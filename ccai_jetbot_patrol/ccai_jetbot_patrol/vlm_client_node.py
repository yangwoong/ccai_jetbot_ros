import base64
import json
import os
import threading
import time
from typing import Any, Dict

import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


DEFAULT_PROMPT = (
    "You are monitoring a patrol robot's camera. Reply with exactly one line: "
    "start with 'RISK:' if you see something dangerous or unusual "
    "(person down, fire, smoke, collision risk, intruder, blocked path), "
    "otherwise start with 'NORMAL:'. Follow the prefix with a summary in Korean "
    "of 50 characters or fewer. No markdown, no extra lines."
)


class VlmClientNode(Node):
    def __init__(self) -> None:
        super().__init__("vlm_client_node")
        self.declare_parameter("api_base_url", os.getenv("CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1"))
        self.declare_parameter("api_key", os.getenv("CCAI_VLLM_API_KEY", ""))
        self.declare_parameter("model", os.getenv("CCAI_VLLM_MODEL", "Qwen/Qwen3-VL-70B-Instruct"))
        self.declare_parameter("image_topic", "/image_raw/compressed")
        self.declare_parameter("trigger_topic", "/ccai/vlm_trigger")
        self.declare_parameter("prompt", DEFAULT_PROMPT)
        self.declare_parameter("summary_max_chars", 50)
        self.declare_parameter("min_interval_seconds", 5.0)
        self.declare_parameter("request_timeout_seconds", 20.0)
        self.declare_parameter("error_event_min_interval_seconds", 30.0)
        # "천장 CSI카메라에 천장 구조를 검출하도록... 엣지를 통해 천장 모양을
        # ... LLM에 보내서 분석" - "none" sends the raw camera frame
        # (unchanged default, used for the D435i room-scan instance and
        # general risk monitoring); "edges" draws a Canny edge overlay
        # highlighting structural boundaries (wall/ceiling seams, light
        # fixture outlines, doorway lintels) before sending, for the
        # CSI-facing instance's ceiling/room-shape questions. YOLO itself
        # isn't used here - its COCO object classes (person, chair, etc.)
        # have nothing resembling "ceiling structure" to detect, and it's
        # separately broken on this hardware anyway (see vision_nav_node's
        # TensorRT/ONNX fallback issues) - edge detection is the actual
        # applicable technique for "천장 모양" and doesn't depend on it.
        self.declare_parameter("image_overlay", "none")

        image_topic = str(self.get_parameter("image_topic").value)
        trigger_topic = str(self.get_parameter("trigger_topic").value)
        self.observation_pub = self.create_publisher(String, "/ccai/vlm_observation", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.create_subscription(CompressedImage, image_topic, self.on_image, 1)
        self.create_subscription(String, trigger_topic, self.on_trigger, 10)

        self.last_request_at = 0.0
        self.inflight = False
        self.triggered = False
        self.pending_question = ""
        self.latest_frame = None
        self.last_error_event_at = 0.0
        self.cv2 = None
        self.np = None
        if str(self.get_parameter("image_overlay").value) == "edges":
            try:
                import cv2
                import numpy as np

                self.cv2 = cv2
                self.np = np
            except Exception as exc:
                self.get_logger().warning(f"image_overlay=edges requested but cv2 unavailable ({exc}); sending raw frames")
        self.get_logger().info(f"vlm_client_node ready, image_topic={image_topic}, trigger_topic={trigger_topic}")

    def on_trigger(self, msg: String) -> None:
        self.triggered = True
        question = ""
        try:
            payload = json.loads(msg.data)
            question = str(payload.get("question", "")).strip()
        except (json.JSONDecodeError, AttributeError):
            pass
        if question:
            self.pending_question = question

    def on_image(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        due = now - self.last_request_at >= float(self.get_parameter("min_interval_seconds").value)
        if self.inflight or not (due or self.triggered):
            return
        was_triggered = self.triggered
        self.last_request_at = now
        self.inflight = True
        self.triggered = False
        question = self.pending_question
        self.pending_question = ""
        image_bytes = bytes(msg.data)
        image_format = msg.format
        if self.cv2 is not None:
            overlaid = self.apply_edge_overlay(image_bytes)
            if overlaid is not None:
                image_bytes, image_format = overlaid
        threading.Thread(
            target=self.analyze_image, args=(image_bytes, image_format, question, was_triggered), daemon=True
        ).start()

    def apply_edge_overlay(self, image_bytes: bytes):
        """Draw a Canny edge overlay highlighting structural boundaries
        (wall/ceiling seams, light fixture outlines, doorway lintels) so the
        VLM gets the same kind of visual aid for "천장 모양" that
        depth_nav_node's drivable-floor overlay gives it for the drive view.
        Returns (jpeg_bytes, "jpeg") or None to fall back to the raw frame
        (e.g. on a decode failure)."""
        try:
            array = self.np.frombuffer(image_bytes, dtype=self.np.uint8)
            frame = self.cv2.imdecode(array, self.cv2.IMREAD_COLOR)
            if frame is None:
                return None
            gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
            gray = self.cv2.GaussianBlur(gray, (5, 5), 0)
            edges = self.cv2.Canny(gray, 50, 150)
            edges = self.cv2.dilate(edges, self.np.ones((2, 2), self.np.uint8), iterations=1)
            overlay = frame.copy()
            overlay[edges > 0] = (0, 255, 255)  # bright cyan-yellow, stands out against a ceiling
            blended = self.cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
            ok, encoded = self.cv2.imencode(".jpg", blended)
            if not ok:
                return None
            return bytes(encoded.tobytes()), "jpeg"
        except Exception as exc:
            self.get_logger().debug(f"edge overlay failed, using raw frame: {exc}")
            return None

    def analyze_image(self, image_bytes: bytes, image_format: str, question: str = "", was_triggered: bool = False) -> None:
        try:
            content = self.call_vlm(image_bytes, image_format, question)
            observation = self.parse_observation(content, question)
            self.observation_pub.publish(String(data=json.dumps(observation, ensure_ascii=False)))
        except Exception as exc:
            self.get_logger().warning(f"vlm request failed: {exc}")
            if was_triggered:
                # An explicit on-demand request ("영상 분석해", location inspection)
                # deserves an immediate answer even if it's "it failed" - silently
                # eating this behind the ambient periodic-scan throttle previously
                # made it look like the command was ignored entirely.
                self.event_pub.publish(String(data=f"vlm analysis failed: {exc}"))
            else:
                self.report_error_throttled(f"vlm request failed: {exc}")
        finally:
            self.inflight = False

    def report_error_throttled(self, text: str) -> None:
        # Failures used to only go to the ROS logger, so a misconfigured/unreachable
        # H200 endpoint silently produced zero observations with no visible trace
        # in Telegram/web chat. Surface it (rate-limited so a persistent outage
        # doesn't spam every request).
        min_interval = float(self.get_parameter("error_event_min_interval_seconds").value)
        now = time.monotonic()
        if now - self.last_error_event_at < min_interval:
            return
        self.last_error_event_at = now
        self.event_pub.publish(String(data=text))

    def parse_observation(self, content: str, question: str = "") -> Dict[str, Any]:
        max_chars = int(self.get_parameter("summary_max_chars").value)
        text = content.strip()
        if question:
            # A specific question ("택배가 있는지 확인해줘") isn't a risk judgement,
            # just answer it directly.
            return {"risk": False, "summary": text[:max_chars], "raw": text}
        risk = False
        summary = text
        upper = text.upper()
        if upper.startswith("RISK:"):
            risk = True
            summary = text[len("RISK:"):].strip()
        elif upper.startswith("NORMAL:"):
            risk = False
            summary = text[len("NORMAL:"):].strip()
        summary = summary[:max_chars]
        return {"risk": risk, "summary": summary, "raw": text}

    def build_prompt(self, question: str) -> str:
        max_chars = int(self.get_parameter("summary_max_chars").value)
        if question:
            return (
                "You are a patrol robot's camera assistant. Answer the following "
                f"question about what the camera currently sees, in Korean, in "
                f"{max_chars} characters or fewer, one line, no markdown: {question}"
            )
        return str(self.get_parameter("prompt").value)

    def call_vlm(self, image_bytes: bytes, image_format: str, question: str = "") -> str:
        api_base_url = self.param_or_env("api_base_url", "CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1").rstrip("/")
        api_key = self.param_or_env("api_key", "CCAI_VLLM_API_KEY", "")
        model = self.param_or_env("model", "CCAI_VLLM_MODEL", "Qwen/Qwen3-VL-70B-Instruct")
        prompt = self.build_prompt(question)
        timeout = float(self.get_parameter("request_timeout_seconds").value)

        mime_type = "image/jpeg"
        if "png" in image_format.lower():
            mime_type = "image/png"
        b64_image = base64.b64encode(image_bytes).decode("ascii")

        payload: Dict[str, Any] = {
            "model": model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:{mime_type};base64,{b64_image}"},
                        },
                    ],
                }
            ],
            "max_tokens": 120,
            "temperature": 0.1,
        }
        headers = {"Content-Type": "application/json"}
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"

        response = requests.post(f"{api_base_url}/chat/completions", headers=headers, data=json.dumps(payload), timeout=timeout)
        response.raise_for_status()
        data = response.json()
        return str(data["choices"][0]["message"]["content"]).strip()

    def param_or_env(self, parameter_name: str, env_name: str, default: str) -> str:
        value = str(self.get_parameter(parameter_name).value or "")
        return value or os.getenv(env_name, default)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmClientNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

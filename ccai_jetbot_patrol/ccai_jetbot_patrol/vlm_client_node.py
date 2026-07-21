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


class VlmClientNode(Node):
    def __init__(self) -> None:
        super().__init__("vlm_client_node")
        self.declare_parameter("api_base_url", os.getenv("CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1"))
        self.declare_parameter("api_key", os.getenv("CCAI_VLLM_API_KEY", ""))
        self.declare_parameter("model", os.getenv("CCAI_VLLM_MODEL", "Qwen/Qwen3-VL-32B-Instruct"))
        self.declare_parameter("image_topic", "/image_raw/compressed")
        self.declare_parameter("prompt", "Describe patrol-relevant safety issues in this robot camera image. Be concise.")
        self.declare_parameter("min_interval_seconds", 3.0)
        self.declare_parameter("request_timeout_seconds", 20.0)

        image_topic = str(self.get_parameter("image_topic").value)
        self.observation_pub = self.create_publisher(String, "/ccai/vlm_observation", 10)
        self.create_subscription(CompressedImage, image_topic, self.on_image, 1)

        self.last_request_at = 0.0
        self.inflight = False
        self.get_logger().info(f"vlm_client_node ready, image_topic={image_topic}")

    def on_image(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if self.inflight or now - self.last_request_at < float(self.get_parameter("min_interval_seconds").value):
            return
        self.last_request_at = now
        self.inflight = True
        image_bytes = bytes(msg.data)
        threading.Thread(target=self.analyze_image, args=(image_bytes, msg.format), daemon=True).start()

    def analyze_image(self, image_bytes: bytes, image_format: str) -> None:
        try:
            result = self.call_vlm(image_bytes, image_format)
            self.observation_pub.publish(String(data=result))
        except Exception as exc:
            self.get_logger().warning(f"vlm request failed: {exc}")
        finally:
            self.inflight = False

    def call_vlm(self, image_bytes: bytes, image_format: str) -> str:
        api_base_url = str(self.get_parameter("api_base_url").value).rstrip("/")
        api_key = str(self.get_parameter("api_key").value)
        model = str(self.get_parameter("model").value)
        prompt = str(self.get_parameter("prompt").value)
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
            "max_tokens": 180,
            "temperature": 0.1,
        }
        headers = {"Content-Type": "application/json"}
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"

        response = requests.post(f"{api_base_url}/chat/completions", headers=headers, data=json.dumps(payload), timeout=timeout)
        response.raise_for_status()
        data = response.json()
        return str(data["choices"][0]["message"]["content"]).strip()


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

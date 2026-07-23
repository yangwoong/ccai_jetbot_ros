import json
import os
import threading
import time
from typing import Any, Dict, Optional

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ccai_jetbot_patrol.mission import command_to_json, is_direct_robot_command, parse_mission_command


SYSTEM_PROMPT = """You are the command router for a ROS2 patrol robot.
Return only compact JSON. No markdown.
Allowed command types:
- status: report current robot state
- patrol_start: start autonomous patrol
- patrol_stop: stop robot motion and patrol
- go_home: return to home or charging station
- inspect: go check something, possibly at a named location the robot was
  previously taught (e.g. "정문", "주방", "복도"). Include target as the
  location name if one is mentioned, or empty if the request is about the
  robot's current position. Include text as the specific thing to check,
  in the user's own words (e.g. "택배가 있는지 확인해줘"). If the robot
  doesn't know that location yet it will say so and check from where it is.
- follow_person: follow the requested person or object using the robot camera. Include target (e.g. "person", "backpack").
- move_forward: drive forward continuously until told to stop. Include target as "slow" if the user asked for reduced speed, otherwise leave target empty.
- move_backward: drive backward continuously until told to stop. Include target as "slow" if reduced speed was requested.
- turn_left: turn left in place briefly
- turn_right: turn right in place briefly
- set_speed: change driving speed. Include target as "up" or "down".
- analyze: analyze the current camera view right now and report back
- remember_start: begin recording the path driven from here so it can be saved as a named location
- remember_save: save the current location under a name, whether or not remember_start was used first (if the user just says "here is X, remember/save it" with no prior recording, that's still remember_save - it will save the current view's visual features even without a travel path). Include target as the location name mentioned by the user (e.g. "정문", "작은방").
- say: if no robot action is requested. Include text.
JSON schema: {"type":"status|patrol_start|patrol_stop|go_home|inspect|follow_person|move_forward|move_backward|turn_left|turn_right|set_speed|analyze|remember_start|remember_save|say","target":"","text":""}
"""


class LlmControlNode(Node):
    def __init__(self) -> None:
        super().__init__("llm_control_node")
        self.declare_parameter("api_base_url", os.getenv("CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1"))
        self.declare_parameter("api_key", os.getenv("CCAI_VLLM_API_KEY", ""))
        self.declare_parameter("model", os.getenv("CCAI_VLLM_MODEL", "Qwen/Qwen3-VL-70B-Instruct"))
        self.declare_parameter("request_timeout_seconds", 20.0)
        self.declare_parameter("health_check_seconds", 30.0)

        self.command_pub = self.create_publisher(String, "/ccai/mission_command", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.llm_response_pub = self.create_publisher(String, "/ccai/llm_response", 10)
        self.llm_status_pub = self.create_publisher(String, "/ccai/llm_status", 10)
        self.create_subscription(String, "/ccai/admin_text", self.on_admin_text, 10)

        self.last_status = {
            "connected": False,
            "api_base_url": self.param_or_env("api_base_url", "CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1"),
            "model": self.param_or_env("model", "CCAI_VLLM_MODEL", "Qwen/Qwen3-VL-70B-Instruct"),
            "message": "not checked",
            "checked_at": 0.0,
        }
        self.create_timer(float(self.get_parameter("health_check_seconds").value), self.check_llm)
        self.check_llm()
        self.get_logger().info("llm_control_node ready")

    def on_admin_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        lowered = text.lower()
        if lowered in {"llm status", "llm 연결상태", "llm 상태", "ai 상태", "ai 연결상태"}:
            self.check_llm()
            self.publish_llm_status_event()
            return

        direct_command = parse_mission_command(text)
        if is_direct_robot_command(direct_command):
            self.publish_command(direct_command, "direct")
            return

        threading.Thread(target=self.resolve_with_llm, args=(text,), daemon=True).start()

    def resolve_with_llm(self, text: str) -> None:
        try:
            content = self.call_text_llm(text)
            command_text = self.extract_json_object(content) or content
            command = parse_mission_command(command_text)
            if command.type == "say":
                command.text = command.text or text
            self.publish_response("llm", text, command, content)
            self.publish_command(command, "llm")
        except Exception as exc:
            self.last_status.update({"connected": False, "message": str(exc), "checked_at": time.time()})
            fallback = parse_mission_command(text)
            self.publish_response("fallback", text, fallback, "llm unavailable")
            if fallback.type == "say":
                self.publish_event("LLM unavailable and command was not recognized: " + text)
            else:
                self.publish_command(fallback, "fallback")

    def call_text_llm(self, text: str) -> str:
        api_base_url = self.param_or_env("api_base_url", "CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1").rstrip("/")
        api_key = self.param_or_env("api_key", "CCAI_VLLM_API_KEY", "")
        model = self.param_or_env("model", "CCAI_VLLM_MODEL", "Qwen/Qwen3-VL-70B-Instruct")
        timeout = float(self.get_parameter("request_timeout_seconds").value)
        payload = {
            "model": model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": text},
            ],
            "max_tokens": 120,
            "temperature": 0.0,
        }
        headers = {"Content-Type": "application/json"}
        if api_key:
            headers["Authorization"] = "Bearer " + api_key
        response = requests.post(api_base_url + "/chat/completions", headers=headers, data=json.dumps(payload), timeout=timeout)
        response.raise_for_status()
        data = response.json()
        self.last_status.update({"connected": True, "message": "ok", "checked_at": time.time()})
        return str(data["choices"][0]["message"]["content"]).strip()

    def check_llm(self) -> None:
        api_base_url = self.param_or_env("api_base_url", "CCAI_VLLM_API_BASE_URL", "http://127.0.0.1:8000/v1").rstrip("/")
        api_key = self.param_or_env("api_key", "CCAI_VLLM_API_KEY", "")
        headers = {}
        if api_key:
            headers["Authorization"] = "Bearer " + api_key
        try:
            response = requests.get(api_base_url + "/models", headers=headers, timeout=5)
            response.raise_for_status()
            self.last_status.update({"connected": True, "message": "ok", "checked_at": time.time()})
        except Exception as exc:
            self.last_status.update({"connected": False, "message": str(exc), "checked_at": time.time()})
        self.llm_status_pub.publish(String(data=json.dumps(self.last_status, ensure_ascii=False)))

    def publish_command(self, command, source: str) -> None:
        payload = command_to_json(command)
        self.command_pub.publish(String(data=payload))
        self.publish_event("command routed by " + source + ": " + payload)

    def publish_response(self, source: str, prompt: str, command, raw_response: str) -> None:
        payload = {
            "source": source,
            "prompt": prompt,
            "command": {
                "type": command.type,
                "target": command.target,
                "text": command.text,
            },
            "raw_response": raw_response,
        }
        self.llm_response_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_llm_status_event(self) -> None:
        self.publish_event("LLM status: " + json.dumps(self.last_status, ensure_ascii=False))

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def param_or_env(self, parameter_name: str, env_name: str, default: str) -> str:
        value = str(self.get_parameter(parameter_name).value or "")
        return value or os.getenv(env_name, default)

    def extract_json_object(self, text: str) -> Optional[str]:
        start = text.find("{")
        end = text.rfind("}")
        if start == -1 or end == -1 or end <= start:
            return None
        return text[start : end + 1]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

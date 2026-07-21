import os
import time

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelegramBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("telegram_bridge_node")
        self.declare_parameter("bot_token", os.getenv("CCAI_TELEGRAM_BOT_TOKEN", ""))
        self.declare_parameter("allowed_chat_id", os.getenv("CCAI_TELEGRAM_ALLOWED_CHAT_ID", ""))
        self.declare_parameter("poll_seconds", 2.0)
        self.command_pub = self.create_publisher(String, "/ccai/mission_command", 10)
        self.create_subscription(String, "/ccai/events", self.on_event, 10)
        self.offset = 0
        self.create_timer(float(self.get_parameter("poll_seconds").value), self.poll)
        self.get_logger().info("telegram_bridge_node ready")

    def api_url(self, method: str) -> str:
        token = str(self.get_parameter("bot_token").value)
        return f"https://api.telegram.org/bot{token}/{method}"

    def poll(self) -> None:
        token = str(self.get_parameter("bot_token").value)
        if not token:
            return
        try:
            response = requests.get(self.api_url("getUpdates"), params={"timeout": 1, "offset": self.offset}, timeout=5)
            response.raise_for_status()
            for update in response.json().get("result", []):
                self.offset = max(self.offset, int(update["update_id"]) + 1)
                message = update.get("message", {})
                chat_id = str(message.get("chat", {}).get("id", ""))
                text = message.get("text", "")
                if self.is_allowed(chat_id) and text:
                    self.command_pub.publish(String(data=text))
                    self.send_message(chat_id, f"accepted: {text}")
        except Exception as exc:
            self.get_logger().warning(f"telegram poll failed: {exc}")

    def on_event(self, msg: String) -> None:
        chat_id = str(self.get_parameter("allowed_chat_id").value)
        if chat_id:
            self.send_message(chat_id, msg.data)

    def is_allowed(self, chat_id: str) -> bool:
        allowed = str(self.get_parameter("allowed_chat_id").value)
        return bool(chat_id) and (not allowed or chat_id == allowed)

    def send_message(self, chat_id: str, text: str) -> None:
        token = str(self.get_parameter("bot_token").value)
        if not token:
            return
        try:
            requests.post(self.api_url("sendMessage"), data={"chat_id": chat_id, "text": text[:3900]}, timeout=5)
            time.sleep(0.1)
        except Exception as exc:
            self.get_logger().warning(f"telegram send failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelegramBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

import json
import os
import subprocess
from pathlib import Path
from typing import Dict

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OtaAgentNode(Node):
    def __init__(self) -> None:
        super().__init__("ota_agent_node")
        self.declare_parameter("manifest_url", os.getenv("CCAI_OTA_MANIFEST_URL", ""))
        self.declare_parameter("current_version_file", "data/current_version.txt")
        self.declare_parameter("check_interval_seconds", 300.0)
        self.declare_parameter("auto_apply", False)
        self.declare_parameter("workdir", ".")
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.create_timer(float(self.get_parameter("check_interval_seconds").value), self.check_once)
        self.get_logger().info("ota_agent_node ready")

    def check_once(self) -> None:
        manifest_url = self.param_or_env("manifest_url", "CCAI_OTA_MANIFEST_URL", "")
        if not manifest_url:
            return
        try:
            manifest = requests.get(manifest_url, timeout=10).json()
            target_version = str(manifest.get("version", ""))
            current_version = self.current_version()
            if target_version and target_version != current_version:
                self.report(f"ota available: {current_version or 'none'} -> {target_version}")
                if bool(self.get_parameter("auto_apply").value):
                    self.apply_manifest(manifest)
            else:
                self.get_logger().debug("ota up to date")
        except Exception as exc:
            self.get_logger().warning(f"ota check failed: {exc}")

    def current_version(self) -> str:
        path = Path(str(self.get_parameter("current_version_file").value))
        if path.exists():
            return path.read_text(encoding="utf-8").strip()
        return ""

    def apply_manifest(self, manifest: Dict) -> None:
        commands = manifest.get("commands", [])
        if not isinstance(commands, list):
            self.report("ota manifest rejected: commands must be a list")
            return
        workdir = str(self.get_parameter("workdir").value)
        for command in commands:
            if not isinstance(command, str):
                self.report("ota command skipped: non-string command")
                continue
            self.report(f"ota running: {command}")
            result = subprocess.run(
                command.split(),
                cwd=workdir,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                timeout=600,
                check=False,
            )
            if result.returncode != 0:
                self.report(f"ota failed: {command}: {result.stderr[-500:]}")
                return
        version_file = Path(str(self.get_parameter("current_version_file").value))
        version_file.parent.mkdir(parents=True, exist_ok=True)
        version_file.write_text(str(manifest.get("version", "")), encoding="utf-8")
        self.report("ota applied")

    def report(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def param_or_env(self, parameter_name: str, env_name: str, default: str) -> str:
        value = str(self.get_parameter(parameter_name).value or "")
        return value or os.getenv(env_name, default)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OtaAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

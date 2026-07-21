import json
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ccai_jetbot_patrol.mission import parse_mission_command


class PatrolState(str, Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    INSPECTING = "inspecting"
    RETURNING_HOME = "returning_home"
    STOPPED = "stopped"


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_node")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("heartbeat_seconds", 2.0)
        self.declare_parameter("safe_stop_on_idle", True)

        self.state = PatrolState.IDLE
        self.current_target = ""
        self.last_vlm_summary = ""

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.create_subscription(String, "/ccai/mission_command", self.on_mission_command, 10)
        self.create_subscription(String, "/ccai/vlm_observation", self.on_vlm_observation, 10)

        heartbeat = float(self.get_parameter("heartbeat_seconds").value)
        self.create_timer(heartbeat, self.publish_status)
        self.create_timer(0.2, self.drive_loop)
        self.get_logger().info("patrol_node ready")

    def on_mission_command(self, msg: String) -> None:
        command = parse_mission_command(msg.data)
        self.get_logger().info(f"mission command: {command.type}")

        if command.type == "patrol_start":
            self.state = PatrolState.PATROLLING
            self.current_target = ""
            self.publish_event("patrol started")
        elif command.type == "patrol_stop":
            self.state = PatrolState.STOPPED
            self.stop_motion()
            self.publish_event("patrol stopped")
        elif command.type == "go_home":
            self.state = PatrolState.RETURNING_HOME
            self.current_target = "home"
            self.publish_event("returning home")
        elif command.type == "inspect":
            self.state = PatrolState.INSPECTING
            self.current_target = command.target
            self.publish_event(f"inspecting {command.target}")
        elif command.type == "status":
            self.publish_status()
            self.publish_event(self.status_text())
        elif command.type == "say":
            self.publish_event(command.text or command.raw)
        else:
            self.publish_event(f"unknown command: {command.raw}")

    def on_vlm_observation(self, msg: String) -> None:
        self.last_vlm_summary = msg.data
        if self.state == PatrolState.PATROLLING and any(word in msg.data.lower() for word in ["person", "hazard", "fire", "blocked"]):
            self.publish_event(f"attention required: {msg.data[:180]}")

    def drive_loop(self) -> None:
        twist = Twist()
        linear_speed = float(self.get_parameter("linear_speed").value)
        angular_speed = float(self.get_parameter("angular_speed").value)

        if self.state == PatrolState.PATROLLING:
            twist.linear.x = linear_speed
        elif self.state == PatrolState.INSPECTING:
            twist.angular.z = angular_speed
        elif self.state == PatrolState.RETURNING_HOME:
            twist.linear.x = linear_speed * 0.7
            twist.angular.z = angular_speed * 0.25
        elif bool(self.get_parameter("safe_stop_on_idle").value):
            self.stop_motion()
            return

        self.cmd_vel_pub.publish(twist)

    def stop_motion(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def publish_status(self) -> None:
        payload = {
            "node": self.get_name(),
            "state": self.state.value,
            "target": self.current_target,
            "last_vlm_summary": self.last_vlm_summary,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def status_text(self) -> str:
        return "status: state={0}, target={1}, last_vlm={2}".format(
            self.state.value,
            self.current_target or "none",
            (self.last_vlm_summary or "none")[:120],
        )

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

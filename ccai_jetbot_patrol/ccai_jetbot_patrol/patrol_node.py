import json
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ccai_jetbot_patrol.mission import parse_mission_command


class PatrolState(str, Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    FOLLOWING_PERSON = "following_person"
    INSPECTING = "inspecting"
    RETURNING_HOME = "returning_home"
    STOPPED = "stopped"
    MANUAL = "manual"


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_node")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("heartbeat_seconds", 2.0)
        self.declare_parameter("safe_stop_on_idle", True)
        self.declare_parameter("patrol_forward_seconds", 4.0)
        self.declare_parameter("patrol_turn_seconds", 1.2)
        self.declare_parameter("use_vision_cmd_vel", True)
        self.declare_parameter("vision_command_timeout_seconds", 0.8)
        self.declare_parameter("manual_move_seconds", 1.5)
        self.declare_parameter("manual_turn_seconds", 0.8)
        self.declare_parameter("speed_step", 0.2)
        self.declare_parameter("min_speed_scale", 0.3)
        self.declare_parameter("max_speed_scale", 2.0)

        self.state = PatrolState.IDLE
        self.current_target = ""
        self.last_vlm_summary = ""
        self.last_vision_status = ""
        self.last_vision_cmd = Twist()
        self.last_vision_cmd_at = 0.0
        self.state_changed_at = time.monotonic()
        self.manual_kind = ""
        self.speed_scale = 1.0
        self.pending_analysis = False

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.vlm_trigger_pub = self.create_publisher(String, "/ccai/vlm_trigger", 10)
        self.create_subscription(String, "/ccai/mission_command", self.on_mission_command, 10)
        self.create_subscription(String, "/ccai/vlm_observation", self.on_vlm_observation, 10)
        self.create_subscription(String, "/ccai/vision_status", self.on_vision_status, 10)
        self.create_subscription(Twist, "/ccai/vision_cmd_vel", self.on_vision_cmd_vel, 10)

        heartbeat = float(self.get_parameter("heartbeat_seconds").value)
        self.create_timer(heartbeat, self.publish_status)
        self.create_timer(0.2, self.drive_loop)
        self.get_logger().info("patrol_node ready")

    def on_mission_command(self, msg: String) -> None:
        command = parse_mission_command(msg.data)
        self.get_logger().info(f"mission command: {command.type}")

        if command.type == "patrol_start":
            self.set_state(PatrolState.PATROLLING)
            self.current_target = ""
            self.publish_event("patrol started")
        elif command.type == "patrol_stop":
            self.set_state(PatrolState.STOPPED)
            self.stop_motion()
            self.publish_event("patrol stopped")
        elif command.type == "go_home":
            self.set_state(PatrolState.RETURNING_HOME)
            self.current_target = "home"
            self.publish_event("returning home")
        elif command.type == "inspect":
            self.set_state(PatrolState.INSPECTING)
            self.current_target = command.target
            self.publish_event(f"inspecting {command.target}")
        elif command.type == "follow_person":
            self.set_state(PatrolState.FOLLOWING_PERSON)
            self.current_target = command.target or "person"
            self.publish_event("following person: {0}".format(self.current_target))
        elif command.type == "status":
            self.publish_status()
            self.publish_event(self.status_text())
        elif command.type in {"move_forward", "move_backward", "turn_left", "turn_right"}:
            self.start_manual_move(command.type)
        elif command.type == "set_speed":
            self.adjust_speed(command.target)
        elif command.type == "analyze":
            self.request_analysis()
        elif command.type == "say":
            self.publish_event(command.text or command.raw)
        else:
            self.publish_event(f"unknown command: {command.raw}")

    def start_manual_move(self, kind: str) -> None:
        self.manual_kind = kind
        self.set_state(PatrolState.MANUAL)
        self.publish_event(f"manual move: {kind}")

    def adjust_speed(self, direction: str) -> None:
        step = float(self.get_parameter("speed_step").value)
        minimum = float(self.get_parameter("min_speed_scale").value)
        maximum = float(self.get_parameter("max_speed_scale").value)
        if direction == "down":
            self.speed_scale = clamp(self.speed_scale - step, minimum, maximum)
        else:
            self.speed_scale = clamp(self.speed_scale + step, minimum, maximum)
        self.publish_event("speed scale set to {0:.2f}".format(self.speed_scale))

    def request_analysis(self) -> None:
        self.pending_analysis = True
        self.vlm_trigger_pub.publish(String(data="on-demand analysis requested"))
        self.publish_event("requesting camera analysis")

    def on_vlm_observation(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            risk = bool(payload.get("risk", False))
            summary = str(payload.get("summary", "")) or msg.data
        except (json.JSONDecodeError, AttributeError):
            summary = msg.data
            risk = any(word in msg.data.lower() for word in ["person", "hazard", "fire", "blocked"])
        self.last_vlm_summary = summary
        if self.pending_analysis:
            self.pending_analysis = False
            self.publish_event(f"analysis result: {summary[:180]}")
        elif risk and self.state in {PatrolState.PATROLLING, PatrolState.FOLLOWING_PERSON, PatrolState.INSPECTING}:
            self.publish_event(f"attention required: {summary[:180]}")

    def on_vision_status(self, msg: String) -> None:
        self.last_vision_status = msg.data

    def on_vision_cmd_vel(self, msg: Twist) -> None:
        self.last_vision_cmd = msg
        self.last_vision_cmd_at = time.monotonic()

    def drive_loop(self) -> None:
        twist = Twist()
        linear_speed = float(self.get_parameter("linear_speed").value) * self.speed_scale
        angular_speed = float(self.get_parameter("angular_speed").value) * self.speed_scale

        if self.state in {PatrolState.PATROLLING, PatrolState.FOLLOWING_PERSON} and self.use_recent_vision_cmd():
            self.cmd_vel_pub.publish(self.last_vision_cmd)
            return

        # Only force a safety stop once vision_nav_node has actually been contributing
        # commands and then goes stale (camera/vision lost mid-mission). If vision has
        # never published anything (disabled, not yet started, or unavailable), fall
        # through to the plain drive pattern below instead of sitting stopped forever.
        if (
            self.state in {PatrolState.PATROLLING, PatrolState.FOLLOWING_PERSON}
            and bool(self.get_parameter("use_vision_cmd_vel").value)
            and self.last_vision_cmd_at > 0.0
        ):
            self.stop_motion()
            return

        if self.state == PatrolState.PATROLLING:
            elapsed = time.monotonic() - self.state_changed_at
            forward_seconds = float(self.get_parameter("patrol_forward_seconds").value)
            turn_seconds = float(self.get_parameter("patrol_turn_seconds").value)
            cycle = max(forward_seconds + turn_seconds, 0.1)
            phase = elapsed % cycle
            if phase < forward_seconds:
                twist.linear.x = linear_speed
            else:
                twist.angular.z = angular_speed
        elif self.state == PatrolState.INSPECTING:
            twist.angular.z = angular_speed
        elif self.state == PatrolState.RETURNING_HOME:
            twist.linear.x = linear_speed * 0.7
            twist.angular.z = angular_speed * 0.25
        elif self.state == PatrolState.MANUAL:
            move_seconds = float(self.get_parameter("manual_move_seconds").value)
            turn_seconds = float(self.get_parameter("manual_turn_seconds").value)
            is_turn = self.manual_kind in {"turn_left", "turn_right"}
            duration = turn_seconds if is_turn else move_seconds
            if time.monotonic() - self.state_changed_at >= duration:
                self.set_state(PatrolState.STOPPED)
                self.stop_motion()
                return
            if self.manual_kind == "move_forward":
                twist.linear.x = linear_speed
            elif self.manual_kind == "move_backward":
                twist.linear.x = -linear_speed
            elif self.manual_kind == "turn_left":
                twist.angular.z = -angular_speed
            elif self.manual_kind == "turn_right":
                twist.angular.z = angular_speed
        elif bool(self.get_parameter("safe_stop_on_idle").value):
            self.stop_motion()
            return

        self.cmd_vel_pub.publish(twist)

    def use_recent_vision_cmd(self) -> bool:
        if not bool(self.get_parameter("use_vision_cmd_vel").value):
            return False
        timeout = float(self.get_parameter("vision_command_timeout_seconds").value)
        return self.last_vision_cmd_at > 0.0 and time.monotonic() - self.last_vision_cmd_at <= timeout

    def stop_motion(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def set_state(self, state: PatrolState) -> None:
        if state != self.state:
            self.state = state
            self.state_changed_at = time.monotonic()

    def publish_status(self) -> None:
        payload = {
            "node": self.get_name(),
            "state": self.state.value,
            "target": self.current_target,
            "last_vlm_summary": self.last_vlm_summary,
            "last_vision_status": self.last_vision_status,
            "patrol_elapsed_seconds": round(time.monotonic() - self.state_changed_at, 1),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def status_text(self) -> str:
        return "status: state={0}, target={1}, last_vlm={2}".format(
            self.state.value,
            self.current_target or "none",
            (self.last_vision_status or self.last_vlm_summary or "none")[:120],
        )

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


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

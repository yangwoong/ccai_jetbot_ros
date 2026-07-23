import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DepthNavNode(Node):
    """Reactive obstacle avoidance + open-direction steering driven by a
    front-facing Intel RealSense D435i's real depth data, instead of the
    monocular CSI heuristics (edge density / floor color / frame-change) that
    vision_nav_node uses. Those heuristics were built because there was no
    depth sensor; with the D435i physically measuring distance, obstacle
    detection no longer has to be inferred indirectly from texture/color and
    is far less prone to the false positives the CSI-only approach hit (see
    docs/vision_and_alerts.md).

    This node publishes to the exact same /ccai/vision_cmd_vel and
    /ccai/vision_status topics vision_nav_node uses, so patrol_node needs no
    changes - it already just drives whatever the most recent vision_cmd_vel
    is. Only one of the two should actually be driving at a time: when this
    node is enabled (D435i connected, pointed forward), set
    vision_nav_node.drive_enabled: false in robot.yaml so the CSI node
    continues doing YOLO object recognition / follow-person duty (its camera
    is ceiling-mounted now and can't usefully see the floor ahead) without
    also fighting this node for the drive topic.

    This is a reactive "seek open space, back off from what's close" patrol
    behavior, not full SLAM/occupancy-grid navigation - there is still no
    odometry or map here. See docs/navigation_roadmap.md for what a further
    RTAB-Map/Nav2 phase on top of this would add.
    """

    def __init__(self) -> None:
        super().__init__("depth_nav_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("depth_image_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("depth_scale_to_meters", 0.001)
        self.declare_parameter("min_valid_depth_m", 0.2)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("linear_speed", 0.045)
        self.declare_parameter("turn_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.22)
        self.declare_parameter("obstacle_stop_distance_m", 0.45)
        self.declare_parameter("min_valid_frame_seconds", 1.0)
        self.declare_parameter("obstacle_avoidance_hold_seconds", 1.0)
        self.declare_parameter("obstacle_clear_confirm_frames", 5)
        self.declare_parameter("obstacle_avoidance_max_seconds", 6.0)
        self.declare_parameter("obstacle_turn_pulse_seconds", 0.3)
        self.declare_parameter("obstacle_pause_seconds", 0.2)
        self.declare_parameter("steer_smoothing_alpha", 0.4)
        self.declare_parameter("speed_ramp_seconds", 1.5)
        self.declare_parameter("speed_ramp_min_factor", 0.35)
        self.declare_parameter("camera_alert_min_interval_seconds", 10.0)

        self.np = None
        self.cv_bridge = None
        self.mode = "idle"
        self.target = ""
        self.last_valid_frame_at = 0.0
        self.forward_streak_started_at = 0.0
        self.obstacle_avoidance_direction = 0
        self.obstacle_avoidance_until = 0.0
        self.obstacle_avoidance_started_at = 0.0
        self.obstacle_clear_streak = 0
        self.smoothed_steer = 0.0
        self.event_throttle_at = {}

        self.cmd_pub = self.create_publisher(Twist, "/ccai/vision_cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/vision_status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("depth_nav_node disabled (enable via 'enabled' param once D435i is connected)")
            return

        try:
            import numpy as np
            from cv_bridge import CvBridge

            self.np = np
            self.cv_bridge = CvBridge()
        except Exception as exc:
            self.publish_event("depth_nav_node unavailable: {0}".format(exc))
            return

        depth_topic = str(self.get_parameter("depth_image_topic").value)
        self.create_subscription(Image, depth_topic, self.on_depth_image, 2)
        self.create_subscription(String, "/ccai/status", self.on_robot_status, 10)
        self.create_timer(0.5, self.watchdog)
        self.publish_event("depth_nav_node ready, depth_topic={0}".format(depth_topic))

    def on_robot_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.mode = str(payload.get("state", "idle"))
        self.target = str(payload.get("target", ""))

    def on_depth_image(self, msg: Image) -> None:
        drives_forward = self.mode == "patrolling" or (self.mode == "manual_drive" and self.target == "move_forward")
        if not drives_forward:
            return
        try:
            depth = self.decode_depth(msg)
        except Exception as exc:
            self.publish_event_throttled("depth frame decode failed: {0}".format(exc), key="depth_decode")
            return
        if depth is None:
            return

        self.last_valid_frame_at = time.monotonic()
        twist, detail = self.compute_patrol_command(depth)
        self.cmd_pub.publish(twist)
        self.publish_status("patrol", detail=detail)

    def decode_depth(self, msg: Image):
        """Convert the raw depth Image to a meters-per-pixel numpy array. D435i
        publishes 16UC1 (millimeters as raw integer counts by default), scaled
        by depth_scale_to_meters - adjust that parameter if a different depth
        unit is configured on the camera.
        """
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        scale = float(self.get_parameter("depth_scale_to_meters").value)
        return frame.astype(self.np.float32) * scale

    def region_distance(self, region) -> float:
        """Median of valid (non-zero = no-return, within sensor range) depth
        readings in a region, in meters. Returns max_valid_depth_m (i.e. "wide
        open") if there's no valid reading at all, rather than treating a
        no-return patch (common on featureless walls/floors just past sensor
        range) as if it were an obstacle at distance zero.
        """
        min_valid = float(self.get_parameter("min_valid_depth_m").value)
        max_valid = float(self.get_parameter("max_valid_depth_m").value)
        valid = region[(region >= min_valid) & (region <= max_valid)]
        if valid.size == 0:
            return max_valid
        return float(self.np.median(valid))

    def compute_patrol_command(self, depth):
        height, width = depth.shape[:2]
        band = depth[int(height * 0.35): int(height * 0.75), :]
        third = width // 3
        left_distance = self.region_distance(band[:, :third])
        center_distance = self.region_distance(band[:, third: 2 * third])
        right_distance = self.region_distance(band[:, 2 * third:])

        twist = Twist()
        stop_distance = float(self.get_parameter("obstacle_stop_distance_m").value)
        turn_speed = float(self.get_parameter("turn_speed").value)
        now = time.monotonic()
        obstacle_now = center_distance < stop_distance

        hold_seconds = float(self.get_parameter("obstacle_avoidance_hold_seconds").value)
        confirm_frames = max(int(self.get_parameter("obstacle_clear_confirm_frames").value), 1)

        if obstacle_now:
            self.obstacle_clear_streak = 0
            was_idle = self.obstacle_avoidance_direction == 0
            if was_idle or now >= self.obstacle_avoidance_until:
                # Real metric distance, so unlike the CSI edge-density proxy
                # this comparison is trustworthy every frame - still commit to
                # one direction per episode for a clean, non-flappy turn.
                self.obstacle_avoidance_direction = -1 if left_distance < right_distance else 1
            if was_idle:
                self.obstacle_avoidance_started_at = now
            self.obstacle_avoidance_until = now + hold_seconds

            max_seconds = float(self.get_parameter("obstacle_avoidance_max_seconds").value)
            if self.obstacle_avoidance_started_at > 0.0 and now - self.obstacle_avoidance_started_at > max_seconds:
                self.obstacle_avoidance_direction = 0
                self.obstacle_avoidance_started_at = 0.0
                self.obstacle_clear_streak = 0
                self.forward_streak_started_at = 0.0
                detail = "depth obstacle avoidance timed out after {0:.1f}s, stopping".format(max_seconds)
                self.publish_event_throttled(detail, key="avoidance_timeout")
                return twist, detail

            pulse = max(float(self.get_parameter("obstacle_turn_pulse_seconds").value), 0.05)
            pause = max(float(self.get_parameter("obstacle_pause_seconds").value), 0.0)
            cycle = pulse + pause
            in_turn_phase = (now % cycle) < pulse
            twist.angular.z = turn_speed * self.obstacle_avoidance_direction if in_turn_phase else 0.0
            detail = "depth obstacle center={0:.2f}m left={1:.2f}m right={2:.2f}m dir={3:+d}".format(
                center_distance, left_distance, right_distance, self.obstacle_avoidance_direction
            )
            self.forward_streak_started_at = 0.0
            return twist, detail

        self.obstacle_clear_streak += 1
        if self.obstacle_avoidance_direction != 0 and (now < self.obstacle_avoidance_until or self.obstacle_clear_streak < confirm_frames):
            pulse = max(float(self.get_parameter("obstacle_turn_pulse_seconds").value), 0.05)
            pause = max(float(self.get_parameter("obstacle_pause_seconds").value), 0.0)
            cycle = pulse + pause
            in_turn_phase = (now % cycle) < pulse
            twist.angular.z = turn_speed * self.obstacle_avoidance_direction if in_turn_phase else 0.0
            detail = "depth clearing obstacle: confirming clear ({0}/{1})".format(self.obstacle_clear_streak, confirm_frames)
            return twist, detail
        self.obstacle_avoidance_direction = 0
        self.obstacle_avoidance_started_at = 0.0

        if self.forward_streak_started_at <= 0.0:
            self.forward_streak_started_at = now
        ramp_seconds = float(self.get_parameter("speed_ramp_seconds").value)
        min_factor = float(self.get_parameter("speed_ramp_min_factor").value)
        ramp_factor = clamp((now - self.forward_streak_started_at) / max(ramp_seconds, 0.01), min_factor, 1.0)

        # Steer toward whichever side has more open space - genuine
        # "seek the clearer path" exploration behavior rather than a fixed
        # route, since there is still no map/localization to plan against.
        max_valid = float(self.get_parameter("max_valid_depth_m").value)
        steer_raw = clamp((right_distance - left_distance) / max_valid, -1.0, 1.0)
        smoothing_alpha = float(self.get_parameter("steer_smoothing_alpha").value)
        self.smoothed_steer = smoothing_alpha * steer_raw + (1.0 - smoothing_alpha) * self.smoothed_steer
        steer = self.smoothed_steer

        twist.linear.x = float(self.get_parameter("linear_speed").value) * ramp_factor
        twist.angular.z = clamp(steer * float(self.get_parameter("max_angular_speed").value), -0.22, 0.22)
        detail = "depth path left={0:.2f}m center={1:.2f}m right={2:.2f}m steer={3:.2f} ramp={4:.2f}".format(
            left_distance, center_distance, right_distance, steer, ramp_factor
        )
        return twist, detail

    def watchdog(self) -> None:
        drives_forward = self.mode == "patrolling" or (self.mode == "manual_drive" and self.target == "move_forward")
        if not drives_forward:
            return
        timeout = float(self.get_parameter("min_valid_frame_seconds").value)
        if self.last_valid_frame_at > 0.0 and time.monotonic() - self.last_valid_frame_at > timeout:
            self.cmd_pub.publish(Twist())
            self.publish_status("depth_camera_timeout", stop=True)
            self.publish_event_throttled("D435i depth frames stopped arriving, stopping motion", key="depth_camera")

    def publish_status(self, state: str, detail: str = "", stop: bool = False) -> None:
        payload = {"state": state, "detail": detail, "stop": stop, "mode": self.mode, "target": self.target}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_event_throttled(self, text: str, key: str = "default") -> None:
        min_interval = float(self.get_parameter("camera_alert_min_interval_seconds").value)
        now = time.monotonic()
        last_at = self.event_throttle_at.get(key, 0.0)
        if now - last_at < min_interval:
            return
        self.event_throttle_at[key] = now
        self.publish_event(text)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

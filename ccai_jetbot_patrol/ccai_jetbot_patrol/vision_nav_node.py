import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class VisionNavNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_nav_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("image_topic", "/image_raw/compressed")
        self.declare_parameter("linear_speed", 0.045)
        self.declare_parameter("turn_speed", 0.16)
        self.declare_parameter("follow_linear_speed", 0.04)
        self.declare_parameter("max_angular_speed", 0.22)
        self.declare_parameter("obstacle_stop_edge_density", 0.16)
        self.declare_parameter("min_valid_frame_seconds", 1.0)
        self.declare_parameter("person_detect_every_n_frames", 5)
        self.declare_parameter("follow_target_area", 0.18)
        self.declare_parameter("follow_min_area", 0.035)

        self.cv2 = None
        self.np = None
        self.hog = None
        self.mode = "idle"
        self.target = ""
        self.frame_count = 0
        self.last_valid_frame_at = 0.0
        self.last_person = None

        self.cmd_pub = self.create_publisher(Twist, "/ccai/vision_cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/vision_status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.create_subscription(CompressedImage, str(self.get_parameter("image_topic").value), self.on_image, 2)
        self.create_subscription(String, "/ccai/status", self.on_robot_status, 10)
        self.create_timer(0.5, self.watchdog)
        self.init_cv()
        self.publish_event("vision_nav_node ready")

    def init_cv(self) -> None:
        try:
            import cv2
            import numpy as np

            self.cv2 = cv2
            self.np = np
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        except Exception as exc:
            self.publish_event("vision unavailable: {0}".format(exc))

    def on_robot_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.mode = str(payload.get("state", "idle"))
        self.target = str(payload.get("target", ""))

    def on_image(self, msg: CompressedImage) -> None:
        if not bool(self.get_parameter("enabled").value) or self.cv2 is None:
            return
        frame = self.decode_frame(msg.data)
        if frame is None or self.is_invalid_frame(frame):
            self.publish_status("invalid_camera", stop=True)
            self.publish_stop()
            return

        self.last_valid_frame_at = time.monotonic()
        self.frame_count += 1
        if self.mode == "patrolling":
            twist, detail = self.compute_patrol_command(frame)
            self.cmd_pub.publish(twist)
            self.publish_status("patrol", detail=detail)
        elif self.mode == "following_person":
            twist, detail = self.compute_follow_command(frame)
            self.cmd_pub.publish(twist)
            self.publish_status("follow_person", detail=detail)

    def decode_frame(self, data) -> object:
        arr = self.np.frombuffer(bytes(data), dtype=self.np.uint8)
        return self.cv2.imdecode(arr, self.cv2.IMREAD_COLOR)

    def is_invalid_frame(self, frame) -> bool:
        if float(frame.std()) < 8.0:
            return True
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        green_mask = self.cv2.inRange(hsv, (45, 60, 40), (85, 255, 255))
        return float(green_mask.mean()) / 255.0 > 0.85

    def compute_patrol_command(self, frame):
        height, width = frame.shape[:2]
        bottom = frame[int(height * 0.52) : height, :]
        gray = self.cv2.cvtColor(bottom, self.cv2.COLOR_BGR2GRAY)
        gray = self.cv2.GaussianBlur(gray, (5, 5), 0)
        edges = self.cv2.Canny(gray, 50, 130)

        third = width // 3
        left_density = self.region_density(edges[:, :third])
        center_density = self.region_density(edges[:, third : 2 * third])
        right_density = self.region_density(edges[:, 2 * third :])
        obstacle_density = self.region_density(edges[int(edges.shape[0] * 0.55) :, third : 2 * third])

        twist = Twist()
        stop_threshold = float(self.get_parameter("obstacle_stop_edge_density").value)
        turn_speed = float(self.get_parameter("turn_speed").value)
        if obstacle_density > stop_threshold:
            twist.angular.z = -turn_speed if left_density < right_density else turn_speed
            detail = "obstacle center={0:.3f}, left={1:.3f}, right={2:.3f}".format(obstacle_density, left_density, right_density)
            return twist, detail

        # Steer away from visual clutter. Lower edge density is treated as clearer floor/path.
        steer = clamp((right_density - left_density) * 2.4, -1.0, 1.0)
        twist.linear.x = float(self.get_parameter("linear_speed").value)
        twist.angular.z = clamp(steer * float(self.get_parameter("max_angular_speed").value), -0.22, 0.22)
        detail = "path left={0:.3f}, center={1:.3f}, right={2:.3f}, steer={3:.2f}".format(
            left_density, center_density, right_density, steer
        )
        return twist, detail

    def compute_follow_command(self, frame):
        person = self.detect_person(frame)
        twist = Twist()
        if person is None:
            twist.angular.z = float(self.get_parameter("turn_speed").value)
            return twist, "person not found, searching"

        x, y, w, h = person
        frame_h, frame_w = frame.shape[:2]
        cx = x + w / 2.0
        offset = (cx - frame_w / 2.0) / max(frame_w / 2.0, 1.0)
        area = float(w * h) / float(frame_w * frame_h)
        target_area = float(self.get_parameter("follow_target_area").value)
        min_area = float(self.get_parameter("follow_min_area").value)

        twist.angular.z = clamp(-offset * float(self.get_parameter("max_angular_speed").value), -0.22, 0.22)
        if area < min_area:
            twist.linear.x = float(self.get_parameter("follow_linear_speed").value)
        elif area < target_area:
            twist.linear.x = float(self.get_parameter("follow_linear_speed").value) * 0.6
        else:
            twist.linear.x = 0.0
        return twist, "person x={0}, y={1}, w={2}, h={3}, area={4:.3f}, offset={5:.2f}".format(x, y, w, h, area, offset)

    def detect_person(self, frame):
        every = max(int(self.get_parameter("person_detect_every_n_frames").value), 1)
        if self.last_person is not None and self.frame_count % every != 0:
            return self.last_person
        try:
            small = self.cv2.resize(frame, (320, 240))
            boxes, _ = self.hog.detectMultiScale(small, winStride=(8, 8), padding=(8, 8), scale=1.05)
        except Exception as exc:
            self.publish_event("person detection failed: {0}".format(exc))
            return self.last_person
        if len(boxes) == 0:
            self.last_person = None
            return None
        frame_h, frame_w = small.shape[:2]
        center_x = frame_w / 2.0
        best = None
        best_score = -1.0
        for x, y, w, h in boxes:
            area = w * h
            center_penalty = abs((x + w / 2.0) - center_x)
            score = area - center_penalty * 2.0
            if score > best_score:
                best_score = score
                best = (int(x), int(y), int(w), int(h))
        self.last_person = best
        return best

    def watchdog(self) -> None:
        if self.mode not in {"patrolling", "following_person"}:
            return
        timeout = float(self.get_parameter("min_valid_frame_seconds").value)
        if time.monotonic() - self.last_valid_frame_at > timeout:
            self.publish_stop()
            self.publish_status("camera_timeout", stop=True)

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_status(self, state: str, detail: str = "", stop: bool = False) -> None:
        payload = {"state": state, "detail": detail, "stop": stop, "mode": self.mode, "target": self.target}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def region_density(self, image) -> float:
        return float(image.mean()) / 255.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


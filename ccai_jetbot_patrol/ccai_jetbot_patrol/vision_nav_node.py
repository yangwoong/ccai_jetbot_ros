import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]

# Exact-match only: short/ambiguous pronouns that would false-positive as substrings
# of unrelated words (e.g. "저" inside "저 가방을 따라가"), so these never shadow a
# real object alias below.
PERSON_ALIASES = {"person", "사람", "me", "나", "저", "사람을", "나를"}

# Substring-matched: distinct enough multi-character nouns that collisions are unlikely.
OBJECT_ALIASES = {
    "가방": "backpack", "배낭": "backpack", "물병": "bottle",
    "의자": "chair", "머그컵": "cup", "컵": "cup",
    "휴대폰": "cell phone", "핸드폰": "cell phone", "스마트폰": "cell phone",
    "책": "book", "우산": "umbrella", "시계": "clock", "노트북": "laptop",
    "자동차": "car", "자전거": "bicycle", "강아지": "dog", "고양이": "cat",
    "박스": "suitcase", "상자": "suitcase",
}


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
        self.declare_parameter("yolo_model_path", "data/models/yolov8n.onnx")
        self.declare_parameter("yolo_input_size", 320)
        self.declare_parameter("yolo_confidence", 0.45)
        self.declare_parameter("yolo_nms_threshold", 0.45)
        self.declare_parameter("yolo_detect_every_n_frames", 3)
        self.declare_parameter("obstacle_box_min_area", 0.05)
        self.declare_parameter("obstacle_path_bottom_fraction", 0.5)
        self.declare_parameter("obstacle_trigger_min_interval_seconds", 4.0)

        self.cv2 = None
        self.np = None
        self.hog = None
        self.yolo_net = None
        self.mode = "idle"
        self.target = ""
        self.frame_count = 0
        self.last_valid_frame_at = 0.0
        self.last_person = None
        self.last_detections = []
        self.last_obstacle_trigger_at = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/ccai/vision_cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/vision_status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.trigger_pub = self.create_publisher(String, "/ccai/vlm_trigger", 10)
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
            return
        self.init_yolo()

    def init_yolo(self) -> None:
        import os

        model_path = str(self.get_parameter("yolo_model_path").value)
        if not os.path.exists(model_path):
            self.publish_event(
                "yolo model not found at {0}; using HOG person detector only "
                "(run scripts/download_yolo_model.sh to enable YOLO)".format(model_path)
            )
            return
        try:
            self.yolo_net = self.cv2.dnn.readNetFromONNX(model_path)
        except Exception as exc:
            self.yolo_net = None
            self.publish_event("yolo model load failed: {0}".format(exc))
            return
        backend = self.select_yolo_backend()
        self.publish_event("yolo model loaded: {0} ({1})".format(model_path, backend))

    def select_yolo_backend(self) -> str:
        """Use CUDA/cuDNN acceleration when the OpenCV build supports it (Jetson L4T
        images ship a CUDA-enabled OpenCV); otherwise CPU. This is the OpenCV DNN
        module, not the standalone TensorRT runtime - use
        scripts/verify_yolo_tensorrt.sh to check TensorRT compatibility separately.
        """
        try:
            if self.cv2.cuda.getCudaEnabledDeviceCount() > 0:
                self.yolo_net.setPreferableBackend(self.cv2.dnn.DNN_BACKEND_CUDA)
                self.yolo_net.setPreferableTarget(self.cv2.dnn.DNN_TARGET_CUDA_FP16)
                return "cuda"
        except Exception as exc:
            self.get_logger().debug("cuda backend unavailable: {0}".format(exc))
        self.yolo_net.setPreferableBackend(self.cv2.dnn.DNN_BACKEND_OPENCV)
        self.yolo_net.setPreferableTarget(self.cv2.dnn.DNN_TARGET_CPU)
        return "cpu"

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

        yolo_obstacle = self.detect_path_obstacle(frame)
        if yolo_obstacle is not None:
            twist.angular.z = -turn_speed if left_density < right_density else turn_speed
            detail = "yolo obstacle: {0} area={1:.3f}".format(yolo_obstacle[0], yolo_obstacle[1])
            self.maybe_trigger_vlm(detail)
            return twist, detail

        if obstacle_density > stop_threshold:
            twist.angular.z = -turn_speed if left_density < right_density else turn_speed
            detail = "obstacle center={0:.3f}, left={1:.3f}, right={2:.3f}".format(obstacle_density, left_density, right_density)
            self.maybe_trigger_vlm(detail)
            return twist, detail

        # Steer away from visual clutter. Lower edge density is treated as clearer floor/path.
        steer = clamp((right_density - left_density) * 2.4, -1.0, 1.0)
        twist.linear.x = float(self.get_parameter("linear_speed").value)
        twist.angular.z = clamp(steer * float(self.get_parameter("max_angular_speed").value), -0.22, 0.22)
        detail = "path left={0:.3f}, center={1:.3f}, right={2:.3f}, steer={3:.2f}".format(
            left_density, center_density, right_density, steer
        )
        return twist, detail

    def detect_path_obstacle(self, frame):
        """Return (class_name, area_fraction) if a YOLO detection blocks the drivable path ahead."""
        if self.yolo_net is None:
            return None
        every = max(int(self.get_parameter("yolo_detect_every_n_frames").value), 1)
        if self.frame_count % every != 0:
            detections = self.last_detections
        else:
            detections = self.run_yolo(frame)
            self.last_detections = detections
        if not detections:
            return None

        frame_h, frame_w = frame.shape[:2]
        bottom_fraction = float(self.get_parameter("obstacle_path_bottom_fraction").value)
        min_area = float(self.get_parameter("obstacle_box_min_area").value)
        path_left = frame_w / 3.0
        path_right = frame_w * 2.0 / 3.0
        path_top = frame_h * (1.0 - bottom_fraction)

        best = None
        for class_id, confidence, x, y, w, h in detections:
            cx = x + w / 2.0
            cy = y + h / 2.0
            area = float(w * h) / float(frame_w * frame_h)
            if area < min_area:
                continue
            if not (path_left <= cx <= path_right and cy >= path_top):
                continue
            if best is None or area > best[1]:
                class_name = COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else "object"
                best = (class_name, area)
        return best

    def maybe_trigger_vlm(self, detail: str) -> None:
        min_interval = float(self.get_parameter("obstacle_trigger_min_interval_seconds").value)
        now = time.monotonic()
        if now - self.last_obstacle_trigger_at < min_interval:
            return
        self.last_obstacle_trigger_at = now
        self.trigger_pub.publish(String(data="obstacle: " + detail))

    def compute_follow_command(self, frame):
        target_class = self.resolve_target_class()
        box = self.detect_target(frame, target_class)
        twist = Twist()
        if box is None:
            twist.angular.z = float(self.get_parameter("turn_speed").value)
            return twist, "{0} not found, searching".format(target_class)

        x, y, w, h = box
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
        return twist, "{0} x={1}, y={2}, w={3}, h={4}, area={5:.3f}, offset={6:.2f}".format(
            target_class, x, y, w, h, area, offset
        )

    def resolve_target_class(self) -> str:
        target = (self.target or "").strip().lower()
        if not target or target in PERSON_ALIASES:
            return "person"
        for alias, class_name in OBJECT_ALIASES.items():
            if alias in target:
                return class_name
        for name in COCO_CLASSES:
            if name in target or target in name:
                return name
        return "person"

    def detect_target(self, frame, target_class: str):
        if self.yolo_net is not None:
            every = max(int(self.get_parameter("yolo_detect_every_n_frames").value), 1)
            if self.frame_count % every != 0 and self.last_detections:
                detections = self.last_detections
            else:
                detections = self.run_yolo(frame)
                self.last_detections = detections
            box = self.best_detection_box(frame, detections, target_class)
            if box is not None or target_class != "person":
                return box
        if target_class == "person":
            return self.detect_person(frame)
        return None

    def best_detection_box(self, frame, detections, target_class: str):
        frame_h, frame_w = frame.shape[:2]
        center_x = frame_w / 2.0
        best = None
        best_score = -1.0
        for class_id, confidence, x, y, w, h in detections:
            class_name = COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else ""
            if class_name != target_class:
                continue
            area = w * h
            center_penalty = abs((x + w / 2.0) - center_x)
            score = area - center_penalty * 2.0
            if score > best_score:
                best_score = score
                best = (int(x), int(y), int(w), int(h))
        return best

    def run_yolo(self, frame):
        """Run the YOLO ONNX model and return [(class_id, confidence, x, y, w, h)] in frame pixel coordinates."""
        try:
            size = int(self.get_parameter("yolo_input_size").value)
            frame_h, frame_w = frame.shape[:2]
            blob = self.cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (size, size), swapRB=True, crop=False)
            self.yolo_net.setInput(blob)
            output = self.yolo_net.forward()
        except Exception as exc:
            self.publish_event("yolo inference failed: {0}".format(exc))
            return []

        # YOLOv8 ONNX export shape: (1, 4 + num_classes, num_boxes)
        output = output[0]
        if output.shape[0] < output.shape[1]:
            output = output.transpose()

        scale_x = frame_w / float(size)
        scale_y = frame_h / float(size)
        confidence_threshold = float(self.get_parameter("yolo_confidence").value)
        nms_threshold = float(self.get_parameter("yolo_nms_threshold").value)

        boxes = []
        confidences = []
        class_ids = []
        for row in output:
            scores = row[4:]
            class_id = int(self.np.argmax(scores))
            confidence = float(scores[class_id])
            if confidence < confidence_threshold:
                continue
            cx, cy, w, h = row[:4]
            x = (cx - w / 2.0) * scale_x
            y = (cy - h / 2.0) * scale_y
            boxes.append([int(x), int(y), int(w * scale_x), int(h * scale_y)])
            confidences.append(confidence)
            class_ids.append(class_id)

        if not boxes:
            return []
        indices = self.cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold)
        detections = []
        for index in self.np.array(indices).flatten():
            x, y, w, h = boxes[int(index)]
            detections.append((class_ids[int(index)], confidences[int(index)], x, y, w, h))
        return detections

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

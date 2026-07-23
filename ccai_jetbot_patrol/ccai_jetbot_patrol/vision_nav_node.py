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
        self.declare_parameter("yolo_engine_path", "data/models/yolov8n.engine")
        self.declare_parameter("yolo_input_size", 320)
        self.declare_parameter("yolo_confidence", 0.45)
        self.declare_parameter("yolo_nms_threshold", 0.45)
        self.declare_parameter("yolo_detect_every_n_frames", 3)
        self.declare_parameter("obstacle_box_min_area", 0.05)
        self.declare_parameter("obstacle_path_bottom_fraction", 0.5)
        self.declare_parameter("obstacle_trigger_min_interval_seconds", 4.0)
        self.declare_parameter("floor_color_diff_threshold", 40.0)
        self.declare_parameter("bottom_change_threshold", 35.0)
        self.declare_parameter("speed_ramp_seconds", 1.5)
        self.declare_parameter("speed_ramp_min_factor", 0.35)
        self.declare_parameter("camera_alert_min_interval_seconds", 10.0)
        self.declare_parameter("debug_image_enabled", True)
        self.declare_parameter("obstacle_avoidance_hold_seconds", 1.0)
        self.declare_parameter("obstacle_clear_confirm_frames", 5)
        self.declare_parameter("steer_direction_noise_floor", 0.01)
        self.declare_parameter("steer_smoothing_alpha", 0.4)
        self.declare_parameter("obstacle_avoidance_max_seconds", 6.0)
        self.declare_parameter("obstacle_turn_pulse_seconds", 0.3)
        self.declare_parameter("obstacle_pause_seconds", 0.2)

        self.cv2 = None
        self.np = None
        self.hog = None
        self.yolo_net = None
        self.trt_yolo = None
        self.mode = "idle"
        self.target = ""
        self.frame_count = 0
        self.last_valid_frame_at = 0.0
        self.last_person = None
        self.last_detections = []
        self.last_obstacle_trigger_at = 0.0
        self.forward_streak_started_at = 0.0
        self.event_throttle_at = {}
        self.yolo_cuda_fallback_tried = False
        self.prev_bottom_mean = None
        self.last_color_distance = 0.0
        self.last_bottom_change = 0.0
        self.last_edge_obstacle_density = 0.0
        self.obstacle_avoidance_direction = 0
        self.obstacle_avoidance_until = 0.0
        self.obstacle_clear_streak = 0
        self.obstacle_avoidance_started_at = 0.0
        self.smoothed_steer = 0.0
        self.last_frame = None
        self.orb = None

        self.cmd_pub = self.create_publisher(Twist, "/ccai/vision_cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/vision_status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.trigger_pub = self.create_publisher(String, "/ccai/vlm_trigger", 10)
        self.debug_image_pub = self.create_publisher(CompressedImage, "/ccai/vision_debug_image", 2)
        self.location_feature_result_pub = self.create_publisher(String, "/ccai/location_feature_result", 10)
        self.create_subscription(CompressedImage, str(self.get_parameter("image_topic").value), self.on_image, 2)
        self.create_subscription(String, "/ccai/status", self.on_robot_status, 10)
        self.create_subscription(String, "/ccai/location_feature_request", self.on_location_feature_request, 10)
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

        engine_path = str(self.get_parameter("yolo_engine_path").value)
        if os.path.exists(engine_path):
            try:
                from ccai_jetbot_patrol.tensorrt_yolo import TensorRTYolo

                self.trt_yolo = TensorRTYolo(engine_path)
                self.publish_event("yolo model loaded via TensorRT: {0}".format(engine_path))
                return
            except Exception as exc:
                self.trt_yolo = None
                self.publish_event(
                    "tensorrt engine load failed ({0}); falling back to OpenCV DNN ONNX".format(exc)
                )

        model_path = str(self.get_parameter("yolo_model_path").value)
        if not os.path.exists(model_path):
            self.publish_event(
                "yolo model not found at {0}; using HOG person detector only "
                "(run scripts/download_yolo_model.sh to enable YOLO, "
                "scripts/build_yolo_tensorrt_engine.sh for TensorRT)".format(model_path)
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

    def yolo_available(self) -> bool:
        return self.trt_yolo is not None or self.yolo_net is not None

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
        drives_forward = self.mode == "patrolling" or (self.mode == "manual_drive" and self.target == "move_forward")
        if frame is None or self.is_invalid_frame(frame):
            self.publish_status("invalid_camera", stop=True)
            self.publish_stop()
            # A blurred/invalid frame (e.g. from a fast turn) shouldn't be able to
            # snap straight back to full speed the instant a good frame reappears -
            # ramp up slowly again, same as after an obstacle turn.
            self.forward_streak_started_at = 0.0
            if drives_forward or self.mode == "following_person":
                self.publish_event_throttled("camera view is invalid, stopping motion", key="camera")
            return

        self.last_valid_frame_at = time.monotonic()
        self.last_frame = frame
        self.frame_count += 1

        # Obstacle signals (and the debug overlay built from them) used to only be
        # computed while actually driving forward, so the "analysis" preview in the
        # web UI would freeze on a stale frame/reading any time the robot was idle,
        # manually turning, or between patrol legs - visibly out of sync with the
        # live camera preview next to it. Now every valid frame is analyzed and
        # published, whether or not it's currently steering anything.
        signals = self.analyze_obstacle(frame)
        if drives_forward:
            twist, detail = self.compute_patrol_command(frame, signals)
            self.cmd_pub.publish(twist)
            self.publish_status("patrol", detail=detail)
        else:
            self.publish_debug_frame(frame, signals["obstacle_now"], self.describe_obstacle(signals, suffix=" (not driving)"))
            if self.mode == "following_person":
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

    def analyze_obstacle(self, frame) -> dict:
        """Compute every obstacle-detection signal for this frame exactly once, so
        the always-on debug overlay and the actual driving decision (when driving)
        see the identical numbers for the identical frame - previously each caller
        recomputed these independently, and the debug overlay was skipped entirely
        outside of drives_forward, which is what made the debug pane visibly lag
        behind the live camera preview.
        """
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
        self.last_edge_obstacle_density = obstacle_density

        stop_threshold = float(self.get_parameter("obstacle_stop_edge_density").value)
        # Edge density alone misses plain/low-texture obstacles: a smooth object
        # (a wall, a box, a leg, anything low-contrast) produces few Canny edges
        # and can read as "clear floor" right up until impact. These two checks
        # don't depend on texture at all - they catch "the floor plane in front
        # of me no longer looks like the floor plane in front of me a moment ago
        # / right under me now", which plain/smooth obstacles still trigger.
        color_obstacle = self.detect_floor_color_obstacle(frame)
        sudden_change = self.detect_sudden_bottom_change(frame)
        yolo_obstacle = self.detect_path_obstacle(frame)
        obstacle_now = yolo_obstacle is not None or obstacle_density > stop_threshold or color_obstacle or sudden_change

        return {
            "left_density": left_density,
            "center_density": center_density,
            "right_density": right_density,
            "obstacle_density": obstacle_density,
            "color_obstacle": color_obstacle,
            "sudden_change": sudden_change,
            "yolo_obstacle": yolo_obstacle,
            "obstacle_now": obstacle_now,
        }

    def describe_obstacle(self, signals: dict, suffix: str = "") -> str:
        yolo_obstacle = signals["yolo_obstacle"]
        if yolo_obstacle is not None:
            return "yolo obstacle: {0} area={1:.3f}{2}".format(yolo_obstacle[0], yolo_obstacle[1], suffix)
        return "obstacle center={0:.3f}, left={1:.3f}, right={2:.3f}, color={3}, sudden={4}{5}".format(
            signals["obstacle_density"], signals["left_density"], signals["right_density"],
            signals["color_obstacle"], signals["sudden_change"], suffix,
        )

    def compute_patrol_command(self, frame, signals: dict):
        left_density = signals["left_density"]
        center_density = signals["center_density"]
        right_density = signals["right_density"]
        color_obstacle = signals["color_obstacle"]
        sudden_change = signals["sudden_change"]
        yolo_obstacle = signals["yolo_obstacle"]
        obstacle_now = signals["obstacle_now"]

        twist = Twist()
        turn_speed = float(self.get_parameter("turn_speed").value)
        now = time.monotonic()

        # Real footage showed the robot flapping rapidly left/right instead of making
        # one clean escape turn. Cause: turn direction was re-decided from
        # left_density vs right_density on *every single frame*, and during motion
        # blur (fast turns, or a close obstacle filling the frame) both densities
        # collapse toward zero - at that point the comparison is deciding on pure
        # noise, and it can flip every 100ms. Now direction is chosen once per
        # avoidance episode and held for obstacle_avoidance_hold_seconds regardless
        # of what later noisy frames say.
        hold_seconds = float(self.get_parameter("obstacle_avoidance_hold_seconds").value)
        confirm_frames = max(int(self.get_parameter("obstacle_clear_confirm_frames").value), 1)

        if obstacle_now:
            self.obstacle_clear_streak = 0
            was_idle = self.obstacle_avoidance_direction == 0
            if was_idle or now >= self.obstacle_avoidance_until:
                noise_floor = float(self.get_parameter("steer_direction_noise_floor").value)
                if abs(left_density - right_density) < noise_floor:
                    # Densities too close to call (often exactly when blur/proximity
                    # makes them meaningless) - keep whatever direction was already
                    # committed, or default to one fixed side rather than guess from noise.
                    self.obstacle_avoidance_direction = self.obstacle_avoidance_direction or 1
                else:
                    self.obstacle_avoidance_direction = -1 if left_density < right_density else 1
            if was_idle:
                self.obstacle_avoidance_started_at = now
            self.obstacle_avoidance_until = now + hold_seconds

            # Real footage in a cluttered/reflective room showed a second failure mode
            # even after the flapping fix above: the robot spun in place *forever*.
            # Cause: turning itself blurs every subsequent frame, and that blur trips
            # the color/sudden-change checks (they were only ever validated against a
            # roughly stationary camera), so obstacle_now stays true for as long as the
            # robot keeps rotating - a self-sustaining loop with no way out. Two
            # independent safeguards:
            #  1. Hard cap: never rotate in avoidance for more than
            #     obstacle_avoidance_max_seconds straight - if exceeded, stop dead and
            #     raise an event instead of spinning indefinitely.
            #  2. Stutter turn: alternate short turn pulses with brief full stops. The
            #     stop portions give the camera time to de-blur, so at least some
            #     frames during avoidance are clean enough to genuinely detect "clear"
            #     and let obstacle_clear_streak progress instead of being reset every
            #     single frame by blur-induced false positives.
            max_seconds = float(self.get_parameter("obstacle_avoidance_max_seconds").value)
            if self.obstacle_avoidance_started_at > 0.0 and now - self.obstacle_avoidance_started_at > max_seconds:
                self.obstacle_avoidance_direction = 0
                self.obstacle_avoidance_started_at = 0.0
                self.obstacle_clear_streak = 0
                self.forward_streak_started_at = 0.0
                detail = "obstacle avoidance timed out after {0:.1f}s, stopping for reassessment".format(max_seconds)
                self.publish_event_throttled(detail, key="avoidance_timeout")
                self.publish_debug_frame(frame, True, detail)
                return twist, detail

            pulse = max(float(self.get_parameter("obstacle_turn_pulse_seconds").value), 0.05)
            pause = max(float(self.get_parameter("obstacle_pause_seconds").value), 0.0)
            cycle = pulse + pause
            in_turn_phase = (now % cycle) < pulse
            twist.angular.z = turn_speed * self.obstacle_avoidance_direction if in_turn_phase else 0.0
            if yolo_obstacle is not None:
                detail = "yolo obstacle: {0} area={1:.3f}, dir={2:+d}".format(
                    yolo_obstacle[0], yolo_obstacle[1], self.obstacle_avoidance_direction
                )
            else:
                detail = "obstacle center={0:.3f}, left={1:.3f}, right={2:.3f}, color={3}, sudden={4}, dir={5:+d}".format(
                    signals["obstacle_density"], left_density, right_density, color_obstacle, sudden_change, self.obstacle_avoidance_direction
                )
            self.maybe_trigger_vlm(detail)
            self.forward_streak_started_at = 0.0
            self.publish_debug_frame(frame, True, detail)
            return twist, detail

        # No obstacle on this frame - but a single clear reading isn't trusted either.
        # Keep turning the committed direction until the hold period has elapsed AND
        # several consecutive frames confirm clear, so one noisy "clear" frame can't
        # let the robot lurch forward into something that's still there (this was the
        # other half of the actual failure: it would occasionally read clear mid-flap
        # and drive straight into the obstacle it was still turning away from).
        self.obstacle_clear_streak += 1
        if self.obstacle_avoidance_direction != 0 and (now < self.obstacle_avoidance_until or self.obstacle_clear_streak < confirm_frames):
            pulse = max(float(self.get_parameter("obstacle_turn_pulse_seconds").value), 0.05)
            pause = max(float(self.get_parameter("obstacle_pause_seconds").value), 0.0)
            cycle = pulse + pause
            in_turn_phase = (now % cycle) < pulse
            twist.angular.z = turn_speed * self.obstacle_avoidance_direction if in_turn_phase else 0.0
            detail = "clearing obstacle: confirming clear ({0}/{1})".format(self.obstacle_clear_streak, confirm_frames)
            self.publish_debug_frame(frame, True, detail)
            return twist, detail
        self.obstacle_avoidance_direction = 0
        self.obstacle_avoidance_started_at = 0.0

        # Steer away from visual clutter. Lower edge density is treated as clearer floor/path.
        # Ramp up from a crawl at the start of every forward run (including right
        # after an obstacle turn above) instead of jumping straight to full speed,
        # in case an obstacle is still close as the path just cleared.
        if self.forward_streak_started_at <= 0.0:
            self.forward_streak_started_at = now
        ramp_seconds = float(self.get_parameter("speed_ramp_seconds").value)
        min_factor = float(self.get_parameter("speed_ramp_min_factor").value)
        ramp_factor = clamp((now - self.forward_streak_started_at) / max(ramp_seconds, 0.01), min_factor, 1.0)

        # Smooth the steering signal too - without this, tiny frame-to-frame noise
        # in left/right density (not just during an obstacle event) made the robot
        # twitch side to side even while just driving down a clear path.
        steer_raw = clamp((right_density - left_density) * 2.4, -1.0, 1.0)
        smoothing_alpha = float(self.get_parameter("steer_smoothing_alpha").value)
        self.smoothed_steer = smoothing_alpha * steer_raw + (1.0 - smoothing_alpha) * self.smoothed_steer
        steer = self.smoothed_steer

        twist.linear.x = float(self.get_parameter("linear_speed").value) * ramp_factor
        twist.angular.z = clamp(steer * float(self.get_parameter("max_angular_speed").value), -0.22, 0.22)
        detail = "path left={0:.3f}, center={1:.3f}, right={2:.3f}, steer={3:.2f}, ramp={4:.2f}".format(
            left_density, center_density, right_density, steer, ramp_factor
        )
        self.publish_debug_frame(frame, False, detail)
        return twist, detail

    def detect_floor_color_obstacle(self, frame) -> bool:
        """Compare a "path ahead" band against a "right under/in front of the
        wheels" reference band, both in the central driving corridor. A plain
        obstacle (wall, box, furniture, a leg) usually differs in color/
        brightness from the floor even when it has almost no texture/edges,
        so this catches cases the Canny edge-density check misses.
        """
        height, width = frame.shape[:2]
        left = width // 3
        right = 2 * width // 3

        reference = frame[int(height * 0.90) : height, left:right]
        path_ahead = frame[int(height * 0.60) : int(height * 0.85), left:right]
        if reference.size == 0 or path_ahead.size == 0:
            return False

        reference_mean = reference.reshape(-1, 3).mean(axis=0)
        path_mean = path_ahead.reshape(-1, 3).mean(axis=0)
        color_distance = float(self.np.linalg.norm(reference_mean - path_mean))
        self.last_color_distance = color_distance
        threshold = float(self.get_parameter("floor_color_diff_threshold").value)
        return color_distance > threshold

    def detect_sudden_bottom_change(self, frame) -> bool:
        """Frame-to-frame color change of the strip right under the wheels. If
        an obstacle has crept close enough to also fill the "reference" floor
        strip used above (so the two bands look similar to each other but
        both changed from a moment ago), the spatial comparison alone won't
        catch it - this temporal check does.
        """
        height, width = frame.shape[:2]
        left = width // 3
        right = 2 * width // 3
        bottom_strip = frame[int(height * 0.90) : height, left:right]
        if bottom_strip.size == 0:
            return False

        current_mean = bottom_strip.reshape(-1, 3).mean(axis=0)
        previous_mean = self.prev_bottom_mean
        self.prev_bottom_mean = current_mean
        if previous_mean is None:
            return False

        change = float(self.np.linalg.norm(current_mean - previous_mean))
        self.last_bottom_change = change
        threshold = float(self.get_parameter("bottom_change_threshold").value)
        return change > threshold

    def publish_debug_frame(self, frame, is_obstacle: bool, detail: str) -> None:
        """Draw what the obstacle detector is looking at directly onto the camera
        frame and publish it, so an admin can watch in the web UI whether a real
        obstacle is being recognized instead of only seeing the raw camera feed.
        Never let a drawing bug take down the vision pipeline - this is purely
        a debug aid.
        """
        if not bool(self.get_parameter("debug_image_enabled").value):
            return
        try:
            debug = frame.copy()
            height, width = debug.shape[:2]
            third = width // 3

            edge_y0 = int(height * 0.52) + int((height - int(height * 0.52)) * 0.55)
            self.cv2.rectangle(debug, (third, edge_y0), (2 * third, height), (0, 255, 255), 1)
            self.cv2.rectangle(debug, (third, int(height * 0.60)), (2 * third, int(height * 0.85)), (255, 255, 0), 1)
            self.cv2.rectangle(debug, (third, int(height * 0.90)), (2 * third, height), (255, 0, 0), 1)

            for class_id, confidence, x, y, w, h in self.last_detections:
                class_name = COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else "obj"
                self.cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
                self.cv2.putText(
                    debug, "{0} {1:.2f}".format(class_name, confidence), (x, max(y - 5, 10)),
                    self.cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1,
                )

            status_text = "OBSTACLE" if is_obstacle else "CLEAR"
            status_color = (0, 0, 255) if is_obstacle else (0, 200, 0)
            self.cv2.putText(debug, status_text, (5, 15), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
            metrics = "edge={0:.3f} color={1:.1f} sudden={2:.1f}".format(
                self.last_edge_obstacle_density, self.last_color_distance, self.last_bottom_change
            )
            self.cv2.putText(debug, metrics, (5, height - 8), self.cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            # A visible timestamp + frame counter lets an admin directly confirm the
            # debug overlay is tracking the live camera in real time rather than
            # trusting it by eye - this is what actually made the earlier "preview
            # and analysis frame look different" report possible to check.
            stamp = "frame #{0} @ {1:.3f}".format(self.frame_count, time.time())
            self.cv2.putText(debug, stamp, (5, 30), self.cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

            ok, encoded = self.cv2.imencode(".jpg", debug, [int(self.cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = encoded.tobytes()
                self.debug_image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug("debug frame draw failed: {0}".format(exc))

    def detect_path_obstacle(self, frame):
        """Return (class_name, area_fraction) if a YOLO detection blocks the drivable path ahead."""
        if not self.yolo_available():
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
        if self.yolo_available():
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
        """Run the YOLO model (TensorRT engine if loaded, else OpenCV DNN ONNX) and
        return [(class_id, confidence, x, y, w, h)] in frame pixel coordinates."""
        size = int(self.get_parameter("yolo_input_size").value)
        frame_h, frame_w = frame.shape[:2]

        if self.trt_yolo is not None:
            try:
                blob = self.cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (size, size), swapRB=True, crop=False)
                output = self.trt_yolo.infer(blob)
                return self.decode_yolo_output(output, frame_w, frame_h, size)
            except Exception as exc:
                self.trt_yolo = None
                self.publish_event(
                    "tensorrt inference failed ({0}); falling back to OpenCV DNN ONNX/HOG".format(exc)
                )
                # Falls through to the cv2.dnn path below, which loads its own
                # net lazily from yolo_model_path if not already loaded.
                if self.yolo_net is None:
                    self.init_yolo_dnn_fallback()

        if self.yolo_net is None:
            return []
        try:
            output = self.run_yolo_inference(frame, size)
        except Exception as exc:
            output = self.recover_from_yolo_failure(frame, size, exc)
            if output is None:
                return []
        return self.decode_yolo_output(output, frame_w, frame_h, size)

    def decode_yolo_output(self, output, frame_w: int, frame_h: int, size: int):
        # YOLOv8 export shape: (1, 4 + num_classes, num_boxes)
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

    def init_yolo_dnn_fallback(self) -> None:
        import os

        model_path = str(self.get_parameter("yolo_model_path").value)
        if not os.path.exists(model_path):
            self.publish_event("yolo onnx fallback unavailable: {0} not found".format(model_path))
            return
        try:
            self.yolo_net = self.cv2.dnn.readNetFromONNX(model_path)
            backend = self.select_yolo_backend()
            self.publish_event("yolo model loaded: {0} ({1})".format(model_path, backend))
        except Exception as exc:
            self.yolo_net = None
            self.publish_event("yolo onnx fallback load failed: {0}".format(exc))

    def run_yolo_inference(self, frame, size: int):
        blob = self.cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (size, size), swapRB=True, crop=False)
        self.yolo_net.setInput(blob)
        return self.yolo_net.forward()

    def recover_from_yolo_failure(self, frame, size: int, exc: Exception):
        """Some OpenCV CUDA DNN builds fail on specific ONNX ops for a given model
        (seen: 'scale_shift' assertion on this L4T OpenCV build with a YOLOv8
        export) even though the model loaded fine. Fall back to the CPU backend
        once rather than spamming an error on every single frame forever; if CPU
        also fails, disable YOLO entirely and keep using the HOG/edge-density path.
        """
        if self.yolo_cuda_fallback_tried:
            self.publish_event_throttled("yolo inference failing repeatedly: {0}".format(exc), key="yolo")
            return None
        self.yolo_cuda_fallback_tried = True
        self.publish_event("yolo inference failed on current backend ({0}); retrying on CPU".format(exc))
        try:
            self.yolo_net.setPreferableBackend(self.cv2.dnn.DNN_BACKEND_OPENCV)
            self.yolo_net.setPreferableTarget(self.cv2.dnn.DNN_TARGET_CPU)
            return self.run_yolo_inference(frame, size)
        except Exception as exc2:
            self.publish_event("yolo inference failed on CPU too ({0}); disabling YOLO, using HOG/edge-density only".format(exc2))
            self.yolo_net = None
            return None

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
        drives_forward = self.mode == "patrolling" or (self.mode == "manual_drive" and self.target == "move_forward")
        if not (drives_forward or self.mode == "following_person"):
            return
        timeout = float(self.get_parameter("min_valid_frame_seconds").value)
        if time.monotonic() - self.last_valid_frame_at > timeout:
            self.publish_stop()
            self.publish_status("camera_timeout", stop=True)
            self.publish_event_throttled("camera frames stopped arriving, stopping motion", key="camera")

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

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

    def region_density(self, image) -> float:
        return float(image.mean()) / 255.0

    def on_location_feature_request(self, msg: String) -> None:
        """Location "teaching" (see LocationStore/patrol_node) so far has only ever
        stored a blind timed move-sequence - it has no idea if it actually arrived
        at the right spot, only that it replayed the same motions. This adds a
        real visual signal: ORB keypoint descriptors captured at teach-time are
        compared against the live frame at arrival-time, so a mismatch (furniture
        moved, wrong location due to drift) can actually be reported instead of
        assumed away.
        """
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        action = payload.get("action", "")
        label = payload.get("label", "")
        if self.cv2 is None or self.last_frame is None:
            self.location_feature_result_pub.publish(String(data=json.dumps(
                {"action": action, "label": label, "error": "no camera frame available"}, ensure_ascii=False
            )))
            return
        try:
            current_descriptors, keypoint_count = self.extract_orb_features(self.last_frame)
        except Exception as exc:
            self.location_feature_result_pub.publish(String(data=json.dumps(
                {"action": action, "label": label, "error": str(exc)}, ensure_ascii=False
            )))
            return

        if action == "capture":
            result = {
                "action": "capture",
                "label": label,
                "descriptors": self.encode_descriptors(current_descriptors),
                "keypoints": keypoint_count,
            }
        elif action == "match":
            stored_descriptors = self.decode_descriptors(payload.get("descriptors", ""))
            ratio, good = self.match_orb_features(stored_descriptors, current_descriptors)
            result = {
                "action": "match",
                "label": label,
                "match_ratio": ratio,
                "good_matches": good,
                "keypoints": keypoint_count,
            }
        else:
            return
        self.location_feature_result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

    def get_orb(self):
        if self.orb is None:
            self.orb = self.cv2.ORB_create(nfeatures=300)
        return self.orb

    def extract_orb_features(self, frame):
        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        _keypoints, descriptors = self.get_orb().detectAndCompute(gray, None)
        count = 0 if descriptors is None else int(descriptors.shape[0])
        return descriptors, count

    def encode_descriptors(self, descriptors) -> str:
        if descriptors is None or descriptors.shape[0] == 0:
            return ""
        import base64

        return "{0}|{1}".format(base64.b64encode(descriptors.tobytes()).decode("ascii"), descriptors.shape[0])

    def decode_descriptors(self, encoded: str):
        if not encoded:
            return None
        import base64

        try:
            data_b64, count_str = encoded.rsplit("|", 1)
            raw = base64.b64decode(data_b64)
            return self.np.frombuffer(raw, dtype=self.np.uint8).reshape(int(count_str), 32)
        except Exception:
            return None

    def match_orb_features(self, stored_descriptors, current_descriptors):
        if (
            stored_descriptors is None
            or current_descriptors is None
            or stored_descriptors.shape[0] == 0
            or current_descriptors.shape[0] == 0
        ):
            return 0.0, 0
        matcher = self.cv2.BFMatcher(self.cv2.NORM_HAMMING, crossCheck=False)
        pairs = matcher.knnMatch(stored_descriptors, current_descriptors, k=2)
        good = 0
        for pair in pairs:
            if len(pair) < 2:
                continue
            best, second = pair
            if best.distance < 0.75 * second.distance:
                good += 1
        ratio = float(good) / float(max(stored_descriptors.shape[0], 1))
        return ratio, good


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

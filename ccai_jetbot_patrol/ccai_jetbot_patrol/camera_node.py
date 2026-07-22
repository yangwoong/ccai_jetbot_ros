import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_backend", "auto")
        self.declare_parameter("use_gstreamer", False)
        self.declare_parameter("force_v4l2", True)
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("jpeg_quality", 45)
        self.declare_parameter("reopen_after_failures", 5)
        self.declare_parameter("reject_invalid_frames", True)
        self.declare_parameter("invalid_green_ratio", 0.85)
        self.declare_parameter("invalid_min_stddev", 8.0)
        self.declare_parameter("output_topic", "/image_raw/compressed")

        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.status_pub = self.create_publisher(String, "/ccai/camera_status", 10)
        self.publisher = self.create_publisher(CompressedImage, str(self.get_parameter("output_topic").value), 2)
        self.cv2 = None
        self.capture = None
        self.failed_reads = 0
        self.backend_index = 0
        self.active_backend = "none"
        self.last_error = ""

        if bool(self.get_parameter("enabled").value):
            self.open_camera()
        else:
            self.publish_event("camera node disabled")

        interval = 1.0 / max(float(self.get_parameter("fps").value), 0.5)
        self.create_timer(interval, self.capture_once)
        self.publish_event("camera node ready")

    def open_camera(self) -> None:
        try:
            import cv2

            self.cv2 = cv2
        except Exception as exc:
            self.publish_event("opencv unavailable: {0}".format(exc))
            return

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        index = int(self.get_parameter("camera_index").value)
        backend = self.next_backend()
        self.active_backend = backend
        if backend == "csi_gstreamer":
            pipeline = (
                "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 "
                "! nvvidconv flip-method=0 ! video/x-raw, width={0}, height={1}, format=BGRx "
                "! videoconvert ! video/x-raw, format=BGR ! appsink"
            ).format(width, height)
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "v4l2_mjpg":
            self.capture = self.cv2.VideoCapture(index, self.cv2.CAP_V4L2)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc("M", "J", "P", "G"))
            self.configure_capture(width, height)
        elif backend == "v4l2_yuyv":
            self.capture = self.cv2.VideoCapture(index, self.cv2.CAP_V4L2)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))
            self.configure_capture(width, height)
        elif backend == "v4l2_default":
            self.capture = self.cv2.VideoCapture(index, self.cv2.CAP_V4L2)
            self.configure_capture(width, height)
        else:
            self.capture = self.cv2.VideoCapture(index)
            self.configure_capture(width, height)

        if not self.capture or not self.capture.isOpened():
            self.last_error = "open failed with backend={0}".format(backend)
            self.publish_event("camera open failed, backend={0}".format(backend))
            self.capture = None
        else:
            self.failed_reads = 0
            self.publish_event("camera opened, backend={0}".format(backend))
        self.publish_status()

    def configure_capture(self, width: int, height: int) -> None:
        self.capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(self.cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))
        self.capture.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)

    def next_backend(self) -> str:
        backend = str(self.get_parameter("camera_backend").value)
        if backend != "auto":
            return backend
        candidates = ["v4l2_mjpg", "v4l2_yuyv", "v4l2_default", "default"]
        if bool(self.get_parameter("use_gstreamer").value):
            candidates.append("csi_gstreamer")
        selected = candidates[self.backend_index % len(candidates)]
        self.backend_index += 1
        return selected

    def capture_once(self) -> None:
        if self.cv2 is None or self.capture is None:
            return
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.failed_reads += 1
            self.last_error = "frame read failed"
            self.publish_event_throttled("camera frame read failed")
            if self.failed_reads >= int(self.get_parameter("reopen_after_failures").value):
                self.reopen_camera()
            self.publish_status()
            return

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        frame = self.cv2.resize(frame, (width, height))
        if bool(self.get_parameter("reject_invalid_frames").value) and self.is_invalid_frame(frame):
            self.failed_reads += 1
            self.last_error = "invalid frame rejected"
            self.publish_event_throttled("camera invalid frame rejected")
            if self.failed_reads >= int(self.get_parameter("reopen_after_failures").value):
                self.reopen_camera()
            self.publish_status()
            return
        self.failed_reads = 0
        self.last_error = ""

        quality = int(self.get_parameter("jpeg_quality").value)
        ok, encoded = self.cv2.imencode(".jpg", frame, [int(self.cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            self.publish_event_throttled("camera jpeg encode failed")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)
        self.publish_status()

    def is_invalid_frame(self, frame) -> bool:
        stddev = float(frame.std())
        if stddev < float(self.get_parameter("invalid_min_stddev").value):
            return True
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        green_mask = self.cv2.inRange(hsv, (45, 60, 40), (85, 255, 255))
        green_ratio = float(green_mask.mean()) / 255.0
        return green_ratio > float(self.get_parameter("invalid_green_ratio").value)

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_event_throttled(self, text: str) -> None:
        now = time.monotonic()
        if not hasattr(self, "_last_event") or now - self._last_event > 5.0:
            self._last_event = now
            self.publish_event(text)

    def reopen_camera(self) -> None:
        self.publish_event("camera reopening after repeated read failures, previous_backend={0}".format(self.active_backend))
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        time.sleep(0.5)
        self.open_camera()

    def publish_status(self) -> None:
        payload = '{{"backend":"{0}","failed_reads":{1},"last_error":"{2}"}}'.format(
            self.active_backend,
            self.failed_reads,
            self.last_error.replace('"', "'"),
        )
        self.status_pub.publish(String(data=payload))

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

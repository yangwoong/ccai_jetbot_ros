import json
import os
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
        self.declare_parameter("use_gstreamer", True)
        self.declare_parameter("force_v4l2", True)
        self.declare_parameter("capture_width", 640)
        self.declare_parameter("capture_height", 480)
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("jpeg_quality", 45)
        self.declare_parameter("reopen_after_failures", 5)
        self.declare_parameter("capture_retry_seconds", 3.0)
        self.declare_parameter("open_probe_frames", 8)
        self.declare_parameter("reject_invalid_frames", True)
        self.declare_parameter("invalid_green_ratio", 0.85)
        self.declare_parameter("invalid_min_stddev", 8.0)
        self.declare_parameter("debug_frame_path", "/tmp/ccai_camera_last_invalid.jpg")
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
        self.last_open_attempt = 0.0

        if bool(self.get_parameter("enabled").value):
            self.open_camera_candidates()
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

        capture_width = int(self.get_parameter("capture_width").value)
        capture_height = int(self.get_parameter("capture_height").value)
        index = int(self.get_parameter("camera_index").value)
        backend = self.next_backend()
        self.active_backend = backend
        if backend == "csi_gstreamer":
            pipeline = (
                "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 "
                "! nvvidconv flip-method=0 ! video/x-raw, width={0}, height={1}, format=BGRx "
                "! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(capture_width, capture_height)
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "csi_gstreamer_legacy":
            pipeline = (
                "nvarguscamerasrc sensor-id={0} ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 "
                "! nvvidconv ! video/x-raw, format=BGRx ! videoconvert "
                "! video/x-raw, width={1}, height={2}, format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(index, capture_width, capture_height)
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_any":
            pipeline = (
                "v4l2src device=/dev/video{0} ! videoconvert "
                "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(index)
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_mjpg_any":
            pipeline = (
                "v4l2src device=/dev/video{0} ! image/jpeg ! jpegdec ! videoconvert "
                "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(index)
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_mjpg":
            pipeline = (
                "v4l2src device=/dev/video{0} ! image/jpeg,width={1},height={2},framerate={3}/1 "
                "! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(index, capture_width, capture_height, int(max(float(self.get_parameter("fps").value), 1.0)))
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_yuyv":
            pipeline = (
                "v4l2src device=/dev/video{0} ! video/x-raw,format=YUY2,width={1},height={2},framerate={3}/1 "
                "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(index, capture_width, capture_height, int(max(float(self.get_parameter("fps").value), 1.0)))
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "v4l2_mjpg":
            self.capture = self.cv2.VideoCapture(index, self.cv2.CAP_V4L2)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc("M", "J", "P", "G"))
            self.configure_capture(capture_width, capture_height)
        elif backend == "v4l2_yuyv":
            self.capture = self.cv2.VideoCapture(index, self.cv2.CAP_V4L2)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))
            self.configure_capture(capture_width, capture_height)
        elif backend == "v4l2_default":
            self.capture = self.cv2.VideoCapture(index, self.cv2.CAP_V4L2)
            self.configure_capture(capture_width, capture_height)
        else:
            self.capture = self.cv2.VideoCapture(index)
            self.configure_capture(capture_width, capture_height)

        if not self.capture or not self.capture.isOpened():
            self.last_error = "open failed with backend={0}".format(backend)
            self.publish_event("camera open failed, backend={0}".format(backend))
            self.capture = None
        elif not self.probe_capture():
            self.last_error = "probe failed with backend={0}".format(backend)
            self.publish_event("camera probe failed, backend={0}".format(backend))
            self.capture.release()
            self.capture = None
        else:
            self.failed_reads = 0
            self.publish_event("camera opened, backend={0}".format(backend))
        self.publish_status()

    def open_camera_candidates(self) -> None:
        self.last_open_attempt = time.monotonic()
        for _ in range(len(self.backend_candidates())):
            self.open_camera()
            if self.capture is not None:
                return

    def configure_capture(self, width: int, height: int) -> None:
        self.capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(self.cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))
        self.capture.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)

    def next_backend(self) -> str:
        candidates = self.backend_candidates()
        selected = candidates[self.backend_index % len(candidates)]
        self.backend_index += 1
        return selected

    def backend_candidates(self):
        backend = str(self.get_parameter("camera_backend").value)
        if backend != "auto":
            return [backend]

        candidates = []
        if bool(self.get_parameter("use_gstreamer").value) or os.path.exists("/tmp/argus_socket"):
            candidates.extend(["csi_gstreamer", "csi_gstreamer_legacy"])

        candidates.extend([
            "gst_v4l2_any",
            "gst_v4l2_mjpg_any",
            "v4l2_mjpg",
            "gst_v4l2_mjpg",
            "v4l2_yuyv",
            "gst_v4l2_yuyv",
            "v4l2_default",
            "default",
        ])
        return candidates

    def probe_capture(self) -> bool:
        probe_frames = max(int(self.get_parameter("open_probe_frames").value), 1)
        last_frame = None
        for _ in range(probe_frames):
            ok, frame = self.capture.read()
            if ok and frame is not None:
                last_frame = frame
                if not self.is_invalid_frame(self.resize_output(frame)):
                    return True
            time.sleep(0.05)
        if last_frame is not None:
            resized = self.resize_output(last_frame)
            self.last_error = self.describe_invalid_frame(resized)
            self.save_debug_frame(resized)
        return False

    def capture_once(self) -> None:
        if self.cv2 is None or self.capture is None:
            retry_seconds = float(self.get_parameter("capture_retry_seconds").value)
            if time.monotonic() - self.last_open_attempt >= retry_seconds:
                self.publish_event_throttled("camera not open, retrying")
                self.open_camera_candidates()
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
        frame = self.resize_output(frame)
        if bool(self.get_parameter("reject_invalid_frames").value) and self.is_invalid_frame(frame):
            self.failed_reads += 1
            self.last_error = self.describe_invalid_frame(frame)
            self.save_debug_frame(frame)
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

    def describe_invalid_frame(self, frame) -> str:
        hsv = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2HSV)
        green_mask = self.cv2.inRange(hsv, (45, 60, 40), (85, 255, 255))
        green_ratio = float(green_mask.mean()) / 255.0
        return "invalid frame stddev={0:.2f}, green_ratio={1:.2f}".format(float(frame.std()), green_ratio)

    def resize_output(self, frame):
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        return self.cv2.resize(frame, (width, height))

    def save_debug_frame(self, frame) -> None:
        path = str(self.get_parameter("debug_frame_path").value)
        if not path:
            return
        try:
            self.cv2.imwrite(path, frame)
        except Exception as exc:
            self.get_logger().debug("failed to write camera debug frame: {0}".format(exc))

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
        self.open_camera_candidates()

    def publish_status(self) -> None:
        payload = {
            "backend": self.active_backend,
            "failed_reads": self.failed_reads,
            "last_error": self.last_error,
            "retry_seconds": float(self.get_parameter("capture_retry_seconds").value),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

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

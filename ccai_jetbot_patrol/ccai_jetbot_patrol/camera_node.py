import json
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class UrlSnapshotCapture:
    def __init__(self, url: str, cv2_module, timeout: float) -> None:
        import numpy as np
        import requests

        self.url = url
        self.cv2 = cv2_module
        self.np = np
        self.requests = requests
        self.timeout = timeout

    def isOpened(self) -> bool:
        return bool(self.url)

    def read(self):
        try:
            response = self.requests.get(self.url, timeout=self.timeout)
        except self.requests.exceptions.RequestException:
            return False, None
        if response.status_code != 200:
            return False, None
        array = self.np.frombuffer(response.content, dtype=self.np.uint8)
        if array.size == 0:
            return False, None
        frame = self.cv2.imdecode(array, self.cv2.IMREAD_COLOR)
        return frame is not None, frame

    def release(self) -> None:
        return


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_device", "")
        self.declare_parameter("camera_url", "")
        self.declare_parameter("camera_url_timeout_seconds", 1.5)
        self.declare_parameter("camera_mode", "usb")
        self.declare_parameter("camera_backend", "auto")
        self.declare_parameter("use_gstreamer", False)
        self.declare_parameter("force_v4l2", True)
        self.declare_parameter("capture_width", 640)
        self.declare_parameter("capture_height", 480)
        self.declare_parameter("csi_sensor_id", 0)
        self.declare_parameter("csi_sensor_mode", 3)
        self.declare_parameter("csi_capture_width", 816)
        self.declare_parameter("csi_capture_height", 616)
        self.declare_parameter("csi_fps", 30)
        self.declare_parameter("csi_flip_method", 0)
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("jpeg_quality", 45)
        self.declare_parameter("reopen_after_failures", 5)
        self.declare_parameter("capture_retry_seconds", 3.0)
        self.declare_parameter("max_open_attempts", 0)
        self.declare_parameter("open_probe_frames", 8)
        self.declare_parameter("reject_invalid_frames", True)
        self.declare_parameter("reject_invalid_on_open", False)
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
        self.active_pipeline = ""
        self.last_error = ""
        self.last_open_attempt = 0.0
        self.open_attempt_rounds = 0
        self.open_retry_exhausted = False

        if self.camera_enabled():
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
        source = self.camera_source()
        backend = self.next_backend()
        self.active_backend = backend
        if backend == "csi_jetbot":
            pipeline = self.csi_pipeline(include_sensor_id=False, include_sensor_mode=True, jetbot_exact=True)
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "csi_jetcam":
            pipeline = self.csi_pipeline(include_sensor_id=True, include_sensor_mode=False, jetbot_exact=True)
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "csi_gstreamer":
            pipeline = (
                "nvarguscamerasrc sensor-id={0} ! video/x-raw(memory:NVMM), width=1280, height=720, "
                "format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv flip-method={1} "
                "! video/x-raw, width=(int){2}, height=(int){3}, format=(string)BGRx "
                "! videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(
                int(self.get_parameter("csi_sensor_id").value),
                int(self.get_parameter("csi_flip_method").value),
                int(self.get_parameter("width").value),
                int(self.get_parameter("height").value),
            )
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "url":
            self.active_pipeline = str(self.get_parameter("camera_url").value)
            self.capture = UrlSnapshotCapture(
                self.active_pipeline,
                self.cv2,
                float(self.get_parameter("camera_url_timeout_seconds").value),
            )
        elif backend == "gst_v4l2_any":
            pipeline = (
                "v4l2src device={0} ! videoconvert "
                "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(source)
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_mjpg_any":
            pipeline = (
                "v4l2src device={0} ! image/jpeg ! jpegdec ! videoconvert "
                "! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(source)
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_mjpg":
            pipeline = (
                "v4l2src device={0} ! image/jpeg,width={1},height={2},framerate={3}/1 "
                "! jpegdec ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(source, capture_width, capture_height, int(max(float(self.get_parameter("fps").value), 1.0)))
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "gst_v4l2_yuyv":
            pipeline = (
                "v4l2src device={0} ! video/x-raw,format=YUY2,width={1},height={2},framerate={3}/1 "
                "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
            ).format(source, capture_width, capture_height, int(max(float(self.get_parameter("fps").value), 1.0)))
            self.active_pipeline = pipeline
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        elif backend == "v4l2_mjpg":
            self.active_pipeline = ""
            self.capture = self.cv2.VideoCapture(source, self.cv2.CAP_V4L2)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc("M", "J", "P", "G"))
            self.configure_capture(capture_width, capture_height)
        elif backend == "v4l2_yuyv":
            self.active_pipeline = ""
            self.capture = self.cv2.VideoCapture(source, self.cv2.CAP_V4L2)
            self.capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))
            self.configure_capture(capture_width, capture_height)
        elif backend == "v4l2_default":
            self.active_pipeline = ""
            self.capture = self.cv2.VideoCapture(source, self.cv2.CAP_V4L2)
            self.configure_capture(capture_width, capture_height)
        else:
            self.active_pipeline = ""
            self.capture = self.cv2.VideoCapture(source)
            self.configure_capture(capture_width, capture_height)

        if not self.capture or not self.capture.isOpened():
            self.last_error = "open failed with backend={0}".format(backend)
            self.publish_event("camera open failed, backend={0}".format(backend))
            self.capture = None
        elif not self.probe_capture():
            if not self.last_error:
                self.last_error = "probe failed with backend={0}".format(backend)
            self.publish_event("camera probe failed, backend={0}".format(backend))
            self.capture.release()
            self.capture = None
        else:
            self.failed_reads = 0
            self.publish_event("camera opened, backend={0}".format(backend))
        self.publish_status()

    def open_camera_candidates(self) -> None:
        if self.open_retry_exhausted:
            return
        max_attempts = int(self.get_parameter("max_open_attempts").value)
        if max_attempts > 0 and self.open_attempt_rounds >= max_attempts:
            self.open_retry_exhausted = True
            self.last_error = "camera open retry limit reached"
            self.publish_event("camera open retry limit reached")
            self.publish_status()
            return
        self.open_attempt_rounds += 1
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

        mode = str(self.get_parameter("camera_mode").value).lower()
        candidates = []
        if mode == "csi":
            candidates.extend(["csi_jetbot", "csi_jetcam", "csi_gstreamer"])
            return candidates
        if mode in {"url", "mjpeg", "http"}:
            return ["url"]

        if mode == "auto" and (bool(self.get_parameter("use_gstreamer").value) or os.path.exists("/tmp/argus_socket")):
            candidates.extend(["csi_jetbot", "csi_jetcam", "csi_gstreamer"])

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
                if not bool(self.get_parameter("reject_invalid_on_open").value):
                    return True
                resized = self.resize_output(frame)
                if not self.is_invalid_frame(resized):
                    return True
            time.sleep(0.05)
        if last_frame is not None:
            resized = self.resize_output(last_frame)
            self.last_error = self.describe_invalid_frame(resized)
            self.save_debug_frame(resized)
        return False

    def capture_once(self) -> None:
        if not self.camera_enabled():
            return
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
            "enabled": self.camera_enabled(),
            "mode": str(self.get_parameter("camera_mode").value),
            "device": self.camera_source(),
            "url": str(self.get_parameter("camera_url").value),
            "backend": self.active_backend,
            "pipeline": self.active_pipeline,
            "csi_sensor_id": int(self.get_parameter("csi_sensor_id").value),
            "csi_sensor_mode": int(self.get_parameter("csi_sensor_mode").value),
            "failed_reads": self.failed_reads,
            "last_error": self.last_error,
            "retry_seconds": float(self.get_parameter("capture_retry_seconds").value),
            "open_attempt_rounds": self.open_attempt_rounds,
            "open_retry_exhausted": self.open_retry_exhausted,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def camera_enabled(self) -> bool:
        if not bool(self.get_parameter("enabled").value):
            return False
        mode = str(self.get_parameter("camera_mode").value).lower()
        return mode != "disabled"

    def camera_source(self):
        device = str(self.get_parameter("camera_device").value)
        if device:
            return device
        return "/dev/video{0}".format(int(self.get_parameter("camera_index").value))

    def csi_pipeline(self, include_sensor_id: bool, include_sensor_mode: bool, jetbot_exact: bool = False) -> str:
        source = "nvarguscamerasrc"
        if include_sensor_id:
            source += " sensor-id={0}".format(int(self.get_parameter("csi_sensor_id").value))
        if include_sensor_mode:
            source += " sensor-mode={0}".format(int(self.get_parameter("csi_sensor_mode").value))
        appsink = "appsink" if jetbot_exact else "appsink drop=true max-buffers=1 sync=false"
        return (
            "{0} ! video/x-raw(memory:NVMM), width={1}, height={2}, format=(string)NV12, framerate=(fraction){3}/1 "
            "! nvvidconv flip-method={4} ! video/x-raw, width=(int){5}, height=(int){6}, format=(string)BGRx "
            "! videoconvert ! video/x-raw, format=(string)BGR ! {7}"
        ).format(
            source,
            int(self.get_parameter("csi_capture_width").value),
            int(self.get_parameter("csi_capture_height").value),
            int(self.get_parameter("csi_fps").value),
            int(self.get_parameter("csi_flip_method").value),
            int(self.get_parameter("width").value),
            int(self.get_parameter("height").value),
            appsink,
        )

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

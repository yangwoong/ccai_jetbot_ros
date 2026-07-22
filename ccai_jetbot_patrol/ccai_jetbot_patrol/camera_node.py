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
        self.declare_parameter("use_gstreamer", False)
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", 5.0)
        self.declare_parameter("jpeg_quality", 45)
        self.declare_parameter("output_topic", "/image_raw/compressed")

        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.publisher = self.create_publisher(CompressedImage, str(self.get_parameter("output_topic").value), 2)
        self.cv2 = None
        self.capture = None

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
        if bool(self.get_parameter("use_gstreamer").value):
            pipeline = (
                "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 "
                "! nvvidconv flip-method=0 ! video/x-raw, width={0}, height={1}, format=BGRx "
                "! videoconvert ! video/x-raw, format=BGR ! appsink"
            ).format(width, height)
            self.capture = self.cv2.VideoCapture(pipeline, self.cv2.CAP_GSTREAMER)
        else:
            self.capture = self.cv2.VideoCapture(index)
            self.capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.capture.set(self.cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))

        if not self.capture or not self.capture.isOpened():
            self.publish_event("camera open failed")
            self.capture = None
        else:
            self.publish_event("camera opened")

    def capture_once(self) -> None:
        if self.cv2 is None or self.capture is None:
            return
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.publish_event_throttled("camera frame read failed")
            return

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        frame = self.cv2.resize(frame, (width, height))
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

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_event_throttled(self, text: str) -> None:
        now = time.monotonic()
        if not hasattr(self, "_last_event") or now - self._last_event > 5.0:
            self._last_event = now
            self.publish_event(text)

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


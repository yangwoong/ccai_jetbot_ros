import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class MotorBackend:
    def set_motors(self, left: float, right: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        self.set_motors(0.0, 0.0)


class JetBotMotorBackend(MotorBackend):
    def __init__(self) -> None:
        from jetbot import Robot

        self.robot = Robot()

    def set_motors(self, left: float, right: float) -> None:
        self.robot.left_motor.value = left
        self.robot.right_motor.value = right


class NullMotorBackend(MotorBackend):
    def __init__(self, logger) -> None:
        self.logger = logger
        self.last_report = 0.0

    def set_motors(self, left: float, right: float) -> None:
        now = time.monotonic()
        if now - self.last_report > 5.0:
            self.logger.warning("motor backend unavailable; ignoring motor command left={0:.2f}, right={1:.2f}".format(left, right))
            self.last_report = now


class StatusLed:
    def __init__(self, pin: int, active_high: bool, logger) -> None:
        self.pin = pin
        self.active_high = active_high
        self.logger = logger
        self.available = False
        self.gpio = None
        if pin < 0:
            return
        try:
            import Jetson.GPIO as GPIO

            self.gpio = GPIO
            self.gpio.setmode(GPIO.BOARD)
            self.gpio.setup(pin, GPIO.OUT, initial=self.off_value())
            self.available = True
        except Exception as exc:
            self.logger.warning("status LED unavailable: {0}".format(exc))

    def on_value(self):
        return self.gpio.HIGH if self.active_high else self.gpio.LOW

    def off_value(self):
        return self.gpio.LOW if self.active_high else self.gpio.HIGH

    def set(self, enabled: bool) -> None:
        if not self.available:
            return
        self.gpio.output(self.pin, self.on_value() if enabled else self.off_value())

    def cleanup(self) -> None:
        if self.available:
            self.set(False)
            self.gpio.cleanup(self.pin)


class OledDisplay:
    def __init__(self, enabled: bool, bus: int, logger) -> None:
        self.enabled = enabled
        self.logger = logger
        self.display = None
        self.image = None
        self.draw = None
        self.font = None
        if not enabled:
            return
        try:
            import Adafruit_SSD1306
            from PIL import Image, ImageDraw, ImageFont

            self.display = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=bus, gpio=1)
            self.display.begin()
            self.display.clear()
            self.display.display()
            self.image = Image.new("1", (self.display.width, self.display.height))
            self.draw = ImageDraw.Draw(self.image)
            self.font = ImageFont.load_default()
        except Exception as exc:
            self.logger.warning("OLED unavailable: {0}".format(exc))
            self.display = None

    def show(self, line1: str, line2: str, line3: str = "") -> None:
        if self.display is None:
            return
        width = self.display.width
        height = self.display.height
        self.draw.rectangle((0, 0, width, height), outline=0, fill=0)
        self.draw.text((0, 0), line1[:21], font=self.font, fill=255)
        self.draw.text((0, 11), line2[:21], font=self.font, fill=255)
        self.draw.text((0, 22), line3[:21], font=self.font, fill=255)
        self.display.image(self.image)
        self.display.display()

    def clear(self) -> None:
        if self.display is not None:
            self.display.clear()
            self.display.display()


class JetBotHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("jetbot_hardware_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motor_backend", "jetbot")
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("left_trim", 1.0)
        self.declare_parameter("right_trim", 1.0)
        self.declare_parameter("command_timeout_seconds", 1.0)
        self.declare_parameter("status_led_pin", -1)
        self.declare_parameter("status_led_active_high", True)
        self.declare_parameter("oled_enabled", True)
        self.declare_parameter("oled_bus", 0)
        self.declare_parameter("oled_refresh_seconds", 2.0)

        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.status_pub = self.create_publisher(String, "/ccai/hardware_status", 10)
        self.last_command_at = 0.0
        self.last_motion = "idle"
        self.heartbeat = False

        self.motor = self.create_motor_backend()
        self.led = StatusLed(
            int(self.get_parameter("status_led_pin").value),
            bool(self.get_parameter("status_led_active_high").value),
            self.get_logger(),
        )
        self.oled = OledDisplay(
            bool(self.get_parameter("oled_enabled").value),
            int(self.get_parameter("oled_bus").value),
            self.get_logger(),
        )

        topic = str(self.get_parameter("cmd_vel_topic").value)
        self.create_subscription(Twist, topic, self.on_cmd_vel, 10)
        self.create_timer(0.1, self.watchdog)
        self.create_timer(float(self.get_parameter("oled_refresh_seconds").value), self.publish_hardware_status)
        self.publish_event("jetbot hardware node ready, cmd_vel_topic={0}".format(topic))

    def create_motor_backend(self) -> MotorBackend:
        if not bool(self.get_parameter("enabled").value):
            self.get_logger().warning("jetbot hardware disabled by parameter")
            return NullMotorBackend(self.get_logger())
        backend = str(self.get_parameter("motor_backend").value)
        if backend == "jetbot":
            try:
                motor = JetBotMotorBackend()
                self.get_logger().info("jetbot motor backend ready")
                return motor
            except Exception as exc:
                self.get_logger().warning("jetbot motor backend unavailable: {0}".format(exc))
        return NullMotorBackend(self.get_logger())

    def on_cmd_vel(self, msg: Twist) -> None:
        max_linear = max(float(self.get_parameter("max_linear_speed").value), 0.01)
        max_angular = max(float(self.get_parameter("max_angular_speed").value), 0.01)
        linear = clamp(msg.linear.x / max_linear, -1.0, 1.0)
        angular = clamp(msg.angular.z / max_angular, -1.0, 1.0)
        left = clamp(linear - angular, -1.0, 1.0) * float(self.get_parameter("left_trim").value)
        right = clamp(linear + angular, -1.0, 1.0) * float(self.get_parameter("right_trim").value)
        left = clamp(left, -1.0, 1.0)
        right = clamp(right, -1.0, 1.0)
        self.motor.set_motors(left, right)
        self.last_command_at = time.monotonic()
        self.last_motion = "left={0:.2f}, right={1:.2f}".format(left, right)

    def watchdog(self) -> None:
        timeout = float(self.get_parameter("command_timeout_seconds").value)
        if self.last_command_at and time.monotonic() - self.last_command_at > timeout:
            self.motor.stop()
            self.last_command_at = 0.0
            self.last_motion = "timeout-stop"
        self.heartbeat = not self.heartbeat
        self.led.set(self.heartbeat)

    def publish_hardware_status(self) -> None:
        ip_address = get_ip_address()
        text = "ip={0}, motion={1}".format(ip_address, self.last_motion)
        self.status_pub.publish(String(data=text))
        self.oled.show("CCAI JetBot", ip_address, self.last_motion)

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def destroy_node(self) -> bool:
        self.motor.stop()
        self.led.cleanup()
        self.oled.clear()
        return super().destroy_node()


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def get_ip_address() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
        return "no-ip"
    finally:
        sock.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetBotHardwareNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


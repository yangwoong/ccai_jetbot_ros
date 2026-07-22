import fcntl
import glob
import os
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


class RawI2cBus:
    I2C_SLAVE = 0x0703

    def __init__(self, bus: int) -> None:
        self.path = "/dev/i2c-{0}".format(bus)
        self.fd = os.open(self.path, os.O_RDWR)
        self.current_address = None

    def select(self, address: int) -> None:
        if self.current_address != address:
            fcntl.ioctl(self.fd, self.I2C_SLAVE, address)
            self.current_address = address

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        self.select(address)
        os.write(self.fd, bytes([register & 0xFF, value & 0xFF]))

    def read_byte_data(self, address: int, register: int) -> int:
        self.select(address)
        os.write(self.fd, bytes([register & 0xFF]))
        return os.read(self.fd, 1)[0]

    def write_i2c_block_data(self, address: int, register: int, values) -> None:
        self.select(address)
        payload = [register & 0xFF]
        payload.extend([int(value) & 0xFF for value in values])
        os.write(self.fd, bytes(payload))

    def close(self) -> None:
        os.close(self.fd)


def open_i2c_bus(bus: int):
    try:
        import smbus

        return smbus.SMBus(bus)
    except Exception:
        return RawI2cBus(bus)


class Pca9685:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, bus: int, address: int = 0x60) -> None:
        self.bus = open_i2c_bus(bus)
        self.address = address
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.01)
        self.set_pwm_freq(1600)

    def set_pwm_freq(self, freq_hz: int) -> None:
        prescale_value = 25000000.0 / 4096.0 / float(freq_hz) - 1.0
        prescale = int(prescale_value + 0.5)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        sleep_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, self.MODE1, sleep_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA1)

    def set_pwm(self, channel: int, on: int, off: int) -> None:
        register = self.LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, register, on & 0xFF)
        self.bus.write_byte_data(self.address, register + 1, on >> 8)
        self.bus.write_byte_data(self.address, register + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, register + 3, off >> 8)

    def set_pin(self, channel: int, value: bool) -> None:
        if value:
            self.set_pwm(channel, 4096, 0)
        else:
            self.set_pwm(channel, 0, 4096)


class Pca9685MotorBackend(MotorBackend):
    # Adafruit Motor HAT channel mapping used by JetBot-style boards.
    MOTOR_CHANNELS = {
        1: (8, 10, 9),
        2: (13, 11, 12),
        3: (2, 4, 3),
        4: (7, 5, 6),
    }

    def __init__(self, bus: int, address: int, left_motor: int, right_motor: int, logger) -> None:
        self.driver = Pca9685(bus, address)
        self.left_channels = self.MOTOR_CHANNELS[left_motor]
        self.right_channels = self.MOTOR_CHANNELS[right_motor]
        self.logger = logger
        self.logger.info(
            "pca9685 motor backend ready, bus={0}, address=0x{1:02x}, left_motor={2}, right_motor={3}".format(
                bus, address, left_motor, right_motor
            )
        )

    def set_motors(self, left: float, right: float) -> None:
        self.set_motor(self.left_channels, left)
        self.set_motor(self.right_channels, right)

    def set_motor(self, channels, speed: float) -> None:
        pwm_channel, in1_channel, in2_channel = channels
        speed = clamp(speed, -1.0, 1.0)
        duty = int(abs(speed) * 4095)
        if speed > 0:
            self.driver.set_pin(in1_channel, True)
            self.driver.set_pin(in2_channel, False)
        elif speed < 0:
            self.driver.set_pin(in1_channel, False)
            self.driver.set_pin(in2_channel, True)
        else:
            self.driver.set_pin(in1_channel, False)
            self.driver.set_pin(in2_channel, False)
        self.driver.set_pwm(pwm_channel, 0, duty)


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
        buses = candidate_i2c_buses(bus)
        last_error = None
        for candidate_bus in buses:
            self.display = self.create_adafruit_display(candidate_bus)
            if self.display is not None:
                return
            try:
                self.display = RawSsd1306Display(candidate_bus)
                self.logger.info("raw OLED fallback ready, bus={0}, address=0x3c".format(candidate_bus))
                return
            except Exception as exc:
                last_error = exc
                self.logger.warning("raw OLED fallback unavailable on bus={0}: {1}".format(candidate_bus, exc))
        if last_error is not None:
            self.logger.warning("OLED unavailable: {0}".format(last_error))

    def create_adafruit_display(self, bus: int):
        try:
            import Adafruit_SSD1306
            from PIL import Image, ImageDraw, ImageFont

            display = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=bus, gpio=1)
            display.begin()
            display.clear()
            display.display()
            self.image = Image.new("1", (display.width, display.height))
            self.draw = ImageDraw.Draw(self.image)
            self.font = ImageFont.load_default()
            self.logger.info("Adafruit OLED ready, bus={0}".format(bus))
            return display
        except Exception as exc:
            return None

    def show(self, line1: str, line2: str, line3: str = "") -> None:
        if self.display is None:
            return
        if isinstance(self.display, RawSsd1306Display):
            self.display.show(line1, line2, line3)
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
            if isinstance(self.display, RawSsd1306Display):
                self.display.clear()
                return
            self.display.clear()
            self.display.display()


class RawSsd1306Display:
    ADDRESS = 0x3C

    def __init__(self, bus: int) -> None:
        from PIL import Image, ImageDraw, ImageFont

        self.bus = open_i2c_bus(bus)
        self.width = 128
        self.height = 32
        self.image = Image.new("1", (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()
        self.init_display()

    def command(self, value: int) -> None:
        self.bus.write_byte_data(self.ADDRESS, 0x00, value)

    def data(self, values) -> None:
        for index in range(0, len(values), 16):
            self.bus.write_i2c_block_data(self.ADDRESS, 0x40, values[index : index + 16])

    def init_display(self) -> None:
        for value in [
            0xAE,
            0x20,
            0x00,
            0xB0,
            0xC8,
            0x00,
            0x10,
            0x40,
            0x81,
            0x7F,
            0xA1,
            0xA6,
            0xA8,
            0x1F,
            0xA4,
            0xD3,
            0x00,
            0xD5,
            0x80,
            0xD9,
            0xF1,
            0xDA,
            0x02,
            0xDB,
            0x40,
            0x8D,
            0x14,
            0xAF,
        ]:
            self.command(value)
        self.clear()

    def show(self, line1: str, line2: str, line3: str = "") -> None:
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.draw.text((0, 0), line1[:21], font=self.font, fill=255)
        self.draw.text((0, 11), line2[:21], font=self.font, fill=255)
        self.draw.text((0, 22), line3[:21], font=self.font, fill=255)
        self.flush()

    def clear(self) -> None:
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.flush()

    def flush(self) -> None:
        pixels = self.image.load()
        buffer = []
        for page in range(0, self.height // 8):
            self.command(0xB0 + page)
            self.command(0x00)
            self.command(0x10)
            buffer = []
            for x in range(self.width):
                byte = 0
                for bit in range(8):
                    if pixels[x, page * 8 + bit]:
                        byte |= 1 << bit
                buffer.append(byte)
            self.data(buffer)


class JetBotHardwareNode(Node):
    def __init__(self) -> None:
        super().__init__("jetbot_hardware_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motor_backend", "auto")
        self.declare_parameter("motor_i2c_bus", -1)
        self.declare_parameter("motor_i2c_address", 0)
        self.declare_parameter("left_motor_channel", 1)
        self.declare_parameter("right_motor_channel", 2)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("left_trim", 1.0)
        self.declare_parameter("right_trim", 1.0)
        self.declare_parameter("command_timeout_seconds", 1.0)
        self.declare_parameter("status_led_pin", -1)
        self.declare_parameter("status_led_active_high", True)
        self.declare_parameter("oled_enabled", True)
        self.declare_parameter("oled_bus", -1)
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
        if backend in {"auto", "jetbot"}:
            try:
                motor = JetBotMotorBackend()
                self.get_logger().info("jetbot motor backend ready")
                return motor
            except Exception as exc:
                self.get_logger().warning("jetbot motor backend unavailable: {0}".format(exc))
                if backend == "jetbot":
                    return NullMotorBackend(self.get_logger())
        if backend in {"auto", "pca9685"}:
            for bus in self.motor_candidate_buses():
                for address in self.motor_candidate_addresses():
                    try:
                        return Pca9685MotorBackend(
                            bus,
                            address,
                            int(self.get_parameter("left_motor_channel").value),
                            int(self.get_parameter("right_motor_channel").value),
                            self.get_logger(),
                        )
                    except Exception as exc:
                        self.get_logger().warning(
                            "pca9685 motor backend unavailable on bus={0}, address=0x{1:02x}: {2}".format(bus, address, exc)
                        )
        return NullMotorBackend(self.get_logger())

    def motor_candidate_buses(self):
        configured = int(self.get_parameter("motor_i2c_bus").value)
        if configured >= 0:
            return [configured]
        buses = []
        for path in sorted(glob.glob("/dev/i2c-*")):
            try:
                buses.append(int(path.rsplit("-", 1)[1]))
            except ValueError:
                pass
        return buses or [1, 0]

    def motor_candidate_addresses(self):
        configured = int(self.get_parameter("motor_i2c_address").value)
        if configured > 0:
            return [configured]
        return [0x60, 0x40]

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


def candidate_i2c_buses(configured: int):
    if configured >= 0:
        return [configured]
    buses = []
    for path in sorted(glob.glob("/dev/i2c-*")):
        try:
            buses.append(int(path.rsplit("-", 1)[1]))
        except ValueError:
            pass
    return buses or [1, 0]


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

"""Convert a joystick into watchdog-protected Ackermann commands."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def scale_axis(value: float, deadzone: float) -> float:
    """Apply a rescaled deadzone while preserving the full output range."""
    if not math.isfinite(value):
        raise ValueError('joystick axis must be finite')
    if not math.isfinite(deadzone) or not 0.0 <= deadzone < 1.0:
        raise ValueError('deadzone must be finite and in [0, 1)')
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    return math.copysign(
        min((magnitude - deadzone) / (1.0 - deadzone), 1.0),
        value,
    )


class JoyToAckermannNode(Node):
    """Publish manual drive commands only while the deadman button is held."""

    def __init__(self) -> None:
        super().__init__('joy_to_ackermann')
        self.declare_parameter('input_topic', '/joy')
        self.declare_parameter('output_topic', '/cmd_ackermann')
        self.declare_parameter('speed_axis', 1)
        self.declare_parameter('steering_axis', 0)
        self.declare_parameter('deadman_button', 5)
        self.declare_parameter('speed_sign', 1.0)
        self.declare_parameter('steering_sign', 1.0)
        self.declare_parameter('max_forward_speed', 0.6)
        self.declare_parameter('max_reverse_speed', 0.3)
        self.declare_parameter('max_steering', 0.55)
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('publish_rate', 30.0)

        self.speed_axis = int(self.get_parameter('speed_axis').value)
        self.steering_axis = int(self.get_parameter('steering_axis').value)
        self.deadman_button = int(
            self.get_parameter('deadman_button').value
        )
        self.speed_sign = float(self.get_parameter('speed_sign').value)
        self.steering_sign = float(
            self.get_parameter('steering_sign').value
        )
        self.max_forward_speed = float(
            self.get_parameter('max_forward_speed').value
        )
        self.max_reverse_speed = float(
            self.get_parameter('max_reverse_speed').value
        )
        self.max_steering = float(
            self.get_parameter('max_steering').value
        )
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._validate_parameters(publish_rate)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Joy, input_topic, self.joy_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self._speed = 0.0
        self._steering = 0.0
        self._enabled = False
        self._last_joy_time: float | None = None
        self._reported_layout_error = False
        self.get_logger().info(
            f'Listening on {input_topic}; hold button {self.deadman_button} '
            f'to command {output_topic}'
        )

    def _validate_parameters(self, publish_rate: float) -> None:
        """Reject unsafe or internally inconsistent controller settings."""
        if min(self.speed_axis, self.steering_axis, self.deadman_button) < 0:
            raise ValueError('axis and button indices must be non-negative')
        if self.speed_sign not in (-1.0, 1.0):
            raise ValueError('speed_sign must be -1.0 or 1.0')
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError('steering_sign must be -1.0 or 1.0')
        limits = (
            self.max_forward_speed,
            self.max_reverse_speed,
            self.max_steering,
            self.command_timeout,
            publish_rate,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in limits):
            raise ValueError('speed, steering, timeout and rate must be positive')
        scale_axis(0.0, self.deadzone)

    def joy_callback(self, message: Joy) -> None:
        """Cache a bounded command from a valid controller report."""
        required_axes = max(self.speed_axis, self.steering_axis) + 1
        required_buttons = self.deadman_button + 1
        if (
            len(message.axes) < required_axes
            or len(message.buttons) < required_buttons
        ):
            self._enabled = False
            self._speed = 0.0
            self._steering = 0.0
            if not self._reported_layout_error:
                self.get_logger().warning(
                    'Joystick report does not contain the configured axes/buttons'
                )
                self._reported_layout_error = True
            return

        self._reported_layout_error = False
        self._last_joy_time = time.monotonic()
        self._enabled = bool(message.buttons[self.deadman_button])
        if not self._enabled:
            self._speed = 0.0
            self._steering = 0.0
            return

        try:
            speed_axis = scale_axis(
                self.speed_sign * float(message.axes[self.speed_axis]),
                self.deadzone,
            )
            steering_axis = scale_axis(
                self.steering_sign * float(message.axes[self.steering_axis]),
                self.deadzone,
            )
        except ValueError as error:
            self.get_logger().warning(f'Ignoring invalid joystick report: {error}')
            self._enabled = False
            self._speed = 0.0
            self._steering = 0.0
            return

        speed_limit = (
            self.max_forward_speed
            if speed_axis >= 0.0
            else self.max_reverse_speed
        )
        self._speed = speed_axis * speed_limit
        self._steering = steering_axis * self.max_steering

    def _publish(self) -> None:
        """Publish continuously and stop on release or stale joystick data."""
        stale = (
            self._last_joy_time is None
            or time.monotonic() - self._last_joy_time > self.command_timeout
        )
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        if self._enabled and not stale:
            message.drive.speed = self._speed
            message.drive.steering_angle = self._steering
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the Xbox-compatible manual controller node."""
    rclpy.init(args=args)
    node = JoyToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._enabled = False
            node._publish()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

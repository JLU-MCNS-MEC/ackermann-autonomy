"""ROS 2 wrapper for the Jetson chassis hardware."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .chassis_controller import ChassisConfig, ChassisController
from .ds20270c_vendor import Ds20270cVendor, FeedbackType
from .pwm_servo import ServoDriver, SysfsPwm
from .socket_can import SocketCan
from .wheel_odometry import WheelOdometry, WheelState


def _key_value(key: str, value: object) -> KeyValue:
    return KeyValue(key=key, value=str(value))


class ChassisNode(Node):
    """Bridge `/drive` to DS20270C/PWM and publish raw wheel feedback."""

    def __init__(self) -> None:
        super().__init__('chassis_driver')
        can_interface = self.declare_parameter('can_interface', 'can0').value
        node_id = self.declare_parameter('ds20270c_node_id', 1).value
        pwm_hint = self.declare_parameter('pwm_controller_hint', '32c0000.pwm').value
        pwm_channel = self.declare_parameter('pwm_channel', 0).value
        self.command_topic = self.declare_parameter('command_topic', '/drive').value
        self.odometry_topic = self.declare_parameter(
            'odometry_topic', '/wheel/odometry'
        ).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.feedback_timeout = self.declare_parameter('feedback_timeout', 0.5).value
        feedback_rate = self.declare_parameter('feedback_rate', 20.0).value
        diagnostics_rate = self.declare_parameter('diagnostics_rate', 2.0).value
        enable_on_start = self.declare_parameter('enable_on_start', False).value
        config = ChassisConfig(
            wheel_diameter_m=self.declare_parameter('wheel_diameter', 0.169).value,
            track_width_m=self.declare_parameter('track_width', 0.40).value,
            wheelbase_m=self.declare_parameter('wheelbase', 0.56).value,
            max_motor_rpm=self.declare_parameter('max_motor_rpm', 300.0).value,
            left_motor_invert=self.declare_parameter('left_motor_invert', False).value,
            right_motor_invert=self.declare_parameter('right_motor_invert', True).value,
            servo_center_deg=self.declare_parameter('servo_center_deg', 150.0).value,
            servo_scale_deg_per_rad=self.declare_parameter(
                'servo_scale_deg_per_rad', 180.0 / math.pi
            ).value,
            max_steering_rad=self.declare_parameter('max_steering', 0.55).value,
            steering_rate_limit_rad_s=self.declare_parameter(
                'steering_rate_limit', math.pi
            ).value,
            watchdog_s=self.declare_parameter('command_timeout', 0.30).value,
        )
        config.validate()
        if (
            not can_interface
            or not self.command_topic
            or not self.odometry_topic
            or not 1 <= node_id <= 127
            or pwm_channel < 0
            or self.feedback_timeout <= 0.0
            or feedback_rate <= 0.0
            or diagnostics_rate <= 0.0
        ):
            raise ValueError('invalid chassis driver parameter')

        self.config = config
        self.can = SocketCan(can_interface)
        if not self.can.open_socket():
            raise RuntimeError(f'cannot open SocketCAN: {self.can.last_error}')
        self.pwm = SysfsPwm(pwm_hint, pwm_channel)
        self.servo = ServoDriver(self.pwm)
        self.drive = Ds20270cVendor(self.can, node_id)
        self.controller = ChassisController(self.drive, self.servo, config)
        self.odometry = WheelOdometry(
            config.wheel_diameter_m,
            config.track_width_m,
            config.left_motor_invert,
            config.right_motor_invert,
        )
        if not self.controller.initialize(enable_on_start):
            self.can.close()
            raise RuntimeError(
                'chassis initialization failed; check DS20270C mode, PWM pinmux and permissions'
            )

        self.odom_publisher = self.create_publisher(Odometry, self.odometry_topic, 10)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self.command_subscription = self.create_subscription(
            AckermannDriveStamped, self.command_topic, self._command_callback, 10
        )
        self.enable_service = self.create_service(SetBool, '~/enable', self._enable_callback)
        self.feedback_timer = self.create_timer(1.0 / feedback_rate, self._feedback_tick)
        self.watchdog_timer = self.create_timer(0.02, self._watchdog_tick)
        self.diagnostics_timer = self.create_timer(
            1.0 / diagnostics_rate, self._publish_diagnostics
        )

        self.last_speed_feedback: float | None = None
        self.last_load_feedback: float | None = None
        self.feedback_cycle = 0
        self.watchdog_active = False
        self.ignored_commands = 0
        self.invalid_commands = 0
        self.io_errors = 0
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.get_logger().info(
            f'chassis driver ready on {can_interface}; motor drive is '
            f'{"enabled" if self.controller.enabled else "disabled"}'
        )

    def _command_callback(self, message: AckermannDriveStamped) -> None:
        if not self.controller.enabled:
            self.ignored_commands += 1
            self.get_logger().warning(
                'ignoring /drive command while chassis is disabled',
                throttle_duration_sec=2.0,
            )
            return
        try:
            success = self.controller.command(
                float(message.drive.speed), float(message.drive.steering_angle)
            )
        except ValueError as error:
            self.invalid_commands += 1
            self.get_logger().error(str(error), throttle_duration_sec=1.0)
            return
        if not success:
            self.io_errors += 1
            self.get_logger().error(
                'failed to apply chassis command', throttle_duration_sec=1.0
            )

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        response.success = self.controller.set_enabled(request.data)
        if response.success:
            response.message = 'chassis enabled' if request.data else 'chassis disabled'
        else:
            response.message = 'DS20270C did not accept the requested enable state'
            self.io_errors += 1
        return response

    def _watchdog_tick(self) -> None:
        if self.controller.watchdog():
            self.watchdog_active = True
            self.get_logger().warning(
                'command timeout: wheel RPM set to zero', throttle_duration_sec=2.0
            )
        elif self.controller.last_command.left_rpm or self.controller.last_command.right_rpm:
            self.watchdog_active = False

    def _feedback_tick(self) -> None:
        if not self.drive.request_speed():
            self.io_errors += 1
        self.feedback_cycle += 1
        if self.feedback_cycle % 10 == 0 and not self.drive.request_load():
            self.io_errors += 1
        for _ in range(32):
            frame = self.can.receive()
            if frame is None:
                break
            feedback_type = self.drive.handle_frame(frame)
            if feedback_type is FeedbackType.SPEED:
                self._handle_speed_feedback()
            elif feedback_type is FeedbackType.LOAD:
                self.last_load_feedback = time.monotonic()

    def _handle_speed_feedback(self) -> None:
        timestamp = time.monotonic()
        feedback = self.drive.feedback
        wheels = self.odometry.wheel_state(feedback.rpm_axis1, feedback.rpm_axis2)
        if self.last_speed_feedback is not None:
            elapsed = timestamp - self.last_speed_feedback
            if self.odometry.integrate(wheels, elapsed):
                radius = self.config.wheel_diameter_m * 0.5
                self.left_wheel_angle += wheels.left_speed_mps / radius * elapsed
                self.right_wheel_angle += wheels.right_speed_mps / radius * elapsed
        self.last_speed_feedback = timestamp
        self._publish_odometry(wheels)
        self._publish_joint_state(wheels)

    def _publish_odometry(self, wheels: WheelState) -> None:
        state = self.odometry.state
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = state.x_m
        message.pose.pose.position.y = state.y_m
        message.pose.pose.orientation.z = math.sin(state.yaw_rad * 0.5)
        message.pose.pose.orientation.w = math.cos(state.yaw_rad * 0.5)
        message.twist.twist.linear.x = wheels.linear_speed_mps
        message.twist.twist.angular.z = wheels.yaw_rate_rad_s
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.10
        message.twist.covariance[0] = 0.02
        message.twist.covariance[35] = 0.05
        self.odom_publisher.publish(message)

    def _front_steering_angles(self) -> tuple[float, float]:
        center = self.controller.steering_rad
        if abs(center) < 1e-9:
            return 0.0, 0.0
        radius = self.config.wheelbase_m / math.tan(abs(center))
        inner = math.atan(self.config.wheelbase_m / (radius - self.config.track_width_m * 0.5))
        outer = math.atan(self.config.wheelbase_m / (radius + self.config.track_width_m * 0.5))
        return (inner, outer) if center > 0.0 else (-outer, -inner)

    def _publish_joint_state(self, wheels: WheelState) -> None:
        left_steer, right_steer = self._front_steering_angles()
        radius = self.config.wheel_diameter_m * 0.5
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'front_left_steering_joint',
            'front_right_steering_joint',
        ]
        message.position = [
            self.left_wheel_angle,
            self.right_wheel_angle,
            left_steer,
            right_steer,
        ]
        message.velocity = [
            wheels.left_speed_mps / radius,
            wheels.right_speed_mps / radius,
            0.0,
            0.0,
        ]
        self.joint_publisher.publish(message)

    def _publish_diagnostics(self) -> None:
        feedback_age = (
            math.inf
            if self.last_speed_feedback is None
            else time.monotonic() - self.last_speed_feedback
        )
        if self.io_errors:
            level, summary = DiagnosticStatus.ERROR, 'CAN or PWM operation failed'
        elif not self.controller.enabled:
            level, summary = DiagnosticStatus.WARN, 'motor drive disabled'
        elif feedback_age > self.feedback_timeout:
            level, summary = DiagnosticStatus.ERROR, 'wheel speed feedback stale'
        elif self.watchdog_active:
            level, summary = DiagnosticStatus.WARN, 'command watchdog active'
        else:
            level, summary = DiagnosticStatus.OK, 'chassis interface operational'
        feedback = self.drive.feedback
        status = DiagnosticStatus(
            level=level,
            name='ackermann_hardware/chassis',
            message=summary,
            hardware_id='DS20270C',
            values=[
                _key_value('enabled', self.controller.enabled),
                _key_value('feedback_age_s', feedback_age),
                _key_value('left_rpm', feedback.rpm_axis1),
                _key_value('right_rpm', feedback.rpm_axis2),
                _key_value('left_load_permille', feedback.load_permille_axis1),
                _key_value('right_load_permille', feedback.load_permille_axis2),
                _key_value('ignored_commands', self.ignored_commands),
                _key_value('invalid_commands', self.invalid_commands),
                _key_value('io_errors', self.io_errors),
            ],
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostic_publisher.publish(message)

    def destroy_node(self) -> bool:
        if hasattr(self, 'controller'):
            self.controller.stop()
        if hasattr(self, 'can'):
            self.can.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the real chassis driver."""
    rclpy.init(args=args)
    node = None
    try:
        node = ChassisNode()
        rclpy.spin(node)
    except (OSError, RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'chassis_driver: {error}')
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

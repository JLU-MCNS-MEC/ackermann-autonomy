"""Exercise the ROS wrapper with CAN and PWM test doubles."""

import time

from ackermann_hardware import chassis_node as module
from ackermann_hardware.socket_can import CanFrame
from ackermann_msgs.msg import AckermannDriveStamped
import pytest
import rclpy
from std_srvs.srv import SetBool


class FakeSocketCan:
    instances = []

    def __init__(self, interface):
        self.interface = interface
        self.last_error = ''
        self.sent = []
        self.received = []
        self.closed = False
        self.instances.append(self)

    def open_socket(self):
        return True

    def close(self):
        self.closed = True

    def send(self, can_id, data):
        self.sent.append((can_id, data))
        return True

    def receive(self, _timeout=0.0):
        return self.received.pop(0) if self.received else None


class FakePwm:
    def __init__(self, controller_hint, channel):
        self.controller_hint = controller_hint
        self.channel = channel
        self.duty_ns = 0
        self.enabled = False

    def initialize(self, _period_ns):
        return True

    def set_duty_ns(self, duty_ns):
        self.duty_ns = duty_ns
        return True

    def enable(self, enabled):
        self.enabled = enabled
        return True


@pytest.fixture
def chassis(monkeypatch):
    FakeSocketCan.instances.clear()
    monkeypatch.setattr(module, 'SocketCan', FakeSocketCan)
    monkeypatch.setattr(module, 'SysfsPwm', FakePwm)
    if not rclpy.ok():
        rclpy.init()
    node = module.ChassisNode()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_enable_command_feedback_and_publications(chassis):
    disabled_command = AckermannDriveStamped()
    disabled_command.drive.speed = 0.2
    chassis._command_callback(disabled_command)
    assert chassis.ignored_commands == 1

    request = SetBool.Request(data=True)
    response = chassis._enable_callback(request, SetBool.Response())
    assert response.success
    assert response.message == 'chassis enabled'

    command = AckermannDriveStamped()
    command.drive.speed = 0.2
    command.drive.steering_angle = 0.2
    chassis._command_callback(command)
    assert chassis.controller.last_command.left_rpm != 0

    can = FakeSocketCan.instances[-1]
    speed = CanFrame(0x581, bytes.fromhex('E800500064009CFF'))
    load = CanFrame(0x581, bytes.fromhex('E80250000A001400'))
    can.received.extend([speed, load])
    chassis._feedback_tick()
    assert chassis.drive.feedback.rpm_axis1 == 100
    assert chassis.drive.feedback.rpm_axis2 == -100
    assert chassis.last_speed_feedback is not None
    assert chassis.last_load_feedback is not None
    time.sleep(0.001)
    can.received.append(speed)
    chassis._feedback_tick()
    assert chassis.odometry.state.x_m > 0.0


def test_invalid_command_watchdog_steering_and_diagnostics(chassis):
    chassis.controller.set_enabled(True)
    command = AckermannDriveStamped()
    command.drive.speed = float('nan')
    chassis._command_callback(command)
    assert chassis.invalid_commands == 1

    chassis.controller._last_command_time = time.monotonic() - 1.0
    chassis._watchdog_tick()
    assert chassis.watchdog_active
    assert chassis._front_steering_angles() == (0.0, 0.0)
    chassis.controller.steering_rad = 0.2
    left, right = chassis._front_steering_angles()
    assert left > right > 0.0
    chassis.controller.steering_rad = -0.2
    left, right = chassis._front_steering_angles()
    assert left > right
    assert right < 0.0

    chassis._publish_diagnostics()
    chassis.io_errors = 1
    chassis._publish_diagnostics()
    chassis.io_errors = 0
    chassis.controller.set_enabled(False)
    chassis._publish_diagnostics()


def test_node_rejects_socket_open_failure(monkeypatch):
    class FailingSocket(FakeSocketCan):
        def open_socket(self):
            self.last_error = 'test failure'
            return False

    monkeypatch.setattr(module, 'SocketCan', FailingSocket)
    monkeypatch.setattr(module, 'SysfsPwm', FakePwm)
    if not rclpy.ok():
        rclpy.init()
    with pytest.raises(RuntimeError, match='test failure'):
        module.ChassisNode()
    rclpy.shutdown()

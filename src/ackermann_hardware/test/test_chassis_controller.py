"""Test servo mapping, vehicle kinematics and fail-safe behavior."""

import math

from ackermann_hardware.chassis_controller import ChassisConfig, ChassisController
from ackermann_hardware.ds20270c_vendor import Ds20270cVendor
from ackermann_hardware.pwm_servo import ServoDriver
import pytest


class FakeCan:
    def __init__(self):
        self.sent = []
        self.send_ok = True

    def send(self, can_id, data):
        self.sent.append((can_id, data))
        return self.send_ok


class FakePwm:
    def __init__(self):
        self.period_ns = 0
        self.duty_ns = 0
        self.enabled = False
        self.duty_ok = True

    def initialize(self, period_ns):
        self.period_ns = period_ns
        return True

    def set_duty_ns(self, duty_ns):
        self.duty_ns = duty_ns
        return self.duty_ok

    def enable(self, enabled):
        self.enabled = enabled
        return True


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def make_controller(config=None):
    can = FakeCan()
    pwm = FakePwm()
    clock = Clock()
    drive = Ds20270cVendor(can, 1)
    controller = ChassisController(drive, ServoDriver(pwm), config, clock)
    return controller, can, pwm, clock


def test_servo_uses_50hz_center_pulse_and_limits():
    pwm = FakePwm()
    servo = ServoDriver(pwm)
    assert servo.initialize(150.0)
    assert pwm.period_ns == 20_000_000
    assert pwm.duty_ns == 1_500_000
    servo.set_angle_limits(140.0, 160.0)
    assert servo.set_angle(200.0)
    assert servo.angle_deg == 160.0
    pwm.duty_ok = False
    assert not servo.set_angle(150.0)


def test_ackermann_command_accounts_for_inner_outer_wheels_and_inversion():
    controller, _, _, _ = make_controller()
    straight = controller.compute_command(0.2, 0.0)
    assert straight.left_rpm > 0
    assert straight.right_rpm < 0
    left_turn = controller.compute_command(0.2, 0.3)
    assert abs(left_turn.left_rpm) < abs(left_turn.right_rpm)


def test_command_limits_and_invalid_input():
    controller, _, _, _ = make_controller()
    limited = controller.compute_command(100.0, 10.0)
    assert abs(limited.left_rpm) <= 300
    assert abs(limited.right_rpm) <= 300
    assert limited.steering_rad == pytest.approx(0.55)
    with pytest.raises(ValueError, match='finite'):
        controller.compute_command(math.nan, 0.0)


def test_disabled_state_enable_sequence_and_watchdog_stop():
    controller, can, _, clock = make_controller()
    assert controller.initialize(False)
    assert not controller.command(0.2, 0.1)
    assert controller.set_enabled(True)
    clock.now = 0.1
    assert controller.command(0.2, 0.1)
    clock.now = 0.41
    assert controller.watchdog()
    assert controller.last_command.left_rpm == 0
    assert can.sent[-1][1] == bytes.fromhex('E918230000000000')


def test_configuration_rejects_invalid_geometry():
    config = ChassisConfig(wheel_diameter_m=0.0)
    with pytest.raises(ValueError, match='positive'):
        make_controller(config)

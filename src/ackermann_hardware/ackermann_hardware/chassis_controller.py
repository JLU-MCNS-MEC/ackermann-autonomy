"""Ackermann kinematics, actuator limits and command watchdog."""

from dataclasses import dataclass
import math
import time
from typing import Callable

from .ds20270c_vendor import Ds20270cVendor
from .pwm_servo import ServoDriver


@dataclass
class ChassisConfig:
    """Measured vehicle geometry and actuator calibration."""

    wheel_diameter_m: float = 0.169
    track_width_m: float = 0.40
    wheelbase_m: float = 0.56
    max_motor_rpm: float = 300.0
    left_motor_invert: bool = False
    right_motor_invert: bool = True
    servo_center_deg: float = 150.0
    servo_scale_deg_per_rad: float = 180.0 / math.pi
    max_steering_rad: float = 0.55
    steering_rate_limit_rad_s: float = math.pi
    watchdog_s: float = 0.30

    def validate(self) -> None:
        values = (
            self.wheel_diameter_m,
            self.track_width_m,
            self.wheelbase_m,
            self.max_motor_rpm,
            self.max_steering_rad,
            self.steering_rate_limit_rad_s,
            self.watchdog_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('chassis dimensions, limits and watchdog must be positive and finite')
        if not math.isfinite(self.servo_center_deg) or not math.isfinite(
            self.servo_scale_deg_per_rad
        ):
            raise ValueError('servo calibration must be finite')


@dataclass(frozen=True)
class WheelCommand:
    """Command after vehicle kinematics and motor direction mapping."""

    left_rpm: int
    right_rpm: int
    steering_rad: float


class ChassisController:
    """Coordinate the dual-axis drive and steering servo."""

    def __init__(
        self,
        drive: Ds20270cVendor,
        servo: ServoDriver,
        config: ChassisConfig | None = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.drive = drive
        self.servo = servo
        self.config = config or ChassisConfig()
        self.config.validate()
        self._monotonic = monotonic
        self.enabled = False
        self.steering_rad = 0.0
        self.last_command = WheelCommand(0, 0, 0.0)
        self._last_command_time = 0.0
        self._last_steering_time = 0.0

    def initialize(self, enable_on_start: bool = False) -> bool:
        if not self.servo.initialize(self.config.servo_center_deg):
            return False
        if not self.drive.enable(False) or not self.drive.set_rpm(0, 0):
            return False
        now = self._monotonic()
        self._last_command_time = now
        self._last_steering_time = now
        return self.set_enabled(True) if enable_on_start else True

    def set_enabled(self, enabled: bool) -> bool:
        if not enabled:
            stopped = self.drive.set_rpm(0, 0)
            disabled = self.drive.enable(False)
            self.enabled = False
            self.last_command = WheelCommand(0, 0, 0.0)
            return stopped and disabled
        if not self.drive.set_rpm(0, 0) or not self.drive.enable(True):
            self.enabled = False
            return False
        self.enabled = True
        self._last_command_time = self._monotonic()
        return True

    def _speed_to_rpm(self, speed_mps: float, inverted: bool) -> int:
        rpm = speed_mps * 60.0 / (math.pi * self.config.wheel_diameter_m)
        if inverted:
            rpm = -rpm
        rpm = min(max(rpm, -self.config.max_motor_rpm), self.config.max_motor_rpm)
        return round(rpm)

    def compute_command(self, speed_mps: float, steering_rad: float) -> WheelCommand:
        if not math.isfinite(speed_mps) or not math.isfinite(steering_rad):
            raise ValueError('speed and steering commands must be finite')
        steering = min(
            max(steering_rad, -self.config.max_steering_rad),
            self.config.max_steering_rad,
        )
        yaw_rate = speed_mps * math.tan(steering) / self.config.wheelbase_m
        left_speed = speed_mps - yaw_rate * self.config.track_width_m * 0.5
        right_speed = speed_mps + yaw_rate * self.config.track_width_m * 0.5
        return WheelCommand(
            self._speed_to_rpm(left_speed, self.config.left_motor_invert),
            self._speed_to_rpm(right_speed, self.config.right_motor_invert),
            steering,
        )

    def _apply_steering(self, target_rad: float, elapsed_s: float) -> bool:
        maximum_step = self.config.steering_rate_limit_rad_s * max(elapsed_s, 0.001)
        error = target_rad - self.steering_rad
        self.steering_rad += min(max(error, -maximum_step), maximum_step)
        servo_angle = self.config.servo_center_deg + (
            self.config.servo_scale_deg_per_rad * self.steering_rad
        )
        return self.servo.set_angle(servo_angle)

    def command(self, speed_mps: float, steering_rad: float) -> bool:
        if not self.enabled:
            return False
        command = self.compute_command(speed_mps, steering_rad)
        now = self._monotonic()
        elapsed = min(max(now - self._last_steering_time, 0.0), 0.5)
        self._last_steering_time = now
        self._last_command_time = now
        steering_ok = self._apply_steering(command.steering_rad, elapsed)
        drive_ok = self.drive.set_rpm(command.left_rpm, command.right_rpm)
        if steering_ok and drive_ok:
            self.last_command = command
        return steering_ok and drive_ok

    def watchdog(self) -> bool:
        command_age = self._monotonic() - self._last_command_time
        if not self.enabled or command_age <= self.config.watchdog_s:
            return False
        now = self._monotonic()
        elapsed = min(max(now - self._last_steering_time, 0.0), 0.5)
        self._last_steering_time = now
        self.drive.set_rpm(0, 0)
        self._apply_steering(0.0, elapsed)
        self.last_command = WheelCommand(0, 0, 0.0)
        return True

    def stop(self) -> None:
        self.drive.set_rpm(0, 0)
        self.servo.set_angle(self.config.servo_center_deg)
        self.drive.enable(False)
        self.enabled = False
        self.last_command = WheelCommand(0, 0, 0.0)

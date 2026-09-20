"""Raw differential rear-wheel odometry without TF publication."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelState:
    """Physical wheel and vehicle velocities."""

    left_speed_mps: float
    right_speed_mps: float
    linear_speed_mps: float
    yaw_rate_rad_s: float


@dataclass
class OdometryState:
    """Integrated planar pose."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


class WheelOdometry:
    """Convert motor RPM feedback and integrate planar wheel odometry."""

    def __init__(
        self,
        wheel_diameter_m: float,
        track_width_m: float,
        left_motor_invert: bool,
        right_motor_invert: bool,
    ) -> None:
        if wheel_diameter_m <= 0.0 or track_width_m <= 0.0:
            raise ValueError('wheel diameter and track width must be positive')
        self.wheel_diameter_m = wheel_diameter_m
        self.track_width_m = track_width_m
        self.left_motor_invert = left_motor_invert
        self.right_motor_invert = right_motor_invert
        self.state = OdometryState()

    def wheel_state(self, left_axis_rpm: int, right_axis_rpm: int) -> WheelState:
        scale = math.pi * self.wheel_diameter_m / 60.0
        left = left_axis_rpm * scale * (-1.0 if self.left_motor_invert else 1.0)
        right = right_axis_rpm * scale * (-1.0 if self.right_motor_invert else 1.0)
        return WheelState(
            left,
            right,
            0.5 * (left + right),
            (right - left) / self.track_width_m,
        )

    def integrate(self, wheel_state: WheelState, elapsed_s: float) -> bool:
        if not math.isfinite(elapsed_s) or not 0.0 < elapsed_s <= 1.0:
            return False
        heading = self.state.yaw_rad + wheel_state.yaw_rate_rad_s * elapsed_s * 0.5
        self.state.x_m += wheel_state.linear_speed_mps * math.cos(heading) * elapsed_s
        self.state.y_m += wheel_state.linear_speed_mps * math.sin(heading) * elapsed_s
        self.state.yaw_rad = math.remainder(
            self.state.yaw_rad + wheel_state.yaw_rate_rad_s * elapsed_s,
            2.0 * math.pi,
        )
        return True

    def reset(self) -> None:
        self.state = OdometryState()

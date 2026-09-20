"""Test raw wheel feedback conversion and planar integration."""

from ackermann_hardware.wheel_odometry import WheelOdometry
import pytest


def test_inverted_motor_feedback_becomes_forward_vehicle_motion():
    odometry = WheelOdometry(0.169, 0.40, False, True)
    wheels = odometry.wheel_state(100, -100)
    assert wheels.left_speed_mps == pytest.approx(wheels.right_speed_mps)
    assert wheels.linear_speed_mps > 0.0
    assert wheels.yaw_rate_rad_s == pytest.approx(0.0)
    assert odometry.integrate(wheels, 1.0)
    assert odometry.state.x_m > 0.0
    assert odometry.state.y_m == pytest.approx(0.0)


def test_integration_rejects_invalid_time_step_and_can_reset():
    odometry = WheelOdometry(0.169, 0.40, False, False)
    wheels = odometry.wheel_state(100, 100)
    assert not odometry.integrate(wheels, 0.0)
    assert not odometry.integrate(wheels, 1.1)
    assert odometry.integrate(wheels, 0.1)
    odometry.reset()
    assert odometry.state.x_m == 0.0


@pytest.mark.parametrize('diameter,track', [(0.0, 0.4), (0.169, -0.4)])
def test_invalid_geometry_is_rejected(diameter, track):
    with pytest.raises(ValueError, match='positive'):
        WheelOdometry(diameter, track, False, False)

"""Verify joystick mapping, deadman behavior and watchdog safety."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from rclpy.time import Time
from sensor_msgs.msg import Joy

from ackermann_autonomy import joy_to_ackermann_node as module


def controller():
    return SimpleNamespace(
        speed_axis=1,
        steering_axis=0,
        deadman_button=5,
        speed_sign=1.0,
        steering_sign=1.0,
        max_forward_speed=0.6,
        max_reverse_speed=0.3,
        max_steering=0.55,
        deadzone=0.1,
        command_timeout=0.25,
        _speed=0.0,
        _steering=0.0,
        _enabled=False,
        _last_joy_time=None,
        _reported_layout_error=False,
        get_logger=MagicMock(),
        get_clock=MagicMock(),
        publisher=MagicMock(),
    )


def joy(steering=0.0, speed=0.0, deadman=True):
    message = Joy()
    message.axes = [steering, speed, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    message.buttons = [0] * 16
    message.buttons[5] = int(deadman)
    return message


def test_scale_axis_rescales_deadzone_and_rejects_invalid_input():
    assert module.scale_axis(0.05, 0.1) == 0.0
    assert module.scale_axis(0.55, 0.1) == pytest.approx(0.5)
    assert module.scale_axis(-1.0, 0.1) == -1.0
    with pytest.raises(ValueError):
        module.scale_axis(float('nan'), 0.1)


def test_deadman_maps_forward_reverse_and_steering(monkeypatch):
    node = controller()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 10.0)
    module.JoyToAckermannNode.joy_callback(node, joy(0.55, 1.0))
    assert node._enabled is True
    assert node._speed == pytest.approx(0.6)
    assert node._steering == pytest.approx(0.275)
    module.JoyToAckermannNode.joy_callback(node, joy(-1.0, -1.0))
    assert node._speed == pytest.approx(-0.3)
    assert node._steering == pytest.approx(-0.55)


def test_release_and_malformed_report_force_stop():
    node = controller()
    module.JoyToAckermannNode.joy_callback(node, joy(speed=1.0))
    module.JoyToAckermannNode.joy_callback(node, joy(speed=1.0, deadman=False))
    assert (node._enabled, node._speed, node._steering) == (False, 0.0, 0.0)
    module.JoyToAckermannNode.joy_callback(node, Joy())
    assert node._enabled is False
    node.get_logger.return_value.warning.assert_called_once()


@pytest.mark.parametrize('now,expected_speed', [(10.1, 0.6), (10.3, 0.0)])
def test_publish_watchdog(now, expected_speed, monkeypatch):
    node = controller()
    node._enabled = True
    node._speed = 0.6
    node._steering = 0.2
    node._last_joy_time = 10.0
    node.get_clock.return_value.now.return_value = Time()
    monkeypatch.setattr(module.time, 'monotonic', lambda: now)
    module.JoyToAckermannNode._publish(node)
    message = node.publisher.publish.call_args.args[0]
    assert message.drive.speed == pytest.approx(expected_speed)
    assert message.drive.steering_angle == pytest.approx(
        0.2 if expected_speed else 0.0
    )

"""Test SocketCAN frame validation and DS20270C protocol encoding."""

from ackermann_hardware.ds20270c_vendor import Ds20270cVendor, FeedbackType
from ackermann_hardware.socket_can import CanFrame, SocketCan
import pytest


class FakeCan:
    def __init__(self, send_ok=True):
        self.send_ok = send_ok
        self.sent = []

    def send(self, can_id, data):
        self.sent.append((can_id, data))
        return self.send_ok


def test_speed_command_encodes_signed_little_endian_axes():
    can = FakeCan()
    drive = Ds20270cVendor(can, 1)
    assert drive.set_rpm(100, -100)
    assert can.sent == [(0x601, bytes.fromhex('E918230064009CFF'))]


def test_enable_and_read_requests_use_expected_frames():
    can = FakeCan()
    drive = Ds20270cVendor(can, 1)
    assert drive.enable(True)
    assert drive.request_speed()
    assert drive.request_load()
    assert can.sent[0] == (0x601, bytes.fromhex('E900210001000100'))
    assert can.sent[1][1] == bytes.fromhex('E800500000000000')
    assert can.sent[2][1] == bytes.fromhex('E802500000000000')


def test_feedback_parser_handles_speed_load_and_invalid_frames():
    drive = Ds20270cVendor(FakeCan(), 1)
    speed = CanFrame(0x581, bytes.fromhex('E80050002C01D4FE'))
    assert drive.handle_frame(speed) is FeedbackType.SPEED
    assert (drive.feedback.rpm_axis1, drive.feedback.rpm_axis2) == (300, -300)
    load = CanFrame(0x581, bytes.fromhex('E802500064009CFF'))
    assert drive.handle_frame(load) is FeedbackType.LOAD
    assert (drive.feedback.load_permille_axis1, drive.feedback.load_permille_axis2) == (
        100,
        -100,
    )
    assert drive.handle_frame(CanFrame(0x582, speed.data)) is FeedbackType.NONE
    assert drive.handle_frame(CanFrame(0x581, b'\xe8')) is FeedbackType.NONE


def test_protocol_rejects_invalid_ranges_and_propagates_send_failure():
    with pytest.raises(ValueError, match='node ID'):
        Ds20270cVendor(FakeCan(), 0)
    drive = Ds20270cVendor(FakeCan(send_ok=False), 1)
    assert not drive.enable(True)
    with pytest.raises(ValueError, match='16-bit'):
        drive.set_rpm(32768, 0)


def test_can_frame_and_closed_socket_error_paths():
    with pytest.raises(ValueError, match='11-bit'):
        CanFrame(0x800, b'')
    with pytest.raises(ValueError, match='8 bytes'):
        CanFrame(1, b'123456789')
    can = SocketCan('can0')
    assert not can.send(1, b'')
    assert can.receive() is None
    with pytest.raises(ValueError, match='negative'):
        can.receive(-0.1)

"""Test Linux I/O wrappers with filesystem and socket doubles."""

import struct

from ackermann_hardware.pwm_servo import SysfsPwm
from ackermann_hardware.socket_can import CanFrame, SocketCan
import pytest


class FakeSocket:
    def __init__(self, received=b''):
        self.received = received
        self.bound = None
        self.sent = b''
        self.closed = False

    def bind(self, address):
        self.bound = address

    def send(self, data):
        self.sent = data
        return len(data)

    def recv(self, _size):
        return self.received

    def close(self):
        self.closed = True


def test_socketcan_packs_and_unpacks_classic_frames(monkeypatch):
    raw = struct.pack('=IB3x8s', 0x581, 3, b'abc'.ljust(8, b'\x00'))
    fake = FakeSocket(raw)
    monkeypatch.setattr('ackermann_hardware.socket_can.socket.socket', lambda *_: fake)
    monkeypatch.setattr(
        'ackermann_hardware.socket_can.select.select', lambda *_: ([fake], [], [])
    )
    can = SocketCan('can0')
    assert can.open_socket()
    assert fake.bound == ('can0',)
    assert can.send(0x601, b'12345678')
    assert len(fake.sent) == 16
    assert can.receive() == CanFrame(0x581, b'abc')
    can.close()
    assert fake.closed


def test_socketcan_reports_bind_failure(monkeypatch):
    class FailingSocket(FakeSocket):
        def bind(self, _address):
            raise OSError('missing interface')

    fake = FailingSocket()
    monkeypatch.setattr('ackermann_hardware.socket_can.socket.socket', lambda *_: fake)
    can = SocketCan('can9')
    assert not can.open_socket()
    assert 'missing interface' in can.last_error
    assert fake.closed


def test_sysfs_pwm_writes_period_duty_and_enable(tmp_path):
    root = tmp_path / 'pwm'
    chip = root / 'pwmchip0'
    controller = tmp_path / 'devices' / '32c0000.pwm'
    channel = chip / 'pwm0'
    controller.mkdir(parents=True)
    chip.mkdir(parents=True)
    (chip / 'device').symlink_to(controller, target_is_directory=True)
    channel.mkdir()
    for name in ('period', 'duty_cycle', 'enable'):
        (channel / name).write_text('', encoding='ascii')

    pwm = SysfsPwm('32c0000.pwm', 0, root)
    assert pwm.initialize(20_000_000)
    assert (channel / 'period').read_text() == '20000000'
    assert pwm.set_duty_ns(1_500_000)
    assert (channel / 'duty_cycle').read_text() == '1500000'
    assert pwm.enable(True)
    assert (channel / 'enable').read_text() == '1'


def test_sysfs_pwm_rejects_invalid_configuration(tmp_path):
    with pytest.raises(ValueError):
        SysfsPwm('', 0)
    pwm = SysfsPwm('missing', 0, tmp_path / 'absent')
    assert not pwm.initialize(20_000_000)

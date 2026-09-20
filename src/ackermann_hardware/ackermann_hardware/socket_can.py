"""Minimal classic-CAN transport using the Linux SocketCAN ABI."""

from dataclasses import dataclass
import select
import socket
import struct
from typing import Optional


CAN_SFF_MASK = 0x7FF
CAN_MAX_DLEN = 8
_CAN_FRAME = struct.Struct('=IB3x8s')


@dataclass(frozen=True)
class CanFrame:
    """A standard 11-bit CAN frame."""

    can_id: int
    data: bytes

    def __post_init__(self) -> None:
        if not 0 <= self.can_id <= CAN_SFF_MASK:
            raise ValueError('CAN ID must be an 11-bit standard identifier')
        if len(self.data) > CAN_MAX_DLEN:
            raise ValueError('classic CAN payload cannot exceed 8 bytes')


class SocketCan:
    """Own and operate one Linux raw CAN socket."""

    def __init__(self, interface: str = 'can0') -> None:
        if not interface:
            raise ValueError('CAN interface cannot be empty')
        self.interface = interface
        self._socket: Optional[socket.socket] = None
        self.last_error = ''

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    def open_socket(self) -> bool:
        self.close()
        try:
            can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            can_socket.bind((self.interface,))
        except OSError as error:
            self.last_error = str(error)
            try:
                can_socket.close()
            except UnboundLocalError:
                pass
            return False
        self._socket = can_socket
        self.last_error = ''
        return True

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def send(self, can_id: int, data: bytes) -> bool:
        frame = CanFrame(can_id, bytes(data))
        if self._socket is None:
            self.last_error = 'CAN socket is not open'
            return False
        packed = _CAN_FRAME.pack(
            frame.can_id & CAN_SFF_MASK,
            len(frame.data),
            frame.data.ljust(CAN_MAX_DLEN, b'\x00'),
        )
        try:
            return self._socket.send(packed) == _CAN_FRAME.size
        except OSError as error:
            self.last_error = str(error)
            return False

    def receive(self, timeout: float = 0.0) -> Optional[CanFrame]:
        if timeout < 0.0:
            raise ValueError('CAN receive timeout cannot be negative')
        if self._socket is None:
            self.last_error = 'CAN socket is not open'
            return None
        try:
            readable, _, _ = select.select([self._socket], [], [], timeout)
            if not readable:
                return None
            raw = self._socket.recv(_CAN_FRAME.size)
        except OSError as error:
            self.last_error = str(error)
            return None
        if len(raw) != _CAN_FRAME.size:
            self.last_error = f'incomplete CAN frame: {len(raw)} bytes'
            return None
        can_id, length, payload = _CAN_FRAME.unpack(raw)
        if length > CAN_MAX_DLEN:
            self.last_error = f'invalid CAN data length: {length}'
            return None
        return CanFrame(can_id & CAN_SFF_MASK, payload[:length])

    def __enter__(self) -> 'SocketCan':
        if not self.open_socket():
            raise OSError(self.last_error)
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

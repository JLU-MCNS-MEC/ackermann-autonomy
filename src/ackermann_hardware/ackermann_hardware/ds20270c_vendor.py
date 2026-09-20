"""DS20270C manufacturer dual-axis CAN protocol."""

from dataclasses import dataclass
from enum import auto, Enum
import struct
from typing import Protocol

from .socket_can import CanFrame


class CanTransport(Protocol):
    """Transport required by the drive protocol."""

    def send(self, can_id: int, data: bytes) -> bool:
        """Send one classic CAN frame."""


@dataclass
class DriveFeedback:
    """Latest dual-axis speed and load response."""

    rpm_axis1: int = 0
    rpm_axis2: int = 0
    load_permille_axis1: int = 0
    load_permille_axis2: int = 0


class FeedbackType(Enum):
    """Recognized response type."""

    NONE = auto()
    SPEED = auto()
    LOAD = auto()
    ACKNOWLEDGEMENT = auto()


class Ds20270cVendor:
    """Encode and decode the DS20270C internal-speed protocol."""

    def __init__(self, transport: CanTransport, node_id: int = 1) -> None:
        if not 1 <= node_id <= 127:
            raise ValueError('DS20270C node ID must be in [1, 127]')
        self._transport = transport
        self.node_id = node_id
        self.feedback = DriveFeedback()

    @property
    def tx_cob_id(self) -> int:
        return 0x600 + self.node_id

    @property
    def rx_cob_id(self) -> int:
        return 0x580 + self.node_id

    def enable(self, enabled: bool) -> bool:
        state = 1 if enabled else 0
        return self._transport.send(
            self.tx_cob_id, b'\xe9\x00\x21\x00' + struct.pack('<hh', state, state)
        )

    def set_rpm(self, axis1_rpm: int, axis2_rpm: int) -> bool:
        if not -32768 <= axis1_rpm <= 32767 or not -32768 <= axis2_rpm <= 32767:
            raise ValueError('axis RPM must fit a signed 16-bit integer')
        return self._transport.send(
            self.tx_cob_id,
            b'\xe9\x18\x23\x00' + struct.pack('<hh', axis1_rpm, axis2_rpm),
        )

    def request_speed(self) -> bool:
        return self._transport.send(self.tx_cob_id, b'\xe8\x00\x50\x00\x00\x00\x00\x00')

    def request_load(self) -> bool:
        return self._transport.send(self.tx_cob_id, b'\xe8\x02\x50\x00\x00\x00\x00\x00')

    def handle_frame(self, frame: CanFrame) -> FeedbackType:
        if frame.can_id != self.rx_cob_id or len(frame.data) < 8:
            return FeedbackType.NONE
        command = frame.data[:4]
        if command == b'\xe8\x00\x50\x00':
            self.feedback.rpm_axis1, self.feedback.rpm_axis2 = struct.unpack(
                '<hh', frame.data[4:8]
            )
            return FeedbackType.SPEED
        if command == b'\xe8\x02\x50\x00':
            (
                self.feedback.load_permille_axis1,
                self.feedback.load_permille_axis2,
            ) = struct.unpack('<hh', frame.data[4:8])
            return FeedbackType.LOAD
        return FeedbackType.ACKNOWLEDGEMENT if frame.data[0] == 0x60 else FeedbackType.NONE

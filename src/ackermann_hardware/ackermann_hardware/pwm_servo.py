"""Jetson Linux PWM sysfs backend and servo calibration."""

from pathlib import Path
import time
from typing import Protocol


class PwmOutput(Protocol):
    """PWM operations required by the servo driver."""

    def initialize(self, period_ns: int) -> bool:
        """Initialize a PWM channel."""

    def set_duty_ns(self, duty_ns: int) -> bool:
        """Set the high-pulse duration."""

    def enable(self, enabled: bool) -> bool:
        """Enable or disable output."""


class SysfsPwm:
    """Discover a Jetson PWM controller and operate its sysfs channel."""

    def __init__(
        self,
        controller_hint: str = '32c0000.pwm',
        channel: int = 0,
        sysfs_root: Path = Path('/sys/class/pwm'),
    ) -> None:
        if not controller_hint:
            raise ValueError('PWM controller hint cannot be empty')
        if channel < 0:
            raise ValueError('PWM channel must be non-negative')
        self.controller_hint = controller_hint
        self.channel = channel
        self.sysfs_root = Path(sysfs_root)
        self.pwm_path: Path | None = None
        self.enabled = False
        self.last_error = ''

    @staticmethod
    def _write(path: Path, value: str) -> None:
        path.write_text(value, encoding='ascii')

    def _find_chip(self) -> Path | None:
        if not self.sysfs_root.is_dir():
            self.last_error = f'{self.sysfs_root} does not exist'
            return None
        for candidate in self.sysfs_root.glob('pwmchip*'):
            try:
                resolved = str(candidate.resolve())
                device = str((candidate / 'device').resolve())
            except OSError:
                continue
            if self.controller_hint in resolved or self.controller_hint in device:
                return candidate
        self.last_error = f'cannot find PWM controller containing {self.controller_hint}'
        return None

    def initialize(self, period_ns: int) -> bool:
        if period_ns <= 0:
            raise ValueError('PWM period must be positive')
        chip = self._find_chip()
        if chip is None:
            return False
        self.pwm_path = chip / f'pwm{self.channel}'
        try:
            if not self.pwm_path.exists():
                self._write(chip / 'export', str(self.channel))
                for _ in range(20):
                    if self.pwm_path.exists():
                        break
                    time.sleep(0.02)
            if not self.pwm_path.exists():
                self.last_error = f'PWM channel did not appear: {self.pwm_path}'
                return False
            self.enable(False)
            self._write(self.pwm_path / 'period', str(period_ns))
            self._write(self.pwm_path / 'duty_cycle', '0')
        except OSError as error:
            self.last_error = str(error)
            return False
        return True

    def set_duty_ns(self, duty_ns: int) -> bool:
        if self.pwm_path is None:
            self.last_error = 'PWM is not initialized'
            return False
        try:
            self._write(self.pwm_path / 'duty_cycle', str(duty_ns))
        except OSError as error:
            self.last_error = str(error)
            return False
        return True

    def enable(self, enabled: bool) -> bool:
        if self.pwm_path is None:
            self.last_error = 'PWM is not initialized'
            return False
        try:
            self._write(self.pwm_path / 'enable', '1' if enabled else '0')
        except OSError as error:
            self.last_error = str(error)
            return False
        self.enabled = enabled
        return True


class ServoDriver:
    """Map calibrated servo angle to a 50 Hz pulse width."""

    PERIOD_NS = 20_000_000
    PULSE_MIN_US = 1000
    PULSE_MAX_US = 2000
    ANGLE_MIN_DEG = 0.0
    ANGLE_MAX_DEG = 300.0

    def __init__(self, pwm: PwmOutput) -> None:
        self._pwm = pwm
        self.limit_min_deg = self.ANGLE_MIN_DEG
        self.limit_max_deg = self.ANGLE_MAX_DEG
        self.reversed = False
        self.angle_deg = self.ANGLE_MIN_DEG

    def initialize(self, initial_angle_deg: float) -> bool:
        return (
            self._pwm.initialize(self.PERIOD_NS)
            and self.set_angle(initial_angle_deg)
            and self._pwm.enable(True)
        )

    def set_angle_limits(self, minimum_deg: float, maximum_deg: float) -> None:
        minimum = min(max(minimum_deg, self.ANGLE_MIN_DEG), self.ANGLE_MAX_DEG)
        maximum = min(max(maximum_deg, self.ANGLE_MIN_DEG), self.ANGLE_MAX_DEG)
        self.limit_min_deg, self.limit_max_deg = sorted((minimum, maximum))

    def set_angle(self, angle_deg: float) -> bool:
        angle = min(max(angle_deg, self.limit_min_deg), self.limit_max_deg)
        mapped = self.ANGLE_MAX_DEG - angle if self.reversed else angle
        ratio = mapped / self.ANGLE_MAX_DEG
        pulse_us = round(self.PULSE_MIN_US + ratio * (self.PULSE_MAX_US - self.PULSE_MIN_US))
        if not self.set_pulse_us(pulse_us):
            return False
        self.angle_deg = angle
        return True

    def set_pulse_us(self, pulse_us: int) -> bool:
        pulse = min(max(pulse_us, self.PULSE_MIN_US), self.PULSE_MAX_US)
        return self._pwm.set_duty_ns(pulse * 1000)

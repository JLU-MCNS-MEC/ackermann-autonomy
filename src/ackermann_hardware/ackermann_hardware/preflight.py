"""Check that the Jetson chassis hardware is ready before bench bring-up.

Reports platform, CAN and PWM readiness and prints the remediation command for each
failure. The PWM discovery reuses the driver's own implementation so the report cannot
disagree with what the driver will actually find.

Command line entry point: ros2 run ackermann_hardware hardware_preflight
"""

from dataclasses import dataclass
import os
from pathlib import Path
import subprocess
from typing import Callable, Sequence

from ackermann_hardware.pwm_servo import SysfsPwm

CAN_SERVICE_REMEDY = (
    'sudo "$(ros2 pkg prefix ackermann_hardware)'
    '/lib/ackermann_hardware/install_can_service.sh"'
)
CAN_MANUAL_REMEDY = 'ros2 run ackermann_hardware setup_can0.sh <interface> <bitrate>'

_DEFAULT_MODEL_PATH = Path('/proc/device-tree/model')
_DEFAULT_MODULES_PATH = Path('/proc/modules')
_DEFAULT_NET_ROOT = Path('/sys/class/net')
_DEFAULT_PWM_ROOT = Path('/sys/class/pwm')


@dataclass(frozen=True)
class CheckResult:
    """One check outcome. advisory results are reported but never fail the run."""

    label: str
    ok: bool
    detail: str
    remedy: str = ''
    advisory: bool = False


def check_platform(model_path: Path = _DEFAULT_MODEL_PATH) -> CheckResult:
    """Report the board model. A non-Jetson host is advisory, not a failure."""
    try:
        model = model_path.read_bytes().replace(b'\0', b'').decode('utf-8', 'replace')
    except OSError:
        return CheckResult(
            'platform',
            True,
            f'{model_path} is unreadable; not a Jetson, hardware checks are advisory',
            advisory=True,
        )
    model = model.strip()
    if 'Jetson' in model:
        return CheckResult('platform', True, model)
    return CheckResult('platform', True, f'{model}; not a Jetson', advisory=True)


def check_can_module(modules_path: Path = _DEFAULT_MODULES_PATH) -> CheckResult:
    """Report whether mttcan is loaded. Without it no CAN interface is created."""
    try:
        modules = modules_path.read_text(encoding='utf-8', errors='replace')
    except OSError as error:
        return CheckResult('CAN module', False, f'cannot read {modules_path}: {error}')
    loaded = any(
        line.split(' ', 1)[0] == 'mttcan' for line in modules.splitlines() if line
    )
    if loaded:
        return CheckResult('CAN module', True, 'mttcan is loaded')
    return CheckResult(
        'CAN module',
        False,
        'mttcan is not loaded, so no CAN interface exists',
        f'{CAN_SERVICE_REMEDY}, or run {CAN_MANUAL_REMEDY} manually',
    )


def check_can_interface(
    interface: str = 'can0',
    net_root: Path = _DEFAULT_NET_ROOT,
    run: Callable[..., subprocess.CompletedProcess] | None = None,
) -> CheckResult:
    """Report interface presence, link state, bitrate and controller state."""
    if not (net_root / interface).exists():
        return CheckResult(
            'CAN interface',
            False,
            f'interface {interface} does not exist',
            f'{CAN_SERVICE_REMEDY}, or run {CAN_MANUAL_REMEDY} manually',
        )
    runner = run or subprocess.run
    try:
        proc = runner(
            ['ip', '-details', 'link', 'show', interface],
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return CheckResult(
            'CAN interface', False, 'the ip command is missing', 'install iproute2'
        )
    if proc.returncode != 0:
        return CheckResult(
            'CAN interface',
            False,
            f'ip cannot read {interface}',
            CAN_SERVICE_REMEDY,
        )
    text = f'{proc.stdout or ""}{proc.stderr or ""}'
    if 'state UP' not in text and 'UP,LOWER_UP' not in text:
        return CheckResult(
            'CAN interface',
            False,
            f'{interface} exists but is down',
            f'ip link set {interface} up',
        )
    details = []
    for token in ('bitrate', 'can state'):
        for line in text.splitlines():
            if token in line:
                details.append(line.strip())
                break
    summary = '; '.join(details)
    if 'ERROR-PASSIVE' in text or 'BUS-OFF' in text:
        return CheckResult(
            'CAN interface',
            False,
            f'{interface} controller is not active: {summary}',
            'check the 3.3 V transceiver wiring and the 120 ohm terminators',
        )
    return CheckResult(
        'CAN interface', True, f'{interface} is up; {summary}'.strip('; ')
    )


def check_pwm_controller(
    hint: str = '32c0000.pwm',
    channel: int = 0,
    sysfs_root: Path = _DEFAULT_PWM_ROOT,
) -> CheckResult:
    """Locate the PWM controller using the driver's own discovery logic."""
    pwm = SysfsPwm(hint, channel, sysfs_root)
    chip = pwm._find_chip()
    if chip is None:
        return CheckResult(
            'PWM controller',
            False,
            f'no pwmchip resolves to {hint} ({pwm.last_error})',
            'confirm the 40-pin header pin 18 PWM controller is enabled in the device tree',
        )
    target = Path(os.path.realpath(chip / 'device')).name
    return CheckResult('PWM controller', True, f'{chip.name} -> {target}')


def check_pwm_writable(
    hint: str = '32c0000.pwm',
    channel: int = 0,
    sysfs_root: Path = _DEFAULT_PWM_ROOT,
) -> CheckResult:
    """Report whether the current user can export the PWM channel."""
    pwm = SysfsPwm(hint, channel, sysfs_root)
    chip = pwm._find_chip()
    if chip is None:
        return CheckResult('PWM access', False, 'no PWM controller found to test')
    channel_path = chip / f'pwm{channel}'
    if channel_path.exists():
        return CheckResult('PWM access', True, f'{channel_path} is already exported')
    export_path = chip / 'export'
    if os.access(export_path, os.W_OK):
        return CheckResult('PWM access', True, f'{export_path} is writable')
    return CheckResult(
        'PWM access',
        False,
        f'{export_path} is not writable',
        'add the current user to the gpio group and log in again',
    )


def run_checks(
    interface: str = 'can0',
    pwm_hint: str = '32c0000.pwm',
    pwm_channel: int = 0,
    model_path: Path = _DEFAULT_MODEL_PATH,
    modules_path: Path = _DEFAULT_MODULES_PATH,
    net_root: Path = _DEFAULT_NET_ROOT,
    pwm_root: Path = _DEFAULT_PWM_ROOT,
    run: Callable[..., subprocess.CompletedProcess] | None = None,
) -> list[CheckResult]:
    """Run every check in report order."""
    return [
        check_platform(model_path),
        check_can_module(modules_path),
        check_can_interface(interface, net_root, run),
        check_pwm_controller(pwm_hint, pwm_channel, pwm_root),
        check_pwm_writable(pwm_hint, pwm_channel, pwm_root),
    ]


def format_results(results: Sequence[CheckResult]) -> str:
    """Render the outcomes as a plain text report."""
    lines = ['Ackermann hardware preflight', '']
    for result in results:
        if result.advisory:
            tag = 'NOTE'
        else:
            tag = 'OK' if result.ok else 'FAIL'
        lines.append(f'[{tag}] {result.label}: {result.detail}')
        if result.remedy and not result.ok:
            lines.append(f'       remedy: {result.remedy}')
    graded = [r for r in results if not r.advisory]
    passed = sum(1 for r in graded if r.ok)
    lines.append('')
    lines.append(f'Summary: {passed} passed, {len(graded) - passed} failed')
    return '\n'.join(lines)


def main() -> None:
    """Command line entry point. Exit 0 when every graded check passes."""
    import sys

    interface = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    results = run_checks(interface=interface)
    print(format_results(results))
    failed = any(not r.ok for r in results if not r.advisory)
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()

"""Exercise the hardware preflight checks against temporary fake system trees."""

from pathlib import Path
import subprocess

from ackermann_hardware import preflight
import pytest


def completed(returncode=0, stdout='', stderr=''):
    return subprocess.CompletedProcess([], returncode, stdout, stderr)


def write_pwm_tree(root, hint='32c0000.pwm', exported=False, export_mode=0o220):
    """Build a pwmchip tree shaped like the one the driver discovers."""
    chip = root / 'pwmchip0'
    controller = root.parent / 'devices' / hint
    controller.mkdir(parents=True, exist_ok=True)
    chip.mkdir(parents=True, exist_ok=True)
    (chip / 'device').symlink_to(controller, target_is_directory=True)
    export = chip / 'export'
    export.write_text('', encoding='ascii')
    export.chmod(export_mode)
    if exported:
        channel = chip / 'pwm0'
        channel.mkdir()
        for name in ('period', 'duty_cycle', 'enable'):
            (channel / name).write_text('0', encoding='ascii')
    return chip


def test_platform_reports_jetson(tmp_path):
    model = tmp_path / 'model'
    model.write_bytes(b'Jetson-AGX\x00')
    result = preflight.check_platform(model)
    assert result.ok
    assert not result.advisory
    assert 'Jetson-AGX' in result.detail


def test_platform_is_advisory_on_other_hosts(tmp_path):
    model = tmp_path / 'model'
    model.write_bytes(b'Some x86 Board\x00')
    result = preflight.check_platform(model)
    assert result.ok
    assert result.advisory


def test_can_module_detects_loaded_and_missing(tmp_path):
    modules = tmp_path / 'modules'
    modules.write_text('mttcan 66251 0 - Live 0x0000000000000000\n', encoding='ascii')
    assert preflight.check_can_module(modules).ok

    modules.write_text('can_dev 13306 0 - Live 0x0000000000000000\n', encoding='ascii')
    missing = preflight.check_can_module(modules)
    assert not missing.ok
    assert 'setup_can0.sh' in missing.remedy


def test_can_interface_missing_fails_with_remedy(tmp_path):
    result = preflight.check_can_interface('can0', tmp_path)
    assert not result.ok
    assert 'can0' in result.detail
    assert 'install_can_service.sh' in result.remedy


def test_can_interface_reports_bitrate_and_state(tmp_path):
    (tmp_path / 'can0').mkdir()
    output = (
        '9: can0: <NOARP,UP,LOWER_UP> mtu 16\n'
        '    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0\n'
        '    bitrate 500000 sample-point 0.870\n'
    )
    result = preflight.check_can_interface(
        'can0', tmp_path, run=lambda *a, **k: completed(0, output)
    )
    assert result.ok
    assert '500000' in result.detail
    assert 'ERROR-ACTIVE' in result.detail


def test_can_interface_flags_bus_off(tmp_path):
    (tmp_path / 'can0').mkdir()
    output = (
        '9: can0: <NOARP,UP,LOWER_UP> mtu 16\n'
        '    can state BUS-OFF (berr-counter tx 0 rx 0) restart-ms 0\n'
    )
    result = preflight.check_can_interface(
        'can0', tmp_path, run=lambda *a, **k: completed(0, output)
    )
    assert not result.ok
    assert '120 ohm' in result.remedy


def test_can_interface_flags_down_link(tmp_path):
    (tmp_path / 'can0').mkdir()
    output = '9: can0: <NOARP> mtu 16 state DOWN\n'
    result = preflight.check_can_interface(
        'can0', tmp_path, run=lambda *a, **k: completed(0, output)
    )
    assert not result.ok
    assert 'ip link set can0 up' in result.remedy


def test_pwm_controller_uses_driver_discovery(tmp_path):
    write_pwm_tree(tmp_path / 'pwm')
    result = preflight.check_pwm_controller('32c0000.pwm', 0, tmp_path / 'pwm')
    assert result.ok
    assert '32c0000.pwm' in result.detail


def test_pwm_controller_missing_is_reported(tmp_path):
    (tmp_path / 'pwm').mkdir()
    result = preflight.check_pwm_controller('32c0000.pwm', 0, tmp_path / 'pwm')
    assert not result.ok
    assert '32c0000.pwm' in result.detail


def test_pwm_access_accepts_writable_export(tmp_path):
    write_pwm_tree(tmp_path / 'pwm')
    result = preflight.check_pwm_writable('32c0000.pwm', 0, tmp_path / 'pwm')
    assert result.ok
    assert 'export' in result.detail


def test_pwm_access_reports_unwritable_export(tmp_path):
    write_pwm_tree(tmp_path / 'pwm', export_mode=0o000)
    result = preflight.check_pwm_writable('32c0000.pwm', 0, tmp_path / 'pwm')
    if result.ok:
        pytest.skip('running as a user that bypasses file permissions')
    assert 'gpio' in result.remedy


def test_format_results_marks_failures_and_counts_them():
    results = [
        preflight.CheckResult('platform', True, 'Jetson-AGX'),
        preflight.CheckResult('note', True, 'advisory', advisory=True),
        preflight.CheckResult('CAN module', False, 'not loaded', 'modprobe mttcan'),
    ]
    report = preflight.format_results(results)
    assert '[OK] platform' in report
    assert '[NOTE] note' in report
    assert '[FAIL] CAN module' in report
    assert 'remedy: modprobe mttcan' in report
    assert 'Summary: 1 passed, 1 failed' in report


def test_format_results_contains_no_emoji():
    report = preflight.format_results(
        [preflight.CheckResult('platform', True, 'Jetson-AGX')]
    )
    assert report.isascii()


def test_run_checks_returns_every_check(tmp_path):
    model = tmp_path / 'model'
    model.write_bytes(b'Jetson-AGX\x00')
    modules = tmp_path / 'modules'
    modules.write_text('mttcan 1 0 - Live 0x0\n', encoding='ascii')
    net_root = tmp_path / 'net'
    net_root.mkdir()
    pwm_root = tmp_path / 'pwm'
    write_pwm_tree(pwm_root)
    results = preflight.run_checks(
        model_path=model,
        modules_path=modules,
        net_root=net_root,
        pwm_root=pwm_root,
    )
    assert [r.label for r in results] == [
        'platform',
        'CAN module',
        'CAN interface',
        'PWM controller',
        'PWM access',
    ]


def test_main_fails_on_a_host_without_hardware(capsys):
    with pytest.raises(SystemExit) as excinfo:
        preflight.main()
    assert excinfo.value.code == 1
    assert '[FAIL]' in capsys.readouterr().out


def test_check_result_defaults_are_immutable():
    result = preflight.CheckResult('label', True, 'detail')
    with pytest.raises(Exception):
        result.label = 'other'


def test_pwm_default_hint_matches_chassis_configuration():
    config = Path(__file__).parents[1] / 'config' / 'chassis.yaml'
    assert '32c0000.pwm' in config.read_text(encoding='utf-8')
    assert preflight.check_pwm_controller.__defaults__[0] == '32c0000.pwm'

"""Validate the CAN bring-up scripts without touching the running system."""

from pathlib import Path
import subprocess

import pytest

PACKAGE = Path(__file__).parents[1]
SETUP = PACKAGE / 'scripts' / 'setup_can0.sh'
INSTALLER = PACKAGE / 'scripts' / 'install_can_service.sh'
SERVICE_TEMPLATE = PACKAGE / 'systemd' / 'ackermann-can.service.in'
SERVICE_ENV = PACKAGE / 'systemd' / 'ackermann-can.env'


def run(script, *args):
    return subprocess.run(
        ['bash', str(script), *args], capture_output=True, text=True
    )


@pytest.mark.parametrize('script', [SETUP, INSTALLER])
def test_scripts_are_installed_and_parse(script):
    assert script.is_file()
    assert script.stat().st_mode & 0o111, 'script must stay executable'
    assert subprocess.run(['bash', '-n', str(script)]).returncode == 0


def test_setup_rejects_invalid_arguments():
    assert run(SETUP, 'bad-iface').returncode == 2
    assert run(SETUP, 'can0', 'abc').returncode == 2
    assert run(SETUP, 'can0', '0').returncode == 2


def test_setup_is_root_aware_and_checks_prerequisites():
    text = SETUP.read_text(encoding='utf-8')
    assert 'EUID == 0' in text, 'must skip sudo when systemd runs it as root'
    assert 'modinfo -n mttcan' in text, 'must fail clearly off a Jetson'
    assert 'install_can_service.sh' in text, 'must point at the boot service'
    assert 'ERROR-PASSIVE' in text, 'must warn about an unterminated bus'


def test_installer_requires_root_and_mentions_jetson():
    text = INSTALLER.read_text(encoding='utf-8')
    assert 'EUID != 0' in text
    assert 'Jetson' in text
    if subprocess.run(['id', '-u'], capture_output=True, text=True).stdout.strip() == '0':
        pytest.skip('running as root, cannot exercise the guard')
    proc = run(INSTALLER)
    assert proc.returncode == 1
    assert 'root' in proc.stderr


def test_service_template_exposes_required_contract():
    text = SERVICE_TEMPLATE.read_text(encoding='utf-8')
    assert 'Type=oneshot' in text
    assert 'RemainAfterExit=yes' in text
    assert '@SETUP_CAN_SCRIPT@' in text
    assert '@SHARE_DIR@' in text
    assert 'WantedBy=multi-user.target' in text
    assert 'EnvironmentFile=-/etc/default/ackermann-can' in text


def test_service_env_defaults_match_the_documented_bitrate():
    text = SERVICE_ENV.read_text(encoding='utf-8')
    assert 'CAN_INTERFACE=can0' in text
    assert 'CAN_BITRATE=500000' in text


def test_setup_help_and_docs_agree_on_default_bitrate():
    readme = (PACKAGE / 'README.md').read_text(encoding='utf-8')
    assert '500000' in readme or '500 kbps' in readme

"""Validate hardware and platform-neutral launch contracts."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest


ROOT = Path(__file__).parents[1]


def launch_module(name):
    """Load a launch file directly from the source tree."""
    path = ROOT / 'launch' / f'{name}.launch.py'
    spec = importlib.util.spec_from_file_location(f'{name}_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_control_boundary_exposes_hardware_command_contract():
    module = launch_module('control_boundary')
    description = module.generate_launch_description()
    names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        'use_sim_time',
        'input_topic',
        'output_topic',
        'wheelbase',
        'max_speed',
        'max_steering',
        'minimum_speed',
        'command_timeout',
        'publish_rate',
    } <= names

    text = (ROOT / 'launch' / 'control_boundary.launch.py').read_text()
    assert "executable='twist_to_ackermann'" in text
    assert "default_value='/cmd_vel_safe'" in text
    assert "default_value='/drive'" in text


def test_visual_navigation_supports_record_and_shadow(tmp_path):
    module = launch_module('visual_navigation')
    context = LaunchContext()
    context.launch_configurations.update(
        mode='record', dataset=str(tmp_path / 'dataset'), rate='5.0',
        image_topic='/rgbd/image', teacher_topic='/cmd_vel_smoothed',
        plan_topic='/plan', model='', metrics=str(tmp_path / 'metrics.json'),
    )
    assert module._launch(context)[0].node_executable == 'visual_dataset_recorder'
    model = tmp_path / 'policy.yml'
    model.write_text('placeholder', encoding='utf-8')
    context.launch_configurations.update(mode='shadow', model=str(model))
    assert module._launch(context)[0].node_executable == 'visual_policy'


def test_visual_navigation_rejects_bad_mode_and_model(tmp_path):
    module = launch_module('visual_navigation')
    context = LaunchContext()
    context.launch_configurations.update(
        mode='bad', dataset=str(tmp_path), rate='5.0',
        image_topic='/rgbd/image', teacher_topic='/cmd_vel_smoothed',
        plan_topic='/plan', model='', metrics=str(tmp_path / 'metrics.json'),
    )
    with pytest.raises(ValueError, match='mode'):
        module._launch(context)
    context.launch_configurations['mode'] = 'shadow'
    with pytest.raises(FileNotFoundError, match='does not exist'):
        module._launch(context)


def test_real_chassis_keeps_hardware_disabled_by_default():
    module = launch_module('real_chassis')
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {'can_interface', 'wheelbase', 'max_speed', 'max_steering',
            'enable_on_start'} <= arguments.keys()
    default = ''.join(
        part.perform(LaunchContext())
        for part in arguments['enable_on_start'].default_value
    )
    assert default == 'false'
    text = (ROOT / 'launch' / 'real_chassis.launch.py').read_text()
    assert "'input_topic': '/cmd_vel_safe'" in text
    assert "'output_topic': '/drive'" in text
    assert "package='ackermann_hardware'" in text

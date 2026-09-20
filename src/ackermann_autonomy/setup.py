from setuptools import setup
from glob import glob
import os


package_name = 'ackermann_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Platform-independent Ackermann autonomy algorithms.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgbd_audit = ackermann_autonomy.rgbd_audit:main',
            (
                'goal_pose_optimizer = '
                'ackermann_autonomy.goal_pose_optimizer:main'
            ),
            (
                'visual_dataset_recorder = '
                'ackermann_autonomy.visual_dataset_recorder:main'
            ),
            (
                'visual_policy_train = '
                'ackermann_autonomy.visual_policy_train:main'
            ),
            (
                'visual_policy = '
                'ackermann_autonomy.visual_policy_node:main'
            ),
            'navigation_regression = ackermann_autonomy.navigation_regression:main',
            'semantic_navigation = ackermann_autonomy.semantic_navigation_node:main',
            'line_follower = ackermann_autonomy.line_follower_node:main',
            'ackermann_to_twist = ackermann_autonomy.ackermann_to_twist_node:main',
            'twist_to_ackermann = ackermann_autonomy.twist_to_ackermann_node:main',
            'waypoint_tracker = ackermann_autonomy.waypoint_tracker_node:main',
            'nav2_waypoint_sender = ackermann_autonomy.nav2_waypoint_sender:main',
            'ackermann_dynamics_test = ackermann_autonomy.dynamics_test_node:main',
            'navigation_diagnostics = ackermann_autonomy.navigation_diagnostics_node:main',
            'navigation_plotter = ackermann_autonomy.navigation_plotter:main',
            'navigation_experiment = ackermann_autonomy.navigation_experiment_node:main',
        ],
    },
)

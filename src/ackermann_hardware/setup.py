from glob import glob
import os

from setuptools import setup


package_name = 'ackermann_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name, ['README.md']),
        ('lib/' + package_name, ['scripts/setup_can0.sh']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    description='SocketCAN and PWM chassis driver for Jetson AGX Xavier.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'chassis_node = ackermann_hardware.chassis_node:main',
        ],
    },
)

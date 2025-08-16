import os
from glob import glob
from setuptools import setup, find_packages


package_name = 'conveyor_control_topic'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CG',
    maintainer_email='natthawejumjai@gmail.com',
    description='Conveyor control via UDP using ROS 2 Topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor_udp_node = conveyor_control_topic.conveyor_udp_node:main',
            'led_hook_udp_node = conveyor_control_topic.led_hook_udp_node:main',
            'led_relay_udp_node = conveyor_control_topic.led_relay_udp_node:main',
        ],
    },
)

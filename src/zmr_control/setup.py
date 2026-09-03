from glob import glob

from setuptools import setup

package_name = 'zmr_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charan',
    maintainer_email='charanmdnl7456@gmail.com',
    description='Teleoperation, obstacle avoidance and waypoint navigation for ZMR.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_key = zmr_control.teleop_key:main',
            'obstacle_avoidance = zmr_control.obstacle_avoidance:main',
            'waypoint_navigator = zmr_control.waypoint_navigator:main',
        ],
    },
)

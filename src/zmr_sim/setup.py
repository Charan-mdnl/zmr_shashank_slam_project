from glob import glob

from setuptools import setup

package_name = 'zmr_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charan',
    maintainer_email='charanmdnl7456@gmail.com',
    description='Headless 2D simulator for the ZMR AMR.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_node = zmr_sim.sim_node:main',
            'make_world = zmr_sim.worlds:main',
        ],
    },
)

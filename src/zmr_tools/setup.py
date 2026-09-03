from setuptools import setup

package_name = 'zmr_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Charan',
    maintainer_email='charanmdnl7456@gmail.com',
    description='Headless map saving and rendering utilities for ZMR.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_saver = zmr_tools.map_saver:main',
            'path_recorder = zmr_tools.path_recorder:main',
            'render_map = zmr_tools.render_map:main',
            'lifecycle_manager = zmr_tools.lifecycle_manager:main',
        ],
    },
)

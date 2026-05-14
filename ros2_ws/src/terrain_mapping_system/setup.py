from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'terrain_mapping_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 bridge + pose adapter for Gazebo Harmonic + ArduPilot terrain mapping',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pose_adapter = terrain_mapping_system.nodes.pose_adapter:main',
            'mission_controller = terrain_mapping_system.nodes.mission_controller:main',
            'dem_mapper = terrain_mapping_system.nodes.dem_mapper:main',
            'terrain_run_evaluator = terrain_mapping_system.validation.evaluator:main',
            'terrain_batch_experiments = terrain_mapping_system.validation.batch:main',
        ],
    },
)

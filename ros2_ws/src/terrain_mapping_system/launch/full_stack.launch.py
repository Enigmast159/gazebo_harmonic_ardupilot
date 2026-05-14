"""Launch the Gazebo bridge, mission controller, and DEM mapper together."""

import os
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from terrain_mapping_system.paths import default_results_root


def generate_launch_description():
    package_share = get_package_share_directory('terrain_mapping_system')
    mission_config = os.path.join(package_share, 'config', 'mission_controller.yaml')
    mapper_config = os.path.join(package_share, 'config', 'dem_mapper.yaml')
    generated_run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')

    run_id_arg = DeclareLaunchArgument(
        'run_id',
        default_value=generated_run_id,
        description='Shared run identifier for mission and mapper artifacts.',
    )
    results_root_arg = DeclareLaunchArgument(
        'results_root',
        default_value=default_results_root(),
        description='Root directory for run artifacts.',
    )
    mission_config_arg = DeclareLaunchArgument(
        'mission_config_file',
        default_value=mission_config,
        description='Mission controller YAML config file.',
    )
    mapper_config_arg = DeclareLaunchArgument(
        'mapper_config_file',
        default_value=mapper_config,
        description='DEM mapper YAML config file.',
    )
    mavlink_connection_arg = DeclareLaunchArgument(
        'mavlink_connection_string',
        default_value='tcp:127.0.0.1:5760',
        description='pymavlink connection string.',
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Plan and initialize without issuing MAVLink commands.',
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='random_terrain_ardupilot',
        description='Gazebo world name used by the bridge defaults.',
    )
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='iris_with_ardupilot',
        description='Gazebo vehicle model name used by the bridge defaults.',
    )

    sim_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'sim_bridge.launch.py')),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name'),
            'model_name': LaunchConfiguration('model_name'),
        }.items(),
    )

    mission_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'mission_control.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration('mission_config_file'),
            'run_id': LaunchConfiguration('run_id'),
            'results_root': LaunchConfiguration('results_root'),
            'mavlink_connection_string': LaunchConfiguration('mavlink_connection_string'),
            'dry_run': LaunchConfiguration('dry_run'),
        }.items(),
    )

    dem_mapper = Node(
        package='terrain_mapping_system',
        executable='dem_mapper',
        name='dem_mapper',
        output='screen',
        parameters=[
            LaunchConfiguration('mapper_config_file'),
            {
                'run_id': LaunchConfiguration('run_id'),
                'results_root': LaunchConfiguration('results_root'),
                'use_sim_time': True,
            },
        ],
    )

    return LaunchDescription([
        run_id_arg,
        results_root_arg,
        mission_config_arg,
        mapper_config_arg,
        mavlink_connection_arg,
        dry_run_arg,
        world_name_arg,
        model_name_arg,
        sim_bridge,
        mission_controller,
        dem_mapper,
    ])

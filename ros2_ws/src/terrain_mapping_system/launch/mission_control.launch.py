"""Launch the pymavlink-based mission controller."""

import os
from datetime import datetime, timezone

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from terrain_mapping_system.paths import default_results_root


def generate_launch_description():
    package_share = get_package_share_directory('terrain_mapping_system')
    default_config = os.path.join(package_share, 'config', 'mission_controller.yaml')
    generated_run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Mission controller YAML config file.',
    )
    run_id_arg = DeclareLaunchArgument(
        'run_id',
        default_value=generated_run_id,
        description='Run identifier for mission artifacts.',
    )
    results_root_arg = DeclareLaunchArgument(
        'results_root',
        default_value=default_results_root(),
        description='Root directory for mission artifacts.',
    )
    connection_arg = DeclareLaunchArgument(
        'mavlink_connection_string',
        default_value='tcp:127.0.0.1:5760',
        description='pymavlink connection string, e.g. tcp:127.0.0.1:5760 or udp:127.0.0.1:14550.',
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Plan and initialize without issuing MAVLink flight commands.',
    )

    node = Node(
        package='terrain_mapping_system',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'run_id': LaunchConfiguration('run_id'),
                'results_root': LaunchConfiguration('results_root'),
                'mavlink.connection_string': LaunchConfiguration('mavlink_connection_string'),
                'dry_run': LaunchConfiguration('dry_run'),
                'use_sim_time': True,
            },
        ],
    )

    return LaunchDescription([
        config_arg,
        run_id_arg,
        results_root_arg,
        connection_arg,
        dry_run_arg,
        node,
    ])

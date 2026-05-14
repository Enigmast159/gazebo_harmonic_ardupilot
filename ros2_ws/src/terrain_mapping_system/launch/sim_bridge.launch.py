"""ROS 2 ingress bridge for the Gazebo Harmonic terrain simulation."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_defaults():
    package_share = get_package_share_directory('terrain_mapping_system')
    defaults_path = os.path.join(package_share, 'config', 'bridge_defaults.yaml')
    with open(defaults_path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    defaults = _load_defaults()
    gazebo_defaults = defaults['gazebo_topics']
    ros_topics = defaults['ros_topics']
    frames = defaults['frames']
    world_name = LaunchConfiguration('world_name')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=defaults['world_name'],
        description='Gazebo world name. Used only for topic defaults and debugging.',
    )
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value=defaults['model_name'],
        description='Gazebo model name for the vehicle.',
    )
    model_name = LaunchConfiguration('model_name')

    gz_lidar_topic_arg = DeclareLaunchArgument(
        'gz_lidar_topic',
        default_value=gazebo_defaults['lidar_points'],
        description='Raw Gazebo LiDAR PointCloudPacked topic.',
    )
    gz_pose_topic_arg = DeclareLaunchArgument(
        'gz_pose_topic',
        default_value=['/model/', model_name, '/pose'],
        description='Raw Gazebo vehicle pose topic from PosePublisher.',
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish map -> base_link TF from the normalized pose.',
    )

    gz_lidar_topic = LaunchConfiguration('gz_lidar_topic')
    gz_pose_topic = LaunchConfiguration('gz_pose_topic')
    publish_tf = LaunchConfiguration('publish_tf')

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='terrain_mapping_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            [gz_lidar_topic, '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            [gz_pose_topic, '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
        ],
        remappings=[
            (gz_lidar_topic, ros_topics['lidar_points']),
            (gz_pose_topic, ros_topics['vehicle_pose_raw']),
        ],
        parameters=[{'use_sim_time': True}],
    )

    pose_adapter_node = Node(
        package='terrain_mapping_system',
        executable='pose_adapter',
        name='pose_adapter',
        output='screen',
        remappings=[
            ('vehicle/pose/raw', ros_topics['vehicle_pose_raw']),
            ('vehicle/pose', ros_topics['vehicle_pose']),
        ],
        parameters=[
            {'use_sim_time': True},
            {'frame_id': frames['map']},
            {'child_frame_id': frames['base_link']},
            {'publish_tf': publish_tf},
        ],
    )

    return LaunchDescription([
        world_name_arg,
        model_name_arg,
        gz_lidar_topic_arg,
        gz_pose_topic_arg,
        publish_tf_arg,
        bridge_node,
        pose_adapter_node,
    ])

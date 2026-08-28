import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tags_config = os.path.join(
        get_package_share_directory('rescuebot_perception'),
        'config',
        'tags.yaml'
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[tags_config],
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
    )

    tag_marker_node = Node(
        package='rescuebot_perception',
        executable='tag_marker_node',
        name='tag_marker_node',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'tag_frame_prefix': 'tag_',
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        apriltag_node,
        tag_marker_node,
    ])

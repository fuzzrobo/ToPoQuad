import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace='ns',
        output='screen',
    )

    rviz_config_path = PathJoinSubstitution([FindPackageShare("ais_gng"), 'config', 'rviz.rviz'])

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    return LaunchDescription([
        joy_node,
        rviz_node
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    topoquad_master_config = os.path.join(
        get_package_share_directory("topoquad_master"),
        "config",
        "topoquad_master.yaml",
    )

    topoquad_robot_profile = os.path.join(
        get_package_share_directory("topoquad_master"),
        "config",
        "robot_profiles",
        "topoquad.yaml",
    )

    leg_node = Node(
        package="topoquad_master",
        executable="leg_node",
        namespace="topoquad",
        name="leg_node",
        output="screen",
        parameters=[topoquad_master_config, topoquad_robot_profile],
    )

    neck_node = Node(
        package="topoquad_master",
        executable="neck_node",
        namespace="topoquad",
        name="neck_node",
        output="screen",
        parameters=[topoquad_master_config, topoquad_robot_profile],
    )

    return LaunchDescription([leg_node, neck_node])

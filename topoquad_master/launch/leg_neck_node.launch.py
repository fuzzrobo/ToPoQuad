import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("topoquad_master")
    topoquad_master_config = os.path.join(package_share, "config", "topoquad_master.yaml")
    default_model_config = os.path.join(package_share, "config", "models", "topoquad.yaml")
    model_config = LaunchConfiguration("model_config")
    model_config_argument = DeclareLaunchArgument( # yaml ファイルのパスを model_config 引数に紐づける．
        "model_config",
        default_value=default_model_config,
    )
    leg_node = Node(
        package="topoquad_master",
        executable="leg_node",
        namespace="topoquad",
        name="leg_node",
        output="screen",
        parameters=[topoquad_master_config, model_config],
    )
    neck_node = Node(
        package="topoquad_master",
        executable="neck_node",
        namespace="topoquad",
        name="neck_node",
        output="screen",
        parameters=[topoquad_master_config, model_config],
    )
    return LaunchDescription([model_config_argument, leg_node, neck_node])

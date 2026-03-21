import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    topoquad_master_launch = os.path.join(
        get_package_share_directory("topoquad_master"),
        "launch",
        "topoquad_master.launch.py",
    )
    topoquad_master_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(topoquad_master_launch)
    )
    return LaunchDescription([topoquad_master_nodes])

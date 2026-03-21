#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def is_valid_to_launch():
    path = "/sys/firmware/devicetree/base/model"
    return not os.path.exists(path)


def generate_launch_description():
    if not is_valid_to_launch():
        print("Can not launch fake robot in Raspberry Pi")
        return LaunchDescription([])

    model_launch = os.path.join(
        get_package_share_directory("topoquad_description"),
        "launch",
        "model.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(model_launch),
            )
        ]
    )

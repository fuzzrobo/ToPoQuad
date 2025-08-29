from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dynamixel_handler_config = os.path.join(
        get_package_share_directory('topoquad_master'),
        'config',
        'dynamixel_handler.yaml'
    )

    dynamixel_handler_node = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler_node',
        name='dxl_handler',
        namespace='topoquad',
        output='screen',
        emulate_tty=True,
        parameters=[dynamixel_handler_config]
    )

    return LaunchDescription([
        dynamixel_handler_node,
    ])

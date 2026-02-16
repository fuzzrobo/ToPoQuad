from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config1 = os.path.join(
        get_package_share_directory("topoquad_master"),
        'config',
        'dynamixel_unify_baudrate.yaml'
    )

    node1 = Node(
        package="dynamixel_handler",
        executable='dynamixel_unify_baudrate',
        name='dxl_unify_handler',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config1]
    )

    ld.add_action(node1)

    return ld

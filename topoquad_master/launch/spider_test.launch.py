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
        namespace='ns',
        output='screen',
        emulate_tty=True,
        parameters=[dynamixel_handler_config]
    )

    topoquad_master_config = os.path.join(
        get_package_share_directory('topoquad_master'),
        'config',
        'topoquad_master.yaml'
    )
    
    leg_node = Node(
        package='topoquad_master',
        executable='leg_node_smooth',
        namespace='ns',
        name='leg_node_smooth',
        output='screen',
        parameters=[topoquad_master_config]
    )
    
    neck_node = Node(
        package='topoquad_master',
        executable='neck_node',
        namespace='ns',
        name='neck_node',
        output='screen',
    )

    return LaunchDescription([
        dynamixel_handler_node,
        leg_node,
        neck_node
    ])

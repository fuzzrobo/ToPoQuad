import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('topoquad_master')
    dynamixel_handler_config = os.path.join(package_share, 'config', 'dynamixel_handler.yaml')
    default_model_config = os.path.join(package_share, 'config', 'models', 'topoquad.yaml')
    model_config = LaunchConfiguration('model_config')
    model_config_argument = DeclareLaunchArgument( # yaml ファイルのパスを model_config 引数に紐づける． 
        'model_config',
        default_value=default_model_config,
    )
    dynamixel_handler_node = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler',
        name='dxl_handler',
        namespace='topoquad',
        output='screen',
        emulate_tty=True,
        parameters=[
            dynamixel_handler_config,
            model_config,
        ],
    )
    return LaunchDescription([model_config_argument, dynamixel_handler_node])

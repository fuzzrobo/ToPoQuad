import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import launch
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    topoquad_config = os.path.join(
        get_package_share_directory('tb3_driver'),
        'config',
        'topoquad.yaml'
    )
    
    print(topoquad_config)

    ais_gng = Node(
        package='ais_gng',
        executable='ais_gng',
        output='screen',
        parameters=[topoquad_config]
    )

    realsense_driver = Node(
        package='tb3_driver',
        executable='realsense_driver',
        parameters=[topoquad_config],
        output='screen',
    )
    
    teleop_sample = Node(
        package='topoquad_control',
        executable='teleop_node',
        namespace='ns',
        output='screen',
    )
    
    spider_test_launch_path = PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare("topoquad_master"),"launch","spider_test.launch.py"])])

    spider_test_launch = IncludeLaunchDescription(
        spider_test_launch_path,
        launch_arguments={}.items()
    )

    return LaunchDescription([
        ais_gng,
        realsense_driver,
        teleop_sample,
        spider_test_launch
    ])

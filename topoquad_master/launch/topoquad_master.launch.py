import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description():
    package_share = get_package_share_directory("topoquad_master")
    model_name = LaunchConfiguration("model")
    is_topoquad = PythonExpression(["'", model_name, "' == 'topoquad'"])
    is_topoquad_lite = PythonExpression(["'", model_name, "'.replace('_', '-') == 'topoquad-lite'"])
    is_supported_model = PythonExpression(["'", model_name, "'.replace('_', '-') in ['topoquad', 'topoquad-lite']"])
    topoquad_model_config = os.path.join(package_share, "config", "models", "topoquad.yaml")
    topoquad_lite_model_config = os.path.join(package_share, "config", "models", "topoquad-lite.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument( # model 引数にロボット名を紐づける． デフォルトでは環境変数 topoquad_model を参照する．
                "model",
                default_value=EnvironmentVariable("topoquad_model", default_value="topoquad"),
            ),
            LogInfo( # model 引数に紐づけたロボット名がサポートされているかチェック．
                condition=UnlessCondition(is_supported_model),
                msg=[ "\033[31mUnsupported topoquad model '", model_name, "'. Supported values: topoquad, topoquad-lite", "\033[0m"],
            ),
            Shutdown( # サポートされていないロボット名の場合は起動を中止する．
                condition=UnlessCondition(is_supported_model),
                reason="Unsupported topoquad model",
            ),
            LogInfo( # サポートされているロボット名の場合は起動するモデルを表示する．
                condition=IfCondition(is_supported_model),
                msg=["\033[36mResolved topoquad model: ", model_name, "\033[0m"],
            ), 
            IncludeLaunchDescription( # model 引数のロボット名が topoquad の場合
                PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "dynamixel_handler.launch.py")),
                launch_arguments={"model_config": topoquad_model_config}.items(),
                condition=IfCondition(is_topoquad),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "leg_neck_node.launch.py")),
                launch_arguments={"model_config": topoquad_model_config}.items(),
                condition=IfCondition(is_topoquad),
            ),
            IncludeLaunchDescription( # model 引数のロボット名が topoquad-lite の場合
                PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "dynamixel_handler.launch.py")),
                launch_arguments={"model_config": topoquad_lite_model_config}.items(),
                condition=IfCondition(is_topoquad_lite),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_share, "launch", "leg_neck_node.launch.py")),
                launch_arguments={"model_config": topoquad_lite_model_config}.items(),
                condition=IfCondition(is_topoquad_lite),
            ),
        ]
    )

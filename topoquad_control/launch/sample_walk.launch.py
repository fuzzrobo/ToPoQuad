from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    model_name = LaunchConfiguration("model")
    is_topoquad = PythonExpression(["'", model_name, "' == 'topoquad'"])
    is_topoquad_lite = PythonExpression(["'", model_name, "'.replace('_', '-') == 'topoquad-lite'"])
    is_supported_model = PythonExpression(["'", model_name, "'.replace('_', '-') in ['topoquad', 'topoquad-lite']"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=EnvironmentVariable("topoquad_model", default_value="topoquad"),
            ),
            LogInfo(
                condition=UnlessCondition(is_supported_model),
                msg=["\033[31mUnsupported topoquad model '", model_name, "'. Supported values: topoquad, topoquad-lite", "\033[0m",],
            ),
            Shutdown(
                condition=UnlessCondition(is_supported_model),
                reason="Unsupported topoquad model",
            ),
            LogInfo(
                condition=IfCondition(is_supported_model),
                msg=["\033[36mResolved topoquad model: ", model_name, "\033[0m"],
            ),
            Node(
                package="topoquad_control",
                executable="walk_node",
                name="walk_sample",
                namespace="topoquad",
                output="screen",
                parameters=[{"r": 0.00, "s": 0.04, "h": 0.015, "x_offset": 0.095, "y_offset": 0.09, "z_offset": 0.085}],  # r: 周回軌道の半径, s: 左右方向の歩幅オフセット, h: 上下方向の脚上げ高さ, x/y/z_offset: 足先軌道中心の各軸オフセット量
                condition=IfCondition(is_topoquad),
            ),
            Node(
                package="topoquad_control",
                executable="walk_node",
                name="walk_sample",
                namespace="topoquad",
                output="screen",
                parameters=[{"r": 0.00, "s": 0.04, "h": 0.02, "x_offset": 0.08, "y_offset": 0.09, "z_offset": 0.08}],  # r: 周回軌道の半径, s: 左右方向の歩幅オフセット, h: 上下方向の脚上げ高さ, x/y/z_offset: 足先軌道中心の各軸オフセット量
                condition=IfCondition(is_topoquad_lite),
            ),
        ]
    )

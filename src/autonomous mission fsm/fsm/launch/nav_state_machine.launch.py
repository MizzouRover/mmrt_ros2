from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_mock_feed = LaunchConfiguration("use_mock_feed")
    use_ui = LaunchConfiguration("use_ui")
    use_foxglove_bridge = LaunchConfiguration("use_foxglove_bridge")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_feed",
            default_value="true",
            description="Launch the simulated navigation telemetry feed.",
        ),
        DeclareLaunchArgument(
            "use_ui",
            default_value="false",
            description="Launch the state machine vision UI.",
        ),
        DeclareLaunchArgument(
            "use_foxglove_bridge",
            default_value="false",
            description="Launch the Foxglove WebSocket bridge.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="WebSocket port used by foxglove_bridge.",
        ),
        Node(
            package="fsm",
            executable="nav_state_machine",
            name="nav_state_machine",
            output="screen",
        ),
        Node(
            package="fsm",
            executable="mock_nav_feed",
            name="mock_nav_feed",
            output="screen",
            condition=IfCondition(use_mock_feed),
        ),
        Node(
            package="fsm",
            executable="state_machine_ui",
            name="state_machine_ui",
            output="screen",
            condition=IfCondition(use_ui),
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": foxglove_port}],
            condition=IfCondition(use_foxglove_bridge),
        ),
    ])

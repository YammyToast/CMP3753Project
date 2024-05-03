from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    test_config = LaunchConfiguration("test_config", default="../test_config.py")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "test_config",
                default_value=test_config,
                description="Python File detailing the set of algorithms and maps used in testing.",
            ),
            Node(
                package="copland_testing",
                namespace="coplandnavi",
                executable="copland_main",
                name="copland_main",
            ),
            Node(
                package="copland_testing",
                namespace="coplandnavi",
                executable="copland_metrics",
                name="copland_metrics",
            ),
            Node(
                package="copland_testing",
                namespace="coplandnavi",
                executable="copland_environment",
                name="copland_environment",
            ),
            Node(
                package="navi",
                namespace="coplandnavi",
                executable="navi_main",
                name="navi_main",
                parameters=[{"test_config": test_config}],
            ),
            Node(
                package="navi",
                namespace="coplandnavi",
                executable="navi_navigation",
                name="navi_navigation",
            ),
        ]
    )

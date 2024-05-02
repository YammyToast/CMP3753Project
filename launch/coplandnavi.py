from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_main',
            name='copland_main'
        ),
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_metrics',
            name='copland_metrics'
        ),
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_environment',
            name='copland_environment'
        ),
        Node(
            package='navi',
            namespace='coplandnavi',
            executable='navi_main',
            name='navi_main'

        ),
        Node(
            package='navi',
            namespace='coplandnavi',
            executable='navi_navigation',
            name='navi_navigation'
        )

    ])
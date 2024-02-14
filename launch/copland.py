from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_main',
            name='main'
        ),
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_metrics',
            name='metrics'
        ),
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_environment',
            name='environment'
        )

    ])
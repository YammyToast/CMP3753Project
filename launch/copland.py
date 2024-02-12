from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='copland_testing',
            namespace='coplandnavi',
            executable='copland_main',
            name='main'
        ),
        # Node(
        #     package='copland_testing',
        #     namespace='coplandnavi',
        #     executable=''
        # )

    ])
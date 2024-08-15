from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='booblik',
            namespace='booblik',
            executable='motors',
            name='motors',
            output='screen'
        ),
        Node(
            package='booblik',
            namespace='booblik',
            executable='ws_m181',
            name='ws_m181',
            output='screen'
        ),
        Node(
            package='booblik',
            namespace='booblik',
            executable='QMC5883L',
            name='QMC5883L',
            output='screen'
        )
    ]) 

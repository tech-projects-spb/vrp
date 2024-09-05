from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
 

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ping', default_value='false', description='Use sounder'),
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
        ),
        Node(
            package='booblik',
            namespace='booblik',
            executable='ping',
            name='ping',
            output='screen',
            parameters=[{'ping': LaunchConfiguration('ping')}],
            condition=IfCondition(LaunchConfiguration('ping'))
        )
    ]) 

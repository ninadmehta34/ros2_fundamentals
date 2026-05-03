from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='actions_quiz',
            executable='navigate_server_executable',
            output='screen')
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='visualizer',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='joystick',
            executable='joystick',
            name='joystick'
        ),
    
    ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       Node(
            package='camera',
            executable='camera',
            name='camera',
            output="screen"
        ),
        Node(
            package='movement_manager',
            executable='movement_manager',
            name='movement_manager',
            output="screen"
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output="screen",
            arguments= ['serial', '--dev', '/dev/ttyACM0']
        ),
    ])
    
    

    
    

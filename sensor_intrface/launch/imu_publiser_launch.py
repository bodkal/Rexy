from launch import LaunchDescription, substitutions
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    imu_node = Node(
        package='sensor_intrface',
        executable='imu_publiser',
        output='screen',
        respawn=True
    )

    ld.add_action(imu_node)

    return ld
    
    

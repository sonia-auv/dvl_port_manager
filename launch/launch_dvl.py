from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dvl_port_manager",
                namespace="dvl_port_manager",
                executable="dvl_port_manager",
                name="provider_dvl",
            )
        ]
    )

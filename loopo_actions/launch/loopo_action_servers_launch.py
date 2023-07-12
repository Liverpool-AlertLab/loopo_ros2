from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='loopo_driver',
            namespace='loopo_driver',
            executable='loopo_driver_node',
            name='driver'
        ),
        Node(
            package='loopo_actions',
            namespace='loopo_actions',
            executable='extension_actions_server',
            name='extension_action'
        ),
    ])
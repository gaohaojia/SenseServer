from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    small_gicp_node = Node(
        package='small_gicp',
        executable='small_gicp_node',
        name='small_gicp',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(small_gicp_node)
    
    return ld
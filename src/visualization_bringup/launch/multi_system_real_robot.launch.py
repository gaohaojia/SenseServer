import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_file = os.path.join(get_package_share_directory('visualization_bringup'), 'rviz', 'multi_visualization.rviz')

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(start_rviz)

    return ld
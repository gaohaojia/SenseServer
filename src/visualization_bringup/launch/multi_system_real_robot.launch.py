import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    rviz_config_file = os.path.join(get_package_share_directory('visualization_bringup'), 'rviz', 'gicp_visualization.rviz')

    start_gicp_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    start_small_gicp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('small_gicp'), 'launch', 'small_gicp.launch.py'
        ))
    )

    ld = LaunchDescription()

    # Add the actions
    for idx in range(5):
        start_robot_rviz = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('visualization_bringup'), 'rviz', 'robot_' + str(idx) + '_visualization.rviz')],
            output='screen'
        )
        ld.add_action(start_robot_rviz)
        
    ld.add_action(start_gicp_rviz)
    ld.add_action(start_small_gicp)

    return ld
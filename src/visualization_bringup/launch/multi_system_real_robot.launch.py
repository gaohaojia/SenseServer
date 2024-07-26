import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_count = LaunchConfiguration('robot_count')
    declare_robot_count = DeclareLaunchArgument('robot_count', default_value='3', description='')

    rviz_config_file = os.path.join(get_package_share_directory('visualization_bringup'), 'rviz', 'visualization.rviz')

    start_foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(
            get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    start_explored_area = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('explored_area'), 'launch', 'explored_area.launch.py')
        ),
        launch_arguments={
            'robot_count': robot_count
        }.items()
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    start_robot_communication = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_communication'), 'launch', 'robot_communication.launch.py')
        ),
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(declare_robot_count)
    
    ld.add_action(TimerAction(period=5.0, actions=[start_explored_area]))

    ld.add_action(start_foxglove_bridge)
    ld.add_action(start_rviz)
    ld.add_action(start_robot_communication)

    return ld
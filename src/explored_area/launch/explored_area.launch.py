from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_count = LaunchConfiguration('robot_count')
    declare_robot_count = DeclareLaunchArgument('robot_count', default_value='0', description='')

    start_explored_area = Node(
        package='explored_area',
        executable='explored_area_node',
        name='explored_area',
        output='screen',
        parameters=[{
            'robot_count': robot_count
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_count)

    ld.add_action(start_explored_area)
    
    return ld
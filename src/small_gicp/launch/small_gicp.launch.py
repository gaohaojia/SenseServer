from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_count = LaunchConfiguration('robot_count')
    declare_robot_count = DeclareLaunchArgument('robot_count', default_value='5', description='')

    small_gicp_node = Node(
        package='small_gicp',
        executable='small_gicp_node',
        name='small_gicp',
        output='screen',
        parameters=[{
            'robot_count': robot_count
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_count)

    ld.add_action(small_gicp_node)
    
    return ld
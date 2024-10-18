from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_count = LaunchConfiguration('robot_count')
    network_port = LaunchConfiguration('network_port')
    network_ip = LaunchConfiguration('network_ip')

    declare_robot_count = DeclareLaunchArgument('robot_count', default_value='3', description='')
    declare_network_port = DeclareLaunchArgument('network_port', default_value='12130', description='')
    declare_network_ip = DeclareLaunchArgument('network_ip', default_value='192.168.31.208', description='')

    robot_communication_node = Node(
        package='robot_communication',
        executable='robot_communication_node',
        name='robot_communication',
        output='screen',
        respawn=True,
        parameters=[{
            'robot_count': robot_count,
            'network_port': network_port,
            'network_ip': network_ip,
        }]
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_robot_count)
    ld.add_action(declare_network_port)
    ld.add_action(declare_network_ip)
    
    ld.add_action(robot_communication_node)
    return ld
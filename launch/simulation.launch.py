from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='circle',
        choices=['circle', 'straight'],
        description='Path type for simulation (circle or straight)'
    )
    
    path_radius_arg = DeclareLaunchArgument(
        'path_radius',
        default_value='10.0',
        description='Radius of circular path (meters)'
    )
    
    vehicle_speed_arg = DeclareLaunchArgument(
        'vehicle_speed',
        default_value='1.0',
        description='Vehicle forward speed (m/s)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publishing rate (Hz)'
    )
    
    # Get launch configurations
    path_type = LaunchConfiguration('path_type')
    path_radius = LaunchConfiguration('path_radius')
    vehicle_speed = LaunchConfiguration('vehicle_speed')
    publish_rate = LaunchConfiguration('publish_rate')
    
    # Simulation node
    simulation_node = Node(
        package='loader_control',
        executable='simulation_node',
        name='simulation_node',
        output='screen',
        parameters=[{
            'path_type': path_type,
            'path_radius': path_radius,
            'vehicle_speed': vehicle_speed,
            'publish_rate': publish_rate,
        }]
    )
    
    return LaunchDescription([
        path_type_arg,
        path_radius_arg,
        vehicle_speed_arg,
        publish_rate_arg,
        simulation_node,
    ])


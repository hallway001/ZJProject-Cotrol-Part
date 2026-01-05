from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='loader_control').find('loader_control')
    
    # Declare launch arguments
    vehicle_model_arg = DeclareLaunchArgument(
        'vehicle_model',
        default_value='loader_v1',
        description='Vehicle model configuration (loader_v1 or loader_v2)'
    )
    
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='pid',
        choices=['pid', 'pure_pursuit', 'purepursuit'],
        description='Controller type to use'
    )
    
    use_actuator_models_arg = DeclareLaunchArgument(
        'use_actuator_models',
        default_value='true',
        description='Whether to use actuator models for delay/error simulation'
    )
    
    enable_error_monitor_arg = DeclareLaunchArgument(
        'enable_error_monitor',
        default_value='true',
        description='Enable steering error monitoring'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    # Get launch configurations
    vehicle_model = LaunchConfiguration('vehicle_model')
    controller_type = LaunchConfiguration('controller_type')
    use_actuator_models = LaunchConfiguration('use_actuator_models')
    enable_error_monitor = LaunchConfiguration('enable_error_monitor')
    enable_visualization = LaunchConfiguration('enable_visualization')
    
    # Load common configuration files
    common_config_dir = PathJoinSubstitution([pkg_share, 'config', 'common'])
    controller_config_dir = PathJoinSubstitution([pkg_share, 'config', 'controllers'])
    vehicle_config_dir = PathJoinSubstitution([pkg_share, 'config', vehicle_model])
    
    # Control node
    control_node = Node(
        package='loader_control',
        executable='loader_control_node',
        name='loader_control_node',
        output='screen',
        parameters=[
            # Common parameters
            PathJoinSubstitution([common_config_dir, 'sensors.yaml']),
            PathJoinSubstitution([common_config_dir, 'actuator.yaml']),
            
            # Controller parameters - loaded conditionally based on controller_type
            # Note: The node will load controller parameters via declare_parameter
            # If you have controller YAML files, uncomment and fix the path below:
            # PathJoinSubstitution([controller_config_dir, controller_type, '.yaml']),
            
            # Vehicle model parameters (overrides common)
            PathJoinSubstitution([vehicle_config_dir, 'actuator.yaml']),
            
            # Error monitor parameters
            PathJoinSubstitution([pkg_share, 'config', 'error_monitor.yaml']),
            
            # Node parameters
            {
                'controller_type': controller_type,
                'vehicle_model': vehicle_model,
                'use_actuator_models': use_actuator_models,
                'enable_error_monitor': enable_error_monitor,
                'enable_visualization': enable_visualization,
                'control_rate': 50.0,
            }
        ],
        remappings=[
            ('/loader/cmd_vel', '/loader/cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        vehicle_model_arg,
        controller_type_arg,
        use_actuator_models_arg,
        enable_error_monitor_arg,
        enable_visualization_arg,
        control_node,
    ])


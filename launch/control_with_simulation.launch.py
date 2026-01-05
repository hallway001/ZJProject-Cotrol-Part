from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
    
    # Simulation arguments
    path_type_arg = DeclareLaunchArgument(
        'path_type',
        default_value='circle',
        choices=['circle', 'straight'],
        description='Path type for simulation'
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
    
    # Get launch configurations
    vehicle_model = LaunchConfiguration('vehicle_model')
    controller_type = LaunchConfiguration('controller_type')
    use_actuator_models = LaunchConfiguration('use_actuator_models')
    enable_error_monitor = LaunchConfiguration('enable_error_monitor')
    enable_visualization = LaunchConfiguration('enable_visualization')
    path_type = LaunchConfiguration('path_type')
    path_radius = LaunchConfiguration('path_radius')
    vehicle_speed = LaunchConfiguration('vehicle_speed')
    
    # Load common configuration files
    common_config_dir = PathJoinSubstitution([pkg_share, 'config', 'common'])
    controller_config_dir = PathJoinSubstitution([pkg_share, 'config', 'controllers'])
    vehicle_config_dir = PathJoinSubstitution([pkg_share, 'config', vehicle_model])
    
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
            'publish_rate': 50.0,
        }]
    )
    
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
            
            # Controller parameters (will be loaded based on controller_type in node)
            # Note: The node will load controller config based on controller_type parameter
            
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
        path_type_arg,
        path_radius_arg,
        vehicle_speed_arg,
        simulation_node,
        control_node,
    ])


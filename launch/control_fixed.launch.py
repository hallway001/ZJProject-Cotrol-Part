from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
    
    return LaunchDescription([
        vehicle_model_arg,
        controller_type_arg,
        use_actuator_models_arg,
        enable_error_monitor_arg,
        enable_visualization_arg,
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    # Get package directory
    pkg_share = FindPackageShare(package='loader_control').find('loader_control')
    
    # Get launch configurations (now they are resolved)
    vehicle_model = LaunchConfiguration('vehicle_model').perform(context)
    controller_type = LaunchConfiguration('controller_type').perform(context)
    use_actuator_models = LaunchConfiguration('use_actuator_models').perform(context)
    enable_error_monitor = LaunchConfiguration('enable_error_monitor').perform(context)
    enable_visualization = LaunchConfiguration('enable_visualization').perform(context)
    
    # Handle controller type mapping
    if controller_type in ['pure_pursuit', 'purepursuit']:
        controller_config_file = 'pure_pursuit.yaml'
    else:
        controller_config_file = f'{controller_type}.yaml'
    
    # Build paths
    import os
    common_config_dir = os.path.join(pkg_share, 'config', 'common')
    controller_config_dir = os.path.join(pkg_share, 'config', 'controllers')
    vehicle_config_dir = os.path.join(pkg_share, 'config', vehicle_model)
    
    # Build parameter list
    parameters = [
        # Common parameters
        os.path.join(common_config_dir, 'sensors.yaml'),
        os.path.join(common_config_dir, 'actuator.yaml'),
        
        # Controller parameters
        os.path.join(controller_config_dir, controller_config_file),
        
        # Vehicle model parameters
        os.path.join(vehicle_config_dir, 'actuator.yaml'),
        
        # Error monitor parameters
        os.path.join(pkg_share, 'config', 'error_monitor.yaml'),
        
        # Node parameters
        {
            'controller_type': controller_type,
            'vehicle_model': vehicle_model,
            'use_actuator_models': use_actuator_models,
            'enable_error_monitor': enable_error_monitor,
            'enable_visualization': enable_visualization,
            'control_rate': 50.0,
        }
    ]
    
    # Control node
    control_node = Node(
        package='loader_control',
        executable='loader_control_node',
        name='loader_control_node',
        output='screen',
        parameters=parameters,
        remappings=[
            ('/loader/cmd_vel', '/loader/cmd_vel'),
        ]
    )
    
    return [control_node]


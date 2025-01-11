# hexapod_bringup/launch/complete_robot.launch.py
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    robot_namespace = LaunchConfiguration('robot_namespace', default='hexapod')
    
    # Package paths
    bringup_pkg = FindPackageShare('hexapod_bringup')
    
    # Include hardware and control launch files
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_pkg, '/launch/hardware.launch.py']),
        launch_arguments={'robot_namespace': robot_namespace}.items()
    )
    
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_pkg, '/launch/control.launch.py']),
        launch_arguments={'robot_namespace': robot_namespace}.items()
    )
    
    # Define our perception nodes
    sensor_fusion_node = Node(
        package='hexapod_perception',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        namespace=robot_namespace,
        parameters=[{
            'map_update_rate': 10.0,
            'local_map_size': 10.0,
            'global_map_resolution': 0.1
        }]
    )
    
    object_detection_node = Node(
        package='hexapod_perception',
        executable='object_detection',
        name='object_detection',
        namespace=robot_namespace,
        parameters=[{
            'confidence_threshold': 0.7,
            'detection_frequency': 10.0
        }]
    )
    
    health_monitor_node = Node(
        package='hexapod_perception',
        executable='health_monitor',
        name='health_monitor',
        namespace=robot_namespace,
        parameters=[{
            'signal_threshold_warning': 0.7,
            'signal_threshold_critical': 0.5,
            'stuck_timeout': 45.0,
            'health_check_rate': 1.0
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'robot_namespace',
        default_value='hexapod',
        description='Namespace for the robot'
    ))
    
    # Add all launch elements in sequence
    ld.add_action(LogInfo(msg='Starting Hexapod Robot...'))
    
    # Launch hardware nodes first
    ld.add_action(hardware_launch)
    
    # Launch control nodes
    ld.add_action(control_launch)
    
    # Launch perception nodes
    ld.add_action(sensor_fusion_node)
    ld.add_action(object_detection_node)
    ld.add_action(health_monitor_node)
    
    return ld
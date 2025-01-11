#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Locomotion Controller Node (from our previous C++ implementation)
    locomotion_controller_node = Node(
        package='hexapod_control',
        executable='locomotion_controller',
        name='locomotion_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gait_type': 'tripod',
            'update_rate': 50.0,
            'leg_height': 0.1,
            'stride_length': 0.2
        }],
        remappings=[
            ('joint_commands', '/hexapod/joint_commands'),
            ('joint_states', '/hexapod/joint_states')
        ]
    )

    # State Machine Node (from our previous implementation)
    state_machine_node = Node(
        package='hexapod_control',
        executable='state_machine',
        name='state_machine',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autonomous_timeout': 45.0,  # seconds
            'signal_threshold_warning': 0.7,
            'signal_threshold_critical': 0.5
        }]
    )

    # Path Planner Node (from our previous A* implementation)
    path_planner_node = Node(
        package='hexapod_control',
        executable='path_planner',
        name='path_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'update_rate': 10.0,
            'grid_resolution': 0.05,  # meters
            'planning_horizon': 2.0   # meters
        }],
        remappings=[
            ('map', '/hexapod/local_map'),
            ('path', '/hexapod/planned_path')
        ]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the arguments
    ld.add_action(use_sim_time_arg)

    # Add the nodes
    ld.add_action(locomotion_controller_node)
    ld.add_action(state_machine_node)
    ld.add_action(path_planner_node)

    # Add a log info action
    ld.add_action(LogInfo(msg='Hexapod Control System Launched'))

    return ld
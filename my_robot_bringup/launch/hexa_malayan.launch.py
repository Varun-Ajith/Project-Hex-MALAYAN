from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
  
    MasterNode = Node(
        package='master_pkg',
        executable='master_node'
    )
    GaitControllerNode = Node(
        package='hexapod_control_pkg',
        executable='gait_controller'
    )
    ServoControllerNode = Node(
        package='hexapod_control_pkg',
        executable="servo_controller")
    
    PerceptionNode = Node(
        package='perception_pkg',
        executable='lidar_processing'
    )
    NavigationNode = Node(
        package='navigation_pkg',
        executable='path_planning'
    )
    ObstacleAvoidanceNode = Node(
        package="navigation_pkg",
        executable="obstacle_avoidance"
    )
    CameraServoNode = Node(
        package='camera_servo_pkg',  
        executable='camera_servo'
    )

    ld.add_action(MasterNode)
    ld.add_action(GaitControllerNode)
    ld.add_action(PerceptionNode)
    ld.add_action(NavigationNode)
    ld.add_action(CameraServoNode)
    ld.add_action(ServoControllerNode)
    ld.add_action(ObstacleAvoidanceNode)

    return ld

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
    PerceptionNode = Node(
        package='perception_pkg',
        executable='lidar_processing_node'
    )
    NavigationNode = Node(
        package='navigation_pkg',
        executable='pathplanner'
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

    return ld

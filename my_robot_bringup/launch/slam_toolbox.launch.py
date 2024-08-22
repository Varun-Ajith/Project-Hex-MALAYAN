from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', 
        name="slam_toolbox",
        parameters=[
            {"use_sim_time":False},
            {"resolution": 0.05}, 
            {"max_laser_range":30.0}, 
            {"minimum_timr_travel":1.0}
            ], 
            remappings=[("scan","/scan")])
    
    ld.add_action(slam)

    return ld


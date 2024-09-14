from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    Brain_node = Node(package = "brainsr", 
                      executable = "brain", 
                      name = "brain_node", 
                      parameters = [{"survivor_detection_topic" : "survivor_detection"}, 
                                    {"navigation_command_topic" : "navigation_command"},
                                    {"communication_topic" : "communication_topic"}] )
    
    perception_node = Node(package = "perceptionSR", 
                           executable = "perception", 
                           name = "perception_node", 
                           parameters = [{"survivor_detection_topic" : "survivor_detection"}] )
    
    Navigation_node = Node(package = "navigation",
                           executable = "navigation",
                           name = "navigation_node",
                           parameters = [{"navigation_command_topic" : "navigation_command"}])
    
    Communication_node = Node(package = "communication",
                              executable = "communication",
                              name = "communication_node",
                              parameters = [{"communication_topic" : "communication_topic"}])
    
    ld.add_action(Brain_node)
    ld.add_action(perception_node)
    ld.add_action(Navigation_node)
    ld.add_action(Communication_node)


    return ld
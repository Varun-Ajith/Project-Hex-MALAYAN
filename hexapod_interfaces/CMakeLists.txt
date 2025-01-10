cmake_minimum_required(VERSION 3.8)
project(hexapod_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/RobotState.msg"
  "msg/HealthStatus.msg"
  "msg/DetectionResult.msg"
  "srv/SetMode.srv"
  "srv/EmergencyStop.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_package()
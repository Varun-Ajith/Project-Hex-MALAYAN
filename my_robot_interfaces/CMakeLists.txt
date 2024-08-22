cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/ComputeRectangleArea.srv"
  "msg/Turtle.msg"
  "msg/TurtleArray.msg"
  "srv/CatchTurtle.srv"
  "srv/ResetPosition.srv"
  "msg/LegCommand.msg"
  "action/MoveSequence.action"
  "action/NavigateHuman.action"
)


ament_package()

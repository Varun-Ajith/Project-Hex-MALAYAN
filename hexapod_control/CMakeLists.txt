# CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(hexapod_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hexapod_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(locomotion_controller src/locomotion_controller.cpp)
add_executable(path_planner src/path_planner.cpp)
add_executable(state_machine src/state_machine.cpp)



ament_target_dependencies(locomotion_controller
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(path_planner
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(state_machine
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  locomotion_controller
  path_planner
  state_machine
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()

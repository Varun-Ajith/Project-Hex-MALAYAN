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

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk/include
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/output/Linux/Release
)

add_executable(servo_controller src/servo_controller.cpp)
add_executable(imu_interface src/imu_interface.cpp)
add_executable(gps_interface src/gps_interface.cpp)
add_executable(lidar_interface src/lidar_interface.cpp)
add_executable(camera_interface src/camera_interface.cpp)

target_link_libraries(lidar_interface
  rplidar_sdk
)

ament_target_dependencies(servo_controller
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(imu_interface
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(lidar_interface
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(lidar_interface
  rclcpp
  hexapod_interfaces
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(camera_interface
  rclcpp
  hexapod_interfaces
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

install(TARGETS
  servo_controller
  imu_interface
  gps_interface
  lidar_interface
  camera_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()

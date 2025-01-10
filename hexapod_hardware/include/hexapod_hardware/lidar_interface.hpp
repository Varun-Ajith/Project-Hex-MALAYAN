#ifndef LIDAR_INTERFACE_HPP
#define LIDAR_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rplidar.h>
#include <vector>
#include <string>

using namespace rp::standalone::rplidar;

class LidarInterface : public rclcpp::Node {
public:
    LidarInterface();
    ~LidarInterface();

private:
    // RPLidar driver instance
    RPlidarDriver* lidar_driver_;
    
    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    float scan_frequency_;
    
    // Methods
    bool initializeLidar();
    void publishScan();
    bool checkLidarHealth();
    void cleanup();
};

#endif // LIDAR_INTERFACE_HPP
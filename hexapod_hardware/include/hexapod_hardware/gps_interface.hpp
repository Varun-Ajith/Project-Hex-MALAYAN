#ifndef GPS_INTERFACE_HPP
#define GPS_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <termios.h>
#include <memory>

class GPSInterface : public rclcpp::Node {
public:
    GPSInterface();
    ~GPSInterface();

private:
    // Serial communication
    int serial_port_;
    static constexpr int BUFFER_SIZE = 256;
    char read_buffer_[BUFFER_SIZE];
    std::string port_name_;
    int baud_rate_;

    // ROS2 interface
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Methods
    bool initializeSerial();
    void closeSerial();
    bool configureGPS();
    void readGPSData();
    bool parseNMEA(const std::string& sentence);
    bool parseGGA(const std::string& sentence);
    bool checksum(const std::string& sentence);
    void publishGPSData(double lat, double lon, double alt, int fix_quality);
};

#endif // GPS_INTERFACE_HPP
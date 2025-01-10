#ifndef CAMERA_INTERFACE_HPP
#define CAMERA_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

class CameraInterface : public rclcpp::Node {
public:
    CameraInterface();
    ~CameraInterface();

private:
    void initializeCamera();
    void captureAndPublish();
    void setNightMode(bool enabled);

    // ROS2 publishers
    image_transport::Publisher image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Camera handling
    cv::VideoCapture cap_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    bool night_mode_;
    int capture_width_;
    int capture_height_;
    int frame_rate_;
};

#endif // CAMERA_INTERFACE_HPP
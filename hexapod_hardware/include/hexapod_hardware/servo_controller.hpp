#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <hexapod_interfaces/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

class ServoController : public rclcpp::Node {
public:
    ServoController();
    ~ServoController();

private:
    // PCA9685 communication
    int i2c_fd_ = -1;
    bool initializePCA9685();
    void writeRegister(uint8_t reg, uint8_t value);
    void setPWM(int channel, uint16_t on, uint16_t off);
    void setServoAngle(int channel, float angle);
    
    // ROS2 interface
    rclcpp::Subscription<hexapod_interfaces::msg::JointCommand>::SharedPtr joint_command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Joint state publishing
    void publishJointStates();
    void jointCommandCallback(const hexapod_interfaces::msg::JointCommand::SharedPtr msg);
    
    // Servo mapping and state tracking
    std::vector<int> servo_channels_;
    std::vector<float> current_positions_;
    std::vector<std::string> joint_names_;
};

#endif // SERVO_CONTROLLER_HPP
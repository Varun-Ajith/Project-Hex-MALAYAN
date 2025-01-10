#include "hexapod_hardware/servo_controller.hpp"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// PCA9685 registers
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define PCA9685_ADDRESS 0x40

// Servo calibration values
#define SERVO_MIN_PWM 150   // ~0.5ms
#define SERVO_MAX_PWM 600   // ~2.5ms
#define ANGLE_MIN 0
#define ANGLE_MAX 180

ServoController::ServoController() : Node("servo_controller") {
    // Initialize parameters with default values
    this->declare_parameter("i2c_device", "/dev/i2c-1");
    this->declare_parameter("update_rate", 50.0);  // Hz
    
    // Initialize joint names for all 18 servos
    joint_names_ = {
        "right_front_hip", "right_front_knee", "right_front_ankle",
        "right_middle_hip", "right_middle_knee", "right_middle_ankle",
        "right_back_hip", "right_back_knee", "right_back_ankle",
        "left_front_hip", "left_front_knee", "left_front_ankle",
        "left_middle_hip", "left_middle_knee", "left_middle_ankle",
        "left_back_hip", "left_back_knee", "left_back_ankle"
    };

    // Initialize servo channels and current positions
    servo_channels_ = std::vector<int>(18);
    current_positions_ = std::vector<float>(18, 90.0);  // Initialize all servos to 90 degrees
    
    for(int i = 0; i < 18; i++) {
        servo_channels_[i] = i;  // Channels 0-17 for 18 servos
    }

    // Initialize PCA9685
    if (!initializePCA9685()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PCA9685!");
        return;
    }

    // Setup ROS2 communications
    joint_command_sub_ = this->create_subscription<hexapod_interfaces::msg::JointCommand>(
        "joint_commands", 10, 
        std::bind(&ServoController::jointCommandCallback, this, std::placeholders::_1));
    
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    // Create timer for joint state publishing
    double update_rate = this->get_parameter("update_rate").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
        std::bind(&ServoController::publishJointStates, this));

    RCLCPP_INFO(this->get_logger(), "Servo Controller initialized successfully");
}

ServoController::~ServoController() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

bool ServoController::initializePCA9685() {
    std::string i2c_device = this->get_parameter("i2c_device").as_string();
    
    // Open I2C device
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device!");
        return false;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd_, I2C_SLAVE, PCA9685_ADDRESS) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address!");
        return false;
    }

    // Initialize PCA9685
    // Put to sleep
    writeRegister(PCA9685_MODE1, 0x10);
    
    // Set PWM frequency to 50Hz
    // Calculation: prescale = round(25MHz/(4096 * freq)) - 1
    writeRegister(PCA9685_PRESCALE, 121);  // 50Hz
    
    // Wake up and enable auto-increment
    writeRegister(PCA9685_MODE1, 0x20);
    
    // Wait for oscillator
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    return true;
}

void ServoController::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to PCA9685!");
    }
}

void ServoController::setPWM(int channel, uint16_t on, uint16_t off) {
    uint8_t buffer[5] = {
        static_cast<uint8_t>(LED0_ON_L + 4 * channel),
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    
    if (write(i2c_fd_, buffer, 5) != 5) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM!");
    }
}

void ServoController::setServoAngle(int channel, float angle) {
    // Clamp angle to valid range
    angle = std::max(std::min(angle, static_cast<float>(ANGLE_MAX)), static_cast<float>(ANGLE_MIN));
    
    // Convert angle to PWM value
    // Map angle (0-180) to PWM range (SERVO_MIN_PWM-SERVO_MAX_PWM)
    float pwm = SERVO_MIN_PWM + (angle / 180.0f) * (SERVO_MAX_PWM - SERVO_MIN_PWM);
    
    // Set PWM value
    setPWM(channel, 0, static_cast<uint16_t>(pwm));
}

void ServoController::jointCommandCallback(const hexapod_interfaces::msg::JointCommand::SharedPtr msg) {
    // Find the corresponding servo channel
    auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->joint_name);
    if (it != joint_names_.end()) {
        int index = std::distance(joint_names_.begin(), it);
        
        // Convert position from radians to degrees
        float angle = msg->position * (180.0f / M_PI);
        
        // Set servo angle and update current position
        setServoAngle(servo_channels_[index], angle);
        current_positions_[index] = msg->position;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Setting joint %s to %.2f degrees (%.2f radians)",
            msg->joint_name.c_str(), angle, msg->position);
    } else {
        RCLCPP_WARN(this->get_logger(), 
            "Received command for unknown joint: %s", 
            msg->joint_name.c_str());
    }
}

void ServoController::publishJointStates() {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->now();
    message.name = joint_names_;
    message.position = current_positions_;
    
    // Velocities and efforts are not measured, so we'll send empty vectors
    message.velocity.resize(joint_names_.size(), 0.0);
    message.effort.resize(joint_names_.size(), 0.0);
    
    joint_state_pub_->publish(message);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
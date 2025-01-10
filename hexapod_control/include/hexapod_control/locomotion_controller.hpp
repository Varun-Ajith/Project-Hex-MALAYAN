#ifndef LOCOMOTION_CONTROLLER_HPP
#define LOCOMOTION_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "hexapod_interfaces/msg/joint_command.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <array>

class LocomotionController : public rclcpp::Node {
public:
    LocomotionController();

private:
    // Constants for hexapod configuration
    static constexpr int NUM_LEGS = 6;
    static constexpr int JOINTS_PER_LEG = 3;
    static constexpr double UPDATE_RATE = 50.0; // Hz
    
    // Leg groups for tripod gait
    const std::vector<int> GROUP_1 = {0, 2, 4}; // Right front, right back, left middle
    const std::vector<int> GROUP_2 = {1, 3, 5}; // Left front, left back, right middle

    // PID Controller parameters
    struct PIDController {
        double kp, ki, kd;
        double integral, prev_error;
        double output_min, output_max;
    };

    // Joint positions and states
    struct LegState {
        double hip, knee, ankle;
    };
    std::array<LegState, NUM_LEGS> current_leg_states_;
    std::array<LegState, NUM_LEGS> target_leg_states_;

    // ROS2 interface
    rclcpp::Publisher<hexapod_interfaces::msg::JointCommand>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // PID Controllers
    PIDController roll_pid_{1.0, 0.1, 0.01, 0.0, 0.0, -0.5, 0.5};
    PIDController pitch_pid_{1.0, 0.1, 0.01, 0.0, 0.0, -0.5, 0.5};

    // Member functions
    void initializeLegs();
    void controlLoop();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // Gait generation
    void generateTripodGait(double phase);
    std::array<LegState, NUM_LEGS> calculateLegPositions(double x, double y, double z);
    
    // Stability control
    double updatePID(PIDController& pid, double error, double dt);
    void stabilityControl(const sensor_msgs::msg::Imu::SharedPtr imu_data);
    
    // Kinematics
    LegState inverseKinematics(double x, double y, double z);
    void publishJointCommands();

    // State variables
    double gait_phase_ = 0.0;
    double walking_height_ = 0.15; // meters
    double step_height_ = 0.05;    // meters
    double step_length_ = 0.1;     // meters
    bool is_moving_ = false;
    
    geometry_msgs::msg::Twist current_velocity_;
    rclcpp::Time last_update_time_;
};

#endif // LOCOMOTION_CONTROLLER_HPP
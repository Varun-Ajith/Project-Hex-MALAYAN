#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hexapod_interfaces/msg/robot_state.hpp>
#include <hexapod_interfaces/msg/health_status.hpp>
#include <hexapod_interfaces/msg/detection_result.hpp>
#include <hexapod_interfaces/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class StateMachine : public rclcpp::Node {
public:
    StateMachine();

private:
    enum class RobotState {
        IDLE,
        AUTONOMOUS_SEARCH,
        MANUAL_CONTROL,
        RETURN_HOME,
        EMERGENCY
    };

    // Current state
    RobotState current_state_;
    geometry_msgs::msg::Pose last_safe_position_;
    std::chrono::steady_clock::time_point last_movement_time_;
    float signal_strength_;
    bool is_stuck_;

    // Publishers
    rclcpp::Publisher<hexapod_interfaces::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    // Subscribers
    rclcpp::Subscription<hexapod_interfaces::msg::HealthStatus>::SharedPtr health_sub_;
    rclcpp::Subscription<hexapod_interfaces::msg::DetectionResult>::SharedPtr detection_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Service
    rclcpp::Service<hexapod_interfaces::srv::SetMode>::SharedPtr mode_service_;

    // Timer
    rclcpp::TimerBase::SharedPtr state_timer_;

    // Callback functions
    void healthCallback(const hexapod_interfaces::msg::HealthStatus::SharedPtr msg);
    void detectionCallback(const hexapod_interfaces::msg::DetectionResult::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void setModeCallback(
        const std::shared_ptr<hexapod_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<hexapod_interfaces::srv::SetMode::Response> response);
    void stateUpdate();

    // State handling functions
    void handleAutonomousSearch();
    void handleManualControl();
    void handleReturnHome();
    void handleEmergency();
    void transitionState(RobotState new_state);
    void publishRobotState();
    void checkStuckCondition(const geometry_msgs::msg::Pose& current_pose);
};

#endif // STATE_MACHINE_HPP
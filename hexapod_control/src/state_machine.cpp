#include "hexapod_control/state_machine.hpp"
#include <chrono>

using namespace std::chrono_literals;

StateMachine::StateMachine() : Node("state_machine"), 
    current_state_(RobotState::IDLE),
    signal_strength_(100.0),
    is_stuck_(false) {
    
    // Initialize publishers
    robot_state_pub_ = this->create_publisher<hexapod_interfaces::msg::RobotState>(
        "robot_state", 10);
    command_pub_ = this->create_publisher<std_msgs::msg::String>(
        "robot_command", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10);

    // Initialize subscribers
    health_sub_ = this->create_subscription<hexapod_interfaces::msg::HealthStatus>(
        "health_status", 10, 
        std::bind(&StateMachine::healthCallback, this, std::placeholders::_1));
    
    detection_sub_ = this->create_subscription<hexapod_interfaces::msg::DetectionResult>(
        "detection_results", 10,
        std::bind(&StateMachine::detectionCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&StateMachine::odomCallback, this, std::placeholders::_1));

    // Initialize service
    mode_service_ = this->create_service<hexapod_interfaces::srv::SetMode>(
        "set_mode",
        std::bind(&StateMachine::setModeCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Initialize timer for state updates (20Hz)
    state_timer_ = this->create_wall_timer(
        50ms, std::bind(&StateMachine::stateUpdate, this));

    RCLCPP_INFO(this->get_logger(), "State Machine initialized in IDLE state");
}

void StateMachine::healthCallback(const hexapod_interfaces::msg::HealthStatus::SharedPtr msg) {
    if (msg->component_name == "system") {
        signal_strength_ = msg->signal_strength;
        
        if (signal_strength_ < 50.0) {
            RCLCPP_WARN(this->get_logger(), "Signal strength critical: %.1f%%", signal_strength_);
            transitionState(RobotState::RETURN_HOME);
        } else if (signal_strength_ < 70.0) {
            RCLCPP_WARN(this->get_logger(), "Signal strength low: %.1f%%", signal_strength_);
        }
    }

    if (msg->status == "ERROR") {
        RCLCPP_ERROR(this->get_logger(), "Component %s error: %s", 
            msg->component_name.c_str(), msg->message.c_str());
        transitionState(RobotState::EMERGENCY);
    }
}

void StateMachine::detectionCallback(const hexapod_interfaces::msg::DetectionResult::SharedPtr msg) {
    if (current_state_ == RobotState::AUTONOMOUS_SEARCH) {
        auto pose_stamped = geometry_msgs::msg::PoseStamped();
        pose_stamped.header = msg->header;
        pose_stamped.pose.position = msg->location;
        
        // Publish detection location as goal
        goal_pub_->publish(pose_stamped);
        
        // Notify base station
        auto command = std_msgs::msg::String();
        command.data = "Target detected at location: " + 
                      std::to_string(msg->location.x) + ", " +
                      std::to_string(msg->location.y);
        command_pub_->publish(command);
    }
}

void StateMachine::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    checkStuckCondition(msg->pose.pose);
    last_safe_position_ = msg->pose.pose;
}

void StateMachine::setModeCallback(
    const std::shared_ptr<hexapod_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<hexapod_interfaces::srv::SetMode::Response> response) {
    
    if (request->mode == "AUTONOMOUS") {
        transitionState(RobotState::AUTONOMOUS_SEARCH);
        response->success = true;
        response->message = "Switched to autonomous mode";
    } else if (request->mode == "MANUAL") {
        transitionState(RobotState::MANUAL_CONTROL);
        response->success = true;
        response->message = "Switched to manual control";
    } else if (request->mode == "RETURN_HOME") {
        transitionState(RobotState::RETURN_HOME);
        response->success = true;
        response->message = "Initiating return home sequence";
    } else {
        response->success = false;
        response->message = "Invalid mode requested";
    }
}

void StateMachine::stateUpdate() {
    switch (current_state_) {
        case RobotState::AUTONOMOUS_SEARCH:
            handleAutonomousSearch();
            break;
        case RobotState::MANUAL_CONTROL:
            handleManualControl();
            break;
        case RobotState::RETURN_HOME:
            handleReturnHome();
            break;
        case RobotState::EMERGENCY:
            handleEmergency();
            break;
        default:
            break;
    }
    publishRobotState();
}

void StateMachine::handleAutonomousSearch() {
    if (is_stuck_) {
        RCLCPP_WARN(this->get_logger(), "Robot appears to be stuck. Switching to manual control");
        transitionState(RobotState::MANUAL_CONTROL);
    }
}

void StateMachine::handleManualControl() {
    // Monitor for conditions to resume autonomous operation
}

void StateMachine::handleReturnHome() {
    auto goal = geometry_msgs::msg::PoseStamped();
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose = last_safe_position_;
    goal_pub_->publish(goal);
}

void StateMachine::handleEmergency() {
    auto command = std_msgs::msg::String();
    command.data = "STOP";
    command_pub_->publish(command);
}

void StateMachine::transitionState(RobotState new_state) {
    if (current_state_ != new_state) {
        RCLCPP_INFO(this->get_logger(), "Transitioning from state %d to %d",
                    static_cast<int>(current_state_), static_cast<int>(new_state));
        current_state_ = new_state;
    }
}

void StateMachine::checkStuckCondition(const geometry_msgs::msg::Pose& current_pose) {
    static geometry_msgs::msg::Pose last_pose;
    static auto last_check_time = std::chrono::steady_clock::now();
    
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - last_check_time).count();
    
    if (elapsed >= 45) {
        double distance = std::sqrt(
            std::pow(current_pose.position.x - last_pose.position.x, 2) +
            std::pow(current_pose.position.y - last_pose.position.y, 2));
        
        if (distance < 0.1) { // 10cm threshold
            is_stuck_ = true;
            RCLCPP_WARN(this->get_logger(), "Robot hasn't moved significantly in 45 seconds");
        } else {
            is_stuck_ = false;
        }
        
        last_pose = current_pose;
        last_check_time = current_time;
    }
}

void StateMachine::publishRobotState() {
    auto msg = hexapod_interfaces::msg::RobotState();
    msg.header.stamp = this->now();
    msg.pose = last_safe_position_;
    msg.current_mode = [this]() {
        switch (current_state_) {
            case RobotState::AUTONOMOUS_SEARCH: return "AUTONOMOUS";
            case RobotState::MANUAL_CONTROL: return "MANUAL";
            case RobotState::RETURN_HOME: return "RETURNING";
            case RobotState::EMERGENCY: return "EMERGENCY";
            default: return "IDLE";
        }
    }();
    msg.is_stable = !is_stuck_;
    
    robot_state_pub_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateMachine>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
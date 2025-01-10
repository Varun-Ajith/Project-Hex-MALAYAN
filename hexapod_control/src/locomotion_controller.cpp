#include "hexapod_control/locomotion_controller.hpp"
#include <cmath>

LocomotionController::LocomotionController() 
    : Node("locomotion_controller"), last_update_time_(this->now()) {
    
    // Initialize publishers
    joint_command_pub_ = this->create_publisher<hexapod_interfaces::msg::JointCommand>(
        "joint_commands", 10);

    // Initialize subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, 
        std::bind(&LocomotionController::imuCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, 
        std::bind(&LocomotionController::cmdVelCallback, this, std::placeholders::_1));

    // Initialize control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / UPDATE_RATE)),
        std::bind(&LocomotionController::controlLoop, this));

    initializeLegs();
    RCLCPP_INFO(this->get_logger(), "Locomotion Controller initialized");
}

void LocomotionController::initializeLegs() {
    // Set initial leg positions for standing
    for (int i = 0; i < NUM_LEGS; i++) {
        current_leg_states_[i] = inverseKinematics(0.0, 0.0, walking_height_);
        target_leg_states_[i] = current_leg_states_[i];
    }
    publishJointCommands();
}

void LocomotionController::controlLoop() {
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    if (is_moving_) {
        gait_phase_ += dt * 2.0 * M_PI * 0.5; // 0.5 Hz gait frequency
        if (gait_phase_ > 2.0 * M_PI) {
            gait_phase_ -= 2.0 * M_PI;
        }
        generateTripodGait(gait_phase_);
    }

    publishJointCommands();
}

void LocomotionController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_velocity_ = *msg;
    is_moving_ = std::abs(msg->linear.x) > 0.01 || 
                 std::abs(msg->linear.y) > 0.01 || 
                 std::abs(msg->angular.z) > 0.01;
}

void LocomotionController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    stabilityControl(msg);
}

void LocomotionController::generateTripodGait(double phase) {
    // Calculate base leg positions from current velocity
    double x_offset = current_velocity_.linear.x * step_length_;
    double y_offset = current_velocity_.linear.y * step_length_;
    double z_offset = walking_height_;

    // Generate leg positions for both tripod groups
    for (int leg : GROUP_1) {
        double leg_phase = phase;
        double x = x_offset * std::cos(leg_phase);
        double y = y_offset * std::cos(leg_phase);
        double z = z_offset - step_height_ * std::max(0.0, std::sin(leg_phase));
        target_leg_states_[leg] = inverseKinematics(x, y, z);
    }

    for (int leg : GROUP_2) {
        double leg_phase = phase + M_PI; // 180 degrees out of phase
        double x = x_offset * std::cos(leg_phase);
        double y = y_offset * std::cos(leg_phase);
        double z = z_offset - step_height_ * std::max(0.0, std::sin(leg_phase));
        target_leg_states_[leg] = inverseKinematics(x, y, z);
    }
}

double LocomotionController::updatePID(PIDController& pid, double error, double dt) {
    pid.integral += error * dt;
    double derivative = (error - pid.prev_error) / dt;
    pid.prev_error = error;

    double output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
    return std::clamp(output, pid.output_min, pid.output_max);
}

void LocomotionController::stabilityControl(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    // Extract roll and pitch from quaternion
    double roll = std::atan2(2.0 * (imu_data->orientation.w * imu_data->orientation.x + 
                                   imu_data->orientation.y * imu_data->orientation.z),
                            1.0 - 2.0 * (imu_data->orientation.x * imu_data->orientation.x + 
                                        imu_data->orientation.y * imu_data->orientation.y));
    
    double pitch = std::asin(2.0 * (imu_data->orientation.w * imu_data->orientation.y - 
                                   imu_data->orientation.z * imu_data->orientation.x));

    // Update PID controllers
    double dt = 1.0 / UPDATE_RATE;
    double roll_correction = updatePID(roll_pid_, roll, dt);
    double pitch_correction = updatePID(pitch_pid_, pitch, dt);

    // Apply corrections to leg heights
    for (int i = 0; i < NUM_LEGS; i++) {
        // Adjust z-component based on leg position and corrections
        double x_pos = (i % 2 == 0) ? 1.0 : -1.0;  // Alternating sides
        double y_pos = (i < 2) ? 1.0 : (i < 4 ? 0.0 : -1.0);  // Front/middle/back

        target_leg_states_[i].ankle += roll_correction * x_pos + pitch_correction * y_pos;
    }
}

LocomotionController::LegState LocomotionController::inverseKinematics(double x, double y, double z) {
    // Simple inverse kinematics for a 3-DOF leg
    // This is a basic implementation - you might need to adjust based on your specific leg geometry
    LegState result;
    
    // Calculate hip angle
    result.hip = std::atan2(y, x);
    
    // Calculate leg length in x-z plane
    double L = std::sqrt(x*x + z*z);
    
    // Calculate knee and ankle angles using cosine law
    double leg_length = 0.1; // Length of tibia and femur segments
    double alpha = std::acos((L*L)/(2*leg_length*L));
    double beta = std::atan2(z, x);
    
    result.knee = beta - alpha;
    result.ankle = -beta - alpha;

    return result;
}

void LocomotionController::publishJointCommands() {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        // Publish hip joint
        auto hip_cmd = hexapod_interfaces::msg::JointCommand();
        hip_cmd.joint_name = "leg_" + std::to_string(leg) + "_hip";
        hip_cmd.position = target_leg_states_[leg].hip;
        joint_command_pub_->publish(hip_cmd);

        // Publish knee joint
        auto knee_cmd = hexapod_interfaces::msg::JointCommand();
        knee_cmd.joint_name = "leg_" + std::to_string(leg) + "_knee";
        knee_cmd.position = target_leg_states_[leg].knee;
        joint_command_pub_->publish(knee_cmd);

        // Publish ankle joint
        auto ankle_cmd = hexapod_interfaces::msg::JointCommand();
        ankle_cmd.joint_name = "leg_" + std::to_string(leg) + "_ankle";
        ankle_cmd.position = target_leg_states_[leg].ankle;
        joint_command_pub_->publish(ankle_cmd);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocomotionController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
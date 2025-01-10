#include <rclcpp/rclcpp.hpp>
#include <hexapod_interfaces/msg/health_status.hpp>
#include <hexapod_interfaces/msg/robot_state.hpp>
#include <hexapod_interfaces/srv/set_mode.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <map>

using namespace std::chrono_literals;

class HealthMonitor : public rclcpp::Node {
public:
    HealthMonitor() : Node("health_monitor") {
        // Initialize parameters
        declare_parameters();
        
        // Create publishers
        health_pub_ = this->create_publisher<hexapod_interfaces::msg::HealthStatus>(
            "system_health", 10);
        
        // Create subscribers
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10, 
            std::bind(&HealthMonitor::gpsCallback, this, std::placeholders::_1));
            
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, 
            std::bind(&HealthMonitor::lidarCallback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot_pose", 10, 
            std::bind(&HealthMonitor::poseCallback, this, std::placeholders::_1));
            
        signal_strength_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "signal_strength", 10, 
            std::bind(&HealthMonitor::signalStrengthCallback, this, std::placeholders::_1));

        // Create client for mode changes
        mode_client_ = this->create_client<hexapod_interfaces::srv::SetMode>("set_mode");

        // Initialize component timestamps
        last_gps_time_ = this->now();
        last_lidar_time_ = this->now();
        last_pose_time_ = this->now();
        
        // Create timer for health check
        health_timer_ = this->create_wall_timer(
            1s, std::bind(&HealthMonitor::healthCheckCallback, this));

        RCLCPP_INFO(this->get_logger(), "Health Monitor initialized");
    }

private:
    void declare_parameters() {
        this->declare_parameter("signal_strength_threshold_warning", 70.0);
        this->declare_parameter("signal_strength_threshold_critical", 50.0);
        this->declare_parameter("component_timeout", 5.0);  // seconds
        this->declare_parameter("position_stuck_timeout", 45.0);  // seconds
        this->declare_parameter("gps_health_topic", "gps/health");
        this->declare_parameter("lidar_health_topic", "lidar/health");
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        last_gps_time_ = this->now();
        last_gps_fix_ = *msg;
        checkGPSHealth();
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_lidar_time_ = this->now();
        checkLidarHealth();
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        auto current_time = this->now();
        
        if (last_pose_) {
            double distance = calculateDistance(*last_pose_, *msg);
            if (distance < 0.1) {  // If robot hasn't moved more than 10cm
                if ((current_time - last_movement_time_).seconds() > 
                    this->get_parameter("position_stuck_timeout").as_double()) {
                    RCLCPP_WARN(this->get_logger(), "Robot appears to be stuck!");
                    initiateFailsafe("STUCK");
                }
            } else {
                last_movement_time_ = current_time;
            }
        }
        
        last_pose_ = msg;
        last_pose_time_ = current_time;
    }

    void signalStrengthCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        float warning_threshold = 
            this->get_parameter("signal_strength_threshold_warning").as_double();
        float critical_threshold = 
            this->get_parameter("signal_strength_threshold_critical").as_double();

        if (msg->data < critical_threshold) {
            RCLCPP_ERROR(this->get_logger(), 
                "Signal strength critical: %.1f%%", msg->data);
            initiateFailsafe("LOW_SIGNAL");
        } else if (msg->data < warning_threshold) {
            RCLCPP_WARN(this->get_logger(), 
                "Signal strength warning: %.1f%%", msg->data);
            publishHealthStatus("SIGNAL", "WARNING", 
                "Signal strength below warning threshold");
        }
    }

    void healthCheckCallback() {
        auto current_time = this->now();
        double timeout = this->get_parameter("component_timeout").as_double();

        // Check component timeouts
        if ((current_time - last_gps_time_).seconds() > timeout) {
            RCLCPP_ERROR(this->get_logger(), "GPS timeout detected!");
            initiateFailsafe("GPS_TIMEOUT");
        }

        if ((current_time - last_lidar_time_).seconds() > timeout) {
            RCLCPP_ERROR(this->get_logger(), "LiDAR timeout detected!");
            initiateFailsafe("LIDAR_TIMEOUT");
        }

        publishSystemHealth();
    }

    void initiateFailsafe(const std::string& reason) {
        RCLCPP_INFO(this->get_logger(), "Initiating failsafe: %s", reason.c_str());
        
        auto request = std::make_shared<hexapod_interfaces::srv::SetMode::Request>();
        request->mode = "MANUAL";  // Switch to manual mode
        
        if (!mode_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Set mode service not available!");
            return;
        }
        
        auto result_future = mode_client_->async_send_request(request);
        
        // Publish emergency health status
        publishHealthStatus("SYSTEM", "ERROR", 
            "Failsafe initiated: " + reason);
    }

    void publishHealthStatus(const std::string& component, 
                           const std::string& status, 
                           const std::string& message) {
        auto health_msg = hexapod_interfaces::msg::HealthStatus();
        health_msg.header.stamp = this->now();
        health_msg.component_name = component;
        health_msg.status = status;
        health_msg.message = message;
        health_pub_->publish(health_msg);
    }

    // Previous code remains the same until publishSystemHealth() function

    void publishSystemHealth() {
        auto health_msg = hexapod_interfaces::msg::HealthStatus();
        health_msg.header.stamp = this->now();
        health_msg.component_name = "SYSTEM";
        health_msg.status = "OK";  // Default to OK
        std::string message = "All systems operational";
        
        // Check all component statuses and aggregate
        bool has_warning = false;
        bool has_error = false;
        std::vector<std::string> warnings;
        std::vector<std::string> errors;
        
        // Check GPS
        if ((this->now() - last_gps_time_).seconds() > 
            this->get_parameter("component_timeout").as_double()) {
            has_error = true;
            errors.push_back("GPS timeout");
        } else if (last_gps_fix_.status.status == 
                  sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
            has_warning = true;
            warnings.push_back("No GPS fix");
        }
        
        // Check LiDAR
        if ((this->now() - last_lidar_time_).seconds() > 
            this->get_parameter("component_timeout").as_double()) {
            has_error = true;
            errors.push_back("LiDAR timeout");
        } else if (last_lidar_status_ == "WARNING") {
            has_warning = true;
            warnings.push_back("LiDAR degraded");
        }
        
        // Check Position
        if (last_pose_) {
            if ((this->now() - last_movement_time_).seconds() > 
                this->get_parameter("position_stuck_timeout").as_double()) {
                has_warning = true;
                warnings.push_back("Robot potentially stuck");
            }
        }
        
        // Aggregate status and create message
        if (has_error) {
            health_msg.status = "ERROR";
            message = "Errors detected: " + 
                     std::accumulate(errors.begin(), errors.end(), std::string(),
                                   [](const std::string& a, const std::string& b) {
                                       return a + (a.empty() ? "" : ", ") + b;
                                   });
        } else if (has_warning) {
            health_msg.status = "WARNING";
            message = "Warnings detected: " + 
                     std::accumulate(warnings.begin(), warnings.end(), std::string(),
                                   [](const std::string& a, const std::string& b) {
                                       return a + (a.empty() ? "" : ", ") + b;
                                   });
        }
        
        health_msg.message = message;
        health_pub_->publish(health_msg);
    }

    void checkLidarHealth() {
        if (!last_lidar_scan_) {
            return;
        }

        int valid_readings = 0;
        int total_readings = last_lidar_scan_->ranges.size();
        float min_intensity = std::numeric_limits<float>::max();
        float max_intensity = -std::numeric_limits<float>::max();
        
        // Check range readings and intensities
        for (size_t i = 0; i < total_readings; ++i) {
            if (last_lidar_scan_->ranges[i] >= last_lidar_scan_->range_min &&
                last_lidar_scan_->ranges[i] <= last_lidar_scan_->range_max) {
                valid_readings++;
            }
            
            if (!last_lidar_scan_->intensities.empty()) {
                min_intensity = std::min(min_intensity, 
                                       last_lidar_scan_->intensities[i]);
                max_intensity = std::max(max_intensity, 
                                       last_lidar_scan_->intensities[i]);
            }
        }
        
        float valid_percentage = 
            (static_cast<float>(valid_readings) / total_readings) * 100.0;
        
        // Update LiDAR status based on health checks
        if (valid_percentage < 50.0) {
            last_lidar_status_ = "ERROR";
            publishHealthStatus("LIDAR", "ERROR", 
                              "Less than 50% valid readings");
        } else if (valid_percentage < 80.0) {
            last_lidar_status_ = "WARNING";
            publishHealthStatus("LIDAR", "WARNING", 
                              "Degraded performance: " + 
                              std::to_string(valid_percentage) + 
                              "% valid readings");
        } else {
            last_lidar_status_ = "OK";
            publishHealthStatus("LIDAR", "OK", 
                              "Operating normally");
        }
        
        // Log detailed diagnostics
        RCLCPP_DEBUG(this->get_logger(),
            "LiDAR Diagnostics:\n"
            "Valid Readings: %.1f%%\n"
            "Total Readings: %d\n"
            "Min Intensity: %.2f\n"
            "Max Intensity: %.2f",
            valid_percentage, total_readings, 
            min_intensity, max_intensity);
    }

    // Add these to the private member variables section
    sensor_msgs::msg::LaserScan::SharedPtr last_lidar_scan_;
    std::string last_lidar_status_{"OK"};

    // Update lidarCallback to store the scan
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_lidar_time_ = this->now();
        last_lidar_scan_ = msg;
        checkLidarHealth();
    }

    double calculateDistance(const geometry_msgs::msg::Pose& pose1, 
                           const geometry_msgs::msg::Pose& pose2) {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        double dz = pose1.position.z - pose2.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // Publishers
    rclcpp::Publisher<hexapod_interfaces::msg::HealthStatus>::SharedPtr health_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr signal_strength_sub_;

    // Service clients
    rclcpp::Client<hexapod_interfaces::srv::SetMode>::SharedPtr mode_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr health_timer_;

    // State tracking
    rclcpp::Time last_gps_time_;
    rclcpp::Time last_lidar_time_;
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_movement_time_;
    sensor_msgs::msg::NavSatFix last_gps_fix_;
    geometry_msgs::msg::Pose::SharedPtr last_pose_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HealthMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "hexapod_hardware/lidar_interface.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

LidarInterface::LidarInterface() : Node("lidar_interface") {
    // Declare parameters with default values
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("frame_id", "laser_frame");
    this->declare_parameter("scan_frequency", 10.0f);
    
    // Get parameters
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();
    scan_frequency_ = this->get_parameter("scan_frequency").as_double();
    
    // Create publisher
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    
    // Initialize LiDAR
    if (!initializeLidar()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize LiDAR");
        return;
    }
    
    // Create timer for scanning
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / scan_frequency_)),
        std::bind(&LidarInterface::publishScan, this));
    
    RCLCPP_INFO(this->get_logger(), "LiDAR Interface initialized successfully");
}

LidarInterface::~LidarInterface() {
    cleanup();
}

bool LidarInterface::initializeLidar() {
    // Create driver instance
    lidar_driver_ = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!lidar_driver_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create driver");
        return false;
    }
    
    // Connect to LiDAR
    u_result result = lidar_driver_->connect(port_name_.c_str(), baud_rate_);
    if (IS_FAIL(result)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to LiDAR on port %s", port_name_.c_str());
        return false;
    }
    
    // Check health
    if (!checkLidarHealth()) {
        return false;
    }
    
    // Start motor
    lidar_driver_->startMotor();
    
    // Start scan
    result = lidar_driver_->startScan(false, true);
    if (IS_FAIL(result)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start scan");
        return false;
    }
    
    return true;
}

bool LidarInterface::checkLidarHealth() {
    rplidar_response_device_health_t health_info;
    u_result result = lidar_driver_->getHealth(health_info);
    
    if (IS_FAIL(result)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get LiDAR health status");
        return false;
    }
    
    if (health_info.status == RPLIDAR_STATUS_ERROR) {
        RCLCPP_ERROR(this->get_logger(), "LiDAR in error state");
        return false;
    }
    
    return true;
}

void LidarInterface::publishScan() {
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t node_count = sizeof(nodes) / sizeof(nodes[0]);
    
    u_result result = lidar_driver_->grabScanDataHq(nodes, node_count);
    
    if (IS_FAIL(result)) {
        RCLCPP_WARN(this->get_logger(), "Failed to get scan data");
        return;
    }
    
    result = lidar_driver_->ascendScanData(nodes, node_count);
    if (IS_FAIL(result)) {
        RCLCPP_WARN(this->get_logger(), "Failed to process scan data");
        return;
    }
    
    // Prepare LaserScan message
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = frame_id_;
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2.0 * M_PI;
    scan_msg.angle_increment = (2.0 * M_PI) / node_count;
    scan_msg.time_increment = 1.0 / scan_frequency_ / node_count;
    scan_msg.scan_time = 1.0 / scan_frequency_;
    scan_msg.range_min = 0.15;  // RPLiDAR A1 specifications
    scan_msg.range_max = 12.0;  // RPLiDAR A1 specifications
    
    // Fill in measurement data
    scan_msg.ranges.resize(node_count);
    scan_msg.intensities.resize(node_count);
    
    for (size_t i = 0; i < node_count; i++) {
        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float distance = nodes[i].dist_mm_q2 / 4000.f;
        int index = (angle / 360.0 * node_count);
        
        if (index >= 0 && index < static_cast<int>(node_count)) {
            if (distance > scan_msg.range_min && distance < scan_msg.range_max) {
                scan_msg.ranges[index] = distance;
                scan_msg.intensities[index] = nodes[i].quality;
            } else {
                scan_msg.ranges[index] = std::numeric_limits<float>::infinity();
                scan_msg.intensities[index] = 0;
            }
        }
    }
    
    scan_pub_->publish(scan_msg);
}

void LidarInterface::cleanup() {
    if (lidar_driver_) {
        lidar_driver_->stop();
        lidar_driver_->stopMotor();
        RPlidarDriver::DisposeDriver(lidar_driver_);
        lidar_driver_ = nullptr;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.hpp>
#include <pcl/segmentation/extract_clusters.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <mutex>
#include <cmath>

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node") {
        // Initialize parameters with realistic values
        this->declare_parameter("map_resolution", 0.05);     // 5cm per pixel
        this->declare_parameter("map_width", 400);           // 20m x 20m map
        this->declare_parameter("map_height", 400);
        this->declare_parameter("local_map_size", 10.0);     // 10m local map
        this->declare_parameter("min_height_thresh", 0.1);   // 10cm min height
        this->declare_parameter("max_height_thresh", 2.0);   // 2m max height
        this->declare_parameter("voxel_size", 0.05);        // 5cm voxel grid
        this->declare_parameter("obstacle_threshold", 0.3);  // 30cm obstacle height

        // Publishers
        local_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map", 10);
        robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("robot_pose", 10);
        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);

        // Subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "scan", 10, std::bind(&SensorFusionNode::lidarCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1));
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10, std::bind(&SensorFusionNode::gpsCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize state
        robot_position_ = Eigen::Vector3d::Zero();
        robot_orientation_ = Eigen::Quaterniond::Identity();
        initializeLocalMap();

        // Create update timer (10Hz)
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorFusionNode::updateAndPublish, this));

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node initialized");
    }

private:
    // ... (previous publisher, subscriber, and state declarations remain the same)

    void initializeLocalMap() {
        local_map_.header.frame_id = "map";
        local_map_.info.resolution = this->get_parameter("map_resolution").as_double();
        local_map_.info.width = this->get_parameter("map_width").as_int();
        local_map_.info.height = this->get_parameter("map_height").as_int();
        local_map_.info.origin.position.x = -local_map_.info.resolution * local_map_.info.width / 2;
        local_map_.info.origin.position.y = -local_map_.info.resolution * local_map_.info.height / 2;
        local_map_.data.resize(local_map_.info.width * local_map_.info.height, -1);
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Apply voxel grid filter for downsampling
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(
            this->get_parameter("voxel_size").as_double(),
            this->get_parameter("voxel_size").as_double(),
            this->get_parameter("voxel_size").as_double()
        );
        voxel_filter.filter(*cloud_filtered);

        // Remove statistical outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setMeanK(50);    // Consider 50 neighbors
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered);

        // Update occupancy grid
        updateLocalMapFromPointCloud(cloud_filtered);

        // Publish filtered cloud for visualization
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered, filtered_msg);
        filtered_msg.header = msg->header;
        filtered_cloud_pub_->publish(filtered_msg);
    }

    void updateLocalMapFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        double min_height = this->get_parameter("min_height_thresh").as_double();
        double max_height = this->get_parameter("max_height_thresh").as_double();
        double obstacle_threshold = this->get_parameter("obstacle_threshold").as_double();
        double resolution = local_map_.info.resolution;

        // Clear the local area around the robot
        int center_x = local_map_.info.width / 2;
        int center_y = local_map_.info.height / 2;
        int local_size = static_cast<int>(this->get_parameter("local_map_size").as_double() / resolution);

        // Create height map for obstacle detection
        std::vector<std::vector<float>> height_map(local_map_.info.width, 
            std::vector<float>(local_map_.info.height, std::numeric_limits<float>::lowest()));

        // Process each point in the filtered cloud
        for (const auto& point : cloud->points) {
            // Skip points outside height range
            if (point.z < min_height || point.z > max_height) {
                continue;
            }

            // Convert point to map coordinates
            int map_x = static_cast<int>((point.x + local_map_.info.width * resolution / 2) / resolution);
            int map_y = static_cast<int>((point.y + local_map_.info.height * resolution / 2) / resolution);

            // Check if point is within map bounds
            if (map_x >= 0 && map_x < static_cast<int>(local_map_.info.width) &&
                map_y >= 0 && map_y < static_cast<int>(local_map_.info.height)) {
                // Update height map with maximum height at this cell
                height_map[map_x][map_y] = std::max(height_map[map_x][map_y], point.z);
            }
        }

        // Convert height map to occupancy grid
        for (size_t x = 0; x < local_map_.info.width; ++x) {
            for (size_t y = 0; y < local_map_.info.height; ++y) {
                size_t index = y * local_map_.info.width + x;
                
                if (height_map[x][y] > std::numeric_limits<float>::lowest()) {
                    // Calculate local height variation
                    float max_local_height = height_map[x][y];
                    float min_local_height = height_map[x][y];
                    
                    // Check neighboring cells
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < static_cast<int>(local_map_.info.width) &&
                                ny >= 0 && ny < static_cast<int>(local_map_.info.height)) {
                                if (height_map[nx][ny] > std::numeric_limits<float>::lowest()) {
                                    max_local_height = std::max(max_local_height, height_map[nx][ny]);
                                    min_local_height = std::min(min_local_height, height_map[nx][ny]);
                                }
                            }
                        }
                    }

                    // Mark as obstacle if height difference exceeds threshold
                    if (max_local_height - min_local_height > obstacle_threshold) {
                        local_map_.data[index] = 100;  // Occupied
                    } else {
                        local_map_.data[index] = 0;    // Free
                    }
                } else {
                    local_map_.data[index] = -1;  // Unknown
                }
            }
        }
    }

    // ... (imuCallback and gpsCallback remain the same as in previous version)

    void updateAndPublish() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto current_time = this->now();

        // Update local map timestamp
        local_map_.header.stamp = current_time;
        local_map_pub_->publish(local_map_);

        // Publish robot pose with updated covariance
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "map";
        
        // Set position and orientation
        pose_msg.pose.pose.position.x = robot_position_.x();
        pose_msg.pose.pose.position.y = robot_position_.y();
        pose_msg.pose.pose.position.z = robot_position_.z();
        
        pose_msg.pose.pose.orientation.w = robot_orientation_.w();
        pose_msg.pose.pose.orientation.x = robot_orientation_.x();
        pose_msg.pose.pose.orientation.y = robot_orientation_.y();
        pose_msg.pose.pose.orientation.z = robot_orientation_.z();

        // Update covariance based on sensor reliability
        updatePoseCovariance(pose_msg.pose.covariance);
        robot_pose_pub_->publish(pose_msg);

        // Broadcast transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = robot_position_.x();
        transform.transform.translation.y = robot_position_.y();
        transform.transform.translation.z = robot_position_.z();
        
        transform.transform.rotation.w = robot_orientation_.w();
        transform.transform.rotation.x = robot_orientation_.x();
        transform.transform.rotation.y = robot_orientation_.y();
        transform.transform.rotation.z = robot_orientation_.z();

        tf_broadcaster_->sendTransform(transform);
    }

    void updatePoseCovariance(std::array<double, 36>& covariance) {
        // Update position covariance based on GPS accuracy
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                covariance[i * 6 + j] = position_covariance_(i, j);
            }
        }

        // Update orientation covariance based on IMU accuracy
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                covariance[(i + 3) * 6 + (j + 3)] = orientation_covariance_(i, j);
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
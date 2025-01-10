#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>

struct GridPoint {
    int x, y;
    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for GridPoint
namespace std {
    template<>
    struct hash<GridPoint> {
        size_t operator()(const GridPoint& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

struct Node {
    GridPoint pos;
    double g_cost;
    double h_cost;
    std::shared_ptr<Node> parent;
    
    double f_cost() const { return g_cost + h_cost; }
};

class PathPlanner : public rclcpp::Node {
public:
    PathPlanner();

private:
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    // Map data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    
    // Path planning
    std::vector<GridPoint> getNeighbors(const GridPoint& point);
    double calculateHeuristic(const GridPoint& start, const GridPoint& goal);
    nav_msgs::msg::Path createPathMsg(const std::vector<GridPoint>& path);
    bool isValidCell(const GridPoint& point);
    bool findPath(const GridPoint& start, const GridPoint& goal, std::vector<GridPoint>& path);
    bool worldToGrid(const geometry_msgs::msg::Point& world, GridPoint& grid);
    geometry_msgs::msg::Point gridToWorld(const GridPoint& grid);
    
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // Constants
    static constexpr int OBSTACLE_THRESHOLD = 50;
    static constexpr double INFLATION_RADIUS = 0.2; // meters
};

#endif // PATH_PLANNER_HPP
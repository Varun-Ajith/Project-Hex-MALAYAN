#include "hexapod_control/path_planner.hpp"
#include <cmath>

PathPlanner::PathPlanner() : Node("path_planner") {
    // Initialize TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize subscribers
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&PathPlanner::mapCallback, this, std::placeholders::_1));
        
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "robot_pose", 10, std::bind(&PathPlanner::poseCallback, this, std::placeholders::_1));
        
    // Initialize publishers
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    
    RCLCPP_INFO(get_logger(), "Path Planner initialized");
}

void PathPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
}

void PathPlanner::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void PathPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!current_map_) {
        RCLCPP_WARN(get_logger(), "No map received yet");
        return;
    }

    GridPoint start_grid, goal_grid;
    if (!worldToGrid(current_pose_.pose.position, start_grid) ||
        !worldToGrid(msg->pose.position, goal_grid)) {
        RCLCPP_ERROR(get_logger(), "Invalid coordinates");
        return;
    }

    std::vector<GridPoint> path;
    if (findPath(start_grid, goal_grid, path)) {
        path_pub_->publish(createPathMsg(path));
        RCLCPP_INFO(get_logger(), "Path published");
    } else {
        RCLCPP_WARN(get_logger(), "No path found");
    }
}

bool PathPlanner::findPath(const GridPoint& start, const GridPoint& goal, std::vector<GridPoint>& path) {
    auto compare = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
        return a->f_cost() > b->f_cost();
    };
    
    std::priority_queue<std::shared_ptr<Node>, 
                       std::vector<std::shared_ptr<Node>>, 
                       decltype(compare)> open_set(compare);
    std::unordered_set<GridPoint> closed_set;
    
    open_set.push(std::make_shared<Node>(Node{start, 0.0, calculateHeuristic(start, goal), nullptr}));
    
    while (!open_set.empty()) {
        auto current = open_set.top();
        open_set.pop();
        
        if (current->pos == goal) {
            // Reconstruct path
            path.clear();
            auto node = current;
            while (node) {
                path.push_back(node->pos);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            return true;
        }
        
        closed_set.insert(current->pos);
        
        for (const auto& neighbor_pos : getNeighbors(current->pos)) {
            if (closed_set.find(neighbor_pos) != closed_set.end()) {
                continue;
            }
            
            double new_g_cost = current->g_cost + 
                (neighbor_pos.x != current->pos.x && neighbor_pos.y != current->pos.y ? 1.414 : 1.0);
                
            auto neighbor = std::make_shared<Node>(
                Node{neighbor_pos, new_g_cost, calculateHeuristic(neighbor_pos, goal), current});
                
            open_set.push(neighbor);
        }
    }
    
    return false;
}

std::vector<GridPoint> PathPlanner::getNeighbors(const GridPoint& point) {
    std::vector<GridPoint> neighbors;
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    for (int i = 0; i < 8; ++i) {
        GridPoint neighbor{point.x + dx[i], point.y + dy[i]};
        if (isValidCell(neighbor)) {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

bool PathPlanner::isValidCell(const GridPoint& point) {
    if (!current_map_) return false;
    
    if (point.x < 0 || point.x >= static_cast<int>(current_map_->info.width) ||
        point.y < 0 || point.y >= static_cast<int>(current_map_->info.height)) {
        return false;
    }
    
    int index = point.y * current_map_->info.width + point.x;
    return current_map_->data[index] < OBSTACLE_THRESHOLD;
}

double PathPlanner::calculateHeuristic(const GridPoint& start, const GridPoint& goal) {
    return std::hypot(goal.x - start.x, goal.y - start.y);
}

bool PathPlanner::worldToGrid(const geometry_msgs::msg::Point& world, GridPoint& grid) {
    if (!current_map_) return false;
    
    grid.x = static_cast<int>((world.x - current_map_->info.origin.position.x) / 
                             current_map_->info.resolution);
    grid.y = static_cast<int>((world.y - current_map_->info.origin.position.y) / 
                             current_map_->info.resolution);
    
    return isValidCell(grid);
}

geometry_msgs::msg::Point PathPlanner::gridToWorld(const GridPoint& grid) {
    geometry_msgs::msg::Point world;
    world.x = grid.x * current_map_->info.resolution + current_map_->info.origin.position.x;
    world.y = grid.y * current_map_->info.resolution + current_map_->info.origin.position.y;
    world.z = 0.0;
    return world;
}

nav_msgs::msg::Path PathPlanner::createPathMsg(const std::vector<GridPoint>& path) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = current_map_->header.frame_id;
    
    for (const auto& point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position = gridToWorld(point);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    
    return path_msg;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
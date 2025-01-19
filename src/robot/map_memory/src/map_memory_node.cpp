#include "map_memory_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

using namespace std;

const int THRESHOLD = 1.5;
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    width_ = this->declare_parameter<int>("width", 100);
    height_ = this->declare_parameter<int>("height", 100);
    resolution_ = this->declare_parameter<double>("resolution", 0.05);

    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "Invalid map dimensions or resolution!");
        throw std::runtime_error("Invalid map dimensions or resolution");
    }

    global_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    if (!global_map) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize global_map!");
        throw std::runtime_error("global_map initialization failed");
    }

    global_map->info.width = width_;
    global_map->info.height = height_;
    global_map->info.resolution = resolution_;
    global_map->data.assign(width_ * height_, 0);

    pos_ = {0.0, 0.0};
    update_map_state_ = false;

    grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallBack, this, std::placeholders::_1));

    odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallBack, this, std::placeholders::_1));

    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MapMemoryNode::updateMap, this));

    RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized successfully.");
}

void MapMemoryNode::costmapCallBack(const nav_msgs::msg::OccupancyGrid &msg) {
    local_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(msg);
    update_map_state_ = true;
}

void MapMemoryNode::odomCallBack(const nav_msgs::msg::Odometry &msg) {
    // Get the current position
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    //RCLCPP_INFO(this->get_logger(), "Position  (%f, %f)", x, y);

    
    // Convert to euler angles 
    auto orient = msg.pose.pose.orientation;
    tf2::Quaternion quat(orient.x, orient.y, orient.z, orient.w);
    tf2::Matrix3x3 m(quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    theta_ = yaw;

    // Updating current position
    dist_ = std::sqrt(std::pow(x - pos_.first, 2) + std::pow(y - pos_.second, 2));

    if (dist_ < THRESHOLD) {
        return;
    }

    pos_ = {x, y};
    update_map_ = true;
}

void MapMemoryNode::updateMap() {
    if (update_map_ && update_map_state_) {
        linearFusion();
        global_map.header.stamp = this->now();
        global_map.header.frame_id = "sim_world";
        map_pub->publish(*global_map);
        update_map_ = false;
    }
}

void MapMemoryNode::linearFusion() {
    int global_h = global_map->info.height;
    int global_w = global_map->info.width;
    int local_h = local_map->info.height;
    int local_w = local_map->info.width;

    double res = local_map->info.resolution;

    // Center of local map
    int center_x = local_w * res / 2;
    int center_y = local_h * res / 2;

    // Center of the global map
    int origin_x = global_w * res / 2;
    int origin_y = global_h * res / 2;


    for (int i = 0; i < local_h; i++) {
        for (int j = 0; j < local_w; j++) {
            int8_t cost = local_map->data[i * local_w + j];

            // Local coordiantes: Convert the Array coords -> Local coords
            double x = i * res - center_x;
            double y = j * res - center_y;
            
            // Convert Local coords -> Global coords
            double newX = x * cos(theta_) - y * sin(theta_) + pos_.first;
            double newY = x * sin(theta_) + y * cos(theta_) + pos_.second;

            // Converts Global coords -> Array coords
            int arrX = static_cast<int>((newX + origin_x)/res);
            int arrY = static_cast<int>((newY + origin_y)/res);

            // Boundary check
            if (arrX < 0 || arrX >= global_w || arrY < 0 || arrY >= global_h) {
                continue;
            }

            //RCLCPP_INFO(this->get_logger(), "Global Position  (%f, %f)", newX, newY);
            //RCLCPP_INFO(this->get_logger(), "Array Global Position  (%i, %i)", arrX, arrY);

            global_map->data[arrY * global_h + arrX] = std::max(global_map->data[arrY * global_h + arrX], static_cast<signed char>(cost));
            //RCLCPP_INFO(this->get_logger(), "Is this real %f", global_map->data[arrY*global_h+arrX]);

        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}

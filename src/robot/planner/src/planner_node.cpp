#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    // Create subscribers
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallBack, this, std::placeholders::_1));
    goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallBack, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filter", 10, std::bind(&PlannerNode::odomCallBack, this, std::placeholders::_1));

    // Create publisher
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallBack, this));
}

void PlannerNode::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    grid_ = msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        pathPlan();
    }
}

void PlannerNode::goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    dest_ = msg;
    dest_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    pathPlan();
}

void PlannerNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
}

void PlannerNode::timerCallBack() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            pathPlan();
        }
    }
}

bool PlannerNode::goalReached() {
    if (!pose_ || !dest_) return false;
    double dx = dest_->point.x - pose_->position.x;
    double dy = dest_->point.y - pose_->position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching goal
}

void PlannerNode::pathPlan() {
    if (!dest_received_ || !grid_ || grid_->data.empty()) {
        RCLCPP_INFO(this->get_logger(), "Cannot plan path: Missing map or goal");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    std::vector<CellIndex> path_nodes;
    if (Astar(path_nodes)) {
        RCLCPP_INFO(this->get_logger(), "Path found, publishing...");
        path.poses.clear();

        for (const auto &node : path_nodes) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = node.x;
            p.pose.position.y = node.y;
            p.pose.orientation.w = 1.0;
            path.poses.push_back(p);
        }
        path_pub->publish(path);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to find a path!");
    }
}

bool PlannerNode::Astar(std::vector<CellIndex> &path) {
    if (!pose_ || !dest_) return false;

    CellIndex start(static_cast<int>(pose_->position.x), static_cast<int>(pose_->position.y));
    CellIndex goal(static_cast<int>(dest_->point.x), static_cast<int>(dest_->point.y));

    auto manhattanDist = [](const CellIndex &a, const CellIndex &b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    };

    std::priority_queue<Node, std::vector<Node>, CompareF> open_set;
    std::unordered_set<CellIndex, CellIndexHash> closed_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    g_score[start] = 0.0;
    open_set.emplace(start, 0.0, manhattanDist(start, goal));

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        if (current.index == goal) {
            CellIndex backtrack = goal;
            while (backtrack != start) {
                path.push_back(backtrack);
                backtrack = came_from[backtrack];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return true;
        }

        closed_set.insert(current.index);

        for (const auto &direction : dir_) {
            CellIndex neighbor(current.index.x + direction[0], current.index.y + direction[1]);

            if (neighbor.x < 0 || neighbor.x >= grid_->info.width ||
                neighbor.y < 0 || neighbor.y >= grid_->info.height) {
                continue;
            }

            int cost = getCost(neighbor);
            if (cost > 80) continue; // High cost means obstacle

            if (closed_set.find(neighbor) != closed_set.end()) continue;

            double tentative_g_score = g_score[current.index] + 1.0;

            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                came_from[neighbor] = current.index;
                g_score[neighbor] = tentative_g_score;
                double h_score = manhattanDist(neighbor, goal);
                open_set.emplace(neighbor, tentative_g_score, h_score);
            }
        }
    }

    return false;
}

double PlannerNode::manhattanDist(const CellIndex &pose, const CellIndex &dest) {
    return std::abs(dest.x - pose.x) + std::abs(dest.y - pose.y);
}

std::vector<PlannerNode::CellIndex> PlannerNode::neigh(const CellIndex &cidx) {
    std::vector<CellIndex> res;
    for (const auto &d : dir_) {
        int newX = cidx.x + d[0];
        int newY = cidx.y + d[1];
        res.emplace_back(newX, newY);
    }
    return res;
}

bool PlannerNode::convertToMap(double x, double y, int &arrX, int &arrY) {
    int global_h = grid_->info.height;
    int global_w = grid_->info.width;
    double res = grid_->info.resolution;

    int origin_x = static_cast<int>(grid_->info.origin.position.x);
    int origin_y = static_cast<int>(grid_->info.origin.position.y);

    arrX = static_cast<int>((x - origin_x) / res);
    arrY = static_cast<int>((y - origin_y) / res);

    return (arrX >= 0 && arrX < global_w && arrY >= 0 && arrY < global_h);
}

int PlannerNode::getCost(const CellIndex &id) {
    int arrX, arrY;
    if (!convertToMap(id.x, id.y, arrX, arrY)) return 150; // Out of bounds

    int index = arrY * grid_->info.width + arrX;
    int val = grid_->data[index];
    return (val < 0) ? 150 : val;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}

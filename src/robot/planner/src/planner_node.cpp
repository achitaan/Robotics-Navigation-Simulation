#include <algorithm>
#include <unordered_map>
#include "planner_node.hpp"

PlannerNode::PlannerNode() : rclcpp::Node("planner") {

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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallBack, this));
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
            RCLCPP_INFO(this->get_logger(), "Goal Reached!!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            pathPlan();
        }
    }
}

bool PlannerNode::goalReached() {
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
      path.poses.clear();

    for (const auto &node : path_nodes) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = node.x;
      p.pose.position.y = node.y;
      p.pose.orientation.w = 1.0;

      path.poses.push_back(p);
    }
  } 

  else {
    RCLCPP_INFO(this->get_logger(), "Path planning failed.");
    return;
  }

  path_pub->publish(path);
}

bool PlannerNode::Astar(std::vector<CellIndex> &path) {
    std::priority_queue<Node, std::vector<Node>, CompareF> q;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> prev_node;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    CellIndex start(static_cast<int>(pose_->position.x), static_cast<int>(pose_->position.y));
    CellIndex goal(static_cast<int>(dest_->point.x), static_cast<int>(dest_->point.y));

    g_score[start] = 0.0;
    q.emplace(start, 0.0, manhattanDist(start, goal));

    while (!q.empty()) {
        Node curr = q.top();
        q.pop();

        if (curr.index == goal) {
            CellIndex prev = curr.index;
            while (prev != start) {
                path.push_back(prev);
                prev = prev_node[prev];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return true;
        }

        for (const auto &dir : dir_) {
            CellIndex next(curr.index.x + dir[0], curr.index.y + dir[1]);
            if (next.x < 0 || next.y < 0 || next.x >= static_cast<int>(grid_->info.width) ||
                next.y >= static_cast<int>(grid_->info.height)) {
                continue;
            }

            int cost = getCost(next);
            if (cost > 80) {
                continue;
            }

            double new_g = g_score[curr.index] + 1.0;
            if (g_score.find(next) == g_score.end() || new_g < g_score[next]) {
                g_score[next] = new_g;
                prev_node[next] = curr.index;
                q.emplace(next, new_g, manhattanDist(next, goal));
            }
        }
    }
    return false;
}

std::vector<PlannerNode::CellIndex> PlannerNode::neigh(const CellIndex &cidx) {
    std::vector<CellIndex> res;
    for (const auto &d : dir_) {
        res.emplace_back(cidx.x + d[0], cidx.y + d[1]);
    }
    return res;
}

double PlannerNode::manhattanDist(const CellIndex &pose, const CellIndex &dest) {
    return std::abs(dest.x - pose.x) + std::abs(dest.y - pose.y);
}

bool PlannerNode::convertToMap(double x, double y, int &arrX, int &arrY) {
    int global_h = grid_->info.height;
    int global_w = grid_->info.width;
    double res = grid_->info.resolution;

    int origin_x = global_w * res / 2;
    int origin_y = global_h * res / 2;

    int newX = static_cast<int>((x + origin_x) / res);
    int newY = static_cast<int>((y + origin_y) / res);

    if (newX < 0 || newX >= global_w || newY < 0 || newY >= global_h) {
        return false;
    }

    arrX = newX;
    arrY = newY;
    return true;
}

int PlannerNode::getCost(const CellIndex &id) {
    int arrX, arrY;
    if (!convertToMap(id.x, id.y, arrX, arrY)) {
        return 150;
    }

    int idx = arrY * grid_->info.width + arrX;
    int val = grid_->data[idx];
    return val < 0 ? 150 : val;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}

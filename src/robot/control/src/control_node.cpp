#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#include "control_node.hpp"


ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Subscribers and Publishers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
  // Skip control if no path or odometry data is available
  if (!current_path_ || !robot_odom_) {
      return;
  }

  // Find the lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
      return;  // No valid lookahead point found
  }

  // Compute velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);

  // Publish the velocity command
  cmd_vel_pub_->publish(cmd_vel);
}
 
std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    // TODO: Implement logic to find the lookahead point on the path
    for (auto &p : current_path_){
      int x = p.pose.position.x, y = p.pose.position.y

      

    }

    return std::nullopt;  // Replace with a valid point when implemented
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  // TODO: Implement logic to compute velocity commands

  

  geometry_msgs::msg::Twist cmd_vel;

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // Convert to euler angles 
  tf2::Quaternion quat(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(quat);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  theta_ = yaw;

  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <unordered_map>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  // Create subscribers
  map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>('/map', 10, std::bind(PlannerNode::mapCallBack, this, std::placeholders::_1));
  goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>('/goal_point', 10, std::bind(PlannerNode::goalCallBack, this, std::placeholders::_1));
  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>('/odom/filter', 10, std::bind(PlannerNode::odomCallBack, this, std::placeholders::_1));

  // Create Publisher
  path_pub = this->create_publisher<nav_msgs::msg::Path>('/path', 10);

  // Creater timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallBack, this));
}

void PlannerNode::mapCallBacker(nav_msgs::msg::OccupancyGrid &msg){
  grid_ = msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL){
    pathPlan();
  }
  
}

void PlannerNode::goalCallBack(geometry_msgs::msg::PointStamped &msg){
  dest_ = msg;
  dest_recieved_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  pathPlan();
}

void PlannerNode::odomCallBack(nav_msgs::msg::Odometry &msg){
  pose_ = msg->pose.pose;
}

void PlannerNode::timerCallBack(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL){
    if (goalReached()){
      RCLCPP_INFO(this->get_logger(), "Goal Reacherd!!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      pathPlan();
    }
  }
}

void PlannerNode::goalReached(){
  double dx = dest_.point.x - pose_.position.x, dy = dest_.point.y - pose_.position.y;
  return std::sqrt(dx*dx _ dy*dy) < 0.5; // Threshold for reaching goal
}

void PlannerNode::pathPlan(){
  if (!dest_recieved_ || grid_.data.empty()){
    RCLCPP_INFO(this->get_logger(), "Cannot plan path: Missing map or goal");
    return;
  }
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";
  
  vector<CellIndex> path_nodes;
  if (Astar(path_nodes)){
    RCLCPP_INFO(this->get_logger(), "It aint cooked yet");

    if (path->poses.size() > 0){
      path->poses.clear();
    }

    for (auto &node : path_nodes){
      geometry_msgs::msg::PoseStamped p;

      p.pose.position.x = node.index.x;
      p.pose.position.y = node.index.y;
      p.pose.position.w = node.index.1.0;



      path->poses.push_back(p);
    }
    
  }
  
  else{
    RCLCPP_INFO(this->get_logger(), "Its cooked bruh");
    return;
  }
  path.header.stamp = this->now();
  path.header.frame_id = grid_->header.frame_id;
  path_pub->publish(path);
}

bool PlannerNode::Astar(vector<CellIndex> path){
    // Use A* 
  priority_queue<Node, vetor<Node>, CompareF> q;
  std::unordered_set<CellIndex, CellIndex. CellIndexHash> visited; 
  std::unordered_set<CellIndex, CellIndex. CellIndexHash> prev_graph; 
  std::unordered_set<CellIndex, double. CellIndexHash> score; 

  // Set up starting node
  CellIndex start = CellIndex(pose_.postion.x, pose_.position.y);
  CellIndex dest = CellIndex(dest_.point.x, dest_.position.y);
  score[(start)] = 0;

  double manhattan_dist = manhattan_dist(start, dest);
  q.insert(Node(start, 0, manhattan_dist))

  while (!q.empty()){
    Node curr = q.top(); q.pop();

    if (curr.index==dest_.index){
      // reconstruct the path
      c = curr.index
      while (c != start){
        path.push_back(c);
        c = prev_graph[c];
      }
      path.push_back(start);
      reverse(path.begin(), path.end());
      
    }

    visited.insert((curr.index));

    dir = neigh(curr.index);
    for (auto &next : dir){
      if (newIdx.x < 0 || newIdx.x >= grid_size || newIdx.y < 0 || newIdx.y >= grid_size)
        continue;
      
      cost = get_cost(next);
      
      if (cost > 80){
        // Check if the area is free and if it has not been visited
        continue;
      }

      int newG = score[(cur.index)] + 1;
      if (newG < score[(next)] || visited.find((next)) == visited.end()){
        nextNode = Node(next, newG, manhattan_dist(next, dest));
        score[(next)] = newG;

        prev_graph[(next)] = curr.index;
        visited.insert((next));
        q.push(nextNode);
      }
    }
  }
  return false;
}

std::vector<CellIndex> PlannerNode::neigh(CellIndex &cidx){
  std::vector<CellIndex> res;

  for (auto &d : dir_){
    int newY = cidx.y + d[0], newX = cidx.x + d[1];
    res.push_back(CellIndex(newX, newY));
  }
  return res;
}

double PlannerNode::manhattan_dist(CellIndex pose, CellIndex dest){
  return std::abs(dest.x - pose.x) + std::abs(dest.y - pose..y);
}

bool PlannerNode::convertToMap(double x, double y, int &arrX, int &arrY){
  int global_h = global_map->info.height;
  int global_w = global_map->info.width;
  int local_h = local_map->info.height;
  int local_w = local_map->info.width;

  int origin_x = global_w * res / 2;
  int origin_y = global_h * res / 2;

  newX = static_cast<int>((x + origin_x)/res);
  newY = static_cast<int>((y + origin_y)/res);

  // Boundary check
  if (newX < 0 || newX >= global_w || newY < 0 || newY >= global_h) 
    return false;
  
  arrX = newX;
  arrY = newY;

  return true;
}

int PlannerNode::getCost(CellIndex &id){
  int arrX, arrY;

  if (!convertToMap(arrX, arrY)) {
      return 150;
  }

  int val = grid_->data[arrY * global_h + arrX];
  if (val < 0)
    return 150;

  return static_cast<int>(val);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

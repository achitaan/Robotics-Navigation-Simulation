#include <chrono>
#include <memory>
#include <queue>
#include <utility>
#include <unordered_set>   
#include <cmath>           

#include "costmap_node.hpp"

// If needed for messages
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std;

CostmapNode::CostmapNode() 
: Node("costmap"), 
  costmap_(robot::CostmapCore(this->get_logger())){
  grid_data_.size = 100;
  grid_data_.max_cost = 100;
  grid_data_.radius = 10;
  grid_data_.resolution = 0.1;
  grid_data_.center = {
    grid_data_.size * grid_data_.resolution /2, // Convert coord to grid points
    grid_data_.size * grid_data_.resolution /2
    };

  occupancy_grid_ = new int*[grid_data_.size];
  for(int i = 0; i < grid_data_.size; i++){
    occupancy_grid_[i] = new int[grid_data_.size];
    for(int j = 0; j < grid_data_.size; j++){
      occupancy_grid_[i][j] = 0;
    }
  }

  // Create Subscriber
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallBack, this, std::placeholders::_1));
  // Create Publisher
  costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&CostmapNode::publishGrid, this)
  );
}

void CostmapNode::lidarCallBack(const sensor_msgs::msg::LaserScan &msg){
  int center_x = grid_data_.center.first;
  int center_y = grid_data_.center.second;

  //RCLCPP_INFO(this->get_logger(), "nagle max = %f", msg.angle_max);

  for (size_t i = 0; i < msg.ranges.size(); i++){
    double range = msg.ranges[i];
    double angle = msg.angle_min + i * msg.angle_increment;

    if (range > grid_data_.size/2)
      continue;

    // Relative (x, y) coords
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    //RCLCPP_INFO(this->get_logger(), "x  %f", x);


    int grid_x = static_cast<int>((x + center_x)/grid_data_.resolution);
    int grid_y = static_cast<int>((y + center_y)/grid_data_.resolution);

    //RCLCPP_INFO(this->get_logger(), "Range  %f", range);

    //RCLCPP_INFO(this->get_logger(), "Global coords %f, Local grid?  %f", x + center_x, (x + center_x)/grid_data_.resolution);


    //RCLCPP_INFO(this->get_logger(), "Okay asoidjaslknd ");

    if (grid_x >= 0 && grid_x < grid_data_.size && grid_y >= 0 && grid_y < grid_data_.size){
      //RCLCPP_INFO(this->get_logger(), "Position  (%f, %f)", x + center_x, y + center_y);


      occupancy_grid_[grid_y][grid_x] = 100;
      //RCLCPP_INFO(this->get_logger(), "occupancy grid %f", occupancy_grid_[grid_y][grid_x]);


      // Inflate obstacles with BFS
      this->inflateNeighbours(occupancy_grid_,
        grid_x,
        grid_y,
        grid_data_.radius,
        grid_data_.size,
        grid_data_.max_cost
      );
    }
  }
}


void CostmapNode::inflateNeighbours(
  int **occupancy_grid, 
  int x, 
  int y, 
  double inflation_radius, 
  int grid_size,
  double max_cost
){
  // normal bfs
  queue<pair<int, int>> q;
  std::unordered_set<long long> visited; 

  auto encode = [&](int r, int c){
    return (static_cast<long long>(r) << 32) ^ static_cast<long long>(c);
  };

  int dir[4][2] = {
    {1, 0}, {0, 1}, {-1, 0}, {0, -1}
  };

  q.push({y, x});
  visited.insert(encode(y, x));

  while (!q.empty()){
    auto curr = q.front();
    q.pop();

    int cy = curr.first;
    int cx = curr.second;

    for (auto &d : dir){
      int newY = cy + d[0];
      int newX = cx + d[1];

      if (newX < 0 || newX >= grid_size || newY < 0 || newY >= grid_size)
        continue;

      double dist_squared = std::pow(static_cast<double>(newX - x), 2.0)
                          + std::pow(static_cast<double>(newY - y), 2.0);

      if (dist_squared < (inflation_radius * inflation_radius)){
        if (visited.find(encode(newY, newX)) == visited.end()){
          visited.insert(encode(newY, newX));

          double cost = max_cost * (1.0 - (std::sqrt(dist_squared) / inflation_radius));
          int int_cost = std::max(occupancy_grid[newY][newX], static_cast<int>(cost));

          occupancy_grid[newY][newX] = int_cost;
          q.push({newY, newX});
        }
      }
    }
  }
}

int* CostmapNode::flattenArray(int **arr, int grid_size){
  int *new_occupancy = new int[grid_size * grid_size];

  for (int i = 0; i < grid_size; i++){
    for (int j = 0; j < grid_size; j++){
      new_occupancy[i * grid_size + j] = arr[i][j];
    }
  }
  return new_occupancy;
}

void CostmapNode::publishGrid(){
  nav_msgs::msg::OccupancyGrid message;

  message.header.stamp = this->now();
  message.header.frame_id = "map";
  message.info.width = grid_data_.size;
  message.info.height = grid_data_.size;
  message.info.resolution = grid_data_.resolution; 

  int *flat = CostmapNode::flattenArray(occupancy_grid_, grid_data_.size);

  // OccupancyGrid's 'data' is std::vector<int8_t>
  message.data.resize(grid_data_.size * grid_data_.size);
  for(int i = 0; i < grid_data_.size * grid_data_.size; i++){
    int val = flat[i];
    //RCLCPP_INFO(this->get_logger(), "Publishing costmap of size %d", flat[i]);

    if (val > 100) val = 100;
    if (val < 0)   val = 0;
    message.data[i] = static_cast<int8_t>(val);
  }
  delete [] flat; 

  costmap_pub->publish(message);
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

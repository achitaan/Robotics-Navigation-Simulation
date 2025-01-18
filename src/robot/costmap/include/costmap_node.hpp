#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

// If using LaserScan messages and OccupancyGrid:
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// If you have a custom costmap_core.hpp:
#include "costmap_core.hpp"

// Definition of GridData
struct GridData {
  int size;
  std::pair<int, int> center;
  int max_y;
  double max_cost;
  double radius;
  double resolution;
};

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  // CostmapCore
  robot::CostmapCore costmap_;

  // Class members to store the occupancy grid and grid data
  int **occupancy_grid_;
  GridData grid_data_;

  // Subscription and publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub;

  // Timer for periodic publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback that processes the LaserScan and updates the occupancy grid
  // (Only takes a LaserScan now)
  void lidarCallBack(const sensor_msgs::msg::LaserScan &msg);

  // Inflate neighbors around an obstacle using BFS
  void inflateNeighbours(
    int **occupancy_grid,
    int x,
    int y,
    double inflation_radius,
    int grid_size,
    double max_cost
  );

  // Flatten the 2D occupancy grid into a 1D array
  int* flattenArray(int **arr, int grid_size);

  // Publish a nav_msgs::msg::OccupancyGrid 
  void publishGrid();
};

#endif  // COSTMAP_NODE_HPP_

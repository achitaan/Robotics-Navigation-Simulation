#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    struct CellIndex{
      int x, y;

      CellIndex(int xx, int yy) : x(xx), y(yy){}
      CellIndex() : x(0), y(0) {}

      bool operator==(const CellIndex &other) const{
        return (x==other.x && y==other.y);
      }

      bool operator!=(const CellIndex &other) const{
        return (x!=other.x && y!=other.y);
      }
    };

    // Hash function
    struct CellIndexHash{
      std::size_t operator()(const CellIndex &idx) const{
        return std::hash<int>(idx.x) ^ (std::hash<int>(idx.y) << 1);
      }
    };

    struct Node{
      CellIndex index;
      double g_score;
      double h_score;
      double f_score;



      AStarNode(CellIndex indx, double g, double h) : index(idx), g_score(f), h_score{h}{
        f_score = g_score + h_score;
      }
    };

    struct CompareF{
      bool operator()(const Node &a, const Node &b){
        return a.f_score > b.f_score;
      }
    }


  private:
    robot::PlannerCore planner_;
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    // Init subscribers and Publishers
    rlccpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rlccpp::Subscription<eometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
    rlccpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr odom_sub;

    rclcpp::Publisher<av_msgs::msg::Path>::SharedPtr path_pub;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Init Variables
    bool dest_recieved_ = false;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
    geometry_msgs::msg::PointStamped::SharedPtr dest_;
    nav_msgs::msg::Odometry::SharedPtr pose_;
    nav_msgs::msg::Path::

    int dir_[8][2] = {
      {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1. -1}, {-1, -1}, {-1, 1}
    };
    

    void mapCallBacker(nav_msgs::msg::OccupancyGrid &msg)
    void goalCallBack(geometry_msgs::msg::PointStamped &msg)
    void odomCallBack(nav_msgs::msg::Odometry &msg)
    void timerCallBack()

    void goalReached()
    void pathPlan()
    bool Astar(vector<CellIndex> path)
    double manhattan_dist(CellIndex pose, CellIndex dest)
    std::vector<CellIndex> neigh(CellIndex &cidx)
    bool convertToMap(double x, double y, int &arrX, int &arrY)
    int getCost(CellIndex &id)


};

#endif 

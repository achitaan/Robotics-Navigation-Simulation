#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

    struct CellIndex {
        int x, y;

        CellIndex(int xx, int yy) : x(xx), y(yy) {}
        CellIndex() : x(0), y(0) {}

        bool operator==(const CellIndex &other) const {
            return (x == other.x && y == other.y);
        }

        bool operator!=(const CellIndex &other) const {
            return (x != other.x || y != other.y);
        }
    };

    struct CellIndexHash {
        std::size_t operator()(const CellIndex &idx) const {
            return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
        }
    };

    struct Node {
        CellIndex index;
        double g_score;
        double h_score;
        double f_score;

        Node(CellIndex idx, double g, double h)
            : index(idx), g_score(g), h_score(h), f_score(g + h) {}
    };

    struct CompareF {
        bool operator()(const Node &a, const Node &b) {
            return a.f_score > b.f_score;
        }
    };

private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    bool dest_received_ = false;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
    geometry_msgs::msg::PointStamped::SharedPtr dest_;
    geometry_msgs::msg::Pose::SharedPtr pose_;

    int dir_[8][2] = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1},
        {1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

    void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallBack();

    bool goalReached();
    void pathPlan();
    bool Astar(std::vector<CellIndex> &path);
    double manhattanDist(const CellIndex &pose, const CellIndex &dest);
    std::vector<CellIndex> neigh(const CellIndex &cidx);
    bool convertToMap(double x, double y, int &arrX, int &arrY);
    int getCost(const CellIndex &id);
};

#endif // PLANNER_NODE_HPP_
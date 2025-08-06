#ifndef SIMPLE_PLANNER_HPP
#define SIMPLE_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "simple_planner/map_utils.hpp"  // <-- Add this

class SimplePlanner : public rclcpp::Node
{
public:
  SimplePlanner();

private:
  // Callback for map
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // 2.1 
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);  // 2.2
  void timerCallback();  // 2.3


  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;  // 2.1 
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;  // 2.2 

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;  // 2.3 

  // TF2 listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;    // 2.3 
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   // 2.3 

  // Flags
  bool map_received_ = false;  // 2.1
  bool goal_received_ = false;  // 2.2
  bool robot_pose_valid_ = false;  // 2.3 

  // Data storage
  nav_msgs::msg::OccupancyGrid current_map_;   // 2.1
  geometry_msgs::msg::PoseStamped goal_pose_;  // 2.2
  geometry_msgs::msg::PoseStamped robot_pose_;   // 2.3 

  // Parsed map
  simple_planner::map_utils::MapData map_data_;  // <-- Add this
};

#endif  // SIMPLE_PLANNER_HPP

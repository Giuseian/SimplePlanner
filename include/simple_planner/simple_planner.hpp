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
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // <-- Add this

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2 listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Flags
  bool map_received_ = false;
  bool goal_received_ = false;
  bool robot_pose_valid_ = false;

  // Data storage
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped robot_pose_;

  // Parsed map
  simple_planner::map_utils::MapData map_data_;  // <-- Add this
};

#endif  // SIMPLE_PLANNER_HPP

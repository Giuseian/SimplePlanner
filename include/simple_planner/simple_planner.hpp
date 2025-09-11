#ifndef SIMPLE_PLANNER_HPP
#define SIMPLE_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

#include "simple_planner/map_utils.hpp"

class SimplePlanner : public rclcpp::Node
{
public:
  SimplePlanner();

private:
  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void timerCallback();

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Flags
  bool map_received_ = false;
  bool goal_received_ = false;
  bool robot_pose_valid_ = false;
  bool use_manual_start_ = false;
  bool manual_start_received_ = false;

  // Data
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::PoseStamped manual_start_pose_;
  simple_planner::map_utils::MapData map_data_;
};

#endif  // SIMPLE_PLANNER_HPP



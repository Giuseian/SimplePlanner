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
#include "sensor_msgs/msg/image.hpp"

#include "simple_planner/map_utils.hpp"
#include "simple_planner/cost_utils.hpp"
#include "simple_planner/a_star.hpp"

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

  // Helpers
  void publishPath(const std::vector<std::pair<int,int>>& path_cells);
  void planOnce();  
  void publishCostmapDebug();  // Step 5: visualizzazione costmap

  // === Helper per validazione celle ===
  bool isInBounds(const std::pair<int,int>& cell) const;
  bool isCellFree(const std::pair<int,int>& cell) const;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr costmap_debug_pub_;

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
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::PoseStamped manual_start_pose_;
  simple_planner::map_utils::MapData map_data_;

  // Step 3.1: grid cell di start e goal
  std::pair<int, int> start_cell_;
  std::pair<int, int> goal_cell_;

  // Step 3.2: mappa 2D derivata da occupancy grid
  std::vector<std::vector<int8_t>> occupancy_grid_;

  // Step 4: change detection
  std::pair<int, int> last_start_cell_ = {-1, -1};
  std::pair<int, int> last_goal_cell_  = {-1, -1};
  int map_version_ = 0;
  int last_map_version_ = -1;

  // Step 5: Distance & Cost Map
  std::vector<std::vector<float>> distance_map_;
  std::vector<std::vector<float>> cost_map_;

  // Parametri cost function
  std::string cost_function_type_;   
  float inflation_radius_;
  float alpha_;
  float W_;
  float epsilon_;
  float lethal_cost_;

  // Step 6: A* weight 
  float lambda_weight_;
};

#endif  // SIMPLE_PLANNER_HPP

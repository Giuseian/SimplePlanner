#ifndef SIMPLE_PLANNER_HPP_
#define SIMPLE_PLANNER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <utility>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/image.hpp"

// Utils
#include "simple_planner/map_utils.hpp"
#include "simple_planner/cost_utils.hpp"
#include "simple_planner/a_star.hpp"

namespace simple_planner
{

// Stato del planner
enum class PlanStatus { WAITING, INVALID, VALID };

class SimplePlanner : public rclcpp::Node
{
public:
  explicit SimplePlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // === Callbacks ROS ===
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);   // riceve mappa 
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);   // riceve goal 
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);   // riceve posa iniziale del robot 
  void timerCallback();   // esegue periodicamente la pianificazione

  // === Helpers ===
  void publishPath(const std::vector<std::pair<int,int>>& path_cells);   // pubblica il percorso 
  void planOnce();   // funzione centrale di pianificazione 
  void publishCostmapDebug();   // pubblica immagine di debug della costmap

  // Validazione celle
  bool isInBounds(const std::pair<int,int>& cell) const;
  bool isCellFree(const std::pair<int,int>& cell) const;

  // === Subscribers ===
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;   // riceve OccupancyGrid 
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;   // riceve goal
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;  // riceve posa iniziale stimata del robot

  // === Publishers ===
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;  // pubblica percorso 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;   // per visualizzare marker start/goal
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr costmap_debug_pub_;  // pubblica immagine debug costmap

  // === Timer ===
  rclcpp::TimerBase::SharedPtr timer_;

  // === TF2 ===
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // === Flags ===
  bool map_received_ = false;
  bool goal_received_ = false;
  bool robot_pose_valid_ = false;
  bool use_manual_start_ = false;
  bool manual_start_received_ = false;

  // === Dati ===
  geometry_msgs::msg::PoseStamped goal_pose_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::PoseStamped manual_start_pose_;
  simple_planner::map_utils::MapData map_data_;

  // Celle di start e goal
  std::pair<int, int> start_cell_;
  std::pair<int, int> goal_cell_;

  // Occupancy grid 2D
  std::vector<std::vector<int8_t>> occupancy_grid_;

  // Change detection
  std::pair<int, int> last_start_cell_ = {-1, -1};
  std::pair<int, int> last_goal_cell_  = {-1, -1};
  int map_version_ = 0;
  int last_map_version_ = -1;

  // Validità ultime celle
  bool last_start_valid_ = false;
  bool last_goal_valid_ = false;

  // Distance & Cost Map
  std::vector<std::vector<float>> distance_map_;
  std::vector<std::vector<float>> cost_map_;

  // Parametri cost function
  std::string cost_function_type_;   
  float inflation_radius_;
  float alpha_;
  float W_;
  float epsilon_;
  float lethal_cost_;

  // Peso A*
  float lambda_weight_;

  // Stato pianificazione
  PlanStatus last_plan_status_;
};

}  // namespace simple_planner

#endif  // SIMPLE_PLANNER_HPP_



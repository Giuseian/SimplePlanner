#include "simple_planner/simple_planner.hpp"
#include "simple_planner/map_utils.hpp"
#include "simple_planner/cost_utils.hpp"

#include <algorithm>
#include <limits>
#include <array>

using std::placeholders::_1;
using namespace simple_planner::map_utils;
using namespace simple_planner::cost_utils;


simple_planner::SimplePlanner::SimplePlanner(const rclcpp::NodeOptions & options)
: rclcpp::Node("simple_planner")
{
  RCLCPP_INFO(this->get_logger(), "Simple Planner Node initialized!");

  // Stato iniziale
  last_plan_status_ = PlanStatus::WAITING;

  // === Parametri ===
  this->declare_parameter("use_manual_start", false);
  this->get_parameter("use_manual_start", use_manual_start_);

  this->declare_parameter("cost_function_type", "exponential");
  this->declare_parameter("inflation_radius", 0.5);
  this->declare_parameter("alpha", 5.0);
  this->declare_parameter("W", 20.0);
  this->declare_parameter("epsilon", 0.1);
  this->declare_parameter("lethal_cost", 1e9);

  this->get_parameter("cost_function_type", cost_function_type_);
  this->get_parameter("inflation_radius", inflation_radius_);
  this->get_parameter("alpha", alpha_);
  this->get_parameter("W", W_);
  this->get_parameter("epsilon", epsilon_);
  this->get_parameter("lethal_cost", lethal_cost_);

  // === Subscribers ===
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SimplePlanner::mapCallback, this, _1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",
    rclcpp::QoS(10),
    std::bind(&SimplePlanner::goalCallback, this, _1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose",
    rclcpp::QoS(10),
    std::bind(&SimplePlanner::initialPoseCallback, this, _1));

  // === Publishers ===
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
  costmap_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/simple_planner/costmap_debug", 1);

  // === TF2 ===
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // === Timer ===
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&SimplePlanner::timerCallback, this));
}

void simple_planner::SimplePlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_data_ = parseOccupancyGrid(*msg);

  if (map_data_.width <= 0 || map_data_.height <= 0 || map_data_.resolution <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid map metadata. Ignoring map.");
    map_received_ = false;
    return;
  }

  if (map_data_.data.size() != static_cast<size_t>(map_data_.width * map_data_.height)) {
    RCLCPP_ERROR(this->get_logger(), "Map data size mismatch");
    map_received_ = false;
    return;
  }

  occupancy_grid_ = to2DGrid(map_data_, true);
  map_received_ = true;
  map_version_++;

  RCLCPP_INFO(this->get_logger(),
              "Map received: %d x %d @ %.3f m resolution",
              map_data_.width, map_data_.height, map_data_.resolution);

  distance_map_ = computeDistanceMap(occupancy_grid_, map_data_);
  cost_map_ = computeCostMap(distance_map_, occupancy_grid_, map_data_,
                             cost_function_type_, inflation_radius_, alpha_,
                             W_, epsilon_, lethal_cost_);

  RCLCPP_INFO(this->get_logger(), "Cost map generated with type=%s",
              cost_function_type_.c_str());

  float min_cost = std::numeric_limits<float>::infinity();
  float max_cost = -std::numeric_limits<float>::infinity();
  double sum_cost = 0.0;
  size_t count = 0;

  for (int r = 0; r < map_data_.height; r++) {
    for (int c = 0; c < map_data_.width; c++) {
      float v = cost_map_[r][c];
      if (v >= lethal_cost_) continue;
      min_cost = std::min(min_cost, v);
      max_cost = std::max(max_cost, v);
      sum_cost += v;
      count++;
    }
  }

  if (count > 0) {
    double mean_cost = sum_cost / count;
    RCLCPP_INFO(this->get_logger(),
                "Cost map stats: min=%.3f, max=%.3f, mean=%.3f (over %zu free cells)",
                min_cost, max_cost, mean_cost, count);
  } else {
    RCLCPP_WARN(this->get_logger(), "Cost map contains no free cells!");
  }

  publishCostmapDebug();
}

void simple_planner::SimplePlanner::publishCostmapDebug()
{
  if (!map_received_ || cost_map_.empty()) return;

  sensor_msgs::msg::Image img;
  img.header.frame_id = "map";
  img.header.stamp = this->now();
  img.height = map_data_.height;
  img.width = map_data_.width;
  img.encoding = "rgb8";
  img.step = img.width * 3;
  img.data.resize(img.height * img.step);

  float max_cost = 0.0;
  for (auto &row : cost_map_) {
    for (auto v : row) {
      if (v < lethal_cost_) max_cost = std::max(max_cost, v);
    }
  }
  if (max_cost <= 0.0) max_cost = 1.0;

  auto costToColor = [](float norm) -> std::array<uint8_t,3> {
    float r=0, g=0, b=0;
    if (norm < 0.25f) { r = 0; g = 4*norm; b = 1; }
    else if (norm < 0.5f) { r = 0; g = 1; b = 1 - 4*(norm-0.25f); }
    else if (norm < 0.75f) { r = 4*(norm-0.5f); g = 1; b = 0; }
    else { r = 1; g = 1 - 4*(norm-0.75f); b = 0; }
    return {static_cast<uint8_t>(r*255),
            static_cast<uint8_t>(g*255),
            static_cast<uint8_t>(b*255)};
  };

  for (int r = 0; r < map_data_.height; r++) {
    for (int c = 0; c < map_data_.width; c++) {
      float v = cost_map_[r][c];
      uint8_t R=0,G=0,B=0;
      if (v >= lethal_cost_) { R=0; G=0; B=0; }
      else {
        float norm = v / max_cost;
        auto color = costToColor(norm);
        R = color[0]; G = color[1]; B = color[2];
      }
      size_t idx = r * img.step + c*3;
      img.data[idx+0] = R;
      img.data[idx+1] = G;
      img.data[idx+2] = B;
    }
  }

  costmap_debug_pub_->publish(img);
}

void simple_planner::SimplePlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  goal_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Goal received at (x=%.2f, y=%.2f)",
              goal_pose_.pose.position.x,
              goal_pose_.pose.position.y);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "goal";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = goal_pose_.pose;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker_pub_->publish(marker);
}

void simple_planner::SimplePlanner::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  manual_start_pose_.header = msg->header;
  manual_start_pose_.pose = msg->pose.pose;
  manual_start_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Manual start pose set at (x=%.2f, y=%.2f)",
              manual_start_pose_.pose.position.x,
              manual_start_pose_.pose.position.y);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "start";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = manual_start_pose_.pose;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = rclcpp::Duration(0, 0);
  marker_pub_->publish(marker);
}


void simple_planner::SimplePlanner::timerCallback()
{
  if (use_manual_start_)
  {
    if (!manual_start_received_) {
      if (last_plan_status_ != PlanStatus::WAITING) {
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for new data (manual start not received).");
        last_plan_status_ = PlanStatus::WAITING;
      }
      robot_pose_valid_ = false;
      return;
    }
    robot_pose_ = manual_start_pose_;
    robot_pose_valid_ = true;
  }
  else
  {
    try {
      auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      robot_pose_.header = transform.header;
      robot_pose_.pose.position.x = transform.transform.translation.x;
      robot_pose_.pose.position.y = transform.transform.translation.y;
      robot_pose_.pose.position.z = transform.transform.translation.z;
      robot_pose_.pose.orientation = transform.transform.rotation;
      robot_pose_valid_ = true;
    }
    catch (const tf2::TransformException & ex) {
      if (last_plan_status_ != PlanStatus::WAITING) {
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for new data (TF unavailable): %s", ex.what());
        last_plan_status_ = PlanStatus::WAITING;
      }
      robot_pose_valid_ = false;
      return;
    }
  }

  if (!map_received_ || !goal_received_ || !robot_pose_valid_) {
    if (last_plan_status_ != PlanStatus::WAITING) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for new data (map=%d goal=%d pose=%d)...",
                  map_received_, goal_received_, robot_pose_valid_);
      last_plan_status_ = PlanStatus::WAITING;
    }
    return;
  }

  start_cell_ = worldToGrid(robot_pose_.pose.position.x,
                            robot_pose_.pose.position.y, map_data_);
  goal_cell_ = worldToGrid(goal_pose_.pose.position.x,
                           goal_pose_.pose.position.y, map_data_);

  bool map_changed   = (map_version_ != last_map_version_);
  bool start_changed = (start_cell_ != last_start_cell_);
  bool goal_changed  = (goal_cell_ != last_goal_cell_);

  if (!(map_changed || start_changed || goal_changed)) {
    if (last_plan_status_ == PlanStatus::VALID) {
      RCLCPP_WARN(this->get_logger(), "Waiting for new data...");
      last_plan_status_ = PlanStatus::WAITING;
    }
    return;
  }

  // === Controllo validità con messaggi specifici ===
  bool start_valid = isInBounds(start_cell_) && isCellFree(start_cell_);
  bool goal_valid  = isInBounds(goal_cell_)  && isCellFree(goal_cell_);

  if (!isInBounds(start_cell_) && last_start_valid_) {
    RCLCPP_WARN(this->get_logger(),
                "No valid path: START out of map bounds (%d,%d).",
                start_cell_.first, start_cell_.second);
    RCLCPP_INFO(this->get_logger(), "Please enter valid data.");
  }
  if (!isInBounds(goal_cell_) && last_goal_valid_) {
    RCLCPP_WARN(this->get_logger(),
                "No valid path: GOAL out of map bounds (%d,%d).",
                goal_cell_.first, goal_cell_.second);
    RCLCPP_INFO(this->get_logger(), "Please enter valid data.");
  }
  if (isInBounds(start_cell_) && !isCellFree(start_cell_) && last_start_valid_) {
    RCLCPP_WARN(this->get_logger(),
                "No valid path: START is in an occupied cell (%d,%d).",
                start_cell_.first, start_cell_.second);
    RCLCPP_INFO(this->get_logger(), "Please enter valid data.");
  }
  if (isInBounds(goal_cell_) && !isCellFree(goal_cell_) && last_goal_valid_) {
    RCLCPP_WARN(this->get_logger(),
                "No valid path: GOAL is in an occupied cell (%d,%d).",
                goal_cell_.first, goal_cell_.second);
    RCLCPP_INFO(this->get_logger(), "Please enter valid data.");
  }

  // Aggiorno flag validità
  last_start_valid_ = start_valid;
  last_goal_valid_  = goal_valid;

  if (start_valid && goal_valid && start_cell_ == goal_cell_) {
    RCLCPP_WARN(this->get_logger(),
                "No valid path: START and GOAL are the same cell (%d,%d).",
                start_cell_.first, start_cell_.second);
    RCLCPP_INFO(this->get_logger(), "Please enter valid data.");
    publishPath({});
    last_plan_status_ = PlanStatus::INVALID;
    return;
  }

  if (!start_valid || !goal_valid) {
    publishPath({});
    last_plan_status_ = PlanStatus::INVALID;
    return;
  }

  // === Se valido → pianifico ===
  planOnce();
  last_map_version_ = map_version_;
  last_start_cell_  = start_cell_;
  last_goal_cell_   = goal_cell_;
}


void simple_planner::SimplePlanner::publishPath(const std::vector<std::pair<int,int>>& path_cells)
{
  if (!map_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish path: no map received yet.");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();

  for (const auto& cell : path_cells) {
    int row = cell.first;
    int col = cell.second;
    auto pose = gridToWorld(row, col, map_data_);
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);

  if (!path_cells.empty()) {
    RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_msg.poses.size());
  }
}

void simple_planner::SimplePlanner::planOnce()
{
  RCLCPP_INFO(this->get_logger(),
              "Planning triggered: start=(%d,%d), goal=(%d,%d)",
              start_cell_.first, start_cell_.second,
              goal_cell_.first, goal_cell_.second);

  bool use_diagonals = true;
  float lambda_weight = 2.0f;

  auto path_cells = simple_planner::a_star::planPath(
      start_cell_, goal_cell_,
      occupancy_grid_,
      cost_map_,
      map_data_,
      use_diagonals,
      lambda_weight);

  if (path_cells.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid path found: A* failed.");
    RCLCPP_INFO(this->get_logger(), "Please enter valid data.");
    publishPath({});
    last_plan_status_ = PlanStatus::INVALID;
    return;
  }

  // Calcolo costo totale e medio
  double total_cost = 0.0;
  for (const auto& cell : path_cells) {
    int r = cell.first;
    int c = cell.second;
    if (isInBounds(cell)) {
      total_cost += cost_map_[r][c];
    }
  }
  double mean_cost = total_cost / path_cells.size();

  RCLCPP_INFO(this->get_logger(),
              "A* found path with %zu cells, total cost=%.2f, mean cost=%.2f",
              path_cells.size(), total_cost, mean_cost);

  publishPath(path_cells);
  last_plan_status_ = PlanStatus::VALID;
}

bool simple_planner::SimplePlanner::isInBounds(const std::pair<int,int>& cell) const
{
  return cell.first >= 0 && cell.first < map_data_.height &&
         cell.second >= 0 && cell.second < map_data_.width;
}

bool simple_planner::SimplePlanner::isCellFree(const std::pair<int,int>& cell) const
{
  int r = cell.first;
  int c = cell.second;

  if (r < 0 || r >= static_cast<int>(occupancy_grid_.size())) return false;
  if (c < 0 || c >= static_cast<int>(occupancy_grid_[0].size())) return false;

  int8_t v = occupancy_grid_[r][c];
  return (v == 0);
}


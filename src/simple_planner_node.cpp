// Integration with additional checks 
#include "simple_planner/simple_planner.hpp"
#include "simple_planner/map_utils.hpp"

using std::placeholders::_1;
using namespace simple_planner::map_utils;

SimplePlanner::SimplePlanner()
: rclcpp::Node("simple_planner")
{
  RCLCPP_INFO(this->get_logger(), "Simple Planner Node initialized!");

  // === Parametri ===
  this->declare_parameter("use_manual_start", false);
  this->get_parameter("use_manual_start", use_manual_start_);

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

  // === TF2 ===
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // === Timer ===
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&SimplePlanner::timerCallback, this));
}

void SimplePlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_data_ = parseOccupancyGrid(*msg);

  // Controlli di validità mappa
  if (map_data_.width <= 0 || map_data_.height <= 0 || map_data_.resolution <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid map metadata (width/height/resolution). Ignoring map.");
    map_received_ = false;
    return;
  }

  if (map_data_.data.size() != static_cast<size_t>(map_data_.width * map_data_.height)) {
    RCLCPP_ERROR(this->get_logger(), "Map data size mismatch: expected %d, got %zu",
                 map_data_.width * map_data_.height, map_data_.data.size());
    map_received_ = false;
    return;
  }

  occupancy_grid_ = to2DGrid(map_data_, true);  // true = treat unknown as obstacle
  map_received_ = true;
  map_version_++;  // incrementa versione mappa

  RCLCPP_INFO(this->get_logger(),
              "Map received: %d x %d @ %.3f m resolution (2D grid built)",
              map_data_.width, map_data_.height, map_data_.resolution);
}

void SimplePlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  goal_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Goal received at (x=%.2f, y=%.2f)",
              goal_pose_.pose.position.x,
              goal_pose_.pose.position.y);

  // Marker rosso per il goal
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

  marker.lifetime = rclcpp::Duration(0, 0);  // permanente
  marker_pub_->publish(marker);
}

void SimplePlanner::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  manual_start_pose_.header = msg->header;
  manual_start_pose_.pose = msg->pose.pose;
  manual_start_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Manual start pose set at (x=%.2f, y=%.2f)",
              manual_start_pose_.pose.position.x,
              manual_start_pose_.pose.position.y);

  // Marker verde per lo start
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

  marker.lifetime = rclcpp::Duration(0, 0);  // permanente
  marker_pub_->publish(marker);
}


void SimplePlanner::timerCallback()
{
  if (use_manual_start_)
  {
    if (!manual_start_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Manual start enabled but not yet received. Waiting...");
      robot_pose_valid_ = false;
      return;
    }

    robot_pose_ = manual_start_pose_;
    robot_pose_valid_ = true;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Using manual start pose at (x=%.2f, y=%.2f)",
      robot_pose_.pose.position.x,
      robot_pose_.pose.position.y);
  }
  else
  {
    // Altrimenti: usa TF
    try
    {
      auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

      robot_pose_.header = transform.header;
      robot_pose_.pose.position.x = transform.transform.translation.x;
      robot_pose_.pose.position.y = transform.transform.translation.y;
      robot_pose_.pose.position.z = transform.transform.translation.z;
      robot_pose_.pose.orientation = transform.transform.rotation;

      robot_pose_valid_ = true;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "TF robot pose: (x=%.2f, y=%.2f)",
        robot_pose_.pose.position.x,
        robot_pose_.pose.position.y);
    }
    catch (const tf2::TransformException & ex)
    {
      robot_pose_valid_ = false;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Could not transform 'map' -> 'base_link': %s", ex.what());
    }
  }

  // === Step 4: orchestrazione pianificazione ===
  if (!map_received_ || !goal_received_ || !robot_pose_valid_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for all data (map=%d goal=%d pose=%d)...",
                         map_received_, goal_received_, robot_pose_valid_);
    return;
  }

  // Conversione world -> grid
  start_cell_ = worldToGrid(robot_pose_.pose.position.x,
                            robot_pose_.pose.position.y,
                            map_data_);
  goal_cell_ = worldToGrid(goal_pose_.pose.position.x,
                           goal_pose_.pose.position.y,
                           map_data_);

  // === Debug: log valori grezzi della mappa ===
  auto debugCell = [&](const std::pair<int,int>& cell, const std::string& label) {
    int row = cell.first;
    int col = cell.second;
    int index = row * map_data_.width + col;

    int8_t value = -99;
    if (index >= 0 && index < static_cast<int>(map_data_.data.size())) {
      value = map_data_.data[index];
    }

    RCLCPP_INFO(this->get_logger(),
                "[DEBUG] %s cell=(%d,%d) index=%d value=%d",
                label.c_str(), row, col, index, value);
  };

  debugCell(start_cell_, "START");
  debugCell(goal_cell_, "GOAL");

  // === Validazioni base (post-Step 4) ===
  if (!isInBounds(start_cell_) || !isInBounds(goal_cell_)) {
    RCLCPP_WARN(this->get_logger(),
                "Start or goal out of map bounds. Skipping planning. start=(%d,%d) goal=(%d,%d)",
                start_cell_.first, start_cell_.second,
                goal_cell_.first, goal_cell_.second);
    publishPath({});
    return;
  }

  if (!isCellFree(start_cell_) || !isCellFree(goal_cell_)) {
    RCLCPP_WARN(this->get_logger(),
                "Start or goal is in an occupied/unknown cell. Skipping planning.");
    publishPath({});
    return;
  }

  // ⚠️ ADESSO QUI: check start==goal sempre valutato
  if (start_cell_ == goal_cell_) {
    RCLCPP_WARN(this->get_logger(),
                "Start and goal are the same cell (%d,%d). Skipping planning.",
                start_cell_.first, start_cell_.second);
    publishPath({});
    return;
  }

  // Change detection
  bool map_changed   = (map_version_ != last_map_version_);
  bool start_changed = (start_cell_ != last_start_cell_);
  bool goal_changed  = (goal_cell_ != last_goal_cell_);

  if (!(map_changed || start_changed || goal_changed)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No changes detected, skipping planning.");
    return;
  }

  // Trigger planning
  planOnce();

  // Aggiorna riferimenti
  last_map_version_ = map_version_;
  last_start_cell_  = start_cell_;
  last_goal_cell_   = goal_cell_;
}



// === Step 3.3: funzione per pubblicare un Path ===
void SimplePlanner::publishPath(const std::vector<std::pair<int,int>>& path_cells)
{
  if (!map_received_) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish path: no map received yet.");
    return;
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();

  for (const auto& cell : path_cells)
  {
    int row = cell.first;
    int col = cell.second;

    auto pose = gridToWorld(row, col, map_data_);
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);

  RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", path_msg.poses.size());
}

// === Step 4: funzione placeholder di planning ===
void SimplePlanner::planOnce()
{
  RCLCPP_INFO(this->get_logger(),
              "Planning triggered: start=(%d,%d), goal=(%d,%d)",
              start_cell_.first, start_cell_.second,
              goal_cell_.first, goal_cell_.second);

  // Placeholder: per ora pubblica un path dummy tra start e goal
  std::vector<std::pair<int,int>> dummy_path;
  dummy_path.push_back(start_cell_);
  dummy_path.push_back(goal_cell_);
  publishPath(dummy_path);
}

// === Helper per validazione celle ===
bool SimplePlanner::isInBounds(const std::pair<int,int>& cell) const
{
  return cell.first >= 0 && cell.first < map_data_.height &&
         cell.second >= 0 && cell.second < map_data_.width;
}

bool SimplePlanner::isCellFree(const std::pair<int,int>& cell) const
{
  int row = cell.first;
  int col = cell.second;
  int index = row * map_data_.width + col;

  if (index < 0 || index >= static_cast<int>(map_data_.data.size())) {
    return false;  // out of range = invalid
  }

  int8_t value = map_data_.data[index];
  return (value == 0);  // 0 = free, treat -1 and 100 as occupied
}



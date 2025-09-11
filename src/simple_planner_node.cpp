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
  map_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Map received: %d x %d @ %.3f m resolution",
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
    return;
  }

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


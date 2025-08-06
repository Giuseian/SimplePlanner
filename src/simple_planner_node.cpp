#include "simple_planner/simple_planner.hpp"
#include "simple_planner/map_utils.hpp"  // Include parser

using std::placeholders::_1;
using namespace simple_planner::map_utils;

SimplePlanner::SimplePlanner()
: rclcpp::Node("simple_planner")
{
  RCLCPP_INFO(this->get_logger(), "Simple Planner Node initialized!");

  // 2.1 
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SimplePlanner::mapCallback, this, _1));
  
  // 2.2 
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",  // or "/move_base_simple/goal" if you're using that
    rclcpp::QoS(10),
    std::bind(&SimplePlanner::goalCallback, this, _1));

  // 2.3 
  // Create the TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a periodic timer to check robot pose
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // Every 500ms
    std::bind(&SimplePlanner::timerCallback, this));  

}

void SimplePlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // RCLCPP_WARN(this->get_logger(), "ENTERED mapCallback");
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
}


void SimplePlanner::timerCallback()
{
  geometry_msgs::msg::TransformStamped transform;

  try
  {
    // Try to get the latest transform from map -> base_link
    transform = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero);

    // Convert to PoseStamped
    robot_pose_.header = transform.header;
    robot_pose_.pose.position.x = transform.transform.translation.x;
    robot_pose_.pose.position.y = transform.transform.translation.y;
    robot_pose_.pose.position.z = transform.transform.translation.z;
    robot_pose_.pose.orientation = transform.transform.rotation;

    robot_pose_valid_ = true;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,  // 2s throttle
      "Robot pose: (x=%.2f, y=%.2f)",
      robot_pose_.pose.position.x,
      robot_pose_.pose.position.y);
  }
  catch (const tf2::TransformException & ex)
  {
    robot_pose_valid_ = false;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,  // Warn every 5s
      "Could not transform 'map' -> 'base_link': %s", ex.what());
  }
}

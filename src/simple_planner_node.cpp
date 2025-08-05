#include "simple_planner/simple_planner.hpp"
#include "simple_planner/map_utils.hpp"  // Include parser

using std::placeholders::_1;
using namespace simple_planner::map_utils;

SimplePlanner::SimplePlanner()
: rclcpp::Node("simple_planner")
{
  RCLCPP_INFO(this->get_logger(), "Simple Planner Node initialized!");

  // Initialize map subscriber
//   map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//     "/map",
//     rclcpp::QoS(10),
//     std::bind(&SimplePlanner::mapCallback, this, _1));
// }

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SimplePlanner::mapCallback, this, _1));
}

void SimplePlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_WARN(this->get_logger(), "ENTERED mapCallback");
  map_data_ = parseOccupancyGrid(*msg);
  map_received_ = true;

  RCLCPP_INFO(this->get_logger(),
              "Map received: %d x %d @ %.3f m resolution",
              map_data_.width, map_data_.height, map_data_.resolution);
}

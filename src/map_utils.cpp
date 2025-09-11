// #include "simple_planner/map_utils.hpp"

// namespace simple_planner::map_utils
// {

// MapData parseOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg)
// {
//   MapData map_data;
//   map_data.resolution = msg.info.resolution;
//   map_data.width = msg.info.width;
//   map_data.height = msg.info.height;
//   map_data.origin = msg.info.origin;
//   map_data.data = msg.data;

//   return map_data;
// }

// } // namespace simple_planner::map_utils



#include "simple_planner/map_utils.hpp"
#include "rclcpp/rclcpp.hpp"  // per timestamp

namespace simple_planner::map_utils
{

MapData parseOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg)
{
  MapData map_data;
  map_data.resolution = msg.info.resolution;
  map_data.width = msg.info.width;
  map_data.height = msg.info.height;
  map_data.origin = msg.info.origin;
  map_data.data = msg.data;

  return map_data;
}

std::pair<int, int> worldToGrid(double x, double y, const MapData & map_data)
{
  int col = static_cast<int>(std::floor((x - map_data.origin.position.x) / map_data.resolution));
  int row = static_cast<int>(std::floor((y - map_data.origin.position.y) / map_data.resolution));
  return {row, col};
}

geometry_msgs::msg::PoseStamped gridToWorld(int row, int col, const MapData & map_data)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = rclcpp::Clock().now();

  pose.pose.position.x = map_data.origin.position.x + (col + 0.5) * map_data.resolution;
  pose.pose.position.y = map_data.origin.position.y + (row + 0.5) * map_data.resolution;
  pose.pose.position.z = 0.0;

  // Orientazione neutra
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  return pose;
}

std::vector<std::vector<int8_t>> to2DGrid(const MapData & map_data, bool treat_unknown_as_obstacle)
{
  std::vector<std::vector<int8_t>> grid(map_data.height, std::vector<int8_t>(map_data.width, 0));

  for (int row = 0; row < map_data.height; ++row)
  {
    for (int col = 0; col < map_data.width; ++col)
    {
      int index = row * map_data.width + col;
      int8_t value = map_data.data[index];

      if (value == -1 && treat_unknown_as_obstacle) {
        value = 100;  // trattiamo unknown come ostacolo
      }
      grid[row][col] = value;
    }
  }

  return grid;
}

} // namespace simple_planner::map_utils

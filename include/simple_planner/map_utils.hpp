#ifndef SIMPLE_PLANNER_MAP_UTILS_HPP
#define SIMPLE_PLANNER_MAP_UTILS_HPP

#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"  
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace simple_planner::map_utils
{

struct MapData
{
  float resolution;
  int width;
  int height;
  geometry_msgs::msg::Pose origin;
  std::vector<int8_t> data;
};

MapData parseOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);

} // namespace simple_planner::map_utils

#endif

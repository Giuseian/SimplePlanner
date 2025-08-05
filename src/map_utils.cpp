#include "simple_planner/map_utils.hpp"

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

} // namespace simple_planner::map_utils

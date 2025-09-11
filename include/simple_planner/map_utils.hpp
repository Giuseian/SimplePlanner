// Up to STEP 2 - kind of working
// #ifndef SIMPLE_PLANNER_MAP_UTILS_HPP
// #define SIMPLE_PLANNER_MAP_UTILS_HPP

// #include <vector>
// #include "geometry_msgs/msg/pose_stamped.hpp"  
// #include <nav_msgs/msg/occupancy_grid.hpp>

// namespace simple_planner::map_utils
// {

// struct MapData
// {
//   float resolution;
//   int width;
//   int height;
//   geometry_msgs::msg::Pose origin;
//   std::vector<int8_t> data;
// };

// MapData parseOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);

// } // namespace simple_planner::map_utils

// #endif


// Integration of STEP 3
#ifndef SIMPLE_PLANNER_MAP_UTILS_HPP
#define SIMPLE_PLANNER_MAP_UTILS_HPP

#include <vector>
#include <utility>  // per std::pair
#include <cmath>

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

// Estrarre i dati della mappa dal messaggio OccupancyGrid
MapData parseOccupancyGrid(const nav_msgs::msg::OccupancyGrid & msg);

// Conversione world → grid (ritorna riga e colonna della cella)
std::pair<int, int> worldToGrid(double x, double y, const MapData & map_data);

// Conversione grid → world (ritorna PoseStamped con coordinate del centro cella)
geometry_msgs::msg::PoseStamped gridToWorld(int row, int col, const MapData & map_data);

// Conversione OccupancyGrid (1D) → matrice 2D
std::vector<std::vector<int8_t>> to2DGrid(const MapData & map_data, bool treat_unknown_as_obstacle = true);

} // namespace simple_planner::map_utils

#endif

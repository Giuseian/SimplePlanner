#include "simple_planner/cost_utils.hpp"

#include <limits>
#include <queue>
#include <cmath>

namespace simple_planner {
namespace cost_utils {

Grid2DFloat computeDistanceMap(
    const Grid2DInt& occupancy_grid,
    const map_utils::MapData& map_data)
{
  int H = map_data.height;
  int Wd = map_data.width;
  float res = map_data.resolution;

  Grid2DFloat dist(H, std::vector<float>(Wd, std::numeric_limits<float>::infinity()));

  std::queue<std::pair<int,int>> q;

  // inizializza: celle occupate/unknown → distanza 0
  for (int r = 0; r < H; r++) {
    for (int c = 0; c < Wd; c++) {
      if (occupancy_grid[r][c] != 0) { // ostacolo o unknown
        dist[r][c] = 0.0f;
        q.push({r,c});
      }
    }
  }

  // BFS multi-source (8 vicini)
  std::vector<std::pair<int,int>> dirs = {
    {1,0},{-1,0},{0,1},{0,-1},
    {1,1},{1,-1},{-1,1},{-1,-1}
  };

  while (!q.empty()) {
    auto [r,c] = q.front(); q.pop();
    for (auto [dr,dc] : dirs) {
      int nr = r+dr, nc = c+dc;
      if (nr < 0 || nr >= H || nc < 0 || nc >= Wd) continue;

      float step = (dr==0 || dc==0) ? res : res * std::sqrt(2.0);
      float ndist = dist[r][c] + step;
      if (ndist < dist[nr][nc]) {
        dist[nr][nc] = ndist;
        q.push({nr,nc});
      }
    }
  }

  return dist;
}

Grid2DFloat computeCostMap(
    const Grid2DFloat& distance_map,
    const Grid2DInt& occupancy_grid,
    const map_utils::MapData& map_data,
    const std::string& cost_function_type,
    float inflation_radius,
    float alpha,
    float W,
    float epsilon,
    float lethal_cost)
{
  int H = map_data.height;
  int Wd = map_data.width;
  Grid2DFloat cost(H, std::vector<float>(Wd, 0.0f));

  for (int r = 0; r < H; r++) {
    for (int c = 0; c < Wd; c++) {
      if (occupancy_grid[r][c] != 0) {
        cost[r][c] = lethal_cost;
        continue;
      }

      float d = distance_map[r][c];
      if (d <= 0.0f) {
        cost[r][c] = lethal_cost;
        continue;
      }

      if (cost_function_type == "exponential") {
        if (d < inflation_radius) {
          cost[r][c] = W * std::exp(-alpha * d);
        } else {
          cost[r][c] = 0.0f;
        }
      }
      else if (cost_function_type == "linear") {
        if (d < inflation_radius) {
          cost[r][c] = W * (1.0f - d/inflation_radius);
        } else {
          cost[r][c] = 0.0f;
        }
      }
      else if (cost_function_type == "inverse") {
        cost[r][c] = W / (d + epsilon);
      }
      else {
        // default: costo binario (solo per sicurezza)
        cost[r][c] = 1.0f;
      }
    }
  }

  return cost;
}

} // namespace cost_utils
} // namespace simple_planner

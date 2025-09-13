#ifndef SIMPLE_PLANNER_A_STAR_HPP
#define SIMPLE_PLANNER_A_STAR_HPP

#include <vector>
#include <utility>
#include <limits>
#include <queue>
#include <cmath>
#include <cstdint>
#include <functional>

#include "simple_planner/map_utils.hpp"  // per MapData (width/height/resolution)

namespace simple_planner {
namespace a_star {

using Grid2DInt   = std::vector<std::vector<int8_t>>;   // 0=free, !=0=blocked/unknown
using Grid2DFloat = std::vector<std::vector<float>>;    // cost map (>=0; “lethal” gestito a monte)

/**
 * Pianifica un path tra start e goal su griglia con A* (8-neighbors).
 *
 * @param start_cell     (row, col) cella di partenza (indici griglia)
 * @param goal_cell      (row, col) cella di arrivo (indici griglia)
 * @param occupancy      griglia occupazione: 0=libero, !=0=non attraversabile
 * @param cost_map       mappa dei costi (stessa dimensione della mappa)
 * @param map            metadati mappa (width, height, resolution)
 * @param use_diagonals  se true usa 8-vicini con “corner check” per diagonali
 * @param lambda_weight  peso da dare ai costi della cost_map nel g-cost
 *
 * @return vettore di celle (row,col) dal start al goal (inclusi). Vuoto se fallisce.
 */
std::vector<std::pair<int,int>> planPath(
  const std::pair<int,int>& start_cell,
  const std::pair<int,int>& goal_cell,
  const Grid2DInt&          occupancy,
  const Grid2DFloat&        cost_map,
  const map_utils::MapData& map,
  bool                      use_diagonals = true,
  float                     lambda_weight = 2.0f);

} // namespace a_star
} // namespace simple_planner

#endif // SIMPLE_PLANNER_A_STAR_HPP

#ifndef COST_UTILS_HPP
#define COST_UTILS_HPP

#include <vector>
#include <string>
#include <limits>
#include <queue>
#include <cmath>
#include "simple_planner/map_utils.hpp"

namespace simple_planner {
namespace cost_utils {

using Grid2DInt = std::vector<std::vector<int8_t>>;
using Grid2DFloat = std::vector<std::vector<float>>;

/// Calcola la distance map (in metri) dagli ostacoli usando BFS multi-source.
/// Celle occupate/unknown → distanza 0.
/// Celle libere → distanza al più vicino ostacolo.
Grid2DFloat computeDistanceMap(
    const Grid2DInt& occupancy_grid,
    const map_utils::MapData& map_data);

/// Genera la cost map a partire dalla distance map e dalla funzione scelta.
/// @param cost_function_type "exponential", "linear", "inverse"
Grid2DFloat computeCostMap(
    const Grid2DFloat& distance_map,
    const Grid2DInt& occupancy_grid,
    const map_utils::MapData& map_data,
    const std::string& cost_function_type,
    float inflation_radius,
    float alpha,
    float W,
    float epsilon,
    float lethal_cost);

}  // namespace cost_utils
}  // namespace simple_planner

#endif  // COST_UTILS_HPP

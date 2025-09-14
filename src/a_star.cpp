#include "simple_planner/a_star.hpp"

#include <limits>
#include <algorithm>
#include <queue>
#include <cmath>

namespace simple_planner {
namespace a_star {

struct Node {
  int r, c;
  float g;   // Costo accumulato
  float f;   // g + h
};

// Comparatore per priority_queue (min-heap su f)
struct NodeGreater {
  bool operator()(const Node& a, const Node& b) const {
    if (a.f == b.f) return a.g > b.g;  // Tie-break: preferisci g minore
    return a.f > b.f;
  }
};

// Controllo limiti mappa
static inline bool inBounds(int r, int c, const map_utils::MapData& map) {
  return (r >= 0 && r < map.height && c >= 0 && c < map.width);
}

// Euristica: distanza euclidea in metri
static inline float heuristicEuclideanM(
    int r1, int c1, int r2, int c2, float res_m)
{
  const float dr = static_cast<float>(r2 - r1) * res_m;
  const float dc = static_cast<float>(c2 - c1) * res_m;
  return std::sqrt(dr * dr + dc * dc);
}

// Evita “corner cutting”: per mosse diagonali richiede che le celle ortogonali siano libere
static inline bool diagonalPassable(
    int r, int c, int nr, int nc,
    const Grid2DInt& occ)
{
  const int dr = nr - r;
  const int dc = nc - c;

  // Se non è mossa diagonale, ok
  if (std::abs(dr) + std::abs(dc) != 2) return true;

  // Celle ortogonali adiacenti al corner
  int r1 = r;
  int c1 = nc;
  int r2 = nr;
  int c2 = c;

  // Politica conservativa: se uno dei due è ostacolo, vieta la diagonale
  if (occ[r1][c1] != 0) return false;
  if (occ[r2][c2] != 0) return false;
  return true;
}

std::vector<std::pair<int,int>> planPath(
  const std::pair<int,int>& start_cell,
  const std::pair<int,int>& goal_cell,
  const Grid2DInt&          occupancy,
  const Grid2DFloat&        cost_map,
  const map_utils::MapData& map,
  bool                      use_diagonals,
  float                     lambda_weight)
{
  std::vector<std::pair<int,int>> empty;

  // Validazioni iniziali
  if (!inBounds(start_cell.first, start_cell.second, map) ||
      !inBounds(goal_cell.first,  goal_cell.second,  map))
    return empty;

  if (occupancy[start_cell.first][start_cell.second] != 0 ||
      occupancy[goal_cell.first][goal_cell.second]   != 0)
    return empty;

  if (start_cell == goal_cell) {
    // Path banale (solo la cella di partenza/arrivo)
    return { start_cell };
  }

  const int H = map.height;
  const int W = map.width;
  const float res = map.resolution;

  // 8-neighbors e 4-neighbors
  static const int DR8[8] = { -1,-1,-1, 0, 0, 1, 1, 1 };
  static const int DC8[8] = { -1, 0, 1,-1, 1,-1, 0, 1 };

  static const int DR4[4] = { -1, 1, 0, 0 };
  static const int DC4[4] = {  0, 0,-1, 1 };

  const int neigh_count = use_diagonals ? 8 : 4;

  // Inizializzazione strutture
  std::vector<std::vector<float>> g_cost(H, std::vector<float>(W, std::numeric_limits<float>::infinity()));
  std::vector<std::vector<std::pair<int,int>>> parent(H, std::vector<std::pair<int,int>>(W, {-1,-1}));
  std::vector<std::vector<bool>> closed(H, std::vector<bool>(W, false));
  std::priority_queue<Node, std::vector<Node>, NodeGreater> open;

  // Nodo iniziale
  g_cost[start_cell.first][start_cell.second] = 0.0f;
  const float h0 = heuristicEuclideanM(start_cell.first, start_cell.second,
                                       goal_cell.first,  goal_cell.second, res);
  open.push(Node{ start_cell.first, start_cell.second, 0.0f, h0 });

  // Ciclo principale A*
  while (!open.empty()) {
    Node cur = open.top(); 
    open.pop();

    if (closed[cur.r][cur.c]) continue;
    closed[cur.r][cur.c] = true;

    // Goal raggiunto
    if (cur.r == goal_cell.first && cur.c == goal_cell.second) {
      std::vector<std::pair<int,int>> path;
      int r = cur.r, c = cur.c;
      path.emplace_back(r, c);
      while (!(r == start_cell.first && c == start_cell.second)) {
        auto p = parent[r][c];
        if (p.first < 0) break;  // Controllo di sicurezza
        r = p.first; 
        c = p.second;
        path.emplace_back(r, c);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Espansione vicini
    for (int k = 0; k < neigh_count; ++k) {
      const int dr = use_diagonals ? DR8[k] : DR4[k];
      const int dc = use_diagonals ? DC8[k] : DC4[k];
      const int nr = cur.r + dr;
      const int nc = cur.c + dc;

      if (!inBounds(nr, nc, map)) continue;
      if (closed[nr][nc]) continue;
      if (occupancy[nr][nc] != 0) continue;  // Ostacolo/unknown

      if (use_diagonals && !diagonalPassable(cur.r, cur.c, nr, nc, occupancy)) {
        continue;
      }

      const bool is_diag = (dr != 0 && dc != 0);
      const float step_cost = is_diag ? (res * std::sqrt(2.0f)) : res;

      const float cell_cost = (nr >= 0 && nr < H && nc >= 0 && nc < W)
                                ? cost_map[nr][nc] : 0.0f;

      const float tentative_g = g_cost[cur.r][cur.c] + step_cost + lambda_weight * cell_cost;

      if (tentative_g < g_cost[nr][nc]) {
        g_cost[nr][nc] = tentative_g;
        parent[nr][nc] = {cur.r, cur.c};
        const float h = heuristicEuclideanM(nr, nc, goal_cell.first, goal_cell.second, res);
        const float f = tentative_g + h;
        open.push(Node{ nr, nc, tentative_g, f });
      }
    }
  }

  // Nessun path trovato
  return empty;
}

} // namespace a_star
} // namespace simple_planner

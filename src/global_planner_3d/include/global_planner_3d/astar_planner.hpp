#pragma once
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include "planner_utils.hpp"

/**
 * @brief 3D A* path planner for grid-based environments
 *
 * Implements a standard A* search algorithm on a 3D occupancy grid.
 * The planner considers 26-connected neighbors (dx, dy, dz ∈ [-1, 1])
 * and avoids occupied or out-of-bounds cells.
 *
 * Output path is returned as a sequence of GridIndex values.
 */
class AStarPlanner
{
public:
  /**
   * @brief Run A* planning on a 3D grid
   *
   * @param grid  Occupancy grid
   * @param start Start grid index
   * @param goal  Goal grid index
   * @param path  Output path (grid indices)
   * @return true if a valid path is found
   */
  bool plan(const Grid3D &grid,
            const GridIndex &start,
            const GridIndex &goal,
            std::vector<GridIndex> &path)
  {
    // Clear previous path
    path.clear();

    // Trivial case: start == goal
    if (start == goal) {
      path.push_back(start);
      return true;
    }

    // Convert 3D grid index to unique linear key
    auto key = [&](const GridIndex &i) -> int64_t {
      return i.x + grid.size_x * (i.y + grid.size_y * i.z);
    };

    // Internal A* node
    struct Node {
      GridIndex idx;  // grid position
      double g;       // cost from start
      double f;       // total cost (g + heuristic)
    };

    // Priority queue sorted by lowest f-cost
    auto cmp = [](const Node &a, const Node &b) {
      return a.f > b.f;
    };

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

    // Cost-to-come for each visited node
    std::unordered_map<int64_t, double> gscore;

    // Parent map for path reconstruction
    std::unordered_map<int64_t, GridIndex> parent;

    // Closed set (already expanded nodes)
    std::unordered_set<int64_t> closed;

    // Initialize start node
    open.push({start, 0.0, heuristic(start, goal)});
    gscore[key(start)] = 0.0;

    // Main A* loop
    while (!open.empty())
    {
      Node cur = open.top();
      open.pop();

      int64_t ck = key(cur.idx);

      // Skip if already expanded
      if (closed.count(ck)) continue;
      closed.insert(ck);

      // Goal reached
      if (cur.idx == goal) {
        reconstructPath(start, goal, parent, grid, path);
        return true;
      }

      // Explore 26-connected neighbors
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
          for (int dz = -1; dz <= 1; dz++)
          {
            // Skip self
            if (dx == 0 && dy == 0 && dz == 0) continue;

            GridIndex n{
              cur.idx.x + dx,
              cur.idx.y + dy,
              cur.idx.z + dz
            };

            // Bounds and collision check
            if (!grid.inBounds(n.x, n.y, n.z)) continue;
            if (grid.isOccupied(n.x, n.y, n.z)) continue;

            int64_t nk = key(n);
            if (closed.count(nk)) continue;

            // Euclidean step cost
            double step = std::sqrt(dx*dx + dy*dy + dz*dz);
            double tentative_g = cur.g + step;

            // Relax edge
            if (!gscore.count(nk) || tentative_g < gscore[nk])
            {
              gscore[nk] = tentative_g;
              parent[nk] = cur.idx;
              open.push({
                n,
                tentative_g,
                tentative_g + heuristic(n, goal)
              });
            }
          }
    }

    // No path found
    return false;
  }

private:
  /**
   * @brief Euclidean distance heuristic
   */
  static double heuristic(const GridIndex &a, const GridIndex &b)
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  /**
   * @brief Reconstruct path from goal to start using parent map
   */
  static void reconstructPath(const GridIndex &start,
                              const GridIndex &goal,
                              const std::unordered_map<int64_t, GridIndex> &parent,
                              const Grid3D &grid,
                              std::vector<GridIndex> &path)
  {
    auto key = [&](const GridIndex &i) -> int64_t {
      return i.x + grid.size_x * (i.y + grid.size_y * i.z);
    };

    GridIndex cur = goal;

    // Backtrack from goal to start
    while (!(cur == start)) {
      path.push_back(cur);
      cur = parent.at(key(cur));
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());
  }
};
#pragma once
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include "grid3d.hpp"

/**
 * @brief Discrete 3D grid index
 *
 * Represents a cell index in the 3D occupancy grid.
 */
struct GridIndex
{
  int x, y, z;

  /// Equality operator
  bool operator==(const GridIndex& o) const
  {
    return x == o.x && y == o.y && z == o.z;
  }
};

/**
 * @brief Euclidean distance heuristic in grid space
 *
 * Used by A* planner to estimate distance between two grid cells.
 */
inline double heuristic(const GridIndex& a, const GridIndex& b)
{
  return std::sqrt(
    (a.x - b.x) * (a.x - b.x) +
    (a.y - b.y) * (a.y - b.y) +
    (a.z - b.z) * (a.z - b.z));
}

/**
 * @brief Convert world coordinates to grid indices
 *
 * Maps a 3D point in world frame into the corresponding grid cell.
 */
inline GridIndex worldToGrid(const geometry_msgs::msg::Point& p,
                             const Grid3D& g)
{
  return {
    int((p.x - g.origin_x) / g.resolution),
    int((p.y - g.origin_y) / g.resolution),
    int((p.z - g.origin_z) / g.resolution)
  };
}

/**
 * @brief Direction change metric for path simplification
 *
 * Computes how much the direction changes at point p1
 * using three consecutive points (p0 -> p1 -> p2).
 *
 * Used to extract key waypoints where the path bends.
 */
inline double directionChangeMetric(
  const geometry_msgs::msg::Point& p0,
  const geometry_msgs::msg::Point& p1,
  const geometry_msgs::msg::Point& p2)
{
  // Vector p0 -> p1
  double ax = p1.x - p0.x;
  double ay = p1.y - p0.y;
  double az = p1.z - p0.z;

  // Vector p1 -> p2
  double bx = p2.x - p1.x;
  double by = p2.y - p1.y;
  double bz = p2.z - p1.z;

  // Difference between consecutive direction vectors
  double dx = ax - bx;
  double dy = ay - by;
  double dz = az - bz;

  // L1 norm of direction change
  return std::abs(dx) + std::abs(dy) + std::abs(dz);
}
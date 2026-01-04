#pragma once
#include <vector>


/**
 * @brief Simple 3D occupancy grid structure
 *
 * - Fixed-resolution 3D grid representation
 * - Each cell is either free (0) or occupied (1)
 * - Designed for grid-based planners such as A*
 *
 * Grid indices (x, y, z) can be converted to world coordinates.
 */
struct Grid3D
{
  // Grid resolution in meters per cell
  double resolution = 0.2;

  // World-frame origin of the grid
  double origin_x = -50.0, origin_y = -50.0, origin_z = 0.0;

  // Grid dimensions (number of cells)
  int size_x = 500, size_y = 500, size_z = 25;

  // Occupancy data (0: free, 1: occupied)
  std::vector<uint8_t> data;

  // Initialize an empty grid
  Grid3D() { data.resize(size_x * size_y * size_z, 0); }

  // Convert 3D grid index to 1D array index
  inline int idx(int x,int y,int z) const
  { return x + size_x * (y + size_y * z); }

  // Check if a cell is within grid bounds
  inline bool inBounds(int x,int y,int z) const
  { return x>=0&&x<size_x&&y>=0&&y<size_y&&z>=0&&z<size_z; }

  // Mark a cell as occupied
  inline void setOccupied(int x,int y,int z)
  { if(inBounds(x,y,z)) data[idx(x,y,z)] = 1; }

  // Check if a cell is occupied
  inline bool isOccupied(int x,int y,int z) const
  { return !inBounds(x,y,z) || data[idx(x,y,z)]==1; }

  // Convert grid indices to world coordinates
  inline void gridToWorld(int gx, int gy, int gz,
                          double &wx, double &wy, double &wz) const
  {
    wx = origin_x + (gx + 0.5) * resolution;
    wy = origin_y + (gy + 0.5) * resolution;
    wz = origin_z + (gz + 0.5) * resolution;
  }

  // Clear the grid (set all cells to free)
  inline void clear()
  {
    std::fill(data.begin(), data.end(), 0);
  }
};
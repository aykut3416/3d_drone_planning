#pragma once
#include <random>
#include <vector>
#include <cmath>
#include "grid3d.hpp"

/**
 * @brief Random 3D occupancy grid generator
 *
 * Generates a simple synthetic 3D map by placing
 * randomly sized cubic obstacles into the grid.
 *
 * This is mainly used for:
 *  - Testing 3D planners
 *  - Simulation and visualization
 *  - Benchmarking path planning algorithms
 */
class RandomMapGenerator
{
public:
  /**
   * @brief Generate a random 3D occupancy grid
   *
   * Clears the grid and fills it with randomly
   * placed cubic obstacles of varying sizes.
   */
  static void generate(Grid3D &grid)
  {
    // Clear existing grid data
    grid.clear();
    
    // Random number generator
    std::mt19937 gen(std::random_device{}());

    // Random distributions for obstacle center positions
    std::uniform_real_distribution<double> xy(-40, 40);
    std::uniform_real_distribution<double> z(1, 4);

    // Possible obstacle sizes (meters)
    std::vector<double> sizes = {1, 2, 3, 4};
    std::uniform_int_distribution<int> si(0, sizes.size() - 1);

    // Generate multiple random obstacles
    for (int i = 0; i < 180; i++)
    {
      // Random obstacle center in world coordinates
      double cx = xy(gen);
      double cy = xy(gen);
      double cz = z(gen);

      // Half-size of obstacle in grid cells
      int hv = std::ceil(
        sizes[si(gen)] / grid.resolution / 2.0);

      // Convert world center to grid indices
      int gx = (cx - grid.origin_x) / grid.resolution;
      int gy = (cy - grid.origin_y) / grid.resolution;
      int gz = (cz - grid.origin_z) / grid.resolution;

      // Mark occupied cells (cube obstacle)
      for (int x = gx - hv; x <= gx + hv; x++)
        for (int y = gy - hv; y <= gy + hv; y++)
          for (int z = gz - hv; z <= gz + hv; z++)
            grid.setOccupied(x, y, z);
    }
  }
};
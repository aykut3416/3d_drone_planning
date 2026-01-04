#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "grid3d.hpp"

/**
 * @brief Utility class for publishing occupied grid cells as a PointCloud2
 *
 * - Converts occupied cells of a Grid3D into a ROS2 PointCloud2 message
 * - Each occupied grid cell is published as a 3D point in the "map" frame
 * - Used mainly for visualization in RViz
 */
class MapPointCloudPublisher
{
public:
  /**
   * @brief Publish occupied grid cells as a point cloud
   *
   * @param grid 3D occupancy grid
   * @param pub ROS2 PointCloud2 publisher
   * @param stamp Timestamp for the point cloud message
   */
  static void publish(const Grid3D& grid,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
    const rclcpp::Time& stamp)
  {
    // Count occupied cells
    int count = 0;
    for (int x = 0; x < grid.size_x; x++)
      for (int y = 0; y < grid.size_y; y++)
        for (int z = 0; z < grid.size_z; z++)
          if (grid.isOccupied(x, y, z)) count++;

    // Initialize PointCloud2 message
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = count;

    // Define XYZ fields
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(count);

    // Iterators for point access
    sensor_msgs::PointCloud2Iterator<float>
      ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");

    // Fill point cloud with occupied grid cells
    for (int x = 0; x < grid.size_x; x++)
      for (int y = 0; y < grid.size_y; y++)
        for (int z = 0; z < grid.size_z; z++)
          if (grid.isOccupied(x, y, z))
          {
            double wx, wy, wz;
            grid.gridToWorld(x, y, z, wx, wy, wz);

            *ix = static_cast<float>(wx);
            *iy = static_cast<float>(wy);
            *iz = static_cast<float>(wz);

            ++ix; ++iy; ++iz;
          }

    // Publish point cloud
    pub->publish(cloud);
  }
};

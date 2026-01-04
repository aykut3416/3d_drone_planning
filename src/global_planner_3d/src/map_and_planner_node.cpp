#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* -------- Global planner includes -------- */
#include "global_planner_3d/grid3d.hpp"
#include "global_planner_3d/planner_utils.hpp"
#include "global_planner_3d/random_map_generator.hpp"
#include "global_planner_3d/map_pointcloud_publisher.hpp"
#include "global_planner_3d/astar_planner.hpp"

#include <random>
#include <vector>
#include <cmath>

/* ============================================================
 * MapAndPlannerNode
 * ============================================================
 * Responsibilities:
 *  - Generate random 3D occupancy grid
 *  - Publish occupied cells as PointCloud2
 *  - Subscribe to vehicle pose (ENU)
 *  - Generate random goals
 *  - Run A* planner in 3D
 *  - Publish:
 *      • full global path
 *      • reduced PX4 path (direction-change points)
 */

class MapAndPlannerNode : public rclcpp::Node
{
public:
  MapAndPlannerNode()
  : Node("map_and_planner"),
    gen_(std::random_device{}()),
    xy_(-40.0, 40.0),
    z_(2.0, 4.0)
  {
    RCLCPP_INFO(get_logger(),
      "===============================");
    RCLCPP_INFO(get_logger(),
      " MapAndPlannerNode starting ");
    RCLCPP_INFO(get_logger(),
      "===============================");

    /* ---------- Publishers ---------- */

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_occupied", 1);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_path", 1);

    px4_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/path", 1);

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal", 10);

    RCLCPP_INFO(get_logger(), "Publishers created:");
    RCLCPP_INFO(get_logger(), "  • /grid_occupied");
    RCLCPP_INFO(get_logger(), "  • /global_path");
    RCLCPP_INFO(get_logger(), "  • /path (PX4 reduced)");
    RCLCPP_INFO(get_logger(), "  • /goal");

    /* ---------- QoS for PX4 ---------- */

    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.durability_volatile();

    /* ---------- Subscribers ---------- */

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vehicle_pose_enu",
      rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&MapAndPlannerNode::poseCb, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal",
      px4_qos,
      std::bind(&MapAndPlannerNode::goalCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribers created:");
    RCLCPP_INFO(get_logger(), "  • /vehicle_pose_enu");
    RCLCPP_INFO(get_logger(), "  • /goal");

    /* ---------- Map generation ---------- */

    RCLCPP_INFO(get_logger(), "Generating random 3D grid map...");
    RandomMapGenerator::generate(grid_);
    RCLCPP_INFO(get_logger(), "Map generation finished");

    /* ---------- Map visualization ---------- */

    MapPointCloudPublisher::publish(grid_, cloud_pub_, now());
    RCLCPP_INFO(get_logger(),
      "Occupied grid published as PointCloud2");

    /* ---------- Initial goal ---------- */

    geometry_msgs::msg::PoseStamped init_goal;
    init_goal.header.frame_id = "map";
    init_goal.header.stamp = now();
    init_goal.pose.position.x = 0.0;
    init_goal.pose.position.y = 0.0;
    init_goal.pose.position.z = 3.0;

    goal_pub_->publish(init_goal);

    RCLCPP_WARN(get_logger(),
      "Initial goal published at [%.2f %.2f %.2f]",
      init_goal.pose.position.x,
      init_goal.pose.position.y,
      init_goal.pose.position.z);
  }

private:
  /* ============================================================
   * Vehicle pose callback
   * ============================================================
   * - Stores current pose
   * - Checks distance to goal
   * - Generates new goal when reached
   */
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!have_pose_) {
      RCLCPP_INFO(get_logger(),
        "First pose received: [%.2f %.2f %.2f]",
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);
    }

    current_pose_ = *msg;
    have_pose_ = true;

    if (!have_goal_) return;

    double dx = current_pose_.pose.position.x - goal_pose_.pose.position.x;
    double dy = current_pose_.pose.position.y - goal_pose_.pose.position.y;
    double dz = current_pose_.pose.position.z - goal_pose_.pose.position.z;

    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Distance to goal: %.2f m", dist);

    if (dist < 0.4) {
      RCLCPP_WARN(get_logger(),
        "Goal reached → generating new goal");
      goalGenerate();
    }
  }

  /* ============================================================
   * Random goal generator
   * ============================================================
   */
  void goalGenerate()
  {
    geometry_msgs::msg::PoseStamped new_goal;
    new_goal.header.frame_id = "map";
    new_goal.header.stamp = now();

    new_goal.pose.position.x = xy_(gen_);
    new_goal.pose.position.y = xy_(gen_);
    new_goal.pose.position.z = z_(gen_);

    goal_pub_->publish(new_goal);
    have_goal_ = false;

    RCLCPP_WARN(get_logger(),
      "New random goal generated at [%.2f %.2f %.2f]",
      new_goal.pose.position.x,
      new_goal.pose.position.y,
      new_goal.pose.position.z);
  }

  /* ============================================================
   * Goal callback → run A* planner
   * ============================================================
   */
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(),
      "===============================");
    RCLCPP_INFO(get_logger(),
      " New goal received ");
    RCLCPP_INFO(get_logger(),
      "===============================");

    if (!have_pose_) {
      RCLCPP_WARN(get_logger(),
        "No vehicle pose yet. Waiting...");
      return;
    }

    goal_pose_ = *msg;
    have_goal_ = true;

    /* ---------- World → Grid ---------- */

    GridIndex start = worldToGrid(
      current_pose_.pose.position, grid_);

    GridIndex goal = worldToGrid(
      goal_pose_.pose.position, grid_);

    RCLCPP_INFO(get_logger(),
      "Start grid: (%d %d %d)",
      start.x, start.y, start.z);

    RCLCPP_INFO(get_logger(),
      "Goal grid:  (%d %d %d)",
      goal.x, goal.y, goal.z);

    /* ---------- Collision checks ---------- */

    if (grid_.isOccupied(start.x, start.y, start.z) ||
        grid_.isOccupied(goal.x, goal.y, goal.z))
    {
      RCLCPP_ERROR(get_logger(),
        "Start or goal is occupied → regenerating map & goal");

      RandomMapGenerator::generate(grid_);
      MapPointCloudPublisher::publish(grid_, cloud_pub_, now());
      goalGenerate();
      return;
    }

    /* ---------- A* planning ---------- */

    std::vector<GridIndex> path_idx;
    AStarPlanner planner;

    RCLCPP_INFO(get_logger(), "Running A* planner...");

    if (!planner.plan(grid_, start, goal, path_idx)) {
      RCLCPP_ERROR(get_logger(), "A* planning failed");
      return;
    }

    RCLCPP_INFO(get_logger(),
      "A* succeeded. Path length: %zu", path_idx.size());

    /* ---------- Full path ---------- */

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();

    for (const auto &g : path_idx) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;

      grid_.gridToWorld(
        g.x, g.y, g.z,
        ps.pose.position.x,
        ps.pose.position.y,
        ps.pose.position.z);

      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }

    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(),
      "Published global path");

    /* ---------- Reduced PX4 path ---------- */

    nav_msgs::msg::Path px4_path;
    px4_path.header = path.header;

    for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
      double change = directionChangeMetric(
        path.poses[i-1].pose.position,
        path.poses[i].pose.position,
        path.poses[i+1].pose.position);

      if (change > 0.05) {
        px4_path.poses.push_back(path.poses[i]);
      }
    }

    px4_path.poses.push_back(path.poses.back());
    px4_path_pub_->publish(px4_path);

    RCLCPP_INFO(get_logger(),
      "Published PX4 path (direction-change points)");
  }

  /* ============================================================
   * Members
   * ============================================================
   */

  Grid3D grid_;

  std::mt19937 gen_;
  std::uniform_real_distribution<double> xy_;
  std::uniform_real_distribution<double> z_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  bool have_pose_ = false;
  bool have_goal_ = false;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr px4_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

/* ============================================================
 * main
 * ============================================================
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAndPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

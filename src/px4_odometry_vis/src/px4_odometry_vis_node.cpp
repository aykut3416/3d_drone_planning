#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cmath>
#include <array>

using px4_msgs::msg::VehicleOdometry;
using geometry_msgs::msg::PoseStamped;
using visualization_msgs::msg::Marker;

/* ============================================================
 * Quaternion helper functions
 * ============================================================
 * PX4 odometry quaternion is given in NED frame.
 * These helpers are used to convert orientation from NED → ENU.
 */

/* Quaternion multiplication: q = q1 ⊗ q2 */
std::array<double, 4> quat_mul(
  const std::array<double,4>& q1,
  const std::array<double,4>& q2)
{
  return {
    q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
    q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
    q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
    q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
  };
}

/* Quaternion inverse */
std::array<double, 4> quat_inv(const std::array<double,4>& q)
{
  double norm =
    q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

  return { q[0]/norm, -q[1]/norm, -q[2]/norm, -q[3]/norm };
}

/* ============================================================
 * PX4OdometryVis Node
 * ============================================================
 * - Subscribes to PX4 VehicleOdometry (NED frame)
 * - Converts pose to ROS ENU frame
 * - Publishes:
 *     • geometry_msgs/PoseStamped
 *     • visualization_msgs/Marker (mesh-based drone model)
 */

class PX4OdometryVis : public rclcpp::Node
{
public:
  PX4OdometryVis()
  : Node("px4_odometry_vis")
  {
    /* PX4 QoS: BEST_EFFORT + VOLATILE */
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    /* PX4 odometry subscriber */
    odom_sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry",
      qos,
      std::bind(&PX4OdometryVis::odomCb, this, std::placeholders::_1));

    /* Pose publisher for RViz / planning */
    pose_pub_ = create_publisher<PoseStamped>(
      "/vehicle_pose_enu", 10);

    /* Marker publisher for drone visualization */
    marker_pub_ = create_publisher<Marker>(
      "/drone_marker", 10);

    RCLCPP_INFO(get_logger(),
      "px4_odometry_vis node started");
    RCLCPP_INFO(get_logger(),
      "Subscribing to /fmu/out/vehicle_odometry");
    RCLCPP_INFO(get_logger(),
      "Publishing:");
    RCLCPP_INFO(get_logger(),
      "  • /vehicle_pose_enu (PoseStamped)");
    RCLCPP_INFO(get_logger(),
      "  • /drone_marker (RViz mesh)");
  }

private:
  /* ============================================================
   * Odometry callback
   * ============================================================
   * Converts PX4 NED odometry to ROS ENU pose.
   */
  void odomCb(const VehicleOdometry::SharedPtr msg)
  {
    /* ---------- Position: NED → ENU ---------- */
    double x_enu = msg->position[1];   // East
    double y_enu = msg->position[0];   // North
    double z_enu = -msg->position[2];  // Up

    /* ---------- Orientation: NED → ENU ---------- */
    std::array<double,4> q_ned {
      msg->q[0], msg->q[1], msg->q[2], msg->q[3]
    };

    /* Rotation quaternion between NED and ENU frames */
    std::array<double,4> q_rot {
      0.0,
      1.0/std::sqrt(2.0),
      1.0/std::sqrt(2.0),
      0.0
    };

    /* Apply frame rotation: q_enu = R * q_ned * R⁻¹ */
    auto q_enu = quat_mul(
      quat_mul(q_rot, q_ned),
      quat_inv(q_rot)
    );

    /* Publish outputs */
    publishPose(x_enu, y_enu, z_enu, q_enu);
    publishMarker(x_enu, y_enu, z_enu, q_enu);

    /* Throttled debug info */
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ENU Pose: [x=%.2f y=%.2f z=%.2f]",
      x_enu, y_enu, z_enu);
  }

  /* ============================================================
   * Pose publisher
   * ============================================================
   */
  void publishPose(
    double x, double y, double z,
    const std::array<double,4>& q)
  {
    PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    pose.pose.orientation.w = q[0];
    pose.pose.orientation.x = q[1];
    pose.pose.orientation.y = q[2];
    pose.pose.orientation.z = q[3];

    pose_pub_->publish(pose);
  }

  /* ============================================================
   * Marker publisher (DAE mesh)
   * ============================================================
   */
  void publishMarker(
    double x, double y, double z,
    const std::array<double,4>& q)
  {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();

    marker.ns = "drone_mesh";
    marker.id = 0;
    marker.action = Marker::ADD;

    /* Use mesh instead of arrow */
    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource =
      "package://px4_odometry_vis/meshes/drone2.dae";

    marker.mesh_use_embedded_materials = true;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.w = q[0];
    marker.pose.orientation.x = q[1];
    marker.pose.orientation.y = q[2];
    marker.pose.orientation.z = q[3];

    /* Scale depends on mesh units */
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;

    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker_pub_->publish(marker);
  }

  /* ---------- ROS interfaces ---------- */
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
};

/* ============================================================
 * main
 * ============================================================
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4OdometryVis>());
  rclcpp::shutdown();
  return 0;
}
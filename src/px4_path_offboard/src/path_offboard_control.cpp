#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <vector>
#include <cmath>

/* ============================================================
 * PathOffboardControl
 * ============================================================
 * Responsibilities:
 *  - Subscribe to reduced path (/path)
 *  - Subscribe to vehicle pose in ENU frame
 *  - Enable PX4 offboard mode
 *  - Arm the vehicle
 *  - Follow path waypoint-by-waypoint
 *
 * PX4 Interface:
 *  Publishers:
 *    • /fmu/in/offboard_control_mode
 *    • /fmu/in/trajectory_setpoint
 *    • /fmu/in/vehicle_command
 *
 *  Subscribers:
 *    • /path              (nav_msgs/Path)
 *    • /vehicle_pose_enu  (PoseStamped, ENU)
 *
 * Notes:
 *  - ENU → NED conversion is applied for PX4 setpoints
 *  - Dummy setpoints are sent before offboard activation
 *  - Waypoints are considered reached within 0.45 m
 * ============================================================
 */

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleOdometry;

class PathOffboardControl : public rclcpp::Node
{
public:
  PathOffboardControl() : Node("path_offboard_control")
  {
    RCLCPP_INFO(get_logger(), "=================================");
    RCLCPP_INFO(get_logger(), "Starting PathOffboardControl node");
    RCLCPP_INFO(get_logger(), "=================================");

    /* ---------------- PX4 QoS ---------------- */
    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();
    px4_qos.durability_volatile();

    /* ---------------- PX4 Publishers ---------------- */

    offboard_control_mode_pub_ =
      create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ =
      create_publisher<TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);

    vehicle_command_pub_ =
      create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", 10);

    RCLCPP_INFO(get_logger(), "PX4 publishers created:");
    RCLCPP_INFO(get_logger(), "  • /fmu/in/offboard_control_mode");
    RCLCPP_INFO(get_logger(), "  • /fmu/in/trajectory_setpoint");
    RCLCPP_INFO(get_logger(), "  • /fmu/in/vehicle_command");

    /* ---------------- Subscribers ---------------- */

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", px4_qos,
      std::bind(&PathOffboardControl::path_callback,
                this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vehicle_pose_enu",
      rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&PathOffboardControl::pose_callback,
                this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribers created:");
    RCLCPP_INFO(get_logger(), "  • /path");
    RCLCPP_INFO(get_logger(), "  • /vehicle_pose_enu");

    /* ---------------- Timer (10 Hz) ---------------- */

    timer_ = create_wall_timer(
      100ms,
      std::bind(&PathOffboardControl::timer_callback, this));

    RCLCPP_INFO(get_logger(), "Timer started at 10 Hz");
    RCLCPP_INFO(get_logger(), "Path Offboard Controller ready");
  }

private:
  /* ================= ROS INTERFACES ================= */

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ================= STATE ================= */

  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t waypoint_idx_{0};
  bool path_received_{false};

  bool offboard_enabled_{false};
  bool armed_{false};

  float curr_x_{0.0f}, curr_y_{0.0f}, curr_z_{0.0f};
  bool odom_received_{false};

  uint64_t setpoint_counter_{0};

  /* ============================================================
   * Path callback
   * ============================================================ */
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path");
      return;
    }

    path_ = msg->poses;
    waypoint_idx_ = 0;
    path_received_ = true;

    RCLCPP_INFO(get_logger(),
      "Path received with %lu waypoints", path_.size());
  }

  /* ============================================================
   * Vehicle pose callback (ENU)
   * ============================================================ */
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    curr_x_ = msg->pose.position.x;
    curr_y_ = msg->pose.position.y;
    curr_z_ = msg->pose.position.z;
    odom_received_ = true;

    RCLCPP_DEBUG(get_logger(),
      "Current pose [ENU]: x=%.2f y=%.2f z=%.2f",
      curr_x_, curr_y_, curr_z_);
  }

  /* ============================================================
   * Timer callback (main control loop)
   * ============================================================ */
  void timer_callback()
  {
    /* PX4 requires offboard control mode to be sent continuously */
    publish_offboard_control_mode();

    /* Send dummy setpoints before offboard activation */
    if (!path_received_) {
      TrajectorySetpoint sp{};
      sp.position = {0.0f, 0.0f, -3.0f};
      sp.yaw = 0.0f;
      sp.timestamp = now_us();
      trajectory_setpoint_pub_->publish(sp);
      return;
    }

    /* Allow PX4 to receive several setpoints first */
    if (setpoint_counter_ < 20) {
      setpoint_counter_++;
      return;
    }

    /* Enable offboard mode once */
    if (!offboard_enabled_) {
      set_offboard_mode();
      offboard_enabled_ = true;
      return;
    }

    /* Arm the vehicle once */
    if (!armed_) {
      arm();
      armed_ = true;
      return;
    }

    /* Follow the path */
    if (path_received_ && odom_received_) {
      publish_path_setpoint();
    }
  }

  /* ============================================================
   * Publish OffboardControlMode
   * ============================================================ */
  void publish_offboard_control_mode()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = now_us();

    offboard_control_mode_pub_->publish(msg);
  }

  /* ============================================================
   * Waypoint reached check
   * ============================================================ */
  bool waypoint_reached(const geometry_msgs::msg::Point &wp)
  {
    float dx = curr_x_ - wp.x;
    float dy = curr_y_ - wp.y;
    float dz = curr_z_ - wp.z;

    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    return dist < 0.45f;
  }

  /* ============================================================
   * Publish trajectory setpoint for current waypoint
   * ============================================================ */
  void publish_path_setpoint()
  {
    if (waypoint_idx_ >= path_.size()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "All waypoints reached");
      return;
    }

    const auto &wp = path_[waypoint_idx_].pose.position;

    /* ENU → NED conversion for PX4 */
    TrajectorySetpoint msg{};
    msg.position = {
      static_cast<float>(wp.y),
      static_cast<float>(wp.x),
      static_cast<float>(-wp.z)
    };
    msg.yaw = 0.0f;
    msg.timestamp = now_us();

    trajectory_setpoint_pub_->publish(msg);

    if (waypoint_reached(wp)) {
      RCLCPP_INFO(get_logger(),
        "Waypoint %lu reached", waypoint_idx_);
      waypoint_idx_++;
    }
  }

  /* ============================================================
   * PX4 Commands
   * ============================================================ */
  void arm()
  {
    send_vehicle_command(
      VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void set_offboard_mode()
  {
    send_vehicle_command(
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(get_logger(), "Offboard mode enabled");
  }

  void send_vehicle_command(
    uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();

    vehicle_command_pub_->publish(msg);
  }

  /* ============================================================
   * Time helper (microseconds)
   * ============================================================ */
  uint64_t now_us()
  {
    return this->get_clock()->now().nanoseconds() / 1000;
  }
};

/* ============================================================
 * main
 * ============================================================ */
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathOffboardControl>());
  rclcpp::shutdown();
  return 0;
}
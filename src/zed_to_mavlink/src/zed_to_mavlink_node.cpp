#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/mocap/mocap.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ZedToMavlinkNode final : public rclcpp::Node
{
public:
  ZedToMavlinkNode()
  : Node("zed_to_mavlink"),
    mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}}
  {
    const std::string default_connection_url = "udpin://0.0.0.0:14590";
    this->declare_parameter<std::string>("connection_url", default_connection_url);
    connection_url_ = this->get_parameter("connection_url").as_string();

    pose_topic_ = "/zed/zed_node/odom";
    this->declare_parameter<std::string>("pose_topic", pose_topic_);
    pose_topic_ = this->get_parameter("pose_topic").as_string();

    local_pose_topic_ = "/uav1/local_position/pose";
    this->declare_parameter<std::string>("local_pose_topic", local_pose_topic_);
    local_pose_topic_ = this->get_parameter("local_pose_topic").as_string();

    // Keep rate modest — high-rate vision floods ArduPilot VisOdom
    auto period_ms = 100;
    this->declare_parameter<int>("publish_period_ms", period_ms);
    period_ms = this->get_parameter("publish_period_ms").as_int();
    if (period_ms < 50) {
      period_ms = 50;
    }

    RCLCPP_INFO(this->get_logger(), "Connecting MAVSDK via: %s", connection_url_.c_str());
    const mavsdk::ConnectionResult connection_result = mavsdk_.add_any_connection(connection_url_);
    if (connection_result != mavsdk::ConnectionResult::Success) {
      throw std::runtime_error("MAVSDK connection failed");
    }

    while (rclcpp::ok() && !system_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Waiting for autopilot on %s (start second_task / mavproxy with --out 127.0.0.1:14590)...",
        connection_url_.c_str());
      system_ = mavsdk_.first_autopilot(5.0);
      if (!system_) {
        std::this_thread::sleep_for(1s);
      }
    }
    if (!system_) {
      throw std::runtime_error("Interrupted while waiting for autopilot");
    }

    mocap_ = std::make_shared<mavsdk::Mocap>(system_.value());
    RCLCPP_INFO(this->get_logger(), "Autopilot connected");

    local_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      local_pose_topic_, rclcpp::SensorDataQoS());

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ZedToMavlinkNode::pose_callback, this, std::placeholders::_1));

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&ZedToMavlinkNode::publish_vision_estimate, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed to: %s -> MAVLink VisOdom (~%d ms) and ROS %s",
      pose_topic_.c_str(), period_ms, local_pose_topic_.c_str());
  }

private:
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(latest_pose_mutex_);
      latest_pose_ = msg;
    }

    const auto & position = msg->pose.pose.position;
    const auto & orientation = msg->pose.pose.orientation;

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quaternion_to_euler(
      orientation.x, orientation.y, orientation.z, orientation.w,
      roll, pitch, yaw);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    if (pose_msg.header.frame_id.empty()) {
      pose_msg.header.frame_id = "map";
    }

    // Publish the ROS local pose in the same FLU->FRD convention used for ArduPilot VisOdom.
    pose_msg.pose.position.x = position.x;
    pose_msg.pose.position.y = -position.y;
    pose_msg.pose.position.z = -position.z;
    pose_msg.pose.orientation = euler_to_quaternion(roll, -pitch, -yaw);

    local_pose_pub_->publish(pose_msg);
  }

  static void quaternion_to_euler(
    double x, double y, double z, double w,
    double & roll, double & pitch, double & yaw)
  {
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
      pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
      pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  static geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
  {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }

  void publish_vision_estimate()
  {
    nav_msgs::msg::Odometry::SharedPtr odom;
    {
      std::lock_guard<std::mutex> lock(latest_pose_mutex_);
      odom = latest_pose_;
    }

    if (!odom) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No odometry yet on %s", pose_topic_.c_str());
      return;
    }

    if (!system_ || !mocap_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "System not available");
      return;
    }

    const auto & position = odom->pose.pose.position;
    const auto & orientation = odom->pose.pose.orientation;

    mavsdk::Mocap::VisionPositionEstimate vision_estimate{};

    // 0 = time-of-receipt on FC. ROS/unix stamps often disagree with FC clock
    // and ArduPilot then rejects vision → VisOdom not healthy / no position.
    vision_estimate.time_usec = 0;

    // Unknown covariance: single NaN per MAVLink / ArduPilot convention
    vision_estimate.pose_covariance.covariance_matrix = {NAN};

    // ROS (ZED): FLU -> ArduPilot external vision FRD
    vision_estimate.position_body.x_m = static_cast<float>(position.x);
    vision_estimate.position_body.y_m = static_cast<float>(-position.y);
    vision_estimate.position_body.z_m = static_cast<float>(-position.z);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quaternion_to_euler(
      orientation.x, orientation.y, orientation.z, orientation.w,
      roll, pitch, yaw);

    // FLU->FRD: flip pitch/yaw signs with y/z flip for attitude consistency
    vision_estimate.angle_body.roll_rad = static_cast<float>(roll);
    vision_estimate.angle_body.pitch_rad = static_cast<float>(-pitch);
    vision_estimate.angle_body.yaw_rad = static_cast<float>(-yaw);

    try {
      const mavsdk::Mocap::Result result = mocap_->set_vision_position_estimate(vision_estimate);
      if (result != mavsdk::Mocap::Result::Success) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Failed to send vision estimate: %d", static_cast<int>(result));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Exception while sending vision estimate: %s", e.what());
    }
  }

private:
  mavsdk::Mavsdk mavsdk_;
  std::optional<std::shared_ptr<mavsdk::System>> system_;
  std::shared_ptr<mavsdk::Mocap> mocap_;

  std::string connection_url_;
  std::string pose_topic_;
  std::string local_pose_topic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex latest_pose_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ZedToMavlinkNode>());
  } catch (const std::exception & e) {
    fprintf(stderr, "zed_to_mavlink aborted: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}

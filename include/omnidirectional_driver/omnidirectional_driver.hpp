#ifndef OMNIDIRECTIONAL_DRIVER__OMNIDIRECTIONAL_DRIVER_HPP_
#define OMNIDIRECTIONAL_DRIVER__OMNIDIRECTIONAL_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "omnidirectional_driver/visibility_control.hpp"
#include "omnidirectional_driver/omni_kinematics.hpp"

namespace omnidirectional_driver
{

class OmniDriver final : public rclcpp::Node
{
public:
  OMNIDIRECTIONAL_DRIVER_PUBLIC
  explicit OmniDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  OMNIDIRECTIONAL_DRIVER_PUBLIC
  ~OmniDriver() override = default;

private:
  // --- Initialization ---
  void load_parameters();
  void init_interfaces();

  // --- Callbacks ---
  void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

  // --- Helpers ---
  void publish_odometry(const OdometryState & state,const Eigen::Vector3d & vel,const Eigen::Matrix3d & Q_vel,const rclcpp::Time & time_now);

  void publish_twist_covariance(const Eigen::Matrix3d & Q_vel,const rclcpp::Time & time_now);

  void publish_pose_covariance(const OdometryState & state,const rclcpp::Time & time_now);

  // --- Core Logic ---
  OmniKinematics kinematics_;

  // --- ROS Infrastructure ---
  std::string odom_frame_id_;
  std::string base_frame_id_;
  rclcpp::Time last_time_;
  bool first_joint_state_received_{false};
  bool publish_tf_{false};

  // Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Pre-allocated messages to avoid runtime allocation
  std_msgs::msg::Float64MultiArray motor_cmd_msg_;

  // Buffer for desired wheel commands
  Eigen::VectorXd wheel_cmds_;
};

}  // namespace omnidirectional_driver

#endif  // OMNIDIRECTIONAL_DRIVER__OMNIDIRECTIONAL_DRIVER_HPP_
#include "omnidirectional_driver/omnidirectional_driver.hpp"

#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace omnidirectional_driver
{

OmniDriver::OmniDriver(const rclcpp::NodeOptions & options)
: Node("omnidirectional_driver", options)
{
  load_parameters();
  init_interfaces();
  
  last_time_ = this->get_clock()->now();
  
  if (!kinematics_.get_params().wheel_names.empty()) {
    motor_cmd_msg_.data.resize(kinematics_.get_params().wheel_names.size(), 0.0);
    wheel_cmds_ = Eigen::VectorXd::Zero(kinematics_.get_params().wheel_names.size());
  }

  RCLCPP_INFO(this->get_logger(), "OmniDriver (Unified C++) Initialized.");
}

void OmniDriver::load_parameters()
{
  RobotParams p;
  this->declare_parameter("wheel_names", std::vector<std::string>());
  this->declare_parameter("robot_radius", 0.0);
  this->declare_parameter("wheel_radius", 0.0);
  this->declare_parameter("wheel_angles_deg", std::vector<double>());
  this->declare_parameter("roller_angle_deg", 0.0);
  this->declare_parameter("use_field_centric", false);
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_frame_id", "robot_footprint");
  
  // Covariance params
  this->declare_parameter("encoder_ppr", 1024);
  this->declare_parameter("nominal_dt", 0.01);  // 1 / expected joint_state Hz
  
  this->declare_parameter("covariance_scale_xy", 1.0);
  this->declare_parameter("covariance_scale_yaw", 1.0);
  this->declare_parameter("covariance_scale_velxy", 1.0);
  this->declare_parameter("covariance_scale_velyaw", 1.0);
  
  this->declare_parameter("covariance_offset_xy", 0.0);
  this->declare_parameter("covariance_offset_yaw", 0.0);
  this->declare_parameter("covariance_offset_velxy", 0.0);
  this->declare_parameter("covariance_offset_velyaw", 0.0);
  
  this->declare_parameter("acceleration_limit", 1.0);

  this->declare_parameter("publish_tf", false);

  try {
    p.wheel_names = this->get_parameter("wheel_names").as_string_array();
    p.robot_radius = this->get_parameter("robot_radius").as_double();
    p.wheel_radius = this->get_parameter("wheel_radius").as_double();
    p.wheel_angles_deg = this->get_parameter("wheel_angles_deg").as_double_array();
    p.roller_angle_deg = this->get_parameter("roller_angle_deg").as_double();
    p.use_field_centric = this->get_parameter("use_field_centric").as_bool();
    
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();

    p.encoder_ppr = this->get_parameter("encoder_ppr").as_int();
    p.nominal_dt  = this->get_parameter("nominal_dt").as_double();
    
    p.covariance_scale_xy = this->get_parameter("covariance_scale_xy").as_double();
    p.covariance_scale_yaw = this->get_parameter("covariance_scale_yaw").as_double();
    p.covariance_scale_velxy = this->get_parameter("covariance_scale_velxy").as_double();
    p.covariance_scale_velyaw = this->get_parameter("covariance_scale_velyaw").as_double();
    
    p.covariance_offset_xy = this->get_parameter("covariance_offset_xy").as_double();
    p.covariance_offset_yaw = this->get_parameter("covariance_offset_yaw").as_double();
    p.covariance_offset_velxy = this->get_parameter("covariance_offset_velxy").as_double();
    p.covariance_offset_velyaw = this->get_parameter("covariance_offset_velyaw").as_double();
    
    p.acceleration_limit = this->get_parameter("acceleration_limit").as_double();

    publish_tf_ = this->get_parameter("publish_tf").as_bool();

  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_FATAL(this->get_logger(), "Parameter Error: %s", e.what());
    throw;
  }

  kinematics_.configure(p);

  if (p.wheel_names.empty()) {
    RCLCPP_WARN(this->get_logger(), "WARNING: No wheel names found in parameters! Driver will be inactive.");
  }
}

void OmniDriver::init_interfaces()
{
  auto sensor_qos = rclcpp::SensorDataQoS();
  auto odom_qos = rclcpp::SystemDefaultsQoS();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", sensor_qos, std::bind(&OmniDriver::cmd_vel_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", sensor_qos, std::bind(&OmniDriver::joint_state_callback, this, std::placeholders::_1));

  motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_velocity_controller/commands", sensor_qos);

  // Odometry topics use reliable delivery for consistency with state estimation nodes (e.g., EKF)
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", odom_qos);
  
  twist_cov_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/twistWithCovariance", odom_qos);

  pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/poseWithCovariance", odom_qos);

  if(publish_tf_) {
    RCLCPP_INFO(this->get_logger(), "TF publishing enabled. Will publish %s -> %s transform.", odom_frame_id_.c_str(), base_frame_id_.c_str());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  } else {
    RCLCPP_INFO(this->get_logger(), "TF publishing disabled. Not publishing %s -> %s transform.", odom_frame_id_.c_str(), base_frame_id_.c_str());
  }
}

void OmniDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  wheel_cmds_ = kinematics_.calculate_wheel_commands(msg->linear.x, msg->linear.y, msg->angular.z, kinematics_.get_state().theta);
}

void OmniDriver::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  const auto & params = kinematics_.get_params();
  size_t n_wheels = params.wheel_names.size();
  if (n_wheels == 0) return;

  // 1. Time Logic Handling & micro-ROS Sync Fix
  rclcpp::Time msg_time = msg->header.stamp;
  rclcpp::Time pc_time = this->now();

  // If ESP32 time is uninitialized (1970) or drifted by > 1s, override with PC time
  if (msg_time.nanoseconds() <= 0 || std::abs((msg_time - pc_time).seconds()) > 1.0) {
    msg_time = pc_time;
  }

  // 2. Parse Joint States
  Eigen::VectorXd wheel_vels = Eigen::VectorXd::Zero(n_wheels);
  int found = 0;

  for (size_t i = 0; i < n_wheels; ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), params.wheel_names[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      
      // Prevent segfault if /joint_states does not populate velocity
      if (idx < msg->velocity.size()) {
        wheel_vels(i) = msg->velocity[idx];
      } else {
        RCLCPP_WARN_ONCE(this->get_logger(), "Joint %s found, but no velocity data published!", params.wheel_names[i].c_str());
        wheel_vels(i) = 0.0;
      }
      found++;
    }
  }

  if (found < (int)n_wheels) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
      "Received /joint_states but found only %d/%zu matching wheel names.", found, n_wheels);
    return;
  }

  // Case: First run initialization
  if (!first_joint_state_received_) {
    first_joint_state_received_ = true;
    last_time_ = msg_time;
    RCLCPP_INFO_ONCE(this->get_logger(), "First joint state received, initialized time.");
    return; // Wait for the next callback to have a valid interval
  }

  double dt = (msg_time - last_time_).seconds();

  // Case: Time went backwards
  if (dt < 0.0) {
    RCLCPP_WARN(this->get_logger(), "Time went backwards (dt=%f). Resetting odometry state.", dt);
    first_joint_state_received_ = false;
    kinematics_.reset_state();
    return;
  }

  // Case: Jitter filter
  if (dt < 0.0005) {
    RCLCPP_DEBUG(this->get_logger(), "dt too small (%f), skipping update.", dt);
    return;
  }

  size_t n = wheel_cmds_.size();
  if (n == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Kinematics returned 0 commands. Have you sent a /cmd_vel yet?");
    return;
  }

  if (motor_cmd_msg_.data.size() != n) {
      motor_cmd_msg_.data.resize(n);
  }

  // Safe acceleration math to prevent division by zero
  if (params.wheel_radius > 1e-5) {
    double angular_accel_limit = params.acceleration_limit / params.wheel_radius;
    double max_step = angular_accel_limit * dt;

    for (size_t i = 0; i < n; ++i) {
      double step = wheel_cmds_(i) - wheel_vels(i);
      motor_cmd_msg_.data[i] = wheel_vels(i) + std::clamp(step, -max_step, max_step);
    }
  } else {
    // If wheel_radius is not loaded, fallback directly to commands
    for (size_t i = 0; i < n; ++i) {
      motor_cmd_msg_.data[i] = wheel_cmds_(i);
    }
  }

  // 3. Main Update Loop (Only runs if dt is valid)
  last_time_ = msg_time;

  // Forward Kinematics: Calculate Robot Velocity
  const Eigen::Vector3d & robot_vel = kinematics_.calculate_robot_velocity(wheel_vels, dt);

  // Debug logging (log first update and then every 100 updates to reduce spam)
  static int debug_count = 0;
  if (debug_count == 0 || debug_count % 100 == 0) {
    RCLCPP_DEBUG(this->get_logger(), 
      "Wheel velocities: [%.3f, %.3f, %.3f] rad/s | "
      "Robot velocity: [vx=%.3f, vy=%.3f, omega=%.3f] m/s | "
      "Position: [x=%.3f, y=%.3f, theta=%.3f]",
      wheel_vels(0), wheel_vels(1), wheel_vels(2),
      robot_vel(0), robot_vel(1), robot_vel(2),
      kinematics_.get_state().x, kinematics_.get_state().y, kinematics_.get_state().theta);
  }
  debug_count++;

  motor_cmd_pub_->publish(motor_cmd_msg_);

  // Integration: Update Odometry Position (x, y, theta)
  const OdometryState & new_state = kinematics_.integrate_odometry(robot_vel, dt);

  const Eigen::Matrix3d & Q_vel = kinematics_.get_twist_covariance();

  // Publish
  publish_odometry(new_state, robot_vel, Q_vel, msg_time);
  // publish_twist_covariance(Q_vel, msg_time);
}

void OmniDriver::publish_odometry(
  const OdometryState & state,
  const Eigen::Vector3d & vel,
  const Eigen::Matrix3d & Q_vel,
  const rclcpp::Time & time_now)
{
  tf2::Quaternion q_odom;
  q_odom.setRPY(0, 0, state.theta);

  // TF
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = time_now;
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id = base_frame_id_;
    t.transform.translation.x = state.x;
    t.transform.translation.y = state.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(q_odom);
    tf_broadcaster_->sendTransform(t);
  }

  // Odom Message
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time_now;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(q_odom);
  
  odom.twist.twist.linear.x = vel(0);
  odom.twist.twist.linear.y = vel(1);
  odom.twist.twist.angular.z = vel(2);

  // Fill Covariances
  // Default scale multiplication for Pose (Empirical tuning)
  const auto & p = kinematics_.get_params();
  
  // Row-major 6x6. Indices: 0=xx, 7=yy, 35=thth
  // Internal matrix is 3x3: 0,1,2 maps to x,y,th
  odom.pose.covariance.fill(0.0);

  double scale_xy_yaw = std::sqrt(p.covariance_scale_xy * p.covariance_scale_yaw);
  
  // Map Pose Covariance (Accumulated)
  // X-row
  odom.pose.covariance[0] = state.pose_covariance(0,0) * p.covariance_scale_xy + p.covariance_offset_xy;
  odom.pose.covariance[1] = state.pose_covariance(0,1) * p.covariance_scale_xy;
  odom.pose.covariance[5] = state.pose_covariance(0,2) * scale_xy_yaw;

  // Y-row
  odom.pose.covariance[6] = state.pose_covariance(1,0) * p.covariance_scale_xy;
  odom.pose.covariance[7] = state.pose_covariance(1,1) * p.covariance_scale_xy + p.covariance_offset_xy;
  odom.pose.covariance[11] = state.pose_covariance(1,2) * scale_xy_yaw;

  // Theta-row (mapped to instate.pose_covariancex 35 -> element (5,5))
  odom.pose.covariance[30] = state.pose_covariance(2,0) * scale_xy_yaw;
  odom.pose.covariance[31] = state.pose_covariance(2,1) * scale_xy_yaw;
  odom.pose.covariance[35] = state.pose_covariance(2,2) * p.covariance_scale_yaw + p.covariance_offset_yaw;

  // Twist Covariance
  // NOTE: Q_vel is twist_covariance_ from calculate_robot_velocity, which already has
  // covariance_scale_vel / covariance_scale_velyaw baked in via S·Σ·S.
  // Applying any scale here would double-scale. Only add the diagonal offset.
  odom.twist.covariance.fill(0.0);

  odom.twist.covariance[0]  = Q_vel(0,0) + p.covariance_offset_velxy;
  odom.twist.covariance[1]  = Q_vel(0,1);
  odom.twist.covariance[5]  = Q_vel(0,2);

  odom.twist.covariance[6]  = Q_vel(1,0);
  odom.twist.covariance[7]  = Q_vel(1,1) + p.covariance_offset_velxy;
  odom.twist.covariance[11] = Q_vel(1,2);

  odom.twist.covariance[30] = Q_vel(2,0);
  odom.twist.covariance[31] = Q_vel(2,1);
  odom.twist.covariance[35] = Q_vel(2,2) + p.covariance_offset_velyaw;

  odom_pub_->publish(odom);
}

void OmniDriver::publish_twist_covariance(
  const Eigen::Matrix3d & Q_vel,
  const rclcpp::Time & time_now
)
{
  // Twist Covariance
  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = time_now;
  twist_msg.header.frame_id = base_frame_id_;
  twist_msg.twist.covariance.fill(0.0);

  // NOTE: Q_vel is already scaled in calculate_robot_velocity via S·Σ·S.
  // Only add the diagonal offset here — no additional scale.
  const auto & p = kinematics_.get_params();

  twist_msg.twist.covariance[0]  = Q_vel(0,0) + p.covariance_offset_velxy;
  twist_msg.twist.covariance[1]  = Q_vel(0,1);
  twist_msg.twist.covariance[5]  = Q_vel(0,2);

  twist_msg.twist.covariance[6]  = Q_vel(1,0);
  twist_msg.twist.covariance[7]  = Q_vel(1,1) + p.covariance_offset_velxy;
  twist_msg.twist.covariance[11] = Q_vel(1,2);

  twist_msg.twist.covariance[30] = Q_vel(2,0);
  twist_msg.twist.covariance[31] = Q_vel(2,1);
  twist_msg.twist.covariance[35] = Q_vel(2,2) + p.covariance_offset_velyaw;

  twist_cov_pub_->publish(twist_msg);
}

// void OmniDriver::publish_pose_covariance(
//   const OdometryState & state,
//   const rclcpp::Time & time_now)
// {
  
}  // namespace omnidirectional_driver

RCLCPP_COMPONENTS_REGISTER_NODE(omnidirectional_driver::OmniDriver)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<omnidirectional_driver::OmniDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
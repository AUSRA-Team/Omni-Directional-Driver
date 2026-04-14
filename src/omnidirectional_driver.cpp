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
  }

  RCLCPP_INFO(this->get_logger(), "OmniDriver (Simplified) Initialized.");
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
  this->declare_parameter("publish_tf", true);  // Disable when using EKF for TF

  try {
    p.wheel_names = this->get_parameter("wheel_names").as_string_array();
    p.robot_radius = this->get_parameter("robot_radius").as_double();
    p.wheel_radius = this->get_parameter("wheel_radius").as_double();
    p.wheel_angles_deg = this->get_parameter("wheel_angles_deg").as_double_array();
    p.roller_angle_deg = this->get_parameter("roller_angle_deg").as_double();
    p.use_field_centric = this->get_parameter("use_field_centric").as_bool();
    
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
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
  // We use SensorDataQoS because it is "Best Effort"
  // This matches your Teleop node and your ESP32 micro-ROS settings
  auto qos = rclcpp::SensorDataQoS();

  // 1. Subscribe to Teleop (cmd_vel)
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos, 
    std::bind(&OmniDriver::cmd_vel_callback, this, std::placeholders::_1));

  // 2. Subscribe to ESP32 Feedback (joint_states)
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", qos,
    std::bind(&OmniDriver::joint_state_callback, this, std::placeholders::_1));

  // 3. Publish to ESP32 (motor commands)
  motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "joint_group_velocity_controller/commands", qos);

  // 4. Publish Odometry (Best Effort is usually better for high-rate odom)
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OmniDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  const Eigen::VectorXd & wheel_cmds = kinematics_.calculate_wheel_commands(
    msg->linear.x, 
    msg->linear.y, 
    msg->angular.z, 
    kinematics_.get_state().theta
  );

  size_t n = wheel_cmds.size();
  if (n == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Kinematics returned 0 commands. Check 'wheel_names' parameter!");
    return;
  }

  if (motor_cmd_msg_.data.size() != n) {
      motor_cmd_msg_.data.resize(n);
  }

  for (size_t i = 0; i < n; ++i) {
    motor_cmd_msg_.data[i] = wheel_cmds(i);
  }
  
  motor_cmd_pub_->publish(motor_cmd_msg_);
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
      wheel_vels(i) = msg->velocity[idx];
      found++;
    }
  }

  if (found < (int)n_wheels) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
      "Received /joint_states but found only %d/%zu matching wheel names. Check ESP32 code vs hardware_params.yaml!", found, n_wheels);
    return;
  }

  // Case: First run initialization
  if (!first_joint_state_received_) {
    first_joint_state_received_ = true;
    last_time_ = msg_time;
    return;
  }

  double dt = (msg_time - last_time_).seconds();

  // Case: Time went backwards (Simulation reset or clock jump)
  if (dt < 0.0) {
    RCLCPP_WARN(this->get_logger(), "Time went backwards (dt=%f). Resetting odometry state.", dt);
    first_joint_state_received_ = false;
    kinematics_.reset_state();
    return;
  }

  // Case: Duplicate message or negligible time step (Jitter filter)
  if (dt < 0.0005) {
    return;
  }

  // 3. Main Update Loop (Only runs if dt is valid)
  last_time_ = msg_time;

  // Forward Kinematics: Calculate Robot Velocity
  const Eigen::Vector3d & robot_vel = kinematics_.calculate_robot_velocity(wheel_vels);

  // Integration: Update Odometry Position (x, y, theta)
  const OdometryState & new_state = kinematics_.integrate_odometry(robot_vel, dt);

  // Publish
  publish_odometry(new_state, robot_vel, msg_time);
}

void OmniDriver::publish_odometry(
  const OdometryState & state,
  const Eigen::Vector3d & vel,
  const rclcpp::Time & time_now)
{
  tf2::Quaternion q_odom;
  q_odom.setRPY(0, 0, state.theta);

  // TF - Only publish if enabled (disable when using EKF for TF)
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

  // Simple fixed covariance values
  odom.pose.covariance.fill(0.0);
  odom.pose.covariance[0] = 0.01;   // x
  odom.pose.covariance[7] = 0.01;   // y
  odom.pose.covariance[35] = 0.01;  // yaw
  
  odom.twist.covariance.fill(0.0);
  odom.twist.covariance[0] = 0.01;   // vx
  odom.twist.covariance[7] = 0.01;   // vy
  odom.twist.covariance[35] = 0.01;  // vtheta

  odom_pub_->publish(odom);
}

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

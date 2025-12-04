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
  
  // Pre-allocate the vector size to avoid reallocation in the loop
  if (!kinematics_.get_params().wheel_names.empty()) {
    motor_cmd_msg_.data.resize(kinematics_.get_params().wheel_names.size(), 0.0);
  }

  RCLCPP_INFO(this->get_logger(), "OmniDriver (Unified C++) Initialized.");
}

void OmniDriver::load_parameters()
{
  RobotParams p;
  // Declare parameters
  this->declare_parameter("wheel_names", std::vector<std::string>());
  this->declare_parameter("robot_radius", 0.0);
  this->declare_parameter("wheel_radius", 0.0);
  this->declare_parameter("wheel_angles_deg", std::vector<double>());
  this->declare_parameter("roller_angle_deg", 0.0);
  this->declare_parameter("use_field_centric", false);
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_frame_id", "robot_footprint");

  try {
    p.wheel_names = this->get_parameter("wheel_names").as_string_array();
    p.robot_radius = this->get_parameter("robot_radius").as_double();
    p.wheel_radius = this->get_parameter("wheel_radius").as_double();
    p.wheel_angles_deg = this->get_parameter("wheel_angles_deg").as_double_array();
    p.roller_angle_deg = this->get_parameter("roller_angle_deg").as_double();
    p.use_field_centric = this->get_parameter("use_field_centric").as_bool();
    

    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();

  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_FATAL(this->get_logger(), "Parameter Error: %s", e.what());
    throw;
  }

  // Configure Math Logic
  kinematics_.configure(p);

  // Check if configuration succeeded
  if (p.wheel_names.empty()) {
    RCLCPP_WARN(this->get_logger(), "WARNING: No wheel names found in parameters! Driver will be inactive.");
  }
}

void OmniDriver::init_interfaces()
{
  auto qos = rclcpp::QoS(10);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", qos, 
    std::bind(&OmniDriver::cmd_vel_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", qos,
    std::bind(&OmniDriver::joint_state_callback, this, std::placeholders::_1));

  motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/joint_group_velocity_controller/commands", qos);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OmniDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  // 1. Calculate Motor Commands
  const Eigen::VectorXd & wheel_cmds = kinematics_.calculate_wheel_commands(
    msg->linear.x, 
    msg->linear.y, 
    msg->angular.z, 
    kinematics_.get_state().theta
  );

  // 2. Publish (using pre-allocated message)
  size_t n = wheel_cmds.size();

  // FIX: Safety check. If n is 0, DO NOT publish an empty message.
  if (n == 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
        "Kinematics returned 0 commands. Check 'wheel_names' parameter in ausrabot_controller.yaml!");
    return;
  }

  if (motor_cmd_msg_.data.size() != n) {
      motor_cmd_msg_.data.resize(n);
  }

  for (size_t i = 0; i < n; ++i) {
    // Removed correction multiplication
    motor_cmd_msg_.data[i] = wheel_cmds(i);
  }
  
  motor_cmd_pub_->publish(motor_cmd_msg_);
}

void OmniDriver::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  const auto & params = kinematics_.get_params();
  size_t n_wheels = params.wheel_names.size();
  if (n_wheels == 0) return;

  // 1. Parse Joint States
  // Using Eigen vector directly
  Eigen::VectorXd wheel_vels = Eigen::VectorXd::Zero(n_wheels);
  int found = 0;

  for (size_t i = 0; i < n_wheels; ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), params.wheel_names[i]);
    if (it != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it);
      // Removed correction multiplication
      wheel_vels(i) = msg->velocity[idx];
      found++;
    }
  }

  if (found < 3) return;

  // 2. Forward Kinematics
  const Eigen::Vector3d & robot_vel = kinematics_.calculate_robot_velocity(wheel_vels);

  // 3. Integration
  rclcpp::Time now = this->get_clock()->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  const OdometryState & new_state = kinematics_.integrate_odometry(robot_vel, dt);

  // 4. Publish
  publish_odometry(new_state, robot_vel, now);
}

void OmniDriver::publish_odometry(
  const OdometryState & state, 
  const Eigen::Vector3d & vel, 
  const rclcpp::Time & time_now)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, state.theta);
  geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);

  // TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = time_now;
  t.header.frame_id = odom_frame_id_;
  t.child_frame_id = base_frame_id_;
  t.transform.translation.x = state.x;
  t.transform.translation.y = state.y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = q_msg;
  tf_broadcaster_->sendTransform(t);

  // Odom
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = time_now;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = q_msg;
  
  odom.twist.twist.linear.x = vel(0);
  odom.twist.twist.linear.y = vel(1);
  odom.twist.twist.angular.z = vel(2);

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
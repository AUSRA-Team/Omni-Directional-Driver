#include "omnidirectional_driver/omni_kinematics.hpp"

namespace omnidirectional_driver
{

void OmniKinematics::configure(const RobotParams & params)
{
  params_ = params;
  
  params_.phi_rad.clear();
  params_.phi_rad.reserve(params_.wheel_angles_deg.size());
  for (double deg : params_.wheel_angles_deg) {
    params_.phi_rad.push_back(deg * M_PI / 180.0);
  }
  params_.gamma_rad = params_.roller_angle_deg * M_PI / 180.0;

  size_t n = params_.wheel_names.size();
  if (n < 3) return;

  coupling_matrix_.resize(n, 3);
  inv_coupling_matrix_.resize(3, n);
  
  wheel_vels_.resize(n);
  temp_wheel_lin_vels_.resize(n);

  for (size_t i = 0; i < n; ++i) {
    coupling_matrix_(i, 0) = std::cos(params_.phi_rad[i] + params_.gamma_rad);
    coupling_matrix_(i, 1) = std::sin(params_.phi_rad[i] + params_.gamma_rad);
    coupling_matrix_(i, 2) = -1.0 * std::cos(params_.gamma_rad) * params_.robot_radius;
  }

  inv_coupling_matrix_ = coupling_matrix_.completeOrthogonalDecomposition().pseudoInverse();
}

const Eigen::VectorXd & OmniKinematics::calculate_wheel_commands(
  double vx, double vy, double omega, double current_heading)
{
  if (params_.wheel_radius <= 1e-6) {
    wheel_vels_.setZero();
    return wheel_vels_;
  }

  if (params_.use_field_centric) {
    double global_vx = vx;
    double global_vy = vy;
    vx = global_vx * std::cos(current_heading) + global_vy * std::sin(current_heading);
    vy = -global_vx * std::sin(current_heading) + global_vy * std::cos(current_heading);
  }

  robot_vel_ << vx, vy, omega;
  temp_wheel_lin_vels_.noalias() = coupling_matrix_ * robot_vel_;
  wheel_vels_.noalias() = temp_wheel_lin_vels_ / (std::cos(params_.gamma_rad) * params_.wheel_radius);

  return wheel_vels_;
}

const Eigen::Vector3d & OmniKinematics::calculate_robot_velocity(const Eigen::VectorXd & wheel_angular_vels)
{
  if (wheel_angular_vels.size() != (long)params_.wheel_names.size()) {
    calc_robot_vel_.setZero();
    return calc_robot_vel_;
  }

  // Calculate Linear Wheel Velocities
  temp_wheel_lin_vels_.noalias() = wheel_angular_vels * (std::cos(params_.gamma_rad) * params_.wheel_radius);
  
  // Forward Kinematics
  calc_robot_vel_.noalias() = inv_coupling_matrix_ * temp_wheel_lin_vels_;

  return calc_robot_vel_;
}

const OdometryState & OmniKinematics::integrate_odometry(const Eigen::Vector3d & robot_vel, double dt)
{
  double vx = robot_vel(0);
  double vy = robot_vel(1);
  double omega = robot_vel(2);
  double th = state_.theta;

  // Position Integration
  double dx_global = (vx * std::cos(th) - vy * std::sin(th)) * dt;
  double dy_global = (vx * std::sin(th) + vy * std::cos(th)) * dt;
  double dth = omega * dt;

  state_.x += dx_global;
  state_.y += dy_global;
  state_.theta += dth;
  
  return state_;
}

}  // namespace omnidirectional_driver
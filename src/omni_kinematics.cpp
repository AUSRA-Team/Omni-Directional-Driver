#include "omnidirectional_driver/omni_kinematics.hpp"

namespace omnidirectional_driver
{

void OmniKinematics::configure(const RobotParams & params)
{
  params_ = params;
  
  // 1. Pre-compute radians
  params_.phi_rad.clear();
  params_.phi_rad.reserve(params_.wheel_angles_deg.size());
  for (double deg : params_.wheel_angles_deg) {
    params_.phi_rad.push_back(deg * M_PI / 180.0);
  }
  // Gamma is likely not used in your specific formula, but we calculate it just in case
  params_.gamma_rad = params_.roller_angle_deg * M_PI / 180.0;

  size_t n = params_.wheel_names.size();
  if (n < 3) return;

  // 2. Pre-allocate Eigen Matrices once
  coupling_matrix_.resize(n, 3);
  inv_coupling_matrix_.resize(3, n);
  
  // Resize reuseable buffers
  wheel_vels_.resize(n);
  temp_wheel_lin_vels_.resize(n);

  // 3. Fill Matrix (Inverse Kinematics) based on your custom formula:
  // wheel_vel = cos(th)*vx + sin(th)*vy - R * omega
  for (size_t i = 0; i < n; ++i) {
    double angle = params_.phi_rad[i]; // This is 'th'
    
    // Vx term: cos(th)
    coupling_matrix_(i, 0) = std::cos(angle);
    
    // Vy term: sin(th)
    coupling_matrix_(i, 1) = std::sin(angle);
    
    // Omega term: -1.0 * R
    coupling_matrix_(i, 2) = -1.0 * params_.robot_radius;
  }

  // 4. Compute Inverse (Forward Kinematics) once
  // Uses CompleteOrthogonalDecomposition for numerical stability
  inv_coupling_matrix_ = coupling_matrix_.completeOrthogonalDecomposition().pseudoInverse();
}

const Eigen::VectorXd & OmniKinematics::calculate_wheel_commands(
  double vx, double vy, double omega, double current_heading)
{
  // Optimization: Early exit for uninitialized config
  if (params_.wheel_radius <= 1e-6) {
    wheel_vels_.setZero();
    return wheel_vels_;
  }

  // 1. Field Centric Rotation
  // We modify the inputs directly to avoid allocating a new vector
  if (params_.use_field_centric) {
    double c_th = std::cos(current_heading);
    double s_th = std::sin(current_heading);
    
    double global_vx = vx;
    double global_vy = vy;
    
    // Rotate global vector into local frame
    vx = global_vx * c_th + global_vy * s_th;
    vy = -global_vx * s_th + global_vy * c_th;
  }

  // 2. Load input buffer
  robot_vel_ << vx, vy, omega;

  // 3. Matrix Math (Optimized)
  // .noalias() avoids creating a temporary return object
  temp_wheel_lin_vels_.noalias() = coupling_matrix_ * robot_vel_;
  
  // 4. Convert to Angular Velocity
  // Array-wise division
  wheel_vels_.noalias() = temp_wheel_lin_vels_ / params_.wheel_radius;

  return wheel_vels_;
}

const Eigen::Vector3d & OmniKinematics::calculate_robot_velocity(const Eigen::VectorXd & wheel_angular_vels)
{
  if (wheel_angular_vels.size() != (long)params_.wheel_names.size()) {
    calc_robot_vel_.setZero();
    return calc_robot_vel_;
  }

  // v_wheels_lin = w * r
  temp_wheel_lin_vels_.noalias() = wheel_angular_vels * params_.wheel_radius;
  
  // v_robot = Inv * v_wheels_lin
  calc_robot_vel_.noalias() = inv_coupling_matrix_ * temp_wheel_lin_vels_;

  return calc_robot_vel_;
}

const OdometryState & OmniKinematics::integrate_odometry(const Eigen::Vector3d & robot_vel, double dt)
{
  double vx = robot_vel(0);
  double vy = robot_vel(1);
  double omega = robot_vel(2);

  // Simple Euler Integration
  double c_th = std::cos(state_.theta);
  double s_th = std::sin(state_.theta);

  // Rotate local velocity to global frame
  double dx = (vx * c_th - vy * s_th) * dt;
  double dy = (vx * s_th + vy * c_th) * dt;
  double dth = omega * dt;

  state_.x += dx;
  state_.y += dy;
  state_.theta += dth;

  return state_;
}

}  // namespace omnidirectional_driver
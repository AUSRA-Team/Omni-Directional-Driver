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
  
  // Initialize noise matrix size
  wheel_noise_matrix_ = Eigen::MatrixXd::Zero(n, n);
  twist_covariance_.setZero();
  state_.pose_covariance.setZero();

  for (size_t i = 0; i < n; ++i) {
    coupling_matrix_(i, 0) = 1.0 * std::cos(params_.phi_rad[i] + params_.gamma_rad);
    coupling_matrix_(i, 1) = 1.0 * std::sin(params_.phi_rad[i] + params_.gamma_rad);
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

const Eigen::Vector3d & OmniKinematics::calculate_robot_velocity(const Eigen::VectorXd & wheel_angular_vels, double dt)
{
  if (wheel_angular_vels.size() != (long)params_.wheel_names.size()) {
    calc_robot_vel_.setZero();
    twist_covariance_.setZero();
    return calc_robot_vel_;
  }

  // 1. Calculate Linear Wheel Velocities
  temp_wheel_lin_vels_.noalias() = wheel_angular_vels * (std::cos(params_.gamma_rad) * params_.wheel_radius);
  
  // 2. Forward Kinematics
  calc_robot_vel_.noalias() = inv_coupling_matrix_ * temp_wheel_lin_vels_;

  // 3. Calculate Theoretical Covariance
  // Base Assumption: Quantization error is uniform distribution of 1 encoder tick width.
  // One tick distance (linear)
  double tick_dist = (2.0 * M_PI * params_.wheel_radius) / params_.encoder_ppr;
  
  // Variance of position error for uniform distribution = width^2 / 12
  double pos_var = (tick_dist * tick_dist) / 12.0;
  
  // Variance of velocity derived from position diff: Var(v) = (Var(p1) + Var(p2)) / dt^2 = 2*Var(p) / dt^2
  // We clamp dt to avoid division by zero
  double safe_dt = (dt < 1e-4) ? 1e-4 : dt;
  double vel_var = (2.0 * pos_var) / (safe_dt * safe_dt);

  // Fill diagonal noise matrix
  wheel_noise_matrix_.setZero();
  wheel_noise_matrix_.diagonal().fill(vel_var);

  // Propagate to Robot Frame: Sigma_robot = J_inv * Sigma_wheel * J_inv^T
  twist_covariance_ = inv_coupling_matrix_ * wheel_noise_matrix_ * inv_coupling_matrix_.transpose();

  // Apply empirical scale
  twist_covariance_ *= params_.covariance_scale_vel;

  return calc_robot_vel_;
}

const OdometryState & OmniKinematics::integrate_odometry(const Eigen::Vector3d & robot_vel, double dt)
{
  double vx = robot_vel(0);
  double vy = robot_vel(1);
  double omega = robot_vel(2);
  double th = state_.theta;

  // 1. Position Integration
  double dx_global = (vx * std::cos(th) - vy * std::sin(th)) * dt;
  double dy_global = (vx * std::sin(th) + vy * std::cos(th)) * dt;
  double dth = omega * dt;

  state_.x += dx_global;
  state_.y += dy_global;
  state_.theta += dth;

  // 2. Covariance Propagation (EKF Prediction Step)
  // Jacobian F of the motion model w.r.t State (x, y, theta)
  // x_new = x + (vx*c - vy*s)*dt
  // y_new = y + (vx*s + vy*c)*dt
  // th_new = th + w*dt
  
  // F is the partial derivative of the motion model with respect to the state x=[x,y,θ].  
  // F = [1, 0, (-vx*s - vy*c)*dt]
  //     [0, 1, (vx*c - vy*s)*dt ]
  //     [0, 0, 1                ]
  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  F(0, 2) = (-vx * std::sin(th) - vy * std::cos(th)) * dt;
  F(1, 2) = (vx * std::cos(th) - vy * std::sin(th)) * dt;

  // Jacobian G of motion model w.r.t Control Input (vx, vy, w) - for Process Noise
  // G is the partial derivative of the motion model with respect to the inputs u=[vx​,vy​,ω]
  // G = Rotation Matrix * dt
  Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
  G(0, 0) = std::cos(th) * dt; G(0, 1) = -std::sin(th) * dt;
  G(1, 0) = std::sin(th) * dt; G(1, 1) =  std::cos(th) * dt;
  G(2, 2) = dt;

  // Process Noise Q = G * TwistCovariance * G^T
  Eigen::Matrix3d Q = G * twist_covariance_ * G.transpose();

  // Apply empirical scales to process each wheel's noise contribution [Future Work: More Sophisticated Noise Models]
  // P_k = F * P_k-1 * F^T + Q
  state_.pose_covariance = F * state_.pose_covariance * F.transpose() + Q;
  
  return state_;
}

}  // namespace omnidirectional_driver
#ifndef OMNIDIRECTIONAL_DRIVER__OMNI_KINEMATICS_HPP_
#define OMNIDIRECTIONAL_DRIVER__OMNI_KINEMATICS_HPP_

#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace omnidirectional_driver
{

struct RobotParams
{
  std::vector<std::string> wheel_names;
  double robot_radius = 0.0;
  double wheel_radius = 0.0;
  std::vector<double> wheel_angles_deg;
  double roller_angle_deg = 0.0;
  bool use_field_centric = false;
  
  // Covariance / Noise Parameters
  int encoder_ppr = 1024;    // Pulses Per Revolution

  // Minimum dt used in the velocity-noise model (= 1 / nominal joint-state rate).
  // Clamping to this prevents jitter-induced messages (dt < nominal) from inflating
  // vel_var to astronomically large values. Default: 0.01 s (100Hz).
  double nominal_dt = 0.01;
  
  double covariance_scale_xy = 1.0;
  double covariance_scale_yaw = 1.0;
  double covariance_scale_velxy = 1.0;
  double covariance_scale_velyaw = 1.0;

  double covariance_offset_xy = 0.0;
  double covariance_offset_yaw = 0.0;
  double covariance_offset_velxy = 0.0;
  double covariance_offset_velyaw = 0.0;

  // Computed radians
  std::vector<double> phi_rad;
  double gamma_rad = 0.0;

  double acceleration_limit = 1.0; // m/s^2
};

struct OdometryState
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  
  // 3x3 Covariance Matrix (x, y, theta)
  Eigen::Matrix3d pose_covariance = Eigen::Matrix3d::Zero();
};

class OmniKinematics
{
public:
  OmniKinematics() = default;
  ~OmniKinematics() = default;

  void configure(const RobotParams & params);

  void reset_state() { state_ = OdometryState(); };

  const Eigen::VectorXd & calculate_wheel_commands(double vx, double vy, double omega, double current_heading);

  /**
   * @brief Calculates robot velocity and its covariance based on wheel velocities and dt.
   * @param dt Time delta in seconds (required for theoretical noise calc)
   */
  const Eigen::Vector3d & calculate_robot_velocity(const Eigen::VectorXd & wheel_angular_vels, double dt);

  const OdometryState & integrate_odometry(const Eigen::Vector3d & robot_vel, double dt);

  // Getters
  const OdometryState & get_state() const { return state_; }
  const RobotParams & get_params() const { return params_; }
  
  // Returns the instantaneous velocity covariance (3x3) calculated in calculate_robot_velocity
  const Eigen::Matrix3d & get_twist_covariance() const { return twist_covariance_; }

private:
  RobotParams params_;
  OdometryState state_;

  // Matrices
  Eigen::MatrixXd coupling_matrix_;      // 3xN
  Eigen::MatrixXd inv_coupling_matrix_;  // Nx3
  
  // Buffers
  Eigen::Vector3d robot_vel_;
  Eigen::VectorXd wheel_vels_;
  Eigen::VectorXd temp_wheel_lin_vels_;
  Eigen::Vector3d calc_robot_vel_;
  
  // Covariance Buffers
  Eigen::Matrix3d twist_covariance_;     // Current velocity covariance (local frame)
  Eigen::MatrixXd wheel_noise_matrix_;   // NxN diagonal matrix
};

}  // namespace omnidirectional_driver

#endif  // OMNIDIRECTIONAL_DRIVER__OMNI_KINEMATICS_HPP_
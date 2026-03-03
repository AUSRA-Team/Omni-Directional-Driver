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
  std::vector<double> wheel_direction_correction;
  bool use_field_centric = false;
  
  // Computed radians
  std::vector<double> phi_rad;
  double gamma_rad = 0.0;
};

struct OdometryState
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

class OmniKinematics
{
public:
  OmniKinematics() = default;
  ~OmniKinematics() = default;

  /**
   * @brief Pre-allocates matrices and pre-calculates trig values.
   * Called once at startup to avoid runtime allocation.
   */
  void configure(const RobotParams & params);

  /**
   * @brief High-performance Inverse Kinematics
   * @return Reference to the internal result vector (avoids copy)
   */
  const Eigen::VectorXd & calculate_wheel_commands(double vx, double vy, double omega, double current_heading);

  /**
   * @brief High-performance Forward Kinematics
   */
  const Eigen::Vector3d & calculate_robot_velocity(const Eigen::VectorXd & wheel_angular_vels);

  /**
   * @brief Updates internal state
   */
  const OdometryState & integrate_odometry(const Eigen::Vector3d & robot_vel, double dt);

  /**
   * @brief Reset odometry state to zero
   */
  void reset_state() { state_ = OdometryState(); }

  // Getters
  const OdometryState & get_state() const { return state_; }
  const RobotParams & get_params() const { return params_; }

private:
  RobotParams params_;
  OdometryState state_;

  // Pre-allocated Memory (Member Variables)
  // We keep these here so we don't re-create them in the loop
  Eigen::MatrixXd coupling_matrix_;      // 3xN
  Eigen::MatrixXd inv_coupling_matrix_;  // Nx3
  
  // Reuseable vectors for intermediate calculations
  Eigen::Vector3d robot_vel_;            // [vx, vy, w]
  Eigen::VectorXd wheel_vels_;           // Result buffer
  Eigen::VectorXd temp_wheel_lin_vels_;  // Intermediate buffer
  Eigen::Vector3d calc_robot_vel_;       // Result buffer
};

}  // namespace omnidirectional_driver

#endif  // OMNIDIRECTIONAL_DRIVER__OMNI_KINEMATICS_HPP_
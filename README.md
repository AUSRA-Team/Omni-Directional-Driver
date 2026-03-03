# Omni-Directional Driver

A configurable ROS 2 C++ driver for a 3-wheeled omnidirectional (holonomic) mobile robot using inverse kinematics.

## Package Overview

This package converts linear and angular velocity commands (`/cmd_vel`) from the navigation stack into individual wheel angular speeds for the three omni-wheels. It computes the inverse kinematics based on the robot's specific geometry and sends the corresponding actuator commands to a `ros2_control` velocity controller.

### Supported Features
*   **ROS 2 Humble** (Ubuntu 22.04)
*   **C++ Node** (Replacing the older Python version)

## Build Instructions

If you previously used the Python version, ensure you clean up properly before building the new C++ driver:
```bash
cd ~/ausra_gp
rm -rf build/omnidirectional_driver/ install/omnidirectional_driver/
colcon build --packages-select omnidirectional_driver --symlink-install
source install/setup.bash
```

## Running the Simulation (Workflow)

This driver is automatically loaded and utilized as part of the robot's bringup stack. You do not typically run this node standalone in the final simulation workflow.

However, if you wish to verify the driver in isolation after building it:
```bash
ros2 run omnidirectional_driver omni_driver
```
*Note: This driver listens to `/cmd_vel` and expects `joint_group_velocity_controller/commands` to be active in the system.*

## Key Parameters & Tuning

The robot geometry used for inverse kinematics calculations is defined in `config/<your-robot-params>.yaml`.

### Physical Robot Parameters

*   `robot_radius`: `0.1105` (Distance from center to wheel contact point in meters).
*   `wheel_radius`: `0.034` (Physical radius of the wheel in meters).
*   `wheel_names`: Defines the ROS 2 Control joint names (e.g., `joint_1`, `joint_2`, `joint_3`).

### Kinematic Geometry

*   `wheel_angles_deg`: `[270.0, 30.0, 150.0]`
    *   Mounting angle of each wheel relative to the robot's base frame. Must match the physical (or URDF) wheel order.
*   `roller_angle_deg`: `0.0`
    *   Roller angle for omni wheels (for Mecanum wheels, this would typically be 45 degrees).

## ROS 2 Interfaces

### Subscribed Topics
*   `/cmd_vel` (`geometry_msgs/Twist`): Receives the velocity command from the Navigation stack (or teleoperation).

### Published Topics
*   `/joint_group_velocity_controller/commands` (`std_msgs/Float64MultiArray`): Publishes the computed angular velocities for the 3 wheels, directly feeding them to `ros2_control`.

## Troubleshooting

1.  **Robot moves sideways when commanded forward, or rotates unexpectedly**:
    *   Check `wheel_angles_deg` order in the YAML configuration. It MUST match the physical/URDF order of the wheels (`joint_1`, `joint_2`, `joint_3`).
2.  **A specific wheel spins in the wrong direction**:
    *   Verify the URDF joint axis sign. Alternatively, negate the corresponding angle in the YAML config.
3.  **No motion output**:
    *   Ensure the `joint_group_velocity_controller` is running via `ros2 control list_controllers`.
    *   Echo the commands to verify the driver is calculating velocities: `ros2 topic echo /joint_group_velocity_controller/commands`.
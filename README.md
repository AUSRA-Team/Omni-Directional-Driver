# Omni-Driver for 3-Wheeled Omni-Directional Robot  
A fully configurable ROS 2 driver for a **3-wheeled omni-directional (holonomic) mobile robot** using inverse kinematics.  
This node converts `/cmd_vel` velocity commands into **wheel angular velocities** using your robot's geometry from a config file.

Supports:  
- **ROS 2 Humble** (Ubuntu 22.04)  
- **ROS 2 Jazzy** (Ubuntu 24.04)

---

# 1. Introduction  

This package provides a complete motion interface for omni-directional mobile robots that use **three omni wheels** positioned around the robot chassis. Unlike differential or mecanum drives, a 3-wheel omni robot can move:

- Forward/backward  
- Sideways (holonomic)  
- Rotate while moving  
- Move in any direction without changing orientation  

This driver handles the **inverse kinematics**, takes velocity commands from `/cmd_vel`, computes each wheel’s angular speed, and sends them to a **ros2_control** velocity controller.

---

# 2. Features  

✔ Fully configurable robot geometry via YAML file  
✔ Clean modular parameter class (`robot_params.py`)  
✔ Inverse kinematics automatically computed at runtime  
✔ ROS 2 Control compatible (`joint_group_velocity_controller`)  
✔ Lightweight Python implementation  
✔ Works with real hardware + Gazebo/Ignition simulation  
✔ Clean ROS interface for integration with navigation stacks  

---

# 3. Package Structure  

```

omni_driver/
│── omni_driver.py           # Main ROS 2 node (IK logic)
│── robot_params.py          # Data structure for robot geometry
│── config/
│     └── omni_params.yaml   # Robot parameters
│── launch/
│     └── omni_driver.launch.py
│── package.xml
│── setup.py
│── resource/

```

---

# 4. Installation  

### Install ROS 2 build tools  
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions build-essential
```

---

# 5. Building the Package

### Create a workspace

```bash
mkdir -p ~/omni_ws/src
cd ~/omni_ws/src
git clone <your-repo-url> omni_driver
```

### Build

```bash
cd ~/omni_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
```

---

# 6. Robot Configuration (YAML File)

Your robot geometry must be defined in:

`config/omni_params.yaml`

Example:

```yaml
omnidirectional_driver:
  ros__parameters:

    wheel_names: ["wheel_1_joint", "wheel_2_joint", "wheel_3_joint"]

    robot_radius: 0.17     # Distance from center to wheel contact point
    wheel_radius: 0.05     # Physical radius of the wheel

    # Wheel mounting angles (degrees)
    # 3-wheel omni: 0°, 120°, 240° is standard
    wheel_angles_deg: [0.0, 120.0, 240.0]

    # Omni wheel roller angle (usually 45°)
    roller_angle_deg: 45.0
```

### Parameter explanation

| Parameter          | Meaning                                          |
| ------------------ | ------------------------------------------------ |
| `wheel_names`      | Joint names used by ros2_control                 |
| `robot_radius`     | Distance from the robot center to each wheel     |
| `wheel_radius`     | Physical wheel radius                            |
| `wheel_angles_deg` | Mounting angle of each wheel in your robot frame |
| `roller_angle_deg` | Roller angle for omni wheels                     |

---

# 7. Running the Driver

### **Option 1 — Using a Launch File (Recommended)**

```bash
ros2 launch omni_driver omni_driver.launch.py
```

### **Option 2 — Running the Node Manually**

```bash
ros2 run omni_driver omni_driver --ros-args \
  --params-file config/omni_params.yaml
```

---

# 8. Publishing Commands to the Robot

Send a velocity command to move the robot:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "
linear:
  x: 0.2
  y: 0.0
angular:
  z: 0.4"
```

---

# 9. ROS 2 Interfaces

### **Subscribed Topics**

| Topic      | Type                  | Description            |
| ---------- | --------------------- | ---------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity command |

### **Published Topics**

| Topic                                       | Type                | Description                         |
| ------------------------------------------- | ------------------- | ----------------------------------- |
| `/joint_group_velocity_controller/commands` | `Float64MultiArray` | Angular velocities for the 3 wheels |

---

# 10. ROS 2 Control Integration

You must define a **JointGroupVelocityController**:

Example:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

joint_group_velocity_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointGroupVelocityController
    joints: ["wheel_1_joint", "wheel_2_joint", "wheel_3_joint"]
```

Make sure your URDF uses the same joint names.

---

# 11. Kinematics (Short Explanation)

The 3-wheel omni robot has wheels placed at angles:

[
\phi_1, \ \phi_2, \ \phi_3
]

Each wheel measures motion along its tangent direction.
For a robot twist:

[
[v_x, \ v_y, \ \omega_z]
]

The **inverse kinematics equation** for wheel ( i ) is:

[
\omega_i = \frac{1}{r_w}
(\cos\phi_i , v_x + \sin\phi_i , v_y - R\omega_z)
]

Where:

| Symbol       | Meaning                |
| ------------ | ---------------------- |
| ( r_w )      | Wheel radius           |
| ( R )        | Robot radius           |
| ( \phi_i )   | Wheel mounting angle   |
| ( \omega_i ) | Wheel angular velocity |

### Matrix Form (used in code)

[
\mathbf{ω} =
\frac{1}{r_w}
\begin{bmatrix}
\cos\phi_1 & \sin\phi_1 & -R \
\cos\phi_2 & \sin\phi_2 & -R \
\cos\phi_3 & \sin\phi_3 & -R
\end{bmatrix}
\begin{bmatrix}
v_x \ v_y \ \omega_z
\end{bmatrix}
]

This matrix is automatically generated at startup from your YAML config.

---

# 12. Troubleshooting

### 1. Robot moves sideways or rotates unexpectedly

* Check `wheel_angles_deg` order
* Must match the **physical order** of wheels

### 2. Wheel spins wrong direction

* Fix URDF joint axis sign
* OR negate the angle in YAML

### 3. No motion output

Check if controller is running:

```bash
ros2 control list_controllers
```

Echo motor commands:

```bash
ros2 topic echo /joint_group_velocity_controller/commands
```

---

# 13. Extending the Driver (Optional Future Work)

You can extend the node with:

* Odometry computation
* TF broadcaster (`odom → base_link`)
* Encoder-based forward kinematics
* Limiters / smoothing filters
* Diagnostics and wheel failure monitoring

The structure of the package was written to be easy to extend.

---

# 14. License

MIT | Apache 2.0 | BSD — choose your preferred license.

---

# 15. Author

AUSRA-Team (2025)
Custom 3-Wheeled Omni-Directional Robot Driver

```
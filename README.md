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

# 2. Package Structure  

```

<your-workspace>/src/omnidirectional_driver/
│── include/omnidirectional_driver
│     ├── omni_kinematics.hpp         # Kinematics math functions
│     ├── omnidirectional_driver.hpp  # Driver class Node
│     └── visibility_control.hpp      # 
│── src/
│     ├── omni_kinematics.cpp         # Kinematics math functions
│     └── omnidirectional_driver.cpp  # Driver class Node
├── CMakeLists.txt
└── package.xml

```

---

# 3. Installation  

### Install ROS 2 build tools  
```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions build-essential
```

---

# 4. Building the Package

### Create a workspace

```bash
mkdir -p ~/<your-workspace>/src
cd ~/<your-workspace>/src
git clone https://github.com/AUSRA-Team/Omni-Directional-Driver.git omnidirectional_driver
```

### Build

```bash
cd ~/<your-workspace>
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select omnidirectional_driver
source install/setup.bash
```

---

# 5. Robot Configuration (YAML File)

Your robot geometry must be defined in:

`config/<your-robot-params>.yaml`

Example:

```yaml
omnidirectional_driver:
  ros__parameters:
    # ----------------------------------------
    # Physical Robot Parameters
    # ----------------------------------------

    robot_radius: 0.1105     # Distance from center to wheel contact point
    wheel_radius: 0.034      # Physical radius of the wheel
    
    wheel_names: 
      - joint_1
      - joint_2
      - joint_3

    # ----------------------------------------
    # Kinematic Geometry
    # ----------------------------------------

    # Wheel mounting angles (Wheel_frame -> Robot_frame) [degrees]
    ## 3-wheel omni: 0°, 120°, 240° is standard
    ## For our robot: 270°, 30°, 150°
    wheel_angles_deg: [270.0, 30.0, 150.0]

    # Omni wheel roller angle (Roller_frame -> Wheel_frame) [degrees]
    ## For Omni-wheels: 0°
    ## For Mecanum-wheels: 45°
    roller_angle_deg: 0.0
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

# 6. Publishing Commands to the Robot

Send a velocity command to move the robot:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.1}}"
```

---

# 7. ROS 2 Interfaces

### **Subscribed Topics**

| Topic      | Type                  | Description            |
| ---------- | --------------------- | ---------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity command |

### **Published Topics**

| Topic                                       | Type                | Description                         |
| ------------------------------------------- | ------------------- | ----------------------------------- |
| `/joint_group_velocity_controller/commands` | `Float64MultiArray` | Angular velocities for the 3 wheels |

---

# 8. ROS 2 Control Integration

You must define a **JointGroupVelocityController**:

Example:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 20  # Hz
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # Standard controller to handle the "Command Interface"
    joint_group_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

# Configuration for the standard velocity controller
joint_group_velocity_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
```

Make sure your URDF uses the same joint names.

---

# 9. Troubleshooting

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

# 10. License

MIT | Apache 2.0 | BSD — choose your preferred license.

---

# 11. Author

AUSRA-Team (2025)
Custom 3-Wheeled Omni-Directional Robot Driver
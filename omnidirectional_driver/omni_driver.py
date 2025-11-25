#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster

# Import the separated class
try:
    from robot_params import RobotParams
except ImportError:
    from omnidirectional_driver.robot_params import RobotParams

class OmniDriver(Node):
    def __init__(self):
        super().__init__('omnidirectional_driver') 

        # --- 1. Init Params ---
        self.robot_params = RobotParams()

        # --- 2. Declare Parameters ---
        self.declare_parameter('wheel_names', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('robot_radius', rclpy.Parameter.Type.DOUBLE) 
        self.declare_parameter('wheel_radius', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('wheel_angles_deg', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('roller_angle_deg', rclpy.Parameter.Type.DOUBLE)

        # --- 3. Load Parameters ---
        try:
            self.robot_params.wheel_names = self.get_parameter('wheel_names').value
            self.robot_params.robot_radius = self.get_parameter('robot_radius').value
            self.robot_params.wheel_radius = self.get_parameter('wheel_radius').value
            self.robot_params.wheel_angles_deg = self.get_parameter('wheel_angles_deg').value
            self.robot_params.roller_angle_deg = self.get_parameter('roller_angle_deg').value
        except Exception as e:
            self.get_logger().error(f"CRITICAL ERROR: Could not load parameters! {e}")
            return

        # --- 4. Process Geometry ---
        if self.robot_params.wheel_angles_deg:
            self.robot_params.phi = [np.radians(a) for a in self.robot_params.wheel_angles_deg]
        self.robot_params.gamma = np.radians(self.robot_params.roller_angle_deg)

        # --- 5. Build Kinematic Matrix ---
        if self.robot_params.phi:
            matrix_rows = []
            for i in range(len(self.robot_params.wheel_names)):
                angle = self.robot_params.phi[i]
                
                # Inverse Kinematics Matrix (Robot Twist -> Wheel Speed)
                row = [
                     np.cos(angle),
                     np.sin(angle),
                     -1.0 * self.robot_params.robot_radius
                ]
                matrix_rows.append(row)

            self.coupling_matrix = np.array(matrix_rows)
        else:
             self.get_logger().error("Kinematics could not be calculated: Missing phi angles.")

        # --- 6. Setup ROS Interfaces ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher for ROS 2 Control (Group Velocity Controller)
        self.motor_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/joint_group_velocity_controller/commands', 
            10
        )

        self.get_logger().info("Omni Driver Ready: Publishing to /joint_group_velocity_controller/commands")

    def cmd_vel_callback(self, msg:Twist):
        """
        INVERSE KINEMATICS: Twist -> Wheel Velocities -> Ros2 Control Topic
        """
        if self.robot_params.wheel_radius == 0.0:
            return

        robot_vel = np.array([[msg.linear.x], [msg.linear.y], [msg.angular.z]])
        
        # Calculate Wheel Velocities
        wheel_lin_vels = self.coupling_matrix @ robot_vel
        wheel_ang_vels = wheel_lin_vels / self.robot_params.wheel_radius

        # Publish to Joint Group Velocity Controller
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(val) for val in wheel_ang_vels.flatten()]
        
        self.motor_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
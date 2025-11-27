#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

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

        # --- State Variables for Odometry ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

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
        self.coupling_matrix = np.zeros((len(self.robot_params.wheel_names), 3))

        if self.robot_params.phi:
            for i in range(len(self.robot_params.wheel_names)):
                angle = self.robot_params.phi[i]
                
                # Inverse Kinematics Matrix (Robot Twist -> Wheel Speed)
                self.coupling_matrix[i] = [
                     np.cos(angle),
                     np.sin(angle),
                     -1.0 * self.robot_params.robot_radius
                ]

            self.inv_coupling_matrix = np.linalg.pinv(self.coupling_matrix)
        else:
             self.get_logger().error("Kinematics could not be calculated: Missing phi angles.")

        # --- 6. Setup ROS Interfaces ---
        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for ROS 2 Control (Group Velocity Controller)
        self.motor_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_velocity_controller/commands', 10
        )

        self.get_logger().info("Omni Driver Ready: Publishing to /joint_group_velocity_controller/commands")

    def cmd_vel_callback(
            self,
            vel: Twist
            ) -> None:
        '''
        INVERSE KINEMATICS: Twist -> Wheel Velocities -> Ros2 Control Topic

        :param vel: Desired Robot Velocity (Twist Message)
        
        :return: None
        '''
        if self.robot_params.wheel_radius == 0.0:
            return

        robot_vel = np.array([[vel.linear.x], [vel.linear.y], [vel.angular.z]])
        
        # Calculate Wheel Velocities
        wheel_lin_vels = self.coupling_matrix @ robot_vel
        wheel_ang_vels = wheel_lin_vels / self.robot_params.wheel_radius

        # Publish to Joint Group Velocity Controller
        cmd_vel = Float64MultiArray()
        cmd_vel.data = [float(val) for val in wheel_ang_vels.flatten()]
        
        self.motor_cmd_pub.publish(cmd_vel)

    def joint_state_callback(
            self,
            state: JointState
            ) -> None:
        '''
        FORWARD KINEMATICS: Wheel Velocities -> Twist -> Odometry
        
        :param state: Joint State Message
        
        :return: None
        '''
        if not self.robot_params.wheel_names:
            return

        wheel_vels = np.zeros((len(self.robot_params.wheel_names), 1))
        
        # Match joint names from params
        for i, target_name in enumerate(self.robot_params.wheel_names):
            if target_name in state.name:
                idx = state.name.index(target_name)
                wheel_vels[i] = state.velocity[idx]

        # Forward Kinematics
        v_wheels = wheel_vels * self.robot_params.wheel_radius
        v_robot = self.inv_coupling_matrix @ v_wheels
        
        vx, vy, omega = v_robot[0], v_robot[1], v_robot[2]

        # Integration
        now = self.get_clock().now()
        dt = float((now - self.last_time).nanoseconds / 1e9)
        self.last_time = now

        delta_x = (vx * np.cos(self.theta) - vy * np.sin(self.theta)) * dt
        delta_y = (vx * np.sin(self.theta) + vy * np.cos(self.theta)) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odometry(self.x, self.y, self.theta, vx, vy, omega, now)

    def publish_odometry(
            self,
            x: float,
            y: float,
            theta: float,
            vx: float,
            vy: float,
            vtheta: float,
            time_now: Time
            ) -> None:
        '''
        Publish TF and Odometry Messages
        
        :param x: X Position
        :param y: Y Position
        :param theta: Yaw Angle
        :param vx: Linear Velocity in X
        :param vy: Linear Velocity in Y
        :param vtheta: Angular Velocity
        :param time_now: Current Time

        :return: None
        '''
        q = quaternion_from_euler(0, 0, theta)
        
        # TF Broadcast
        # t = TransformStamped()
        # t.header.stamp = time_now.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'robot_footprint'
        # t.transform.translation.x = float(x)
        # t.transform.translation.y = float(y)
        # t.transform.translation.z = 0.0
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)

        # Pose Odometry (Global frame)
        odom = Odometry()
        odom.header.stamp = time_now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'robot_footprint'
        
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Velocity Twist (Local frame)
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(vtheta)
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OmniDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
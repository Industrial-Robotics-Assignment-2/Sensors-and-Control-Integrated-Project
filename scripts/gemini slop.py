#!/usr/bin/env python3

import math
import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
import control as ct


class LQRTrajectoryTracker(Node):
    def __init__(self):
        super().__init__('lqr_trajectory_tracker')

        # --- Subscriptions & Publishers ---
        # Strictly subscribes to estimated state from EKF (Never ground truth)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ekf_pose',
            self.pose_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Control Loop Parameters ---
        self.dt = 0.05              # 20 Hz control execution rate
        self.v_ref = 0.18           # Nominal forward speed (m/s)
        self.goal_tolerance = 0.20  # Waypoint arrival threshold (meters)

        # --- LQR Cost Matrices (Tuning Weights) ---
        # Q_lqr penalizes state tracking errors [e_x, e_y, e_theta]
        self.Q_lqr = np.diag([2.0, 5.0, 1.2])
        # R_lqr penalizes control effort commands [delta_v, delta_omega]
        self.R_lqr = np.diag([0.2, 0.2])

        # --- Global Waypoints [(x, y), ...] ---
        self.waypoints = [
            (1.5, 0.0),
            (2.5, 1.5),
            (1.0, 2.5),
            (0.0, 0.0)  # Entrance to docking region
        ]
        self.current_wp_idx = 0

        # --- Internal States ---
        self.current_pose = None    # [x, y, theta]
        self.min_front_dist = float('inf')

        # Control loop timer at 20 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("LQR Trajectory Tracking Node Started.")

    def pose_callback(self, msg: PoseStamped):
        """Extract estimated state [x, y, theta] published by EKF node."""
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Convert quaternion (q_z, q_w) back to yaw heading (theta)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)

        self.current_pose = np.array([x, y, theta])

    def scan_callback(self, msg: LaserScan):
        """Process 2D LiDAR scans to monitor obstacle clearance in front cone."""
        if len(msg.ranges) == 0:
            return

        num_ranges = len(msg.ranges)
        # Sector indexing for front 60-degree arc (-30 to +30 degrees)
        front_indices = list(range(0, 30)) + list(range(num_ranges - 30, num_ranges))
        valid_ranges = [msg.ranges[i] for i in front_indices if 0.05 < msg.ranges[i] < msg.range_max]

        self.min_front_dist = min(valid_ranges) if valid_ranges else msg.range_max

    def compute_lqr_gain(self, v_r):
        """Linearize error dynamics around current reference speed v_r and solve DARE."""
        # Continuous linear tracking error Jacobians: e_dot = A*e + B*u
        A = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, v_r],
            [0.0, 0.0, 0.0]
        ])

        B = np.array([
            [1.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0]
        ])

        # Discretize system using Forward Euler approximation (A_d = I + A*dt, B_d = B*dt)
        A_d = np.eye(3) + A * self.dt
        B_d = B * self.dt

        # Solve Discrete Algebraic Riccati Equation (DARE)
        P = scipy.linalg.solve_discrete_are(A_d, B_d, self.Q_lqr, self.R_lqr)

        # Compute optimal LQR gain matrix K (2x3 matrix)
        K = np.linalg.inv(self.R_lqr + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
        return K

    def control_loop(self):
        """Execute LQR tracking and safety check at 20 Hz."""
        if self.current_pose is None:
            self.get_logger().warn("Waiting for EKF pose estimate...", throttle_duration_sec=2.0)
            return

        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info("All room waypoints reached! Ready for visual docking.")
            self.cmd_pub.publish(Twist())  # Stop robot
            return

        # 1. Target Waypoint Kinematics
        target_x, target_y = self.waypoints[self.current_wp_idx]
        curr_x, curr_y, curr_theta = self.current_pose

        dx = target_x - curr_x
        dy = target_y - curr_y
        dist_to_goal = math.hypot(dx, dy)

        # Waypoint Advancement Check
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Reached Waypoint {self.current_wp_idx + 1}")
            self.current_wp_idx += 1
            return

        target_theta = math.atan2(dy, dx)

        # 2. Map Global Error vector to Robot Local Body Frame
        global_err = np.array([dx, dy, self.normalize_angle(target_theta - curr_theta)])

        R_body = np.array([
            [math.cos(curr_theta),  math.sin(curr_theta), 0.0],
            [-math.sin(curr_theta), math.cos(curr_theta), 0.0],
            [0.0,                   0.0,                  1.0]
        ])

        # Local body error vector e = [e_x, e_y, e_theta]^T
        e = R_body @ global_err

        # 3. LiDAR Obstacle Safety Speed Scaling
        v_ref_active = self.v_ref
        if self.min_front_dist < 0.5:
            # Dynamically reduce forward velocity as obstacle approaches
            speed_factor = max(0.0, (self.min_front_dist - 0.15) / 0.35)
            v_ref_active *= speed_factor
            self.get_logger().warn(
                f"Obstacle close! Modulating velocity to {v_ref_active:.2f} m/s",
                throttle_duration_sec=1.0
            )

        # 4. Compute Dynamic LQR Feedback Gain Matrix K
        effective_v = max(v_ref_active, 0.02)  # Avoid zero speed singularity
        K = self.compute_lqr_gain(effective_v)

        # 5. Apply Control Law: u_lqr = K * e
        u_lqr = K @ e

        cmd = Twist()
        cmd.linear.x = max(0.0, v_ref_active * math.cos(e[2]) + u_lqr[0])
        cmd.angular.z = u_lqr[1]

        # Velocity Saturation Thresholds for TurtleBot3
        cmd.linear.x = min(cmd.linear.x, 0.22)
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = LQRTrajectoryTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # Safety stop
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
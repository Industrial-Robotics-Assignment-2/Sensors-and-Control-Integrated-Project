import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
import control as ct
import scipy

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LQR_Controller(Node):
    def __init__(self):
        super().__init__('LQR_Controller')

        qos_profile = QoSProfile(
                            reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=5
                        )

        self.estimated_pose = self.create_subscription(PoseStamped, '/ekf_estimated_pose', self.pose_callback, qos_profile=qos_profile)
        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile)

        # Interals
        self.waypoints = np.array([

        ])
        self.v_ref = 0.18
        self.T = 0.02 # 50hz
        self.goal_tolerance = 0.2

        self.max_ey = 0.1  # m
        self.max_etheta = 0.2  # rad
        self.max_domega = 1.0  # rad/s

        # Q_lqr penalizes state tracking errors [e_x, e_y, e_theta]
        self.Q_lqr = np.diag([2.0, 5.0, 1.2])
        # R_lqr penalizes control effort commands [delta_v, delta_omega]
        self.R_lqr = np.diag([0.2, 0.2])

        self.X = {'x': 0.0,
                  'y': 0.0,
                  'theta': 0.0
                  }

        # timer
        self.timer = self.create_timer(self.T, self.control_loop)

    def pose_callback(self, msg: PoseStamped):

        x = msg.pose.position.x
        y = msg.pose.position.y

        # Convert quaternion (q_z, q_w) back to yaw heading (theta)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)

        self.X['x'] = x
        self.X['y'] = y
        self.X['theta'] = theta

    def scan_callback(self, msg: LaserScan):
        pass

    def compute_lqr_gain(self, v_ref: float):

        A = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, v_ref],
            [0.0, 0.0, 0.0]
        ])

        B = np.array([
            [1.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0]
        ])

        C = np.eye(3)
        D = np.zeros([3,2])

        sys_c = ct.ss(A, B, C, D)
        sys_d = ct.c2d(sys_c, self.T, method="zoh")

        K, S, E = ct.dlqr(sys_d.A, sys_d.B, self.Q_lqr, self.R_lqr)
        # control library wasnt working so commented out

        # A_d = np.eye(3) + A * self.T
        # B_d = B * self.T

        # # Solve Discrete Algebraic Riccati Equation (DARE)
        # P = scipy.linalg.solve_discrete_are(A_d, B_d, self.Q_lqr, self.R_lqr)

        # # Compute optimal LQR gain matrix K (2x3 matrix)
        # K = np.linalg.inv(self.R_lqr + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)

        return K

    def control_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = LQR_Controller()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'LQR Controller Node has stopped'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()


    
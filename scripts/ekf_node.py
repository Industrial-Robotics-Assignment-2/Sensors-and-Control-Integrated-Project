import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
import open3d as o3d

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Turtlebot(Node):

    def __init__(self):
        super().__init__('turtlebot')

        qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=5
                )

        # subscriptions
        self.odom = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos_profile) # NOT GROUND TRUTH
        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile)

        # publishers
        self.pose_ekf = self.create_publisher(PoseStamped,'/ekf_estimated_pose', qos_profile=qos_profile) # dead reckoning pose - GLOBAL
        self.pose_ekf_X = self.create_publisher(PoseStamped,'/ekf_estimated_pose_X', qos_profile=qos_profile) # ekf pose - GLOBAL

        # Internal
        
        # EKF
        self.current_ekf_pose = np.array([ # State Space Model of Turtlebot3
            0.0, # x
            0.0, # y
            0.0  # theta
            ])
        
        self.prev_ekf_pose = None
        self.prev_pts = None
        self.prev_odom_time = None

        self.P = np.diag([0.3**2, 0.3**2, np.deg2rad(10.0)**2]) # Error Covariance Matrix, how certain it is about its position

        self.sigma_xy = 0.02  # position uncertainty std dev per sample (meters)
        self.sigma_theta = np.deg2rad(4.0)  # heading uncertainty std dev per sample (radians)
        self.Q = np.diag([self.sigma_xy**2, self.sigma_xy**2, self.sigma_theta**2]) # Noise Covariance Matrix, rn im using the same matrix for the process noise covariance (Q), and the Measuremnt noise covariance (R). split this if needed
        self.R = self.Q
        self.max_icp_dist = 0.2

        self.get_logger().info(
            'Turtlebot Node is online'
        )

    def scan_callback(self, msg: LaserScan):

        current_pts = self.laser_scan_to_pts(msg)
        current_ekf_pose = np.copy(self.current_ekf_pose)

        if self.prev_pts is not None and self.prev_ekf_pose is not None:

            current_pc = o3d.geometry.PointCloud()
            current_pc.points = o3d.utility.Vector3dVector(current_pts)

            prev_pc = o3d.geometry.PointCloud()
            prev_pc.points = o3d.utility.Vector3dVector(self.prev_pts)

            # 2. Compute global world displacement between scan snapshots
            dx_w = current_ekf_pose[0] - self.prev_ekf_pose[0]
            dy_w = current_ekf_pose[1] - self.prev_ekf_pose[1]
            prev_th = self.prev_ekf_pose[2]

            # 3. Rotate world displacement into local body frame of the previous scan
            dx_b =  np.cos(prev_th) * dx_w + np.sin(prev_th) * dy_w
            dy_b = -np.sin(prev_th) * dx_w + np.cos(prev_th) * dy_w
            dth_b = self.wrap_angle(current_ekf_pose[2] - prev_th)

            odom_trans_body = np.array([dx_b, dy_b, dth_b])

            init = self.x2matrix(odom_trans_body)
            # init = np.eye(4)

            result = o3d.pipelines.registration.registration_icp(
                source=current_pc,
                target=prev_pc,
                max_correspondence_distance=self.max_icp_dist,  # Maximum distance (meters) to pair points
                init=init, #initial guess from the odom prediction, if having issues with the x2matrix() function just replace with the I matrix
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
            )

            T = result.transformation  # 4x4 matrix

            dx = T[0, 3]
            dy = T[1, 3]
            dtheta = np.arctan2(T[1, 0], T[0, 0])
            icp_trans = np.array([dx, dy, dtheta])

            self.ekf_update(icp_trans, (odom_trans_body))

        self.prev_pts = np.copy(current_pts)
        self.prev_ekf_pose = np.copy(current_ekf_pose)

    def odom_callback(self,msg: Odometry):

        # === 1. Extract values from odometry ===

        # extract velocity from wheel odometry
        # vx is in the robots coordinate frame
        vx =  msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        current_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 # get current time in seconds

        # copy current pose
        px = self.current_ekf_pose[0]
        py = self.current_ekf_pose[1]
        theta = self.current_ekf_pose[2]

        # dt = self.T # make in sync with ros2 headers if having sync issues
        if self.prev_odom_time is None:
            self.prev_odom_time = current_odom_time
            return
        
        dt = current_odom_time - self.prev_odom_time

        # === 2. State Propagation === 
        
        EPS = 1e-5

        # straight line motion
        # Euler Integration, works well if moving in a straight line or sample rate is higher than 10Hz about
        px_next = px + vx * dt * np.cos(theta)
        py_next = py + vx * dt * np.sin(theta)
        theta_next = theta + wz * dt
        theta_next = self.wrap_angle(theta_next)
        
        # Update the unified EKF state vector
        self.current_ekf_pose[0] = px_next
        self.current_ekf_pose[1] = py_next
        self.current_ekf_pose[2] = theta_next

        # === 3. Compute state transition jacobian === 

        # again, euler method, implement conditional arc method if needed
        df_dtheta = np.array([-vx * dt * np.sin(theta), vx * dt * np.cos(theta), 0.0])
        
        # State Transition Jacobian
        F = np.eye(3)
        F[0, 2] = df_dtheta[0]
        F[1, 2] = df_dtheta[1]

        # Covariance Propagation
        self.P = F @ self.P @ F.T + self.Q

        # Publish predicted pose
        published_pose = self.x2pose(self.current_ekf_pose)
        self.pose_ekf.publish(published_pose)
        self.prev_odom_time = current_odom_time

    def ekf_update(self, icp_trans: npt.NDArray[np.float64], odom_trans: npt.NDArray[np.float64]):
        """compute the ekf update step and correct the state space vector"""

        # Innovation Residual (difference in odom and lidar measurements)
        y_k = icp_trans - odom_trans
        y_k[2] = self.wrap_angle(y_k[2])

        # Observation Model Matrix, 3x3 identity matrix since icp measurements [x,y,theta]T map exactly to odom prediction [x,y,theta]T
        H = np.eye(3)

        # Innovation Covariance
        S = H @ self.P @ H.T + self.R

        # Compute Kalmann Gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State Correction Vector
        delta_X_body = K @ y_k

        # Rotate local body correction into global world frame
        theta = self.current_ekf_pose[2]
        dx_world = np.cos(theta) * delta_X_body[0] - np.sin(theta) * delta_X_body[1]
        dy_world = np.sin(theta) * delta_X_body[0] + np.cos(theta) * delta_X_body[1]
        dtheta_world = delta_X_body[2]

        # Updtae state with the correction vector
        self.current_ekf_pose[0] += dx_world
        self.current_ekf_pose[1] += dy_world
        self.current_ekf_pose[2] = self.wrap_angle(self.current_ekf_pose[2] + dtheta_world)

        published_pose = self.x2pose(self.current_ekf_pose)
        self.pose_ekf_X.publish(published_pose)

        # Update Error Covariance Matrix P (Joseph Form for numerical stability)
        I = np.eye(3)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    @staticmethod
    def wrap_angle(angle: float):
        """wraps angle to pi,-pi"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    @staticmethod
    def laser_scan_to_pts(msg: LaserScan):
        """Convert LaserScan ranges into 2D Cartesian points in robot frame."""
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)

        # Filter invalid range readings
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        r = ranges[valid]
        a = angles[valid]

        # Convert polar to 2D Cartesian (x, y)
        pts = np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros_like(r))) # add a 3rd dimension for compatibility with open3d pointclouds
        return pts

    @staticmethod
    def x2pose(X: npt.NDArray[np.float64]):
        """convert 3x1 NDArray State Space Vector [x,y,theta] into a ROS2 PoseStamped"""

        published_pose = PoseStamped()

        # Translation
        published_pose.pose.position.x = X[0]
        published_pose.pose.position.y = X[1]
        published_pose.pose.position.z = 0.0

        # Orientation (Quarternions)
        published_pose.pose.orientation.x = 0.0
        published_pose.pose.orientation.y = 0.0
        published_pose.pose.orientation.z = np.sin(X[2] / 2)
        published_pose.pose.orientation.w = np.cos(X[2] / 2)

        return published_pose

    @staticmethod
    def x2matrix(X: npt.NDArray[np.float64]):
        """convert 3x1 NDArray State Space Vector [x,y,theta] into a 4x4 transformation matrix"""

        x,y,yaw = X

        c = np.cos(yaw)
        s = np.sin(yaw)

        matrix = np.array([
            [c,  -s,  0.0,  x  ],
            [s,   c,  0.0,  y  ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        return matrix

def main(args=None):
    rclpy.init(args=args)

    node = Turtlebot()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            'Turtlebot Node has stopped'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
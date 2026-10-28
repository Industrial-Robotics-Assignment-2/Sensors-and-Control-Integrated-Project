import numpy as np
import rclpy
from rclpy.node import Node
import open3d as o3d

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, LaserScan

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
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile)
        self.pose_ekf = self.create_publisher(PoseStamped,'/ekf_estimated_pose', qos_profile=qos_profile)

        # Internal
        self.current_ekf_pose = np.array([
            0, # x
            0, # y
            0  # theta
            ])
        
        self.prev_ekf_pose = None
        self.prev_pts = None
        self.T = 0.02 #50hz time interval
        self.P = np.diag([0.3**2, 0.3**2, np.deg2rad(10.0)**2]) # Error Covariance Matrix, how certain it is about its position
        self.sigma_xy = 0.02  # position uncertainty std dev per sample (meters)
        self.sigma_theta = np.deg2rad(4.0)  # heading uncertainty std dev per sample (radians)
        self.Q = np.diag([self.sigma_xy**2, self.sigma_xy**2, self.sigma_theta**2])
        self.max_icp_dist = 0.2

        self.get_logger().info(
            'Turtlebot Node is online'
        )

    def move(self, v=0.0, w=0.0):
        """Publishes Twist command to /cmd_vel topic"""

        q = Twist()
        q.angular.z = float(w)
        q.linear.x = float(v)

        self.cmd_vel.publish(q)

    def scan_callback(self, msg: LaserScan):

        current_pts = self.laser_scan_to_pts(msg)
        current_ekf_pose = self.current_ekf_pose

        if self.prev_pts is not None and self.prev_ekf_pose is not None:

            current_pc = o3d.geometry.PointCloud()
            current_pc.points = o3d.utility.Vector3dVector(current_pts)

            prev_pc = o3d.geometry.PointCloud()
            prev_pc.points = o3d.utility.Vector3dVector(self.prev_pts)

            init_trans = np.eye(4)

            result = o3d.pipelines.registration.registration_icp(
                source=current_pc,
                target=prev_pc,
                max_correspondence_distance=self.max_icp_dist,  # Maximum distance (meters) to pair points
                init=init_trans,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
            )

            T = result.transformation  # 4x4 matrix

            dx = T[0, 3]
            dy = T[1, 3]
            dtheta = np.arctan2(T[1, 0], T[0, 0])
            trans = np.array([dx, dy])

            if trans is not None:
                # Pass relative laser odometry to your EKF correction function
                self.ekf_correct_scan_match(trans[0], trans[1], dtheta, self.prev_ekf_pose)

        self.prev_pts = current_pts
        self.prev_ekf_pose = current_ekf_pose

    def odom_callback(self,msg: Odometry):

        # === 1. Extract values from odometry ===

        # extract velocity from wheel odometry
        # vx is in the robots coordinate frame
        vx =  msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        # copy
        px = self.current_ekf_pose[0]
        py = self.current_ekf_pose[1]
        theta = self.current_ekf_pose[2]

        dt = self.T # make in sync with ros2 headers if having sync issues

        # === 2. State Propagation === 

        EPS = 1e-5
       
        # straight line motion
        # Euler Integration, works well if moving in a straight line or sample rate is higher than 10Hz about
        px_next = px + vx * dt * np.cos(theta)
        py_next = py + vx * dt * np.sin(theta)
        theta_next = theta + wz * dt

        theta_next = self.wrap_angle(theta_next)

        # implement conditional exact arc integration if needed

        # === 3. Compute state transition jacobian === 

        # again, euler method, implement conditional arc method if needed
        df_dtheta = np.array([-vx * dt * np.sin(theta), vx * dt * np.cos(theta), 0.0])

        F = np.eye(3)
        F[0,2] = df_dtheta[0]
        F[1,2] = df_dtheta[1]

        # === 4. Covariance Propagation / Updating Internals === 

        self.P = F @ self.P @ F.T + self.Q
        self.current_ekf_pose[0] = px_next
        self.current_ekf_pose[1] = py_next
        self.current_ekf_pose[2] = theta_next

        # 5. === Create publish estimated pose ===
        published_pose = PoseStamped()
        published_pose.pose.position.x = self.current_ekf_pose[0]
        published_pose.pose.position.y = self.current_ekf_pose[1]
        published_pose.pose.position.z = 0.0

        published_pose.pose.orientation.x = 0.0
        published_pose.pose.orientation.y = 0.0
        published_pose.pose.orientation.z = np.sin(self.current_ekf_pose[2] / 2)
        published_pose.pose.orientation.w = np.cos(self.current_ekf_pose[2] / 2)

        self.pose_ekf.publish(published_pose)

    @staticmethod
    def wrap_angle(angle):
        """wraps angle to pi,-pi"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def laser_scan_to_pts(self, msg: LaserScan):
        """Convert LaserScan ranges into 2D Cartesian points in robot frame."""
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges)

        # Filter invalid range readings
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        r = ranges[valid]
        a = angles[valid]

        # Convert polar to 2D Cartesian (x, y)
        pts = np.column_stack((r * np.cos(a), r * np.sin(a)))
        return pts

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
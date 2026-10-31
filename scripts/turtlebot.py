import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu
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
        self.odom = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos_profile)
        self.camera_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/camera/aruco_pose',
            self.camera_callback,
            10,
        )
        self.imu = self.create_subscription(Imu, '/imu', self.callback, qos_profile=qos_profile)
        self.scan = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile=qos_profile)

        # publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile)
        self.pose_ekf = self.create_publisher(PoseStamped, '/ekf_estimated_pose', qos_profile=qos_profile)

        self.current_ekf_pose = np.zeros(3)
        self.previous_odom_time = None
        self.covariance = np.diag([1.0**2, 1.0**2, np.deg2rad(30.0)**2])
        self.process_noise = np.diag([0.02**2, 0.02**2, np.deg2rad(4.0)**2])

        self.get_logger().info(
            'Turtlebot Node is online'
        )

    def move(self, v=0.0, w=0.0):
        """Publishes Twist command to /cmd_vel topic"""

        q = Twist()
        q.angular.z = float(w)
        q.linear.x = float(v)

        self.cmd_vel.publish(q)

    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.previous_odom_time is None:
            self.previous_odom_time = current_time
            return

        dt = current_time - self.previous_odom_time
        self.previous_odom_time = current_time
        if dt <= 0.0 or dt > 1.0:
            return

        velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z
        relative_x, relative_y, relative_theta = self.current_ekf_pose
        distance = velocity * dt
        turn = angular_velocity * dt
        cos_turn = np.cos(turn)
        sin_turn = np.sin(turn)

        translated_x = relative_x - distance
        translated_y = relative_y
        self.current_ekf_pose = np.array([
            cos_turn * translated_x + sin_turn * translated_y,
            -sin_turn * translated_x + cos_turn * translated_y,
            self.wrap_angle(relative_theta - turn),
        ])

        transition = np.array([
            [cos_turn, sin_turn, 0.0],
            [-sin_turn, cos_turn, 0.0],
            [0.0, 0.0, 1.0],
        ])
        self.covariance = (
            transition @ self.covariance @ transition.T + self.process_noise * dt
        )
        self.publish_ekf_pose(msg.header)

    def camera_callback(self, msg):
        measurement = msg.pose.pose
        measurement_vector = np.array([
            measurement.position.x,
            measurement.position.y,
            self.quaternion_to_yaw(measurement.orientation),
        ])
        innovation = measurement_vector - self.current_ekf_pose
        innovation[2] = self.wrap_angle(innovation[2])
        measurement_covariance = np.diag([
            msg.pose.covariance[0],
            msg.pose.covariance[7],
            msg.pose.covariance[35],
        ])
        measurement_covariance[measurement_covariance <= 0.0] = np.diag(
            [0.03**2, 0.03**2, 0.08**2]
        )[measurement_covariance <= 0.0]

        innovation_covariance = self.covariance + measurement_covariance
        kalman_gain = np.linalg.solve(
            innovation_covariance,
            self.covariance,
        ).T
        self.current_ekf_pose = self.current_ekf_pose + kalman_gain @ innovation
        self.current_ekf_pose[2] = self.wrap_angle(self.current_ekf_pose[2])
        identity = np.eye(3)
        self.covariance = (identity - kalman_gain) @ self.covariance
        self.publish_ekf_pose(msg.header)

    def publish_ekf_pose(self, header):
        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = self.current_ekf_pose[0]
        pose.pose.position.y = self.current_ekf_pose[1]
        pose.pose.orientation.z = np.sin(self.current_ekf_pose[2] / 2.0)
        pose.pose.orientation.w = np.cos(self.current_ekf_pose[2] / 2.0)
        self.pose_ekf.publish(pose)

    @staticmethod
    def quaternion_to_yaw(orientation):
        return np.arctan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def callback(self, msg):
        pass

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

# while True:
#     rclpy.spin_once(node)
#     w = 0.5
#     v = 1
#     node.move(v=v, w=w)
#     node.get_logger().info(
#         'published to /cmd_vel'
#     )
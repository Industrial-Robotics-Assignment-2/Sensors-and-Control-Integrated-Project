import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
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
        # callbacks are placeholders rn change them if needed
        self.odom = self.create_subscription(Odometry, '/odom', self.callback, qos_profile=qos_profile)
        self.imu = self.create_subscription(Imu, '/imu', self.callback, qos_profile=qos_profile)
        self.scan = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile=qos_profile)

        # publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile)

        self.get_logger().info(
            'Turtlebot Node is online'
        )

    def move(self, v=0.0, w=0.0):
        """Publishes Twist command to /cmd_vel topic"""

        q = Twist()
        q.angular.z = float(w)
        q.linear.x = float(v)

        self.cmd_vel.publish(q)

    # === PLACEHOLDER ===
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
import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import control as ct

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LQR_Controller(Node):
    def __init__(self):
        super().__init__('LQR_Controller')

        qos_profile = QoSProfile(
                            reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=5
                        )

        # subscribers
        self.estimated_pose = self.create_subscription(PoseStamped, '/ekf_estimated_pose_X', self.pose_callback, qos_profile=qos_profile)
        self.scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile)
        self.arcuo_pose = self.create_subscription(PoseWithCovarianceStamped, '/camera/aruco_pose', self.aruco_pose_callback, qos_profile=qos_profile)
        self.tag_in_view = self.create_subscription(Bool, '/camera/tag_in_view', self.tag_in_view_callback, qos_profile=qos_profile)
        # publishers
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos_profile)

        # Interals
        self.current_pose = np.array([
            0.0, # x
            0.0, # y
            0.0  # theta
        ])

        # self.waypoints = [ # just for testing change these later
        #     [1.0, 1.0, 0.0],
        #     [2.0, 2.0, np.pi / 2],
        #     [1.0, 1.0, 0.0],
        #     [0.0, 0.0, np.pi]
        # ]
        self.waypoints = np.array([
            [1.0, -1.0, 0.0]
        ])
        self.current_waypoint_index = 0
        self.current_scan = None
        self.latest_aruco_pose = None
        self.latest_aruco_normal_vec = None
        self.obstacle_avoidance_flag = True
        self.tag_in_view = None
        self.turtlebot_state = 'NAVIGATING'

        self.v_ref = 0.18 # m/s
        self.w_ref = 0.2 # rad/s
        self.k_rep = 0.35 # Repulsive gain for angular steering
        self.max_v = 0.22 # m/s
        self.max_w = 1.0 # rad/s
        self.minimum_obstacle_dist = 0.60 # m
        self.minimum_angle_diff = np.deg2rad(5)
        self.dock_stop_dist = 0.10 # m
        self.T = 0.02 # 50hz
        self.goal_tolerance = 0.20 # m

        self.max_ey = 0.1  # m
        self.max_etheta = 0.2  # rad
        self.max_domega = 1.0  # rad/s

        # Q_lqr penalizes state tracking errors [e_x, e_y, e_theta]
        self.Q_lqr = np.diag([2.0, 5.0, 1.2])
        # R_lqr penalizes control effort commands [delta_v, delta_omega]
        self.R_lqr = np.diag([0.2, 0.2])
        self.K_lqr = self.compute_lqr_gain(self.v_ref)

        self.k_dock_v = 0.25            # Proportional gain for docking linear speed
        self.k_dock_w = 1.50            # Proportional gain for docking steering
        self.dock_stop_dist = 0.18      # Stop distance to ArUco marker (m)

        # timer
        self.timer = self.create_timer(self.T, self.control_loop)
        self.get_logger().info(
            'LQR Controller is Online'
        )

    def pose_callback(self, msg: PoseStamped):

        x = msg.pose.position.x
        y = msg.pose.position.y

        # Convert quaternion (q_z, q_w) back to yaw heading (theta)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)

        pose = np.array([x,y,theta])
        self.current_pose = pose

    def scan_callback(self, msg: LaserScan):
        self.current_scan = msg

    def aruco_pose_callback(self, msg: PoseWithCovarianceStamped):
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y

        # # Convert quaternion (q_z, q_w) back to yaw heading (theta)
        # qz = msg.pose.pose.orientation.z
        # qw = msg.pose.pose.orientation.w
        # theta = 2.0 * np.arctan2(qz, qw)

        # pose = np.array([x,y,theta])
        # self.latest_aruco_pose = pose

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # Full 3D rotation matrix from base_link to ArUco tag frame
        q = [ori.x, ori.y, ori.z, ori.w]
        R_B_D = Rotation.from_quat(q).as_matrix()

        # ArUco tag normal (pointing OUT of tag face) is +Z axis in tag frame
        normal_in_base = R_B_D @ np.array([0.0, 0.0, 1.0])

        # Tag orientation (heading) in 2D ground plane
        normal_yaw = np.arctan2(normal_in_base[1], normal_in_base[0])

        # Store position and 2D ground-plane normal heading
        self.latest_aruco_pose = np.array([pos.x, pos.y, normal_yaw])
        self.latest_aruco_normal_vec = normal_in_base[:2]  # [nx, ny]

    def tag_in_view_callback(self, msg: Bool):
        self.tag_in_view = msg.data

    def compute_lqr_gain(self, v_ref: float):

        A = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, v_ref],
            [0.0, 0.0, 0.0]
        ])

        B = np.array([
            [-1.0, 0.0],
            [0.0, 0.0],
            [0.0, -1.0]
        ])

        # === CONTROL LIBRARY CODE ===
        C = np.eye(3)
        D = np.zeros([3,2])

        sys_c = ct.ss(A, B, C, D)
        sys_d = ct.c2d(sys_c, self.T, method="zoh")

        K, S, E = ct.dlqr(sys_d.A, sys_d.B, self.Q_lqr, self.R_lqr)
        # === CONTROL LIBRARY CODE ===

        """
        for whatever reason the control library must use a later version of numpy that other libraries dont support
        if youre getting an error that says smth like "A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6 as it may crash."
        then comment out the control library code and uncomment the not control library code. They should do the same thing. idfk man
        """

        # === NOT CONTROL LIBRARY CODE ===
        # A_d = np.eye(3) + A * self.T
        # B_d = B * self.T

        # # Solve Discrete Algebraic Riccati Equation (DARE)
        # P = scipy.linalg.solve_discrete_are(A_d, B_d, self.Q_lqr, self.R_lqr)

        # # Compute optimal LQR gain matrix K (2x3 matrix)
        # K = np.linalg.inv(self.R_lqr + B_d.T @ P @ B_d) @ (B_d.T @ P @ A_d)
        # === NOT CONTROL LIBRARY CODE ===

        return K

    def compute_obstacle_repulsion(self):
        if self.current_scan is None or self.obstacle_avoidance_flag is False:
            return 0.0, 0.0

        ranges = np.array(self.current_scan.ranges)
        angles = self.current_scan.angle_min + np.arange(len(ranges)) * self.current_scan.angle_increment

        # Restrict cone to front view (-60 to +60 degrees)
        max_angle = np.deg2rad(60.0)
        valid_mask = (ranges > self.current_scan.range_min) & (ranges < self.minimum_obstacle_dist) & (np.abs(angles) < max_angle)
        
        if not np.any(valid_mask):
            return 0.0, 0.0

        r_valid = ranges[valid_mask]
        a_valid = angles[valid_mask]

        angular_weight = np.cos(a_valid * (np.pi / (2.0 * max_angle)))

        # Calculate APF repulsive forces
        rep_magnitude = (1.0 / r_valid - 1.0 / self.minimum_obstacle_dist) / (r_valid**2) * angular_weight
        Fx = np.mean(-rep_magnitude * np.cos(a_valid))
        Fy = np.mean(-rep_magnitude * np.sin(a_valid))

        return Fx, Fy

    def control_loop(self):
        match self.turtlebot_state:
            case 'NAVIGATING':
                self.navigating_loop()
            case 'SEARCHING':
                self.searching_dock_loop()
            case 'ALIGNING':
                self.aligning_loop()
            case 'DOCKING':
                self.docking_loop()
            case 'DOCKED':
                self.publish_cmd(0.0, 0.0)

    def navigating_loop(self):
        target = self.waypoints[self.current_waypoint_index]
        reached = self.go_to_point(target=target)

        if reached:
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                self.get_logger().info(f'Switching to Waypoint #{self.current_waypoint_index}')
            else:
                # Goal Reached: Stop Robot
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    'Final waypoint reached. Searching for charging dock...'
                )
                self.turtlebot_state = 'SEARCHING'

    def searching_dock_loop(self):
        """Rotate slowly until the ArUco tag is detected by camera."""
        if self.tag_in_view and self.latest_aruco_pose is not None:
            self.get_logger().info('ArUco Tag acquired. Transitioning to ALIGNING visual servoing.')
            self.turtlebot_state = 'ALIGNING'
            return

        # Spin slowly on the spot to acquire tag
        self.publish_cmd(0.0, 0.50)

    def aligning_loop(self):

        if not self.tag_in_view or self.latest_aruco_pose is None:
            self.get_logger().warning('Lost ArUco tag sight during docking. Re-searching...')
            self.turtlebot_state = 'SEARCHING'
            return

        target = self.get_staging_pose(0.60)
        # target = np.array([1.5, -1.0, np.deg2rad(-90.0)])
        reached = self.go_to_point(target)

        if reached:
            self.get_logger().info(
                'Aligned Turtlebot for docking, initialising DOCKING'
            )
            self.turtlebot_state = 'DOCKING'

        pass

    def docking_loop(self):
        """Visual servoing controller driving precisely into the charging dock."""
        if not self.tag_in_view or self.latest_aruco_pose is None:
            self.get_logger().warning('Lost ArUco tag sight during docking! Re-searching...')
            self.turtlebot_state = 'SEARCHING'
            return

        # Extract relative marker position in base_link frame
        tag_x = self.latest_aruco_pose[0]  # Distance forward to marker
        tag_y = self.latest_aruco_pose[1]  # Lateral offset (+ left, - right)

        # Target angle offset to center of marker
        angle_to_tag = np.arctan2(tag_y, tag_x)

        # Stop Check: Terminal docking threshold
        if tag_x <= self.dock_stop_dist:
            self.get_logger().info('CHARGING DOCK ENGAGED! Robot docked successfully.')
            self.turtlebot_state = 'DOCKED'
            self.publish_cmd(0.0, 0.0)
            return

        # Visual Servoing Controls
        w_cmd = self.k_dock_w * angle_to_tag
        v_cmd = self.k_dock_v * tag_x

        # Speed limits during docking
        v_cmd = float(np.clip(v_cmd, 0.02, 0.10))  # Slow controlled creep into dock
        w_cmd = float(np.clip(w_cmd, -0.6, 0.6))

        self.publish_cmd(v_cmd, w_cmd)

    def publish_cmd(self, v: float, w: float):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel.publish(twist)

    def go_to_point(self, target: npt.NDArray[np.float64]):
        if target.size != 3:
            self.get_logger().error(
                'Goal passed is not of the right format. Goals must be NDArray[float64] and structured to be [x, y, theta]'
            )
            return False

        dx, dy, _ = target - self.current_pose
        theta = self.current_pose[2]
        goal_dist = np.hypot(dx, dy)
        angle_diff = theta - target[2]

        # Check if target tolerance is reached
        if goal_dist < self.goal_tolerance and angle_diff < self.minimum_angle_diff:
            return True

        if goal_dist > 0.35:
            target_heading = np.arctan2(dy, dx)
        else:
            target_heading = target[2]

        dtheta = self.wrap_angle(target_heading - theta)

        e_x = np.cos(theta) * dx + np.sin(theta) * dy
        e_y = -np.sin(theta) * dx + np.cos(theta) * dy
        
        e_vec = np.array([e_x, e_y, dtheta])

        # Nominal LQR Control Inputs [delta_v, delta_w]
        u_lqr = -self.K_lqr @ e_vec
        v_cmd = self.v_ref + u_lqr[0]
        w_cmd = u_lqr[1]

        # Apply Reactive Obstacle Avoidance Bias
        Fx_rep, Fy_rep = self.compute_obstacle_repulsion()
        F_norm = np.hypot(Fx_rep, Fy_rep)

        if F_norm > 0.0:
            v_cmd = max(0.03, v_cmd - 0.08 * F_norm)

            if Fx_rep < -0.5 and np.abs(Fy_rep) < 0.05:
                Fy_rep = -0.3

            w_cmd += self.k_rep * Fy_rep

        # Velocity Saturation Limits
        v_cmd = float(np.clip(v_cmd, 0.0, self.max_v))
        w_cmd = float(np.clip(w_cmd, -self.max_w, self.max_w))

        self.publish_cmd(v_cmd, w_cmd)
        return False

    @staticmethod
    def create_se2_matrix(x: float, y: float, theta: float) -> np.ndarray:
        """Helper to build 3x3 SE(2) matrix."""
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta),  np.cos(theta), y],
            [0.0,            0.0,           1.0]
        ])

    def get_staging_pose(self, d_standoff: float = 0.60) -> np.ndarray:
        """Computes the target pose directly along the tag's normal in the global frame."""

        # # 1. T_O^B: Robot pose in World/Odom frame
        # rx, ry, rtheta = self.current_pose
        # T_O_B = self.create_se2_matrix(rx, ry, rtheta)

        # # 2. T_B^D: Dock pose in Robot (base_link) frame
        # dx, dy, dtheta = self.latest_aruco_pose
        # T_B_D = self.create_se2_matrix(dx, dy, dtheta)

        # # 3. T_D^L: Staging pose in Dock frame (d_standoff out along normal, rotated to face dock)
        # T_D_L = self.create_se2_matrix(d_standoff, 0.0, np.pi)

        # # 4. T_O^L = T_O^B * T_B^D * T_D^L
        # T_O_L = T_O_B @ T_B_D @ T_D_L

        # # Extract target (x, y, theta) in global frame
        # target_x = T_O_L[0, 2]
        # target_y = T_O_L[1, 2]
        # target_theta = np.arctan2(T_O_L[1, 0], T_O_L[0, 0])

        # return np.array([target_x, target_y, target_theta])

        rx, ry, rtheta = self.current_pose
        tag_x, tag_y, normal_yaw = self.latest_aruco_pose

        # 1. Staging point in base_link (d_standoff out along the +Z tag normal vector)
        nx, ny = self.latest_aruco_normal_vec
        local_target_x = tag_x + d_standoff * nx
        local_target_y = tag_y + d_standoff * ny
        
        # Target heading faces back TOWARD the dock (opposite of normal vector)
        local_target_theta = self.wrap_angle(normal_yaw + np.pi)

        # 2. Transform local base_link staging point to global (odom) frame
        global_target_x = rx + local_target_x * np.cos(rtheta) - local_target_y * np.sin(rtheta)
        global_target_y = ry + local_target_x * np.sin(rtheta) + local_target_y * np.cos(rtheta)
        global_target_theta = self.wrap_angle(rtheta + local_target_theta)

        return np.array([global_target_x, global_target_y, global_target_theta])

    @staticmethod
    def wrap_angle(angle: float):
        """wraps angle to pi,-pi"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

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

        node.publish_cmd(0.0, 0.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
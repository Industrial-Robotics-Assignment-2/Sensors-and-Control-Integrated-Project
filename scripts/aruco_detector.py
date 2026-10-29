#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from tf2_geometry_msgs import do_transform_pose
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class ArucoDetector(Node):
    """Detect a charging-station ArUco marker and publish its camera-frame pose."""

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.80)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('image_topic', '/camera/camera_sensor/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/camera_sensor/camera_info'
        )
        self.declare_parameter('pose_topic', '/camera/aruco_pose')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('position_stddev', 0.03)
        self.declare_parameter('angle_stddev', 0.08)

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size').value)
        dictionary_name = str(self.get_parameter('dictionary').value)
        self.dictionary = self._get_dictionary(dictionary_name)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_matrix = None
        self.distortion = None

        image_topic = str(self.get_parameter('image_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, pose_topic, 10
        )
        self.debug_publisher = self.create_publisher(
            Image, '/camera/aruco_debug', 10
        )
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Looking for ArUco marker {self.marker_id} using {dictionary_name}'
        )

    @staticmethod
    def _get_dictionary(name):
        try:
            dictionary_id = getattr(cv2.aruco, name)
        except AttributeError as exc:
            raise ValueError(f'Unknown ArUco dictionary: {name}') from exc
        return cv2.aruco.getPredefinedDictionary(dictionary_id)

    def camera_info_callback(self, message):
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(message.d, dtype=np.float64)

    def image_callback(self, message):
        if self.camera_matrix is None or self.distortion is None:
            self.get_logger().warning(
                'Waiting for camera calibration data', throttle_duration_sec=5.0
            )
            return

        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        detection_scale = 0.5
        detection_image = cv2.resize(
            image, None, fx=detection_scale, fy=detection_scale,
            interpolation=cv2.INTER_AREA,
        )
        corners, ids, _ = cv2.aruco.detectMarkers(
            detection_image, self.dictionary
        )

        if ids is not None:
            ids = ids.flatten()
            matching = np.flatnonzero(ids == self.marker_id)
            if matching.size:
                marker_corners = corners[int(matching[0])] / detection_scale
                rotation, translation = self._estimate_pose(marker_corners)
                self._publish_pose(rotation, translation, message)
                cv2.aruco.drawDetectedMarkers(
                    image, [marker_corners], np.array([[self.marker_id]])
                )

        if bool(self.get_parameter('publish_debug_image').value):
            debug_message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            debug_message.header = message.header
            self.debug_publisher.publish(debug_message)

    def _estimate_pose(self, corners):
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                np.asarray(corners, dtype=np.float32),
                self.marker_size,
                self.camera_matrix,
                self.distortion,
            )
            return rvecs[0], tvecs[0]

        half_size = self.marker_size / 2.0
        object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )
        image_points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            raise RuntimeError('Could not estimate the ArUco marker pose')
        return rvec, tvec.reshape(3)

    def _publish_pose(self, rotation, translation, image_message):
        rotation_vector = np.asarray(rotation, dtype=np.float64).reshape(3, 1)
        translation_vector = np.asarray(translation, dtype=np.float64).reshape(3)
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        quaternion = self._rotation_to_quaternion(rotation_matrix)

        camera_pose = PoseStamped()
        camera_pose.header = image_message.header
        camera_pose.pose.position.x = float(translation_vector[0])
        camera_pose.pose.position.y = float(translation_vector[1])
        camera_pose.pose.position.z = float(translation_vector[2])
        camera_pose.pose.orientation.x = quaternion[0]
        camera_pose.pose.orientation.y = quaternion[1]
        camera_pose.pose.orientation.z = quaternion[2]
        camera_pose.pose.orientation.w = quaternion[3]

        target_frame = str(self.get_parameter('target_frame').value)
        try:
            transformed_pose = self.tf_buffer.transform(
                camera_pose,
                target_frame,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Could not transform marker pose to {target_frame}: {error}',
                throttle_duration_sec=5.0,
            )
            return

        pose = PoseWithCovarianceStamped()
        pose.header = transformed_pose.header
        pose.pose.pose = transformed_pose.pose

        position_variance = float(self.get_parameter('position_stddev').value) ** 2
        angle_variance = float(self.get_parameter('angle_stddev').value) ** 2
        pose.pose.covariance[0] = position_variance
        pose.pose.covariance[7] = position_variance
        pose.pose.covariance[14] = position_variance
        pose.pose.covariance[21] = angle_variance
        pose.pose.covariance[28] = angle_variance
        pose.pose.covariance[35] = angle_variance
        self.pose_publisher.publish(pose)

    @staticmethod
    def _rotation_to_quaternion(matrix):
        trace = float(np.trace(matrix))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * scale
            x = (matrix[2, 1] - matrix[1, 2]) / scale
            y = (matrix[0, 2] - matrix[2, 0]) / scale
            z = (matrix[1, 0] - matrix[0, 1]) / scale
        elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif matrix[1, 1] > matrix[2, 2]:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
        return (float(x), float(y), float(z), float(w))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
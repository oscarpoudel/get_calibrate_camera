import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from ament_index_python.packages import get_package_share_directory


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')

        self.declare_parameter('image_topic', '/ip_stream_image')
        self.declare_parameter('board_width', 7)
        self.declare_parameter('board_height', 4)
        self.declare_parameter('output_yaml', './calibration_output/camera_calibration.yaml')
        self.declare_parameter('output_dir', './calibration_output/calibration_images')
        self.declare_parameter('max_images', 100)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.board_width = self.get_parameter('board_width').get_parameter_value().integer_value
        self.board_height = self.get_parameter('board_height').get_parameter_value().integer_value
        self.output_yaml = self.get_parameter('output_yaml').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value

        self.subscription = self.create_subscription(Image, image_topic, self.listener_callback, 10)
        self.bridge = CvBridge()

        self.objp = np.zeros((self.board_width * self.board_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.board_width, 0:self.board_height].T.reshape(-1, 2)

        self.objpoints = []
        self.imgpoints = []
        self.image_count = 0
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Started calibration on topic: {image_topic}")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (self.board_width, self.board_height), None)

        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            self.objpoints.append(self.objp)
            self.imgpoints.append(corners2)
            self.image_count += 1

            vis = cv2.drawChessboardCorners(cv_image.copy(), (self.board_width, self.board_height), corners2, found)
            filename = os.path.join(self.output_dir, f'image_{self.image_count}.png')
            cv2.imwrite(filename, vis)

            self.get_logger().info(f'Captured {self.image_count}/{self.max_images} calibration frames')

        if self.image_count >= self.max_images:
            self.perform_calibration(gray.shape[::-1])

    def perform_calibration(self, image_shape):
        rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_shape, None, None
        )
        self.get_logger().info(f'RMS Error: {rms}')
        self.get_logger().info(f'Camera matrix:\n{mtx}')
        self.get_logger().info(f'Distortion coefficients:\n{dist.ravel()}')

        # Save to YAML
        import yaml
        data = {
            'camera_matrix': {'rows': 3, 'cols': 3, 'data': mtx.flatten().tolist()},
            'distortion_coefficients': {'rows': 1, 'cols': len(dist.flatten()), 'data': dist.flatten().tolist()},
            'rms_error': float(rms)
        }
        with open(self.output_yaml, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f'Calibration saved to: {self.output_yaml}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

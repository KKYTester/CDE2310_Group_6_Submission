import yaml
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        # --- Parameters ---
        self.declare_parameter('marker_size', 0.043)  # 100mm default
        self.marker_size = self.get_parameter('marker_size').value
        
        # --- Camera calibration ---
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Load calibration
        self.load_calibration()

        # --- ArUco setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50  
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        # --- Subscribers ---
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # ✅ ADDED - subscribe to camera
            self.image_callback,
            10
        )

        # --- Publishers ---
        self.image_pub = self.create_publisher(
            Image,
            '/aruco/image',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco/pose',
            10
        )

        self.get_logger().info('ArUco detector started')
        self.get_logger().info(f'Marker size: {self.marker_size}m')
        self.get_logger().info('Listening on /camera/image_raw')

    def load_calibration(self):
        """Load camera calibration from package config"""
        try:
            # ✅ CHANGE 'aruco_detector' to YOUR actual package name
            package_name = 'aruco_detector'  # ← YOUR PACKAGE NAME HERE
            package_share = get_package_share_directory(package_name)
            yaml_path = os.path.join(package_share, 'config', 'camera_calibration.yaml')

            with open(yaml_path, 'r') as f:
                calib = yaml.safe_load(f)

            self.camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
            self.dist_coeffs = np.array(calib['distortion_coefficients']['data'])

            self.get_logger().info(f'✓ Loaded calibration from {yaml_path}')
            self.get_logger().info(f'  Image size: {calib["image_width"]}x{calib["image_height"]}')
            self.get_logger().info(f'  Calibration error: {calib.get("calibration_error", "N/A")}')

        except FileNotFoundError:
            self.get_logger().error(f'✗ Calibration file not found: {yaml_path}')
            self.get_logger().error('  Run camera calibration first!')
            raise
        except Exception as e:
            self.get_logger().error(f'✗ Failed to load calibration: {e}')
            raise

    def image_callback(self, msg):
        """Process camera images and detect ArUco markers"""
        
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                # Calculate distance
                distance = np.linalg.norm(tvec)

                # Log detection
                self.get_logger().info(
                    f"Marker {marker_id} | "
                    f"x={tvec[0]:+.3f} y={tvec[1]:+.3f} z={tvec[2]:+.3f}m | "
                    f"dist={distance:.3f}m"
                )

                # Draw coordinate axes on marker
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    0.05  # Axis length (5cm)
                )

                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'  # ✅ Standard TF frame name

                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])

                # Convert rotation vector to quaternion
                rot_mat, _ = cv2.Rodrigues(rvec)
                quat = self.rotation_matrix_to_quaternion(rot_mat)

                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                self.pose_pub.publish(pose_msg)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(out_msg)

    def rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s
        
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArucoDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml

class CharucoCalibrationNode(Node):
    def __init__(self):
        super().__init__('charuco_calibration')
        
        # ChArUco board
        self.SQUARES_X = 7
        self.SQUARES_Y = 5
        self.SQUARE_LENGTH = 0.057
        self.MARKER_LENGTH = 0.043
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.board = aruco.CharucoBoard(
            (self.SQUARES_X, self.SQUARES_Y),
            self.SQUARE_LENGTH,
            self.MARKER_LENGTH,
            aruco_dict
        )
        self.charuco_detector = aruco.CharucoDetector(self.board)
        
        # Storage
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.image_size = None
        self.captured = 0
        
        # ROS
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('ChArUco calibration node started')
        self.get_logger().info('Press SPACE to capture, C to calibrate, Q to quit')
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect board
        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            self.charuco_detector.detectBoard(gray)
        # Draw
        display = cv_image.copy()
        if charuco_ids is not None and len(charuco_ids) > 4:
            aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)
            if marker_ids is not None:
                aruco.drawDetectedMarkers(display, marker_corners, marker_ids)
            cv2.putText(display, f"Detected: {len(charuco_ids)} corners", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.putText(display, f"Captured: {self.captured}/20", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('ChArUco Calibration', display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('v') and charuco_ids is not None and len(charuco_ids) > 4:
            print("hi")
            self.all_charuco_corners.append(charuco_corners)
            self.all_charuco_ids.append(charuco_ids)
            self.image_size = gray.shape[::-1]
            self.captured += 1
            self.get_logger().info(f'Captured {self.captured}/20')
        
        elif key == ord('c'):
            self.calibrate()
        
        elif key == ord('q'):
            rclpy.shutdown()
    
    def calibrate(self):
        if self.captured < 10:
            self.get_logger().warn(f'Need at least 10 images (have {self.captured})')
            return
        
        self.get_logger().info('Starting calibration...')
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            self.all_charuco_corners,
            self.all_charuco_ids,
            self.board,
            self.image_size,
            None,
            None
        )
        
        if ret:
            self.get_logger().info(f'Calibration successful! RMS error: {ret:.4f}')
            
            calibration_data = {
                'camera_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': camera_matrix.flatten().tolist()
                },
                'distortion_coefficients': {
                    'rows': 1,
                    'cols': 5,
                    'data': dist_coeffs.flatten().tolist()
                },
                'image_width': self.image_size[0],
                'image_height': self.image_size[1],
                'calibration_error': float(ret)
            }
            
            with open('camera_calibration.yaml', 'w') as f:
                yaml.dump(calibration_data, f)
            
            self.get_logger().info('Saved to camera_calibration.yaml')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = CharucoCalibrationNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
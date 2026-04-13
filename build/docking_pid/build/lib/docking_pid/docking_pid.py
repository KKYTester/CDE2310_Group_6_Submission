#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32, String
import time


class PIDController:
    def __init__(self, kp, ki, kd, max_integral=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0


class ArucoDockingPID(Node):
    def __init__(self):
        super().__init__('aruco_docking_pid')

        # ==================== TARGET POSITION ====================
        self.TARGET_DISTANCE = 0.25  # ✅ Changed from 0.30 to 0.25 (25cm)
        self.TARGET_LATERAL = 0.0

        # ==================== DOCKING CONFIRMATION ====================
        self.DOCKING_CONFIRM_TIME = 1.0  # ✅ Must stay docked for 1 second
        self.docking_start_time = None    # ✅ Tracks when docking started
        self.in_docking_position = False  # ✅ Flag for docking position

        # ==================== CONTROL GAINS ====================
        self.KP_DISTANCE = 0.8
        self.KI_DISTANCE = 0.0
        self.KD_DISTANCE = 0.0

        self.KP_ANGULAR = 2.0
        self.KI_ANGULAR = 0.0
        self.KD_ANGULAR = 0.0

        # ==================== ERROR DEAD ZONES ====================
        self.LATERAL_ERROR_DEADZONE = 0.015  # Ignore errors < 1.5cm
        self.DISTANCE_ERROR_DEADZONE = 0.02  # Ignore errors < 2cm

        # ==================== ASYMMETRIC TUNING ====================
        self.RIGHT_TURN_GAIN = 2.5
        self.LEFT_TURN_GAIN = 1.5
        
        self.MIN_RIGHT_TURN = 0.12
        self.MIN_LEFT_TURN = 0.08

        # ==================== LIMITS ====================
        self.MAX_LINEAR_SPEED = 0.12
        self.MAX_ANGULAR_SPEED = 0.6
        self.MAX_ACCEL_LINEAR = 0.3
        self.MAX_ACCEL_ANGULAR = 1.0

        # ==================== TOLERANCES ====================
        self.DISTANCE_TOLERANCE = 0.05
        self.LATERAL_TOLERANCE = 0.03

        # ==================== SAFETY ====================
        self.MIN_SAFE_DISTANCE = 0.20
        self.MARKER_TIMEOUT = 0.8

        # ==================== STATE ====================
        self.last_pose = None
        self.last_seen_time = 0.0
        self.is_docked = False
        self.docking_begin = False
        self.current_marker_id = None
        self.detected_markers = set()
        self.marker_last_seen = {}
        self.completed_markers = set()  # Markers already docked & launched

        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.prev_time = time.time()

        # Initialize PID controllers
        self.distance_pid = PIDController(self.KP_DISTANCE, self.KI_DISTANCE, self.KD_DISTANCE)
        self.angular_pid = PIDController(self.KP_ANGULAR, self.KI_ANGULAR, self.KD_ANGULAR)

        # ==================== ROS INTERFACES ====================
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/pose',
            self.pose_callback,
            10
        )

        self.docking_begin_sub = self.create_subscription(
            Bool,
            '/docking/begin',
            self.docking_begin_callback,
            10
        )

        self.marker_id_sub = self.create_subscription(
            Int32,
            '/aruco/marker_id',
            self.marker_id_callback,
            10
        )

        self.docking_status_sub = self.create_subscription(
            String,
            '/docking/status',
            self.docking_status_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.launch_cmd_pub = self.create_publisher(String, '/docking/launch_command', 10)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("🎯 PID Docking Controller Started")
        self.get_logger().info(f"   Target distance: {self.TARGET_DISTANCE}m")
        self.get_logger().info(f"   Docking confirmation: {self.DOCKING_CONFIRM_TIME}s")
        self.get_logger().info(f"   RIGHT: Gain={self.RIGHT_TURN_GAIN}, Min={self.MIN_RIGHT_TURN}")
        self.get_logger().info(f"   LEFT:  Gain={self.LEFT_TURN_GAIN}, Min={self.MIN_LEFT_TURN}")

    # ==================== CALLBACKS ====================
    def pose_callback(self, msg):
        self.last_pose = msg
        self.last_seen_time = time.time()

    def docking_begin_callback(self, msg):
        self.docking_begin = msg.data
        if msg.data:
            self.is_docked = False
            self.in_docking_position = False  # ✅ Reset confirmation
            self.docking_start_time = None    # ✅ Reset timer
            self.detected_markers.clear()
            self.marker_last_seen.clear()
            self.prev_linear = 0.0
            self.prev_angular = 0.0
            self.distance_pid.reset()
            self.angular_pid.reset()
        else:
            self.stop_robot()
            self.in_docking_position = False
            self.docking_start_time = None

    def marker_id_callback(self, msg):
        self.current_marker_id = msg.data
        self.detected_markers.add(msg.data)
        self.marker_last_seen[msg.data] = time.time()

    def docking_status_callback(self, msg):
        """Launch complete - mark the marker as done"""
        if self.current_marker_id is not None:
            self.completed_markers.add(self.current_marker_id)
            self.get_logger().info(
                f"✅ Marker {self.current_marker_id} completed. "
                f"Done: {self.completed_markers}"
            )
        self.is_docked = False
        self.in_docking_position = False
        self.docking_start_time = None

    # ==================== MAIN CONTROL LOOP ====================
    def control_loop(self):
        """Main PID control loop with docking confirmation delay"""
        
        # Update marker timeout
        current_time = time.time()
        for marker in list(self.marker_last_seen):
            if current_time - self.marker_last_seen[marker] > 1.0:
                self.detected_markers.discard(marker)
                del self.marker_last_seen[marker]

        # Check if docking is active
        if not self.docking_begin:
            self.stop_robot()
            return

        # Safety: No marker visible
        if self.last_pose is None:
            self.stop_robot()
            self.in_docking_position = False  # ✅ Reset if marker lost
            self.docking_start_time = None
            return

        # Safety: Marker timeout
        time_since_seen = current_time - self.last_seen_time
        if time_since_seen > self.MARKER_TIMEOUT:
            self.stop_robot()
            self.in_docking_position = False  # ✅ Reset if timeout
            self.docking_start_time = None
            return

        # Skip already-completed markers
        if self.current_marker_id in self.completed_markers:
            self.stop_robot()
            return

        # ==================== EXTRACT MARKER POSITION ====================
        marker_x = self.last_pose.pose.position.x
        marker_z = self.last_pose.pose.position.z

        # ==================== CALCULATE ERRORS ====================
        distance_error = marker_z - self.TARGET_DISTANCE
        lateral_error = self.TARGET_LATERAL - marker_x
        
        # ==================== APPLY ERROR DEAD ZONES ====================
        if abs(lateral_error) < self.LATERAL_ERROR_DEADZONE:
            lateral_error = 0.0
            
        if abs(distance_error) < self.DISTANCE_ERROR_DEADZONE:
            distance_error = 0.0
        
        # ==================== CHECK IF IN DOCKING POSITION ====================
        within_tolerance = (
            abs(marker_z - self.TARGET_DISTANCE) < self.DISTANCE_TOLERANCE and
            abs(marker_x - self.TARGET_LATERAL) < self.LATERAL_TOLERANCE
        )
        
        if within_tolerance:
            # ✅ DOCKING CONFIRMATION LOGIC
            if not self.in_docking_position:
                # Just entered docking position
                self.in_docking_position = True
                self.docking_start_time = current_time
                self.get_logger().info(f"⏳ Entered docking position, confirming...")
                
            else:
                # Already in position - check if enough time has passed
                time_in_position = current_time - self.docking_start_time
                
                if time_in_position >= self.DOCKING_CONFIRM_TIME:
                    # ✅ CONFIRMED DOCKED - publish launch command
                    if not self.is_docked:
                        self.get_logger().info(
                            f"✅ DOCKED! (confirmed after {time_in_position:.1f}s)"
                        )
                        status_msg = String()
                        if self.current_marker_id == 0:
                            status_msg.data = "static"
                        else:
                            status_msg.data = "dynamic"
                        self.launch_cmd_pub.publish(status_msg)
                        self.is_docked = True
                    
                    self.stop_robot()
                    return
                else:
                    # Still confirming - hold position but keep control active
                    remaining = self.DOCKING_CONFIRM_TIME - time_in_position
                    self.get_logger().info(
                        f"⏳ Holding position... {remaining:.1f}s remaining",
                        throttle_duration_sec=0.5
                    )
                    # Continue with control loop to maintain position
        else:
            # ✅ LEFT DOCKING POSITION - RESET CONFIRMATION
            if self.in_docking_position:
                self.get_logger().info("⚠️  Left docking position - resetting confirmation")
            self.in_docking_position = False
            self.docking_start_time = None
            self.is_docked = False

        # ==================== PID CONTROL ====================
        dt = current_time - self.prev_time
        self.prev_time = current_time
        dt = max(min(dt, 0.1), 0.001)

        # Distance PID
        linear_cmd = self.distance_pid.compute(distance_error, dt)
        
        # ==================== ASYMMETRIC ANGULAR CONTROL ====================
        if lateral_error == 0.0:
            angular_cmd = 0.0
            direction = "CENTER"
        else:
            pid_base = self.angular_pid.compute(lateral_error, dt)
            
            if pid_base < 0:  # RIGHT
                angular_cmd = pid_base * (self.RIGHT_TURN_GAIN / self.KP_ANGULAR)
                if abs(angular_cmd) < self.MIN_RIGHT_TURN:
                    angular_cmd = -self.MIN_RIGHT_TURN
                direction = "RIGHT"
            else:  # LEFT
                angular_cmd = pid_base * (self.LEFT_TURN_GAIN / self.KP_ANGULAR)
                if abs(angular_cmd) < self.MIN_LEFT_TURN:
                    angular_cmd = self.MIN_LEFT_TURN
                direction = "LEFT"
        
        # ==================== SAFETY ====================
        if marker_z < self.MIN_SAFE_DISTANCE and linear_cmd > 0:
            linear_cmd = 0.0
        
        # ==================== VELOCITY CONSTRAINTS ====================
        linear_cmd = max(min(linear_cmd, self.MAX_LINEAR_SPEED), -self.MAX_LINEAR_SPEED)
        angular_cmd = max(min(angular_cmd, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
        
        # ==================== RATE LIMITING ====================
        linear_cmd = self.rate_limit(linear_cmd, self.prev_linear, self.MAX_ACCEL_LINEAR * self.dt)
        angular_cmd = self.rate_limit(angular_cmd, self.prev_angular, self.MAX_ACCEL_ANGULAR * self.dt)
        
        self.prev_linear = linear_cmd
        self.prev_angular = angular_cmd

        # ==================== PUBLISH ====================
        cmd = Twist()
        cmd.linear.x = float(linear_cmd)
        cmd.angular.z = float(angular_cmd)
        self.cmd_pub.publish(cmd)
        
        # ==================== DEBUG LOGGING ====================
        self.get_logger().info(
            f"{direction}: z={marker_z:.3f}m, x={marker_x:+.3f}m | "
            f"v={linear_cmd:+.2f}, ω={angular_cmd:+.2f}",
            throttle_duration_sec=0.5
        )

    # ==================== UTILITIES ====================
    def rate_limit(self, desired, previous, max_change):
        delta = desired - previous
        if delta > max_change:
            return previous + max_change
        elif delta < -max_change:
            return previous - max_change
        else:
            return desired

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.prev_linear = 0.0
        self.prev_angular = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDockingPID()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32, Bool
import RPi.GPIO as GPIO
import time


class LauncherNode(Node):
    def __init__(self):
        super().__init__('launcher_node')

        # ==================== CALLBACK GROUPS ====================
        self.marker_cb_group = MutuallyExclusiveCallbackGroup()
        self.launch_cmd_cb_group = MutuallyExclusiveCallbackGroup()
        # Removed docking_begin_cb_group

        # ==================== GPIO SETUP ====================
        self.IN3 = 23
        self.IN4 = 24
        self.SERVO_PIN = 18

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)

        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)
        self.SERVO_OPEN = 2
        self.SERVO_CLOSE = 7

        # ==================== STATE ====================
        self.current_marker_id = None
        self.last_marker_id = None
        self.docked_marker_id = None
        self.launch_in_progress = False

        # Flags to prevent repeat launches – never reset
        self.static_launch_done = False
        self.dynamic_launch_done = False

        # ==================== ROS INTERFACES ====================
        self.launch_cmd_sub = self.create_subscription(
            String,
            '/docking/launch_command',
            self.launch_command_callback,
            10,
            callback_group=self.launch_cmd_cb_group
        )

        self.marker_id_sub = self.create_subscription(
            Int32,
            '/aruco/marker_id',
            self.marker_id_callback,
            10,
            callback_group=self.marker_cb_group
        )

        # No subscription to /docking/begin

        self.docking_status_pub = self.create_publisher(
            String,
            '/docking/status',
            10
        )

        self.launch_status_pub = self.create_publisher(
            String,
            '/launcher/status',
            10
        )

        self.get_logger().info("🚀 Launcher Node Started")
        self.get_logger().info(f"   Flywheel: IN3={self.IN3}, IN4={self.IN4}")
        self.get_logger().info(f"   Servo: GPIO {self.SERVO_PIN}")

    # ==================== CALLBACKS ====================
    def launch_command_callback(self, msg):
        command = msg.data
        if command == "static" and not self.launch_in_progress:
            if self.static_launch_done:
                self.get_logger().info("⚠️ Static launch already done, ignoring")
                return
            self.get_logger().info("📍 Received static launch command")
            self.docked_marker_id = 0
            self.launch_static()
        elif command == "dynamic" and not self.launch_in_progress:
            if self.dynamic_launch_done:
                self.get_logger().info("⚠️ Dynamic launch already done, ignoring")
                return
            self.get_logger().info("📍 Received dynamic launch command")
            self.docked_marker_id = 1
            self.launch_dynamic()

    def marker_id_callback(self, msg):
        self.last_marker_id = self.current_marker_id
        self.current_marker_id = msg.data

    # ==================== MOTOR & SERVO CONTROL ====================
    def motor_forward(self):
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)

    def motor_stop(self):
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    def launch_ball(self, ball_num):
        self.get_logger().info(f"⚾ Launching ball {ball_num}")
        self.servo_pwm.ChangeDutyCycle(self.SERVO_OPEN)
        time.sleep(0.5)
        self.servo_pwm.ChangeDutyCycle(self.SERVO_CLOSE)

    # ==================== LAUNCH SEQUENCES ====================
    def launch_static(self):
        self.launch_in_progress = True
        self.get_logger().info("🚀 Starting STATIC launch sequence")

        self.motor_forward()
        time.sleep(0.5)

        self.launch_ball(1)
        time.sleep(3.5)

        self.launch_ball(2)
        time.sleep(5.5)

        self.launch_ball(3)
        time.sleep(1.0)

        self.motor_stop()
        self.get_logger().info("✅ Static launch complete")

        self.static_launch_done = True

        self.launch_status_pub.publish(String(data="static_launch_complete"))
        self.docking_status_pub.publish(String(data="static docking is done"))
        self.launch_in_progress = False

    def launch_dynamic(self):
        self.launch_in_progress = True
        self.get_logger().info("🚀 Starting DYNAMIC launch sequence")
        self.get_logger().info("⏳ Waiting for marker 2 to appear...")

        self.motor_forward()
        time.sleep(0.5)

        ball_count = 1
        just_launched = False
        disappear_start_time = None
        marker_visible = False

        while ball_count <= 3:
            current_time = time.time()
            is_visible = (self.current_marker_id == 2)

            if is_visible and not marker_visible:
                if not just_launched:
                    self.get_logger().info(f"👀 Marker 2 detected! Launching ball {ball_count}")
                    self.launch_ball(ball_count)
                    ball_count += 1
                    just_launched = True
                    disappear_start_time = None
                else:
                    self.get_logger().info("⏳ Cooldown active, ignoring detection")

            if not is_visible:
                if just_launched and disappear_start_time is None:
                    disappear_start_time = current_time
                    self.get_logger().info("👋 Marker 2 gone, starting 1s cooldown")
                if just_launched and disappear_start_time is not None:
                    if current_time - disappear_start_time >= 1.0:
                        self.get_logger().info("✅ Cooldown finished, ready for next detection")
                        just_launched = False
                        disappear_start_time = None

            marker_visible = is_visible
            time.sleep(0.01)

        self.motor_stop()
        self.get_logger().info("✅ Dynamic launch complete")

        self.dynamic_launch_done = True

        self.launch_status_pub.publish(String(data="dynamic_launch_complete"))
        self.docking_status_pub.publish(String(data="dynamic docking is done"))
        self.launch_in_progress = False

    def cleanup(self):
        self.motor_stop()
        self.servo_pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("🧹 GPIO cleaned up")


def main(args=None):
    rclpy.init(args=args)
    node = LauncherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down launcher...")
    finally:
        node.cleanup()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

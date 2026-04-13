#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
import random
import math
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return np.array([qx, qy, qz, qw])

class RandomGoalNoAMCL(Node):
    def __init__(self):
        super().__init__('random_goal_no_amcl')
        
        # --- Action client for Nav2 ---
        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available. Is Nav2 running?')
            return
        
        # --- Costmap subscription ---
        self.costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos_profile_sensor_data
        )
        
        # Wait for first costmap
        self.get_logger().info('Waiting for global costmap...')
        while self.costmap is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Global costmap received.')
        self.resume = False  # Start in pause mode by default
        self.resume_sub = self.create_subscription(
            Bool,
            'random_nav/resume',   # topic name – adjust if needed
            self.resume_callback,
            10
        )
        self.get_logger().info('Subscribed to random_nav/resume. Set False to pause, True to resume.')
        # State flag to prevent overlapping goals
        self.navigating = False
        
        # Start the first goal
        self.send_random_goal()
    
    def resume_callback(self, msg: Bool):
        was_resume = self.resume
        self.resume = msg.data
        if self.resume and not was_resume:
            self.get_logger().info('Resumed: will start sending new goals.')
            # If we're not currently navigating, trigger a new goal
            if not self.navigating:
                self.send_random_goal()
        elif not self.resume and was_resume:
            self.get_logger().info('Paused: will not send new goals.')

    def costmap_callback(self, msg):
        self.costmap = msg
    
    def is_pose_valid(self, x, y):
        if self.costmap is None:
            return False
        ox = self.costmap.info.origin.position.x
        oy = self.costmap.info.origin.position.y
        res = self.costmap.info.resolution
        cx = int((x - ox) / res)
        cy = int((y - oy) / res)
        if cx < 0 or cx >= self.costmap.info.width or cy < 0 or cy >= self.costmap.info.height:
            return False
        idx = cy * self.costmap.info.width + cx
        cost = self.costmap.data[idx]
        # Valid only if free space (0-49) and not unknown (-1)
        return cost >= 0 and cost < 50
    
    def generate_random_goal(self):
        ox = self.costmap.info.origin.position.x
        oy = self.costmap.info.origin.position.y
        w = self.costmap.info.width * self.costmap.info.resolution
        h = self.costmap.info.height * self.costmap.info.resolution
        max_attempts = 1000
        for _ in range(max_attempts):
            x = random.uniform(ox, ox + w)
            y = random.uniform(oy, oy + h)
            yaw = random.uniform(-math.pi, math.pi)
            if self.is_pose_valid(x, y):
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, yaw)
                goal.pose.orientation.x = q[0]
                goal.pose.orientation.y = q[1]
                goal.pose.orientation.z = q[2]
                goal.pose.orientation.w = q[3]
                self.get_logger().info(f'Generated valid goal: ({x:.2f}, {y:.2f})')
                return goal
        self.get_logger().error('Could not generate valid goal after many attempts.')
        return None
    
    def send_random_goal(self):
        # Prevent sending if paused
        if not self.resume:
            self.get_logger().debug('Paused – ignoring goal request')
            return
        # Prevent sending a new goal while one is already in progress
        if self.navigating:
            self.get_logger().debug('Already navigating, ignoring new goal request')
            return
        
        goal = self.generate_random_goal()
        if goal is None:
            self.get_logger().warn('No valid goal generated, retrying in 2 seconds...')
            self.create_timer(2.0, lambda: self.send_random_goal())
            return
        
        self.navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        self.get_logger().info('Sending goal to Nav2...')
        send_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        # Optional: log feedback (e.g., distance remaining)
        pass
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.navigating = False
            # Try another goal after a short delay
            self.get_logger().info('Trying another goal in 2 seconds...')
            self.create_timer(2.0, lambda: self.send_random_goal())
            return
        
        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result()
        self.navigating = False
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().warn(f'Navigation failed with status {result.status}')
            if result.status == 6:
                self.get_logger().warn('Goal aborted. Possible causes: path blocked, unknown space, or planning timeout.')
        
        # Wait a few seconds then send another random goal
        self.get_logger().info('Waiting 5 seconds before next goal...')
        self.create_timer(5.0, lambda: self.send_random_goal())

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalNoAMCL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
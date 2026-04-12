#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from std_msgs.msg import Int32          # <-- ADDED for marker ID
import tf2_geometry_msgs  # still used for helper functions? We'll avoid it.
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
import math
import numpy as np

class ArucoNavGoal(Node):
    def __init__(self):
        super().__init__('aruco_nav_goal')
        # marker orientation convention:
        self.declare_parameter('marker_forward_axis', 'z')   # 'x', 'y', or 'z'
        self.marker_forward_axis = self.get_parameter('marker_forward_axis').get_parameter_value().string_value
        self.declare_parameter('flip_forward_direction', False)
        self.flip_forward_direction = self.get_parameter('flip_forward_direction').value

        self.declare_parameter('approach_distance', 0.15)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('aruco_pose_topic', '/aruco/pose')
        self.declare_parameter('aruco_id_topic', '/aruco/marker_id')   # <-- ADDED
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('base_to_camera_translation', [0.036, -0.035, 0.18])
        self.declare_parameter('base_to_camera_rotation', [-90.0, 0.0, -90.0])

        self.approach_distance = self.get_parameter('approach_distance').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.aruco_pose_topic = self.get_parameter('aruco_pose_topic').value
        self.aruco_id_topic = self.get_parameter('aruco_id_topic').value   # <-- ADDED
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # Allowed marker IDs
        self.allowed_ids = {0, 1, 3}          # <-- ADDED
        self.current_marker_id = None         # <-- ADDED

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.sub = self.create_subscription(PoseStamped, self.aruco_pose_topic, self.aruco_callback, 10)
        self.id_sub = self.create_subscription(Int32, self.aruco_id_topic, self.id_callback, 10)   # <-- ADDED

        self.broadcast_static_transform()
        self.get_logger().info('ArucoNavGoal started.')
    
    # ------------------ ID callback (store only allowed IDs) ------------------
    def id_callback(self, msg: Int32):
        marker_id = msg.data
        if marker_id in self.allowed_ids:
            self.current_marker_id = marker_id
            self.get_logger().debug(f'Allowed marker ID received: {marker_id}')
        else:
            self.current_marker_id = None
            self.get_logger().debug(f'Ignoring marker ID {marker_id} (not in {self.allowed_ids})')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
        qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
        qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        return (qx, qy, qz, qw)

    def broadcast_static_transform(self):
        trans = self.get_parameter('base_to_camera_translation').value
        rot_deg = self.get_parameter('base_to_camera_rotation').value
        roll = math.radians(rot_deg[0])
        pitch = math.radians(rot_deg[1])
        yaw = math.radians(rot_deg[2])
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = self.camera_frame
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Static transform {self.robot_base_frame} -> {self.camera_frame} broadcasted')

    def transform_pose(self, pose_stamped: PoseStamped, transform: TransformStamped) -> PoseStamped:
        """
        Manually apply a transform to a PoseStamped.
        """
        # Extract translation and rotation from transform
        t = transform.transform.translation
        q = transform.transform.rotation

        # Convert transform quaternion to rotation matrix (or use quaternion multiplication)
        # We'll use quaternion multiplication: p' = q * p * q^-1 + t
        # But for position: new_pos = q * old_pos * q^-1 + t
        # Simpler: use numpy for matrix multiplication
        # Build 4x4 matrix from transform
        def quaternion_to_matrix(qx, qy, qz, qw):
            return np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw), 0],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw), 0],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2), 0],
                [0, 0, 0, 1]
            ])
        R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        T = np.eye(4)
        T[0:3, 0:3] = R[0:3, 0:3]
        T[0:3, 3] = [t.x, t.y, t.z]

        # Pose as homogeneous vector
        p = np.array([pose_stamped.pose.position.x,
                      pose_stamped.pose.position.y,
                      pose_stamped.pose.position.z, 1.0])
        p_transformed = T @ p

        # Transform orientation: q_new = q_transform * q_old
        # Use quaternion multiplication
        q_old = [pose_stamped.pose.orientation.x,
                 pose_stamped.pose.orientation.y,
                 pose_stamped.pose.orientation.z,
                 pose_stamped.pose.orientation.w]
        q_t = [q.x, q.y, q.z, q.w]
        # Quaternion multiplication
        q_new = [
            q_t[0]*q_old[3] + q_t[3]*q_old[0] + q_t[1]*q_old[2] - q_t[2]*q_old[1],
            q_t[1]*q_old[3] + q_t[3]*q_old[1] + q_t[2]*q_old[0] - q_t[0]*q_old[2],
            q_t[2]*q_old[3] + q_t[3]*q_old[2] + q_t[0]*q_old[1] - q_t[1]*q_old[0],
            q_t[3]*q_old[3] - q_t[0]*q_old[0] - q_t[1]*q_old[1] - q_t[2]*q_old[2]
        ]
        # Normalize (optional but safe)
        norm = math.sqrt(sum([x*x for x in q_new]))
        if norm > 0:
            q_new = [x/norm for x in q_new]

        result = PoseStamped()
        result.header = pose_stamped.header
        result.header.frame_id = transform.header.frame_id  # target frame
        result.pose.position.x = p_transformed[0]
        result.pose.position.y = p_transformed[1]
        result.pose.position.z = p_transformed[2]
        result.pose.orientation.x = q_new[0]
        result.pose.orientation.y = q_new[1]
        result.pose.orientation.z = q_new[2]
        result.pose.orientation.w = q_new[3]
        return result

    def aruco_callback(self, msg: PoseStamped):
        # Ignore pose if we haven't seen an allowed marker ID recently
        if self.current_marker_id is None:
            self.get_logger().debug('Received AruCo pose but no allowed marker ID. Skipping.')
            return
        
        self.get_logger().info(f'Received AruCo pose for marker ID {self.current_marker_id} in frame: {msg.header.frame_id}')

        # Lookup transform from camera_link to map
        try:
            transform = self.tf_buffer.lookup_transform(
                self.goal_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')
            return

        # Manually transform the pose
        marker_in_map = self.transform_pose(msg, transform)
        self.get_logger().info(f'Marker in map: ({marker_in_map.pose.position.x:.2f}, {marker_in_map.pose.position.y:.2f})')

        # Compute goal in front of marker
        goal_pose = self.compute_goal_in_front(marker_in_map)

        # Send goal to Nav2
        self.send_nav_goal(goal_pose)

    def compute_goal_in_front(self, marker_pose: PoseStamped) -> PoseStamped:
        pos = marker_pose.pose.position
        orient = marker_pose.pose.orientation
        qx, qy, qz, qw = orient.x, orient.y, orient.z, orient.w

        # Helper: rotate a vector by quaternion
        def rotate(vx, vy, vz):
            x2 = qx*2; y2 = qy*2; z2 = qz*2
            wx = qw*x2; wy = qw*y2; wz = qw*z2
            xx = qx*x2; xy = qx*y2; xz = qx*z2
            yy = qy*y2; yz = qy*z2; zz = qz*z2
            rot = np.array([
                [1 - (yy+zz), xy - wz, xz + wy],
                [xy + wz, 1 - (xx+zz), yz - wx],
                [xz - wy, yz + wx, 1 - (xx+yy)]
            ])
            return rot @ np.array([vx, vy, vz])

        # Use the user‑selected axis (from parameter)
        axis = self.marker_forward_axis  # 'x', 'y', or 'z'
        if axis == 'x':
            forward_3d = rotate(1, 0, 0)
        elif axis == 'y':
            forward_3d = rotate(0, 1, 0)
        else:  # 'z'
            forward_3d = rotate(0, 0, 1)

        forward_xy = forward_3d[:2]
        self.get_logger().info(f'Using axis {axis} -> horizontal vector: ({forward_xy[0]:.2f}, {forward_xy[1]:.2f})')

        # Optionally flip the direction
        if self.flip_forward_direction:
            forward_xy = -forward_xy
            self.get_logger().info('Flipped forward direction')

        # Goal position = marker position + approach_distance * forward direction
        goal_xy = np.array([pos.x, pos.y]) + self.approach_distance * forward_xy
        goal_pos = np.array([goal_xy[0], goal_xy[1], 0.0])

        # Robot orientation: point toward marker (opposite of forward direction)
        direction_to_marker = -forward_xy
        yaw = math.atan2(direction_to_marker[1], direction_to_marker[0])

        goal = PoseStamped()
        goal.header = marker_pose.header
        goal.header.frame_id = self.goal_frame
        goal.pose.position.x = goal_pos[0]
        goal.pose.position.y = goal_pos[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})  yaw={yaw:.2f} rad')
        return goal

    def send_nav_goal(self, goal_pose: PoseStamped):
        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 server not available')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.nav2_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal sent to Nav2')

    def feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
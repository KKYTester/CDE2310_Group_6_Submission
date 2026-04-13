#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32, String

class Turtlebot3FSM(Node):
    """
    Finite State Machine for a real TurtleBot3.
    States:
        0: EXPLORING          - publishes True to 'explore/resume', listens for ArUco markers.
        1: NAVIGATING         - frontier paused, waiting for nav2aruco to reach goal.
        2: DOCKING            - docking in progress.
    """

    def __init__(self):
        super().__init__('turtlebot3_fsm')

        # Station completion flags
        self.static_complete = False      # marker 0
        self.dynamic_complete = False     # marker 1
        self.lift_complete = False        # marker 3 (only after both 0 and 1)

        # FSM state
        self.state = 0  # EXPLORING

        # Flags to avoid repeated publications
        self.docking_started = False

        # Publishers
        self.explore_resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.docking_begin_pub = self.create_publisher(Bool, 'docking/begin', 10)

        # NEW: Publish station completion status for nav2aruco
        self.static_complete_pub = self.create_publisher(Bool, '/station/static_complete', 10)
        self.dynamic_complete_pub = self.create_publisher(Bool, '/station/dynamic_complete', 10)

        # Subscribers
        self.aruco_sub = self.create_subscription(Int32, 'aruco/marker_id', self.aruco_callback, 10)
        self.docking_status_sub = self.create_subscription(String, '/docking/status', self.docking_status_callback, 10)

        # NEW: Subscribers from nav2aruco
        self.nav_started_sub = self.create_subscription(Bool, 'nav2aruco/started', self.nav_started_callback, 10)
        self.nav_goal_reached_sub = self.create_subscription(Bool, 'nav2aruco/goal_reached', self.nav_goal_callback, 10)

        # Publish initial resume command
        self.publish_explore_resume(True)
        self.publish_completion_status()   # initial publish

        self.get_logger().info('FSM started in EXPLORING state')
        self.get_logger().info(f'Static complete: {self.static_complete}, Dynamic complete: {self.dynamic_complete}')

    # --------------------------------------------------------------------------
    # Helper methods
    # --------------------------------------------------------------------------
    def publish_explore_resume(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self.explore_resume_pub.publish(msg)
        self.get_logger().info(f'Published explore/resume = {resume}')

    def publish_docking_begin(self, begin: bool):
        msg = Bool()
        msg.data = begin
        self.docking_begin_pub.publish(msg)
        self.get_logger().info(f'Published docking/begin = {begin}')

    def publish_completion_status(self):
        self.static_complete_pub.publish(Bool(data=self.static_complete))
        self.dynamic_complete_pub.publish(Bool(data=self.dynamic_complete))
        self.get_logger().debug(f'Published completion status: static={self.static_complete}, dynamic={self.dynamic_complete}')

    # --------------------------------------------------------------------------
    # State entry actions
    # --------------------------------------------------------------------------
    def on_enter_state0(self):
        """EXPLORING: resume frontier, reset docking flag."""
        self.publish_explore_resume(True)
        self.docking_started = False
        self.get_logger().info('Entered EXPLORING state (State 0)')

    def on_enter_state1(self):
        """NAVIGATING: pause frontier, wait for nav2aruco."""
        self.publish_explore_resume(False)
        self.get_logger().info('Entered NAVIGATING state (State 1) - waiting for goal_reached')

    def on_enter_state2(self):
        """DOCKING: start docking procedure."""
        if not self.docking_started:
            self.publish_docking_begin(True)
            self.docking_started = True
        self.get_logger().info('Entered DOCKING state (State 2)')

    # --------------------------------------------------------------------------
    # FSM transitions
    # --------------------------------------------------------------------------
    def transition_to_state1(self):
        if self.state == 0:
            self.state = 1
            self.on_enter_state1()

    def transition_to_state2(self):
        if self.state == 1:
            self.state = 2
            self.on_enter_state2()

    def transition_to_state0(self):
        if self.state == 2:
            self.state = 0
            self.on_enter_state0()

    # --------------------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------------------
    def aruco_callback(self, msg: Int32):
        """
        Called when an ArUco marker is detected.
        We no longer transition directly to docking; nav2aruco handles navigation.
        However, we still track which markers are seen for potential future use.
        For now, this callback only logs.
        """
        marker_id = msg.data
        self.get_logger().info(f'ArUco marker detected, ID = {marker_id}')
        # The actual navigation is triggered by nav2aruco node, which subscribes to the same topic.
        # No state change here.

    def nav_started_callback(self, msg: Bool):
        """nav2aruco has started navigating to a marker -> pause exploration."""
        if msg.data and self.state == 0:
            self.get_logger().info('nav2aruco started navigation -> transitioning to NAVIGATING')
            self.transition_to_state1()

    def nav_goal_callback(self, msg: Bool):
        """nav2aruco reached the goal -> start docking."""
        if msg.data and self.state == 1:
            self.get_logger().info('nav2aruco goal reached -> transitioning to DOCKING')
            self.transition_to_state2()

    def docking_status_callback(self, msg: String):
        if self.state != 2:
            return
        status = msg.data.strip().lower()
        self.get_logger().info(f'Docking status: {status}')
        if status == "static docking is done":
            if not self.static_complete:
                self.static_complete = True
                self.publish_completion_status()
                self.publish_docking_begin(False)
                self.get_logger().info('Static docking complete. Returning to EXPLORING.')
                self.transition_to_state0()
        elif status == "dynamic docking is done":
            if not self.dynamic_complete:
                self.dynamic_complete = True
                self.publish_completion_status()
                self.publish_docking_begin(False)
                self.get_logger().info('Dynamic docking complete. Returning to EXPLORING.')
                self.transition_to_state0()
        elif status == "lift docking is done":
            if not self.lift_complete and self.static_complete and self.dynamic_complete:
                self.lift_complete = True
                self.publish_docking_begin(False)
                self.get_logger().info('Lift docking complete. Returning to EXPLORING.')
                self.transition_to_state0()
            else:
                self.get_logger().warn('Lift docking done but conditions not met (static/dynamic not both complete).')
        else:
            self.get_logger().warn(f'Unknown status: {msg.data}')

# ------------------------------------------------------------------------------
# Main entry point
# ------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3FSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('FSM node shut down by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
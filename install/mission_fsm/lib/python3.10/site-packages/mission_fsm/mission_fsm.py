#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

class Turtlebot3FSM(Node):
    """
    Finite State Machine for a real TurtleBot3.
    States:
        0: EXPLORING   - publishes True to 'explore/resume', listens for ArUco markers.
        1: ARUCO_DETECTED - pauses frontier node, prepares for docking.
        2: DOCKING     - starts docking, listens for completion events.
    """

    def __init__(self):
        super().__init__('turtlebot3_fsm')

        # Global flags (station completion status)
        self.station_a_complete = False
        self.station_b_complete = False

        # FSM state
        self.state = 0  # Start in EXPLORING

        # Flags to avoid repeated publications
        self.docking_started = False

        # Publishers
        self.explore_resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.docking_begin_pub = self.create_publisher(Empty, 'docking/begin', 10)

        # Subscribers
        self.aruco_sub = self.create_subscription(
            Bool, 'aruco/detected', self.aruco_callback, 10)
        self.static_dock_sub = self.create_subscription(
            Bool, 'docking/static/complete', self.static_docking_callback, 10)
        self.dynamic_dock_sub = self.create_subscription(
            Bool, 'docking/dynamic/complete', self.dynamic_docking_callback, 10)

        # Publish initial resume command (State 0 entry)
        self.publish_explore_resume(True)

        self.get_logger().info('FSM started in EXPLORING state')
        self.get_logger().info(f'Station A complete: {self.station_a_complete}, Station B complete: {self.station_b_complete}')

    # --------------------------------------------------------------------------
    # Helper methods
    # --------------------------------------------------------------------------
    def publish_explore_resume(self, resume: bool):
        """Publish True to resume frontier node, False to pause it."""
        msg = Bool()
        msg.data = resume
        self.explore_resume_pub.publish(msg)
        self.get_logger().info(f'Published explore/resume = {resume}')

    def publish_docking_begin(self):
        """Send signal to start the docking procedure."""
        self.docking_begin_pub.publish(Empty())
        self.get_logger().info('Published docking/begin signal')

    # --------------------------------------------------------------------------
    # State callbacks (state-specific logic)
    # --------------------------------------------------------------------------
    def on_enter_state0(self):
        """Actions performed when entering EXPLORING state."""
        self.publish_explore_resume(True)      # Resume frontier exploration
        self.docking_started = False           # Reset docking flag for next time
        self.get_logger().info('Entered EXPLORING state (State 0)')

    def on_enter_state1(self):
        """Actions performed when entering ARUCO_DETECTED state."""
        self.publish_explore_resume(False)     # Pause frontier exploration
        self.get_logger().info('Entered ARUCO_DETECTED state (State 1)')

    def on_enter_state2(self):
        """Actions performed when entering DOCKING state."""
        if not self.docking_started:
            self.publish_docking_begin()
            self.docking_started = True
        self.get_logger().info('Entered DOCKING state (State 2)')

    # --------------------------------------------------------------------------
    # FSM transition logic (called from callbacks)
    # --------------------------------------------------------------------------
    def transition_to_state1(self):
        """Transition from EXPLORING to ARUCO_DETECTED."""
        if self.state == 0:
            self.state = 1
            self.on_enter_state1()
            self.transition_to_state2()   # Immediately go to docking

    def transition_to_state2(self):
        """Transition from ARUCO_DETECTED to DOCKING."""
        if self.state == 1:
            self.state = 2
            self.on_enter_state2()

    def transition_to_state0(self):
        """Transition from DOCKING back to EXPLORING."""
        if self.state == 2:
            self.state = 0
            self.on_enter_state0()

    # --------------------------------------------------------------------------
    # ROS2 callbacks
    # --------------------------------------------------------------------------
    def aruco_callback(self, msg: Bool):
        """
        Called when an ArUco marker is detected.
        Only triggers if we are in EXPLORING state and at least one station is incomplete.
        """
        if self.state == 0 and msg.data:
            # Check if any station is still incomplete
            if not self.station_a_complete or not self.station_b_complete:
                self.get_logger().info('ArUco detected and station(s) incomplete → transitioning to docking')
                self.transition_to_state1()
            else:
                self.get_logger().info('ArUco detected but both stations already complete → ignoring')

    def static_docking_callback(self, msg: Bool):
        """
        Called when static docking completes.
        Sets Station_A_Complete flag and returns to EXPLORING.
        """
        if self.state == 2 and msg.data:
            if not self.station_a_complete:
                self.station_a_complete = True
                self.get_logger().info('Static docking complete → Station_A_Complete set to True')
                self.transition_to_state0()
            else:
                self.get_logger().info('Static docking complete but Station_A already complete (ignored)')

    def dynamic_docking_callback(self, msg: Bool):
        """
        Called when dynamic docking completes.
        Sets Station_B_Complete flag and returns to EXPLORING.
        """
        if self.state == 2 and msg.data:
            if not self.station_b_complete:
                self.station_b_complete = True
                self.get_logger().info('Dynamic docking complete → Station_B_Complete set to True')
                self.transition_to_state0()
            else:
                self.get_logger().info('Dynamic docking complete but Station_B already complete (ignored)')

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

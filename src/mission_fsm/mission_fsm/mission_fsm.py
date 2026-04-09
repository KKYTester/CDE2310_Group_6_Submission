#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32, String

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
        self.station_a_in_progress = False
        self.station_b_in_progress = False

        # FSM state
        self.state = 0  # Start in EXPLORING

        # Flags to avoid repeated publications
        self.docking_started = False

        # Publishers
        self.explore_resume_pub = self.create_publisher(Bool, 'explore/resume', 10)
        self.docking_begin_pub = self.create_publisher(Bool, 'docking/begin', 10)

        # Subscribers
        self.aruco_sub = self.create_subscription(
            Int32, 'aruco/marker_id', self.aruco_callback, 10)
        self.docking_status_sub = self.create_subscription(String, '/docking/status', self.docking_status_callback, 10)

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

    def publish_docking_begin(self, begin: bool):
        """Send signal to start the docking procedure."""
        msg = Bool()
        msg.data = begin
        self.docking_begin_pub.publish(msg)
        self.get_logger().info(f'Published docking/begin = {begin}')

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
            self.publish_docking_begin(True)
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
    def aruco_callback(self, msg: Int32):
        """
        Called when an ArUco marker is detected.
        Only triggers if we are in EXPLORING state and at least one station is incomplete.
        """
        marker_id = msg.data   # <-- get the integer ID
        self.get_logger().info(f'ArUco marker detected, ID = {marker_id}')

        # Station A handler
        if self.state == 0 and marker_id == 0:

            if not self.station_a_complete:
                if self.station_a_in_progress:
                    self.get_logger().info('ArUco detected and station A already in progress → ignoring')
                    return
                self.get_logger().info('ArUco detected and station A incomplete → transitioning to docking')
                self.station_a_in_progress = True
                self.transition_to_state1()
            else:
                self.get_logger().info('Station A detected but already complete → ignoring')
        # Station B handler
        if self.state == 0 and marker_id == 1:    
            if not self.station_b_complete:
                if self.station_b_in_progress:
                    self.get_logger().info('ArUco detected and station B already in progress → ignoring')
                    return
                self.get_logger().info('ArUco detected and station B incomplete → transitioning to docking')
                self.station_a_in_progress = True
                self.transition_to_state1()
            else:
                self.get_logger().info('Station B detected but already complete → ignoring')

    def docking_status_callback(self, msg: String):
            if self.state != 2:
                return
            status = msg.data.strip().lower()
            self.get_logger().info(f'Docking status: {status}')
            if status == "static docking is done":
                if not self.station_a_complete:
                    # set progress flags
                    self.station_a_complete = True
                    self.station_a_in_progress = False
                    # Reset docking flag so next time we can start docking again
                    self.publish_docking_begin(False)
                    self.transition_to_state0()
            elif status == "dynamic docking is done":
                if not self.station_b_complete:
                    # set progress flags
                    self.station_b_complete = True
                    self.station_b_in_progress = False
                    # Reset docking flag so next time we can start docking again
                    self.publish_docking_begin(False)
                    self.transition_to_state0()
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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

class AdaptiveScanResampler(Node):
    def __init__(self):
        super().__init__('adaptive_scan_resampler')
        
        # Subscriber to raw scan
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        
        # Publisher for filtered scan
        self.pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )
        
        self.target_n = None  # Target will be set by the first valid scan
        self.get_logger().info('Adaptive scan resampler started. Waiting for first scan...')
    
    def scan_callback(self, msg):
        original_n = len(msg.ranges)
        
        # Set the target based on the first scan we receive
        if self.target_n is None:
            self.target_n = original_n
            self.get_logger().info(f'Target number of readings set to: {self.target_n}')
            # If first scan already matches its own length, just republish
            self.pub.publish(msg)
            return
        
        # If it already matches, republish as is
        if original_n == self.target_n:
            self.pub.publish(msg)
            return
        
        # Resample to the target number of readings
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = msg.angle_min
        new_scan.angle_max = msg.angle_max
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        
        # Compute new angle increment
        total_angle = msg.angle_max - msg.angle_min
        new_scan.angle_increment = total_angle / (self.target_n - 1)
        
        # Original angles for each reading
        orig_angles = [msg.angle_min + i * msg.angle_increment for i in range(original_n)]
        
        # New angles
        new_angles = [new_scan.angle_min + i * new_scan.angle_increment for i in range(self.target_n)]
        
        # Linear interpolation of ranges
        new_ranges = []
        new_intensities = []  # if intensities exist
        has_intensities = hasattr(msg, 'intensities') and len(msg.intensities) == original_n
        
        for new_ang in new_angles:
            # Find nearest two original angles
            idx = 0
            while idx < original_n - 1 and orig_angles[idx+1] < new_ang:
                idx += 1
            
            if new_ang <= orig_angles[0]:
                # Extrapolate from first two points
                x0, x1 = orig_angles[0], orig_angles[1]
                y0, y1 = msg.ranges[0], msg.ranges[1]
                if has_intensities:
                    i0, i1 = msg.intensities[0], msg.intensities[1]
            elif new_ang >= orig_angles[-1]:
                # Extrapolate from last two points
                x0, x1 = orig_angles[-2], orig_angles[-1]
                y0, y1 = msg.ranges[-2], msg.ranges[-1]
                if has_intensities:
                    i0, i1 = msg.intensities[-2], msg.intensities[-1]
            else:
                # Interpolate between idx and idx+1
                x0, x1 = orig_angles[idx], orig_angles[idx+1]
                y0, y1 = msg.ranges[idx], msg.ranges[idx+1]
                if has_intensities:
                    i0, i1 = msg.intensities[idx], msg.intensities[idx+1]
            
            # Linear interpolation for range
            if x1 - x0 < 1e-9:
                new_range = y0
            else:
                t = (new_ang - x0) / (x1 - x0)
                new_range = y0 + t * (y1 - y0)
            new_ranges.append(new_range)
            
            # Interpolate intensities if present
            if has_intensities:
                new_int = i0 + t * (i1 - i0) if x1 - x0 > 1e-9 else i0
                new_intensities.append(new_int)
        
        new_scan.ranges = new_ranges
        if has_intensities:
            new_scan.intensities = new_intensities
        
        self.pub.publish(new_scan)
        self.get_logger().debug(f'Resampled from {original_n} to {self.target_n} readings')

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveScanResampler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
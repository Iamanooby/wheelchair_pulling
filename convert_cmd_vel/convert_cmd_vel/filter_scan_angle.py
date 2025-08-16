import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class FilterScanNode(Node):
    def __init__(self):
        super().__init__('filter_scan_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/filter_scan', 10)

    def scan_callback(self, msg: LaserScan):
        # Compute angles for each range measurement
        num_ranges = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_ranges)
        
        # Mask out the ranges within ±50 degrees (converted to radians)
        mask = (angles < -np.deg2rad(50)) | (angles > np.deg2rad(50))
        filtered_ranges = np.array(msg.ranges)
        filtered_ranges[~mask] = float('inf')  # Set removed values to infinity
        
        # Create a new LaserScan message with filtered ranges
        filtered_msg = msg
        filtered_msg.ranges = filtered_ranges.tolist()
        
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FilterScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

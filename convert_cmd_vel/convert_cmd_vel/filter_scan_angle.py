import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class FilterScanNode(Node):
    def __init__(self):
        super().__init__('filter_scan_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(LaserScan, '/filter_scan', 10)
        
        # Create static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        # Create a static transform that rotates 180 degrees about z-axis
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_scan'  # Original frame
        transform.child_frame_id = 'rotated_base_scan'  # New rotated frame
        
        # No translation
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # 180 degree rotation about z-axis (quaternion: w=0, x=0, y=0, z=1)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 1.0
        transform.transform.rotation.w = 0.0
        
        self.tf_broadcaster.sendTransform(transform)

    def scan_callback(self, msg: LaserScan):
        # Compute angles for each range measurement
        num_ranges = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_ranges)
        
        # Mask out the ranges within ±60 degrees (converted to radians)
        mask = (angles < -np.deg2rad(60)) | (angles > np.deg2rad(60))
        filtered_ranges = np.array(msg.ranges)
        filtered_ranges[~mask] = float('inf')  # Set removed values to infinity
        
        # Create a new LaserScan message with filtered and rotated ranges
        filtered_msg = LaserScan()
        filtered_msg.header.stamp = msg.header.stamp
        filtered_msg.header.frame_id = 'rotated_base_scan'  # Use rotated frame
        
        # # Reverse the ranges array for 180 degree rotation
        # filtered_msg.ranges = filtered_ranges[::-1].tolist()
        
        # # Adjust angles by adding π (180 degrees) and normalize
        # filtered_msg.angle_min = msg.angle_min + math.pi
        # filtered_msg.angle_max = msg.angle_max + math.pi
        
        # # Normalize angles to [-π, π] range
        # if filtered_msg.angle_min > math.pi:
        #     filtered_msg.angle_min -= 2 * math.pi
        # if filtered_msg.angle_max > math.pi:
        #     filtered_msg.angle_max -= 2 * math.pi
        
        length_ranges = len(filtered_ranges)
        # second half become first half instead
        filtered_msg.ranges = filtered_ranges[length_ranges//2:].tolist() + filtered_ranges[:length_ranges//2].tolist()

        # Copy other fields
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.intensities = msg.intensities[::-1] if msg.intensities else []
        
        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FilterScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

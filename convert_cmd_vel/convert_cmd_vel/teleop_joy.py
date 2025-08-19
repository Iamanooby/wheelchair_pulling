import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        
        # QoS profile
        qos = QoSProfile(depth=10)
        self.max_x = 0.22
        self.max_w = 0.25
        # Publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel_test', qos)
        
        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos
        )
        
        self.get_logger().info('Teleop Joy node started')
    
    def joy_callback(self, msg):
        twist = Twist()
        
        # Map joystick axes to twist message
        # Typically: left stick vertical = linear.x, left stick horizontal = angular.z
        if len(msg.axes) >= 2:
            twist.linear.x = msg.axes[1]*self.max_x  # Forward/backward
            twist.angular.z = msg.axes[3]*self.max_w  # Left/right rotation

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

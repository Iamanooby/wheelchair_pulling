import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

radius = 0.3

class OutputHoloTow(Node):

    def __init__(self):
        super().__init__('output_holo_tow')

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.cmd_vel_subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_safe', 10)


    def cmd_vel_callback(self, msg):
        pubCmdMsg = Twist()
        vx = msg.linear.x
        az = msg.angular.z

        pubCmdMsg.linear.x = vx
        pubCmdMsg.linear.y = radius*az
        pubCmdMsg.angular.z = az
        self.publisher_.publish(pubCmdMsg)

        self.get_logger().info(f'X: {pubCmdMsg.linear.x}, Y : {pubCmdMsg.linear.y}, Z : {pubCmdMsg.angular.z} ')




def main(args=None):
    rclpy.init(args=args)

    output_holo_tow = OutputHoloTow()

    rclpy.spin(output_holo_tow)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    output_holo_tow.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
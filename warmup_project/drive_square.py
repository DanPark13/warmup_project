"""Drive the Neato in a Square"""

import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square')

        # Set up timers for when to turn
        self.start_timestamp = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(0.1, self.run_loop)

        # Set up publisher to push code
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def run_loop(self):
        msg = Twist()
        current_time = self.get_clock().now().nanoseconds

        print((current_time - self.start_timestamp) * (10 ** -9))

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
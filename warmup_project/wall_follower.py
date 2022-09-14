import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # Neato control Messages

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_msg)
        self.speed_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # not sure of scan topic object type
    def process_scan(self, msg):
        ranges = scan_subscriber

def main(args=None):
    rclpy.init(args=args) # initializes ROS connection
    node = Publisher() # create node of type Publisher
    rclpy.spin(node) # keeps the node from 'dying', nodes usually stop after doing their job once
    rclpy.shutdown()



if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 #Neato control Messages
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class PositionPlotterNode(Node):
    def __init__(self):
        super().__init__("position_plotter_node")
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.process_odom, 10)

    def process_odom(self, msg):
        #print(msg.pose.pose.position)
        print(msg.pose.pose.orientation)
        

def main(args=None):
    rclpy.init(args=args)
    node = PositionPlotterNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
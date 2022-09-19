import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 #Neato control Messages
from sensor_msgs.msg import LaserScan


class RangePlotterNode(Node):
    def __init__(self):
        super().__init__("range_plotter_node")
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

    def process_scan(self, msg):
        angles = range(360)
        ranges_dict = {}
        for angle in angles:
            ranges_dict[angle] = msg.ranges[angle]
            #print(ranges_dict[angle])
        for key,value in ranges_dict.items():
            print(str(key) + " => " + str(value))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RangePlotterNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
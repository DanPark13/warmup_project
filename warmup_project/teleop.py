import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        settings = termios.tcgetattr(sys.stdin)
        key = None
        self.process_key(key, settings)


    def process_key(self, key, settings):
        msg = Twist()
        while key != '\x03':
            key = self.getKey(settings)
            print(key)
            if key == '\x77': # w
                msg.linear.x = 1.0
                msg.angular.z = 0.0
            elif key == '\x61': # a
                msg.linear.x = 0.0
                msg.angular.z = 1.0
            elif key == '\x73': # s
                msg.linear.x = -1.0
                msg.angular.z = 0.0
            elif key == '\x64': # d
                msg.linear.x = 0.0
                msg.angular.z = -1.0
            elif key == '\x20':
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            


            self.vel_pub.publish(msg)
 

    def getKey(self, settings):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key





def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == 'main':
    main()
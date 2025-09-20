#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Use keys:
w : move forward
z : move backward
a : turn left
d : turn right
s : stop

CTRL-C to quit
"""

moveBindings = {
    'w': (0.2, 0.0),
    'z': (-0.2, 0.0),
    'a': (0.0, 0.4),
    'd': (0.0, -0.4),
    's': (0.0, 0.0)
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('custom_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Custom Teleop Node Started')
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.x = 0.0
        self.z = 0.0

    def timer_callback(self):
        key = getKey()
        if key in moveBindings.keys():
            self.x, self.z = moveBindings[key]
            twist = Twist()
            twist.linear.x = self.x
            twist.angular.z = self.z
            self.pub.publish(twist)
            self.get_logger().info(f'Move command: linear_x={self.x}, angular_z={self.z}')
        elif key == '\x03':  # CTRL-C
            rclpy.shutdown()

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    print(msg)
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


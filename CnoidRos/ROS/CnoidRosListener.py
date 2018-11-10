#!/usr/bin/python3


import sys

import rclpy
from rclpy.qos import qos_profile_default
from std_msgs.msg import String
import command_dispatcher

def listenerCallback(msg):
    print('Received: [%s]' % msg.data)
    command_dispatcher.do_commands(msg.data)


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args)

    node = rclpy.create_node('CnoidRosListener')

    sub = node.create_subscription(String, 'CnoidRos', listenerCallback, qos_profile_default)
    assert sub  # prevent unused warning

    rclpy.spin(node)


if __name__ == '__main__':
    main()

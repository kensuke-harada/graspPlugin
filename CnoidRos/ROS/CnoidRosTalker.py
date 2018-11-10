#!/usr/bin/python3


import os
import sys
import mmap
import math
import copy
from time import sleep

import rclpy
from rclpy.qos import qos_profile_default
from std_msgs.msg import String


def main(args = None):
    if args is None:
        args = sys.argv

    rclpy.init(args)

    node = rclpy.create_node('CnoidRosTalker')
    pub = node.create_publisher(String, 'CnoidRos', qos_profile_default)

    mmapPath = os.environ['HOME'] + '/.CnoidRosMappedFile'
    msg = String()

    while True:
        with open(mmapPath, 'r+b') as f:
            mapf = mmap.mmap(f.fileno(), 0)
            line = mapf.readline().decode('UTF-8')
            line = line.strip()
            if line:
                mapf.seek(0)
                f.write(bytes('\n', 'UTF-8'))
                msg.data = line
                pub.publish(msg)
                print('Send: [%s]' % msg.data)
            mapf.close()
        sleep(0.1)

if __name__ == '__main__':
    main()

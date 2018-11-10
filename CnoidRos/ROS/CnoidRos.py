#!/usr/bin/env python

import os
import mmap
import math
import copy
import rospy
from hrpsys_ros_bridge.srv import *
from nextage_ros_bridge import nextage_client


class NextageUtil(object):
    def __init__(self):
        # Initial Pose
        self.initPose = nextage_client.NextageClient._InitialPose
        self.offPose = nextage_client.NextageClient.OffPose
        for angles in self.initPose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        for angles in self.offPose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        self.pose = copy.copy(self.initPose)

        # nextage_ros_bridge
        self.setJointAnglesOfGroup = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setJointAnglesOfGroup',
            OpenHRP_SequencePlayerService_setJointAnglesOfGroup)

    def goPose(self, pose, time):
        return self.setJointAnglesOfGroup('torso', pose[0], time) \
            and self.setJointAnglesOfGroup('head', pose[1], time) \
            and self.setJointAnglesOfGroup('rarm', pose[2], time) \
            and self.setJointAnglesOfGroup('larm', pose[3], time)

    def go(self, time):
        result = self.goPose(self.pose, time)
        if result:
            rospy.sleep(time)
        return result

    def goInitial(self, time=3):
        self.pose = copy.copy(self.initPose)
        return self.go(time)

    def goOff(self, time=3):
        self.pose = copy.copy(self.offPose)
        return self.go(time)

    def servoOn(self):
        rospy.loginfo('Called servoOn()...')
        return self.goInitial(5)

    def servoOff(self):
        rospy.loginfo('Called servoOff()...')
        return self.goOff(5)

    def setTorso(self, strArray):
        joint = [
            float(strArray[0])
        ]
        self.pose[0] = joint

    def setHead(self, strArray):
        joint = [
            float(strArray[0]),
            float(strArray[1])
        ]
        self.pose[1] = joint

    def setArmR(self, strArray):
        joint = [
            float(strArray[0]),
            float(strArray[1]),
            float(strArray[2]),
            float(strArray[3]),
            float(strArray[4]),
            float(strArray[5])
        ]
        self.pose[2] = joint

    def setArmL(self, strArray):
        joint = [
            float(strArray[0]),
            float(strArray[1]),
            float(strArray[2]),
            float(strArray[3]),
            float(strArray[4]),
            float(strArray[5])
        ]
        self.pose[3] = joint


def move(util, line):
    if line == 'servoOn()':
        util.servoOn()
    elif line == 'servoOff()':
        util.servoOff()
    elif line == 'goInitial()':
        util.goInitial()
    elif line == 'goOff()':
        util.goOff()
    else:
        for command in line.split(';'):
            command = command.strip()

            if command:
                if command.startswith('rhandOpen'):
                    rospy.loginfo('Not Implemented : ' + command)
                elif command.startswith('rhandClose'):
                    rospy.loginfo('Not Implemented : ' + command)
                elif command.startswith('lhandOpen'):
                    rospy.loginfo('Not Implemented : ' + command)
                elif command.startswith('lhandClose'):
                    rospy.loginfo('Not Implemented : ' + command)
                elif command.startswith('joints'):
                    splitted = command.split(':')
                    start = splitted[0].find('[') + 1
                    end = splitted[0].find(']')
                    if start < end:
                        jointStr = splitted[0][start:end]
                    else:
                        jointStr = ''
                    time = float(splitted[1])

                    angles = jointStr.split(',')
                    util.setTorso(angles[0:1])
                    util.setHead(angles[1:3])
                    util.setArmR(angles[3:9])
                    util.setArmL(angles[9:15])
                    util.go(time)
                else:
                    rospy.loginfo('Command Not Found : ' + command)

if __name__ == '__main__':
    rospy.init_node('CnoidRos')
    try:
        util = NextageUtil()
        rate = rospy.Rate(10)
        mmapPath = os.environ['HOME'] + '/.CnoidRosMappedFile'
        while not rospy.is_shutdown():
            with open(mmapPath, 'r+b') as f:
                mapf = mmap.mmap(f.fileno(), 0)
                line = mapf.readline().strip()
                if line:
                    mapf.seek(0)
                    f.write('\n')
                    move(util, line)
                mapf.close()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

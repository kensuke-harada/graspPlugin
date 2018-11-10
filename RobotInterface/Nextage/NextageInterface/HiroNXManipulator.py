import os
from math import pi

#from hironx_ros_bridge import hironx_client 
from hironx_ros_bridge.ros_client import ROS_Client
# This should come earlier than later import.
# See http://code.google.com/p/rtm-ros-robotics/source/detail?r=6773
from nextage_ros_bridge import nextage_client

from hrpsys import rtm

FILENAME_ROBOTHOST='.robothost'
FILENAME_ROBOTNAME='.robotname'

robot = None

def init():
    global robot
    if os.path.isfile(FILENAME_ROBOTHOST):
        f = open(FILENAME_ROBOTHOST, 'r')
        data =  f.readline().strip().split(':')
        f.close()
        host = data[0]
        port = data[1]
    else:
        host = 'localhost'
        port = 2809
    print 'host:' + host
    print 'port:' + port
    rtm.nshost = host
    rtm.nsport = port
    robot_name = "RobotHardware0" if host != 'localhost' else "HiroNX(Robot)0"
    #robot = hironx_client.HIRONX()
    robot = nxc = nextage_client.NextageClient()

def connect():
    robot_name = "RobotHardware0" #"HiroNX(Robot)0"
    if os.path.isfile(FILENAME_ROBOTNAME):
        f = open(FILENAME_ROBOTNAME, 'r')
        robot_name =  f.readline().strip()
        f.close()    
    robot.init(robotname=robot_name, url="")
    print 'connected'
    
def servoON():
    robot.servoOn()
    
def servoOFF():
    robot.servoOff()
    
def calibrateJoint():
    robot.checkEncoders()
    
def goInitial():
    robot.goInitial()
    
def goOffPose():
    robot.goOffPose()
    
def rhandOpen(distance=100):
    print('rhandOpen')
    #robot.setHandWidth('rhand', distance)
    
def rhandClose(distance=0):
    print('rhandClose')
    #robot.setHandWidth('rhand', distance)
    
def lhandOpen(distance=100):
    print('lhandOpen')
    #robot.setHandWidth('lhand', distance)
    
def lhandClose(distance=0):
    print('lhandClose')
    #robot.setHandWidth('lhand', distance) 

def rhandAttach():
    print('attach')
    robot._hands.handtool_r_attach()
    
def rhandDetach():
    print('detach')
    robot._hands.handtool_r_eject()
    
def lhandAttach():
    print('Attach')
    robot._hands.handtool_l_attach()
    
def lhandDetach():
    print('Detach')
    robot._hands.handtool_l_eject() 

    
def setRHandAnglesDeg(angles):
    robot.setHandJointAngles('rhand', [v * pi / 180.0 for v in angles], 1.0)
    
def setLHandAnglesDeg(angles):
    robot.setHandJointAngles('lhand', [v * pi / 180.0 for v in angles], 1.0)

def moveRArmRel(dx, dy, dz, dr, dp, dw):
    robot.setTargetPoseRelative('rarm', 'RHAND_JOINT5', dx, dy, dz, dr, dp, dw, 1.0)
    
def moveLArmRel(dx, dy, dz, dr, dp, dw):
    robot.setTargetPoseRelative('larm', 'LHAND_JOINT5', dx, dy, dz, dr, dp, dw, 1.0)

def setJointAngles(angles,tm=1.0):
    robot.setJointAnglesOfGroup('torso', angles[0:1], tm, False)
    robot.setJointAnglesOfGroup('head', angles[1:3], tm, False)
    robot.setJointAnglesOfGroup('rarm', angles[3:9], tm, False)
    robot.setJointAnglesOfGroup('larm', angles[9:15], tm, True)


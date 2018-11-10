def init():
    pass

def connect():
    print 'connect()'

def servoON():
    print 'servoOn()'

def servoOFF():
    print 'servoOff()'

def calibrateJoint():
    print 'calibrateJoint()'

def goInitial():
    print 'goInitial()'

def goOffPose():
    print 'goOffPose()'

def rhandOpen(distance=100):
    print 'rhandOpen()'

def rhandClose(distance=0):
    print 'rhandClose()'

def lhandOpen(distance=100):
    print 'lhandOpen()'

def lhandClose(distance=0):
    print 'lhandClose()'

def setRHandAnglesDeg(angles):
    print 'setRHandAnglesDeg(' + str(angles) + ')'

def setLHandAnglesDeg(angles):
    print 'setLHandAnglesDeg(' + str(angles) + ')'

def moveRArmRel(dx, dy, dz, dr, dp, dw):
    print 'moveRArmRel(dx=' + str(dx) + ' dy=' + str(dy) + ' dz=' + str(dz) + \
          ' dr=' + dr + ' dp=' + dp + ' dw=' + dw + ')'

def moveLArmRel(dx, dy, dz, dr, dp, dw):
    print 'moveLArmRel(dx=' + str(dx) + ' dy=' + str(dy) + ' dz=' + str(dz) + \
          ' dr=' + dr + ' dp=' + dp + ' dw=' + dw + ')'

def setJointAngles(angles,tm=1.0):
    print 'setJointAngles(torso=' + str(angles[0:1]) + ' head=' + str(angles[1:3]) + \
          ' rarm=' + str(angles[3:9]) + ' larm=' + str(angles[9:15]) + ' time=' + str(tm) + ')'

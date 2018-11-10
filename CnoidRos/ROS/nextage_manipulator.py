def servoOn():
    print('servoOn()')


def servoOff():
    print('servoOff()')


def goInitial():
    print('goInitial()')


def goOffPose():
    print('goOffPose()')


def rhandOpen():
    print('rhandOpen()')


def rhandClose():
    print('rhandClose()')


def lhandOpen():
    print('lhandOpen()')


def lhandClose():
    print('lhandClose()')


def setJointAngles(angles, time):
    joint_angles = {'torso': angles[0:1], 'head': angles[1:3],
                    'rarm': angles[3:9], 'larm': angles[9:15]}
    print('setJointAngle(')
    print('angle : ' + str(joint_angles))
    print('time : ' + str(time))
    print(')')

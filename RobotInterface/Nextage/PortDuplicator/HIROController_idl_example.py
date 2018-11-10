#!/usr/bin/env python

"""
 \file HIROController_idl_examplefile.py
 \brief Python example implementations generated from HIROController.idl
 \date $Date$


"""

import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA

class RETURN_ID:
    def __init__(self, id, comment):
        self.id = id
        self.comment = comment

class CommonCommands_i (_GlobalIDL__POA.CommonCommands):
    """
    \class CommonCommands_i
    Example class implementing IDL interface CommonCommands
    """

    def __init__(self):
        """
        \brief standard constructor
        Initialise member variables here
        """
        self.if0 = None
        self.if1 = None

    def setInterface(self, if0, if1):
        self.if0 = if0
        self.if1 = if1
        
    # RETURN_ID servoOFF()
    def servoOFF(self):
        ret0 = self.if0._ptr().servoOFF()
        ret1 = self.if1._ptr().servoOFF()
        return self.__make_return(ret0, ret1)

    # RETURN_ID servoON()
    def servoON(self):
        ret0 = self.if0._ptr().servoON()
        ret1 = self.if1._ptr().servoON()
        return self.__make_return(ret0, ret1)

    # RETURN_ID servoOFFArm()
    def servoOFFArm(self):
        ret0 = self.if0._ptr().servoOFFArm()
        ret1 = self.if1._ptr().servoOFFArm()
        return self.__make_return(ret0, ret1)

    # RETURN_ID servoOFFHand()
    def servoOFFHand(self):
        ret0 = self.if0._ptr().servoOFFHand()
        ret1 = self.if1._ptr().servoOFFHand()
        return self.__make_return(ret0, ret1)

    # RETURN_ID servoONArm()
    def servoONArm(self):
        ret0 = self.if0._ptr().servoONArm()
        ret1 = self.if1._ptr().servoONArm()
        return self.__make_return(ret0, ret1)

    # RETURN_ID servoONHand()
    def servoONHand(self):
        ret0 = self.if0._ptr().servoONHand()
        ret1 = self.if1._ptr().servoONHand()
        return self.__make_return(ret0, ret1)

    def __make_return(self, ret0, ret1):
        if ret0.id == 0 or ret1.id == 0:
            return RETURN_ID(0, "OK")
        else:
            return RETURN_ID(-1, "NG")


class MotionCommands_i (_GlobalIDL__POA.MotionCommands):
    """
    \class MotionCommands_i
    Example class implementing IDL interface MotionCommands
    """

    def __init__(self):
        """
        \brief standard constructor
        Initialise member variables here
        """
        self.if0 = None
        self.if1 = None

    def setInterface(self, if0, if1):
        self.if0 = if0
        self.if1 = if1
        
    # RETURN_ID closeGripper()
    def closeGripper(self):
        ret0 = self.if0._ptr().closeGripper()
        ret1 = self.if1._ptr().closeGripper()
        return self.__make_return(ret0, ret1)

    # RETURN_ID moveGripper(in MotionCommands::DoubleSeq r_angle, in MotionCommands::DoubleSeq l_angle)
    def moveGripper(self, r_angle, l_angle):
        ret0 = self.if0._ptr().moveGripper()
        ret1 = self.if1._ptr().moveGripper()
        return self.__make_return(ret0, ret1)

    # RETURN_ID moveLinearCartesianAbs(in CarPosWithElbow rArm, in CarPosWithElbow lArm)
    def moveLinearCartesianAbs(self, rArm, lArm):
        ret0 = self.if0._ptr().moveLinearCartesianAbs(rArm, lArm)
        ret1 = self.if1._ptr().moveLinearCartesianAbs(rArm, lArm)
        return self.__make_return(ret0, ret1)

    # RETURN_ID moveLinearCartesianRel(in CarPosWithElbow rArm, in CarPosWithElbow lArm)
    def moveLinearCartesianRel(self, rArm, lArm):
        ret0 = self.if0._ptr().moveLinearCartesianRel(rArm, lArm)
        ret1 = self.if1._ptr().moveLinearCartesianRel(rArm, lArm)
        return self.__make_return(ret0, ret1)

    # RETURN_ID movePTPJointAbs(in MotionCommands::JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints):
        ret0 = self.if0._ptr().movePTPJointAbs(jointPoints)
        ret1 = self.if1._ptr().movePTPJointAbs(jointPoints)
        return self.__make_return(ret0, ret1) 

    # RETURN_ID movePTPJointRel(in MotionCommands::JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        ret0 = self.if0._ptr().movePTPJointRel(jointPoints)
        ret1 = self.if1._ptr().movePTPJointRel(jointPoints)
        return self.__make_return(ret0, ret1)

    # RETURN_ID movePTPJointAbsSeq(in MotionCommands::JointPosSeq jointPointsSeq, in MotionCommands::DoubleSeq timeSeq)
    def movePTPJointAbsSeq(self, jointPointsSeq, timeSeq):
        ret0 = self.if0._ptr().movePTPJointAbsSeq(jointPointsSeq, timeSeq)
        ret1 = self.if1._ptr().movePTPJointAbsSeq(jointPointsSeq, timeSeq)
        return self.__make_return(ret0, ret1)

    # RETURN_ID openGripper()
    def openGripper(self):
        ret0 = self.if0._ptr().openGripper()
        ret1 = self.if1._ptr().openGripper()
        return self.__make_return(ret0, ret1)

    # RETURN_ID setSpeedCartesian(in MotionCommands::ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        ret0 = self.if0._ptr().setSpeedCartesian(spdRatio)
        ret1 = self.if1._ptr().setSpeedCartesian(spdRatio)
        return self.__make_return(ret0, ret1)

    # RETURN_ID setSpeedJoint(in MotionCommands::ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
        ret0 = self.if0._ptr().setSpeedJoint(spdRatio)
        ret1 = self.if1._ptr().setSpeedJoint(spdRatio)
        return self.__make_return(ret0, ret1)

    # RETURN_ID setMotionTime(in double motionTime)
    def setMotionTime(self, motionTime):
        ret0 = self.if0._ptr().setMotionTime(motionTime)
        ret1 = self.if1._ptr().setMotionTime(motionTime)
        return self.__make_return(ret0, ret1)

    def __make_return(self, ret0, ret1):
        if ret0.id == 0 or ret1.id == 0:
            return RETURN_ID(0, "OK")
        else:
            return RETURN_ID(-1, "NG")
            
if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = CommonCommands_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    print orb.object_to_string(objref)

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()


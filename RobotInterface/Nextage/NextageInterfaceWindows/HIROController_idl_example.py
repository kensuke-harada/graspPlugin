#!/usr/bin/env python

"""
 \file HIROController_idl_examplefile.py
 \brief Python example implementations generated from HIROController.idl
 \date $Date$


"""
import HiroNXManipulator
import util

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
        pass

    # RETURN_ID servoOFF()
    def servoOFF(self):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID servoON()
    def servoON(self):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID servoOFFArm()
    def servoOFFArm(self):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID servoOFFHand()
    def servoOFFHand(self):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID servoONArm()
    def servoONArm(self):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID servoONHand()
    def servoONHand(self):
        print "no implementaion"
        return RETURN_ID(-1, "")
    
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
        #pass
        self.defalut_time = 5.0
        self.time = self.defalut_time
        self.ratio = 0.1

    # RETURN_ID closeGripper()
    def closeGripper(self):
        HiroNXManipulator.rhandClose()
        HiroNXManipulator.lhandClose()
        return RETURN_ID(0, "OK")

    # RETURN_ID moveGripper(in MotionCommands::DoubleSeq r_angle, in MotionCommands::DoubleSeq l_angle)
    def moveGripper(self, r_angle, l_angle):
        HiroNXManipulator.setRHandAnglesDeg(r_angle)
        HiroNXManipulator.setLHandAnglesDeg(l_angle)
        return RETURN_ID(0, "OK")

    # RETURN_ID moveLinearCartesianAbs(in CarPosWithElbow rArm, in CarPosWithElbow lArm)
    def moveLinearCartesianAbs(self, rArm, lArm):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID moveLinearCartesianRel(in CarPosWithElbow rArm, in CarPosWithElbow lArm)
    def moveLinearCartesianRel(self, rArm, lArm):
        dx = rArm.carPos[0][3]
        dy = rArm.carPos[1][3]
        dz = rArm.carPos[2][3]
        dr, dp, dw = util.omegaFromRot(rArm.carPos) 
        HiroNXManipulator.moveRArmRel(dx, dy, dz, dr, dp, dw)
        dx = lArm.carPos[0][3]
        dy = lArm.carPos[1][3]
        dz = lArm.carPos[2][3]
        dr, dp, dw = util.omegaFromRot(lArm.carPos)
        HiroNXManipulator.moveLArmRel(dx, dy, dz, dr, dp, dw)
        return RETURN_ID(0, "OK")

    # RETURN_ID movePTPJointAbs(in MotionCommands::JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints, tm=1.0):
        HiroNXManipulator.setJointAngles(jointPoints, self.time)
        return RETURN_ID(0, "OK")


    # RETURN_ID movePTPJointRel(in MotionCommands::JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID movePTPJointAbsSeq(in MotionCommands::JointPosSeq jointPointsSeq, in MotionCommands::DoubleSeq timeSeq)
    def movePTPJointAbsSeq(self, jointPointsSeq, timeSeq):
        for zp in zip(jointPointsSeq, timeSeq):
            jp, time = zp
            if len(jp) < 15:
                raise 'pose broken.'
            self.movePTPJointAbs(jp, time)
        return RETURN_ID(0, "OK")

    # RETURN_ID openGripper()
    def openGripper(self):
        HiroNXManipulator.rhandOpen()
        HiroNXManipulator.lhandOpen()
        return RETURN_ID(0, "OK")
        
    # RETURN_ID setSpeedCartesian(in MotionCommands::ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID setSpeedJoint(in MotionCommands::ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
        self.time = spdRatio
        return RETURN_ID(0, "OK")

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


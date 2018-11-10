#!/usr/bin/env python

"""
 \file HIROController_idl_examplefile.py
 \brief Python example implementations generated from HIROController.idl
 \date $Date$


"""

import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA

import seed_manipulator

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
        pass

    # RETURN_ID closeGripper()
    def closeGripper(self):
        seed_manipulator.close_hand()
        return RETURN_ID(0, "OK")

    # RETURN_ID moveGripper(in MotionCommands::DoubleSeq r_angle, in MotionCommands::DoubleSeq l_angle)
    def moveGripper(self, r_angle, l_angle):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID moveLinearCartesianAbs(in CarPosWithElbow rArm, in CarPosWithElbow lArm)
    def moveLinearCartesianAbs(self, rArm, lArm):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID moveLinearCartesianRel(in CarPosWithElbow rArm, in CarPosWithElbow lArm)
    def moveLinearCartesianRel(self, rArm, lArm):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID movePTPJointAbs(in MotionCommands::JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID movePTPJointRel(in MotionCommands::JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID movePTPJointAbsSeq(in MotionCommands::JointPosSeq jointPointsSeq, in MotionCommands::DoubleSeq timeSeq)
    def movePTPJointAbsSeq(self, jointPointsSeq, timeSeq):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID openGripper()
    def openGripper(self):
        seed_manipulator.open_hand()
        return RETURN_ID(1, "OK")

    # RETURN_ID setSpeedCartesian(in MotionCommands::ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID setSpeedJoint(in MotionCommands::ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
        print "no implementaion"
        return RETURN_ID(-1, "")

    # RETURN_ID setMotionTime(in double motionTime)
    def setMotionTime(self, motionTime):
        print "no implementaion"
        return RETURN_ID(-1, "")

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


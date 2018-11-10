#!/usr/bin/env python

"""
 \file HiroNX_idl_examplefile.py
 \brief Python example implementations generated from HiroNX.idl
 \date $Date$


"""
import HiroNXManipulator

import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA


class HiroNX_i (_GlobalIDL__POA.HiroNX):
    """
    \class HiroNX_i
    Example class implementing IDL interface HiroNX
    """

    def __init__(self):
        """
        \brief standard constructor
        Initialise member variables here
        """
        pass

    # void setupRobot()
    def setupRobot(self):
        HiroNXManipulator.connect()

    # void restart()
    def restart(self):
        print "no implementaion"

    # void goInitial()
    def goInitial(self):
        HiroNXManipulator.goInitial()

    # void goOffPose()
    def goOffPose(self):
        HiroNXManipulator.goOffPose()

    # void servoOn()
    def servoOn(self):
        HiroNXManipulator.servoON()

    # void servoOff()
    def servoOff(self):
        HiroNXManipulator.servoOFF()

    # void calibrateJoint()
    def calibrateJoint(self):
        HiroNXManipulator.calibrateJoint()

    # void servoOnHands()
    def servoOnHands(self):
        print "no implementaion"

    # void servoOffHands()
    def servoOffHands(self):
        print "no implementaion"

    # void EngageProtectiveStop()
    def EngageProtectiveStop(self):
        print "no implementaion"

    # void DisengageProtectiveStop()
    def DisengageProtectiveStop(self):
        print "no implementaion"

    # void reboot()
    def reboot(self):
        print "no implementaion"

    # void shutdown()
    def shutdown(self):
        print "no implementaion"

    # void rhandOpen()
    def rhandOpen(self):
        HiroNXManipulator.rhandOpen()

    # void rhandClose()
    def rhandClose(self):
        HiroNXManipulator.rhandClose()

    # void lhandOpen()
    def lhandOpen(self):
        HiroNXManipulator.lhandOpen()
        
    # void lhandClose()
    def lhandClose(self):
        HiroNXManipulator.lhandClose()


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = HiroNX_i()

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


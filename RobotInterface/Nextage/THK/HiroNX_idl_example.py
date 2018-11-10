#!/usr/bin/env python

"""
 \file HiroNX_idl_examplefile.py
 \brief Python example implementations generated from HiroNX.idl
 \date $Date$


"""

import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA

import seed_manipulator

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
        seed_manipulator.init()

    # void restart()
    def restart(self):
        print "no implementation"

    # void goInitial()
    def goInitial(self):
        print "no implementation"

    # void goOffPose()
    def goOffPose(self):
        print "no implementation"

    # void servoOn()
    def servoOn(self):
        print "no implementation"

    # void servoOff()
    def servoOff(self):
        print "no implementation"

    # void calibrateJoint()
    def calibrateJoint(self):
        print "no implementation"

    # void servoOnHands()
    def servoOnHands(self):
        print "no implementation"

    # void servoOffHands()
    def servoOffHands(self):
        print "no implementation"

    # void EngageProtectiveStop()
    def EngageProtectiveStop(self):
        print "no implementation"

    # void DisengageProtectiveStop()
    def DisengageProtectiveStop(self):
        print "no implementation"

    # void reboot()
    def reboot(self):
        print "no implementation"

    # void shutdown()
    def shutdown(self):
        print "no implementation"

    # void rhandOpen()
    def rhandOpen(self):
        seed_manipulator.open_hand(2)

    # void rhandClose()
    def rhandClose(self):
        seed_manipulator.close_hand(2)

    # void lhandOpen()
    def lhandOpen(self):
        seed_manipulator.open_hand(1)

    # void lhandClose()
    def lhandClose(self):
        seed_manipulator.close_hand(1)

    def rhandAttach(self):
        print('attach')

    def rhandDetach(self):
        print('detach')

    def lhandAttach(self):
        print('attach')

    def lhandDetach(self):
        print('detach')

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


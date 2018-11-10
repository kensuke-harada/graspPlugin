#!/usr/bin/env python

"""
 \file HiroNX_idl_examplefile.py
 \brief Python example implementations generated from HiroNX.idl
 \date $Date$


"""

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
        self.if0 = None
        self.if1 = None

    def setInterface(self, if0, if1):
        self.if0 = if0
        self.if1 = if1

    # void setupRobot()
    def setupRobot(self):
        self.if0._ptr().setupRobot()
        self.if1._ptr().setupRobot()

    # void restart()
    def restart(self):
        self.if0._ptr().restart()
        self.if1._ptr().restart()

    # void goInitial()
    def goInitial(self):
        self.if0._ptr().goInitial()
        self.if1._ptr().goInitial()
        
    # void goOffPose()
    def goOffPose(self):
        self.if0._ptr().goOffPose()
        self.if1._ptr().goOffPose()

    # void servoOn()
    def servoOn(self):
        self.if0._ptr().servoOn()
        self.if1._ptr().servoOn()

    # void servoOff()
    def servoOff(self):
        self.if0._ptr().servoOff()
        self.if1._ptr().servoOff()

    # void calibrateJoint()
    def calibrateJoint(self):
        self.if0._ptr().calibrateJoint()
        self.if1._ptr().calibrateJoint()

    # void servoOnHands()
    def servoOnHands(self):
        self.if0._ptr().servoOnHands()
        self.if1._ptr().servoOnHands()

    # void servoOffHands()
    def servoOffHands(self):
        self.if0._ptr().servoOffHands()
        self.if1._ptr().servoOffHands()

    # void EngageProtectiveStop()
    def EngageProtectiveStop(self):
        self.if0._ptr().EngageProtectiveStop()
        self.if1._ptr().EngageProtectiveStop()

    # void DisengageProtectiveStop()
    def DisengageProtectiveStop(self):
        self.if0._ptr().DisengageProtectiveStop()
        self.if1._ptr().DisengageProtectiveStop()

    # void reboot()
    def reboot(self):
        self.if0._ptr().reboot()
        self.if1._ptr().reboot()

    # void shutdown()
    def shutdown(self):
        self.if0._ptr().shutdown()
        self.if1._ptr().shutdown()

    # void rhandOpen()
    def rhandOpen(self):
        self.if0._ptr().rhandOpen()
        self.if1._ptr().rhandOpen()

    # void rhandClose()
    def rhandClose(self):
        self.if0._ptr().rhandClose()
        self.if1._ptr().rhandClose()

    # void lhandOpen()
    def lhandOpen(self):
        self.if0._ptr().lhandOpen()
        self.if1._ptr().lhandOpen()

    # void lhandClose()
    def lhandClose(self):
        self.if0._ptr().lhandClose()
        self.if1._ptr().lhandClose()

    def rhandAttach(self):
        self.if0._ptr().rhandAttach()
        self.if1._ptr().rhandAttach()

    def rhandDetach(self):
        self.if0._ptr().rhandDetach()
        self.if1._ptr().rhandDetach()

    def lhandAttach(self):
        self.if0._ptr().lhandAttach()
        self.if1._ptr().lhandAttach()

    def lhandDetach(self):
        self.if0._ptr().lhandDetach()
        self.if1._ptr().lhandDetach()

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


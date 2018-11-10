#!/usr/bin/env python

"""
 \file ManipulatorCommonInterface_Common_idl_examplefile.py
 \brief Python example implementations generated from ManipulatorCommonInterface_Common.idl
 \date $Date$


"""

import sys
import traceback
from subprocess import call
import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA
import RTC


class ManipulatorCommonInterface_Common_i (_GlobalIDL__POA.ManipulatorCommonInterface_Common):
    """
    \class ManipulatorCommonInterface_Common_i
    Example class implementing IDL interface ManipulatorCommonInterface_Common
    """

    def __init__(self):
        """
        \brief standard constructor
        Initialise member variables here
        """
        pass

    # RTC::RETURN_ID clearAlarms()
    def clearAlarms(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID getActiveAlarm(out RTC::AlarmSeq alarms)
    def getActiveAlarm(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, alarms

    # RTC::RETURN_ID getFeedbackPosJoint(out RTC::JointPos pos)
    def getFeedbackPosJoint(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, pos

    # RTC::RETURN_ID getManipInfo(out RTC::ManipInfo manipInfo)
    def getManipInfo(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, manipInfo

    # RTC::RETURN_ID getSoftLimitJoint(out RTC::LimitSeq softLimit)
    def getSoftLimitJoint(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, softLimit

    # RTC::RETURN_ID getState(out RTC::ULONG state)
    def getState(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, state

    # RTC::RETURN_ID servoOFF()
    def servoOFF(self):
        print "servoOFF"
        try:
            ret = call("manip serv_off", shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID servoON()
    def servoON(self):
        print "servoON"
        try:
            ret = call("manip serv_on", shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID setSoftLimitJoint(in RTC::LimitSeq softLimit)
    def setSoftLimitJoint(self, softLimit):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = ManipulatorCommonInterface_Common_i()

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


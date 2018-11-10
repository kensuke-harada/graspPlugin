#!/usr/bin/env python

"""
 \file ManipulatorCommonInterface_MiddleLevel_idl_examplefile.py
 \brief Python example implementations generated from ManipulatorCommonInterface_MiddleLevel.idl
 \date $Date$


"""

import sys
import traceback
from subprocess import call
from math import degrees
import CORBA, PortableServer
import _GlobalIDL, _GlobalIDL__POA
import RTC

class ManipulatorCommonInterface_Middle_i (_GlobalIDL__POA.ManipulatorCommonInterface_Middle):
    """
    \class ManipulatorCommonInterface_Middle_i
    Example class implementing IDL interface ManipulatorCommonInterface_Middle
    """

    def __init__(self):
        """
        \brief standard constructor
        Initialise member variables here
        """
        pass

    # RTC::RETURN_ID closeGripper()
    def closeGripper(self):
        print "closeGripper"
        try:
            ret = call("manip grasp", shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID getBaseOffset(out RTC::HgMatrix offset)
    def getBaseOffset(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, offset

    # RTC::RETURN_ID getFeedbackPosCartesian(out RTC::CarPosWithElbow pos)
    def getFeedbackPosCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, pos

    # RTC::RETURN_ID getMaxSpeedCartesian(out RTC::CartesianSpeed speed)
    def getMaxSpeedCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, speed

    # RTC::RETURN_ID getMaxSpeedJoint(out RTC::DoubleSeq speed)
    def getMaxSpeedJoint(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, speed

    # RTC::RETURN_ID getMinAccelTimeCartesian(out double aclTime)
    def getMinAccelTimeCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, aclTime

    # RTC::RETURN_ID getMinAccelTimeJoint(out double aclTime)
    def getMinAccelTimeJoint(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, aclTime

    # RTC::RETURN_ID getSoftLimitCartesian(out RTC::LimitValue xLimit, out RTC::LimitValue yLimit, out RTC::LimitValue zLimit)
    def getSoftLimitCartesian(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result, xLimit, yLimit, zLimit

    # RTC::RETURN_ID moveGripper(in RTC::ULONG angleRatio)
    def moveGripper(self, angleRatio):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID moveLinearCartesianAbs(in RTC::CarPosWithElbow carPoint)
    def moveLinearCartesianAbs(self, carPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID moveLinearCartesianRel(in RTC::CarPosWithElbow carPoint)
    def moveLinearCartesianRel(self, carPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID movePTPCartesianAbs(in RTC::CarPosWithElbow carPoint)
    def movePTPCartesianAbs(self, carPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID movePTPCartesianRel(in RTC::CarPosWithElbow carPoint)
    def movePTPCartesianRel(self, carPoint):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID movePTPJointAbs(in RTC::JointPos jointPoints)
    def movePTPJointAbs(self, jointPoints):
        print "movePTPJointAbs"
        try:
            degPoints = map( (lambda v: degrees(v)), jointPoints )
            arg = " ".join(map((lambda v: str(v)), degPoints))
            print arg
            ret = call("manip abs_jmove " + arg, shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID movePTPJointRel(in RTC::JointPos jointPoints)
    def movePTPJointRel(self, jointPoints):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID openGripper()
    def openGripper(self):
        print "openGripper"
        try:
            ret = call("manip hand_open", shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID pause()
    def pause(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID resume()
    def resume(self):
        try:
            ret = call("echo resume", shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID stop()
    def stop(self):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setAccelTimeCartesian(in double aclTime)
    def setAccelTimeCartesian(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setAccelTimeJoint(in double aclTime)
    def setAccelTimeJoint(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setBaseOffset(in RTC::HgMatrix offset)
    def setBaseOffset(self, offset):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setControlPointOffset(in RTC::HgMatrix offset)
    def setControlPointOffset(self, offset):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setMaxSpeedCartesian(in RTC::CartesianSpeed speed)
    def setMaxSpeedCartesian(self, speed):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setMaxSpeedJoint(in RTC::DoubleSeq speed)
    def setMaxSpeedJoint(self, speed):
        print "setMaxSpeedJoint: ", speed
        try:
            arg = " ".join(map((lambda v: str(v)), speed))
            ret = call("manip set_max_ang_vel %s" % arg , shell=True)
            if ret < 0:
                return RTC.RETURN_ID(ret, "NG")
            return RTC.RETURN_ID(ret, "OK")
        except:
            print sys.exc_info()[0]
            print sys.exc_info()[1]
            print traceback.print_tb(sys.exc_info()[2])
            return RTC.RETURN_ID(-1, "NG")

    # RTC::RETURN_ID setMinAccelTimeCartesian(in double aclTime)
    def setMinAccelTimeCartesian(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setMinAccelTimeJoint(in double aclTime)
    def setMinAccelTimeJoint(self, aclTime):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setSoftLimitCartesian(in RTC::LimitValue xLimit, in RTC::LimitValue yLimit, in RTC::LimitValue zLimit)
    def setSoftLimitCartesian(self, xLimit, yLimit, zLimit):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setSpeedCartesian(in RTC::ULONG spdRatio)
    def setSpeedCartesian(self, spdRatio):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

    # RTC::RETURN_ID setSpeedJoint(in RTC::ULONG spdRatio)
    def setSpeedJoint(self, spdRatio):
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
    servant = ManipulatorCommonInterface_Middle_i()

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


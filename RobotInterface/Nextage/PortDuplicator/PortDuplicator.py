#!/usr/bin/env python
# -*- Python -*-

"""
 \file PortDuplicator.py
 \brief Duplicate service port
 \date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import OpenRTM_aist
import RTC

# Import Service implementation class
# <rtc-template block="service_impl">
from HiroNX_idl_example import *
from HIROController_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import _GlobalIDL, _GlobalIDL__POA

# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
portduplicator_spec = ["implementation_id", "PortDuplicator", 
		 "type_name",         "PortDuplicator", 
		 "description",       "Duplicate service port", 
		 "version",           "1.0.0", 
		 "vendor",            "AIST", 
		 "category",          "Category", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "0", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

class PortDuplicator(OpenRTM_aist.DataFlowComponentBase):
	
	"""
	\class PortDuplicator
	\brief Duplicate service port
	
	"""
	def __init__(self, manager):
		"""
		\brief constructor
		\param manager Maneger Object
		"""
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)


		"""
		"""
		self._HiroNXPort = OpenRTM_aist.CorbaPort("HiroNX")
		"""
		"""
		self._HIROPort = OpenRTM_aist.CorbaPort("HIRO")
		"""
		"""
		self._HiroNX0Port = OpenRTM_aist.CorbaPort("HiroNX0")
		"""
		"""
		self._HIRO0Port = OpenRTM_aist.CorbaPort("HIRO0")
		"""
		"""
		self._HiroNX1Port = OpenRTM_aist.CorbaPort("HiroNX1")
		"""
		"""
		self._HIRO1Port = OpenRTM_aist.CorbaPort("HIRO1")

		"""
		"""
		self._manipulator = HiroNX_i()
		"""
		"""
		self._common = CommonCommands_i()
		"""
		"""
		self._motion = MotionCommands_i()
		

		"""
		"""
		self._HiroNX0 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.HiroNX)
		"""
		"""
		self._common0 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.CommonCommands)
		"""
		"""
		self._motion0 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.MotionCommands)
		"""
		"""
		self._HiroNX1 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.HiroNX)
		"""
		"""
		self._common1 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.CommonCommands)
		"""
		"""
		self._motion1 = OpenRTM_aist.CorbaConsumer(interfaceType=_GlobalIDL.MotionCommands)

		self._manipulator.setInterface(self._HiroNX0, self._HiroNX1)
                self._common.setInterface(self._common0, self._common1)
                self._motion.setInterface(self._motion0, self._motion1)
		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>


		 
	def onInitialize(self):
		"""
		
		The initialize action (on CREATED->ALIVE transition)
		formaer rtc_init_entry() 
		
		\return RTC::ReturnCode_t
		
		"""
		# Bind variables and configuration variable
		
		# Set InPort buffers
		
		# Set OutPort buffers
		
		# Set service provider to Ports
		self._HiroNXPort.registerProvider("manipulator", "HiroNX", self._manipulator)
		self._HIROPort.registerProvider("common", "CommonCommands", self._common)
		self._HIROPort.registerProvider("motion", "MotionCommands", self._motion)
		
		# Set service consumers to Ports
		self._HiroNX0Port.registerConsumer("manipulator", "HiroNX", self._HiroNX0)
		self._HIRO0Port.registerConsumer("common", "CommonCommands", self._common0)
		self._HIRO0Port.registerConsumer("motion", "MotionCommands", self._motion0)
		self._HiroNX1Port.registerConsumer("manipulator", "HiroNX", self._HiroNX1)
		self._HIRO1Port.registerConsumer("common", "CommonCommands", self._common1)
		self._HIRO1Port.registerConsumer("motion", "MotionCommands", self._motion1)
		
		# Set CORBA Service Ports
		self.addPort(self._HiroNXPort)
		self.addPort(self._HIROPort)
		self.addPort(self._HiroNX0Port)
		self.addPort(self._HIRO0Port)
		self.addPort(self._HiroNX1Port)
		self.addPort(self._HIRO1Port)
		
		return RTC.RTC_OK
	
	#def onFinalize(self, ec_id):
	#	"""
	#
	#	The finalize action (on ALIVE->END transition)
	#	formaer rtc_exiting_entry()
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onStartup(self, ec_id):
	#	"""
	#
	#	The startup action when ExecutionContext startup
	#	former rtc_starting_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onShutdown(self, ec_id):
	#	"""
	#
	#	The shutdown action when ExecutionContext stop
	#	former rtc_stopping_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onActivated(self, ec_id):
	#	"""
	#
	#	The activated action (Active state entry action)
	#	former rtc_active_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onDeactivated(self, ec_id):
	#	"""
	#
	#	The deactivated action (Active state exit action)
	#	former rtc_active_exit()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onExecute(self, ec_id):
	#	"""
	#
	#	The execution action that is invoked periodically
	#	former rtc_active_do()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onAborting(self, ec_id):
	#	"""
	#
	#	The aborting action when main logic error occurred.
	#	former rtc_aborting_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onError(self, ec_id):
	#	"""
	#
	#	The error action in ERROR state
	#	former rtc_error_do()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onReset(self, ec_id):
	#	"""
	#
	#	The reset action that is invoked resetting
	#	This is same but different the former rtc_init_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onStateUpdate(self, ec_id):
	#	"""
	#
	#	The state update action that is invoked after onExecute() action
	#	no corresponding operation exists in OpenRTm-aist-0.2.0
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	
	#def onRateChanged(self, ec_id):
	#	"""
	#
	#	The action that is invoked when execution context's rate is changed
	#	no corresponding operation exists in OpenRTm-aist-0.2.0
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK
	



def PortDuplicatorInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=portduplicator_spec)
    manager.registerFactory(profile,
                            PortDuplicator,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    PortDuplicatorInit(manager)

    # Create a component
    comp = manager.createComponent("PortDuplicator")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()


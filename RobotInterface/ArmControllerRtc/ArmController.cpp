// -*- C++ -*-
/*!
 * @file  ArmController.cpp
 * @brief The controller of the single/dual arm from Choreonoid
 * @date $Date$
 *
 * $Id$
 */

#include "ArmController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* armcontroller_spec[] =
  {
    "implementation_id", "ArmController",
    "type_name",         "ArmController",
    "description",       "The controller of the single/dual arm from Choreonoid",
    "version",           "1.1.0",
    "vendor",            "AIST",
    "category",          "VMRG",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
ArmController::ArmController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_HiroNXPort("HiroNX"),
    m_HIROPort("HIRO"),
    m_ManipulatorPort("Manipulator")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
ArmController::~ArmController()
{
}



RTC::ReturnCode_t ArmController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  m_HiroNXPort.registerConsumer("manipulator", "HiroNX", m_manipulator);
  m_HIROPort.registerConsumer("common", "CommonCommands", m_common);
  m_HIROPort.registerConsumer("motion", "MotionCommands", m_motion);
  m_ManipulatorPort.registerConsumer("manipulator_common", "ManipulatorCommonInterface_Common", m_manipulator_common);
  m_ManipulatorPort.registerConsumer("manipulator_motion", "ManipulatorCommonInterface_Middle", m_manipulator_motion);
  
  // Set CORBA Service Ports
  addPort(m_HiroNXPort);
  addPort(m_HIROPort);
  addPort(m_ManipulatorPort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ArmController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ArmController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void ArmControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(armcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<ArmController>,
                             RTC::Delete<ArmController>);
  }
  
};



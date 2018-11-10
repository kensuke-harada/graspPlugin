// -*- C++ -*-
/*!
 * @file  GraspConsumer.cpp
 * @brief Grasp Consumer
 * @date $Date$
 *
 * $Id$
 */

#include "GraspConsumer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* graspconsumer_spec[] =
  {
    "implementation_id", "GraspConsumer",
    "type_name",         "GraspConsumer",
    "description",       "Grasp Consumer",
    "version",           "1.1.0",
    "vendor",            "AIST",
    "category",          "TestInterfade",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
GraspConsumer::GraspConsumer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_PlannStartPortPort("PlannStartPort"),
    m_ResultPortPort("ResultPort")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
GraspConsumer::~GraspConsumer()
{
}



RTC::ReturnCode_t GraspConsumer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_ResultPortPort.registerProvider("Result", "GraspPlanResult", m_Result);
  
  // Set service consumers to Ports
  m_PlannStartPortPort.registerConsumer("PlanStart", "GraspPlanStart", m_PlanStart);
  
  // Set CORBA Service Ports
  addPort(m_PlannStartPortPort);
  addPort(m_ResultPortPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspConsumer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t GraspConsumer::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspConsumer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspConsumer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void GraspConsumerInit(RTC::Manager* manager)
  {
    coil::Properties profile(graspconsumer_spec);
    manager->registerFactory(profile,
                             RTC::Create<GraspConsumer>,
                             RTC::Delete<GraspConsumer>);
  }
  
};



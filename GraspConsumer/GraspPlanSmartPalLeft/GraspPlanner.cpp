// -*- C++ -*-
/*!
 * @file  GraspPlanner.cpp
 * @brief Grasp Planner
 * @date $Date$
 *
 * $Id$
 */

#include "GraspPlanner.h"
#include "VectorConvert.h"

// Module specification
// <rtc-template block="module_spec">
static const char* graspplanner_spec[] =
  {
    "implementation_id", "GraspPlanner",
    "type_name",         "GraspPlanner",
    "description",       "Grasp Planner",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Planner",
    "activity_type",     "EVENTDRIVEN",
    "kind",              "DataFlowComponent",
    "max_instance",      "0",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.Position", "0,0,-0.136",
    "conf.default.Posture", "1,0,0,0,1,0,0,0,1",
    // Widget
    "conf.__widget__.Position", "text",
    "conf.__widget__.Posture", "slider",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
GraspPlanner::GraspPlanner(RTC::Manager* manager)
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
GraspPlanner::~GraspPlanner()
{
}



RTC::ReturnCode_t GraspPlanner::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_PlannStartPortPort.registerProvider("PlanStart", "GraspPlanStart", m_PlanStart);
  
  // Set service consumers to Ports
  m_ResultPortPort.registerConsumer("Result", "GraspPlanResult", m_Result);
  
  // Set CORBA Service Ports
  addPort(m_PlannStartPortPort);
  addPort(m_ResultPortPort);


	
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("Position", m_Position, "0,0,-0.136");
  bindParameter("Posture", m_Posture, "1,0,0,0,1,0,0,0,1");

   m_PlanStart.GraspPlanConsumer(&m_Result,&m_Position[0], &m_Posture[0]);


  // </rtc-template>
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspPlanner::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPlanner::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void GraspPlannerInit(RTC::Manager* manager)
  {
    coil::Properties profile(graspplanner_spec);
    manager->registerFactory(profile,
                             RTC::Create<GraspPlanner>,
                             RTC::Delete<GraspPlanner>);
  }
  
};



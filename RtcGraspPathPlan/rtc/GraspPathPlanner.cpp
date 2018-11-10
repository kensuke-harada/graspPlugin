// -*- C++ -*-
/*!
 * @file  GraspPathPlanner.cpp
 * @brief grasp path planner
 * @date $Date$
 *
 * $Id$
 */

#include "GraspPathPlanner.h"

// Module specification
// <rtc-template block="module_spec">
static const char* grasppathplanner_spec[] =
  {
    "implementation_id", "GraspPathPlanner",
    "type_name",         "GraspPathPlanner",
    "description",       "grasp path planner",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "planner",
    "activity_type",     "EVENTDRIVEN",
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
GraspPathPlanner::GraspPathPlanner(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_planGraspPathPort("planGraspPath")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
GraspPathPlanner::~GraspPathPlanner()
{
}



RTC::ReturnCode_t GraspPathPlanner::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_planGraspPathPort.registerProvider("planGraspPath", "planGraspPath", m_planGraspPath);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_planGraspPathPort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspPathPlanner::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanner::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void GraspPathPlannerInit(RTC::Manager* manager)
  {
    coil::Properties profile(grasppathplanner_spec);
    manager->registerFactory(profile,
                             RTC::Create<GraspPathPlanner>,
                             RTC::Delete<GraspPathPlanner>);
  }
  
};



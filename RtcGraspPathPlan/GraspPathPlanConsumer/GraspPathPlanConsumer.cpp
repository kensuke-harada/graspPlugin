// -*- C++ -*-
/*!
 * @file  GraspPathPlanConsumer.cpp
 * @brief consumer for GraspPathPlanner
 * @date $Date$
 *
 * $Id$
 */

#include "GraspPathPlanConsumer.h"

using namespace std;

// Module specification
// <rtc-template block="module_spec">
static const char* grasppathplanconsumer_spec[] =
  {
    "implementation_id", "GraspPathPlanConsumer",
    "type_name",         "GraspPathPlanConsumer",
    "description",       "consumer for GraspPathPlanner",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Planne",
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
GraspPathPlanConsumer::GraspPathPlanConsumer(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_GraspPathPlanPort("GraspPathPlan")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
GraspPathPlanConsumer::~GraspPathPlanConsumer()
{
}



RTC::ReturnCode_t GraspPathPlanConsumer::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_GraspPathPlanPort.registerConsumer("planGraspPath", "planGraspPath", m_planGraspPath);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_GraspPathPlanPort);
  
  // </rtc-template>

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t GraspPathPlanConsumer::onExecute(RTC::UniqueId ec_id)
{
  int j;
  std::cout << "input number" << std::endl;
  std::cin >> j;

  //RTC::CorbaConsumer<planGraspPath> c_Result;

  planGraspPath::ObjectInputMode mode2;
  planGraspPath::ObjectInfo obj;

//  mode2 = planGraspPath::APPEAR;
//  mode2 = planGraspPath::REMOVE_RECORD;
  mode2 = planGraspPath::CREATE_RECORD;

  obj.objId = 1;
  obj.tagId="testCan";
  obj.pos.length(3);
  obj.pos[0] = 0.5;
  obj.pos[1] = 0;
  obj.pos[2] = 0.7;

  obj.ori.length(9);
  for(int i=0; i<9; i++)
    obj.ori[i]=0.0;
  obj.ori[0]=obj.ori[4]=obj.ori[8]=1.0;

  mode2 = planGraspPath::CREATE_RECORD;
  m_planGraspPath->SetStatusObject(mode2, obj);
  mode2 = planGraspPath::APPEAR;
  m_planGraspPath->SetStatusObject(mode2, obj);

  //planGraspPath_var *gp;
  //gp->SetStatusObject(mode2, obj);


  planGraspPath::ULONG  mode;
  planGraspPath::DblSeq begin;
  planGraspPath::DblSeq end;
  char* robotId;
  char* objectTagId;
  ::CORBA::Double resolution;
  planGraspPath::ManipInfoSeq_var trajectory;
  planGraspPath::ULONG state;
  
  mode = 0;
  robotId = "HRP2";
  objectTagId="glass_case";
  resolution = 0;
  
  begin.length(30);
  end.length(30);
    
  double r2d = 3.1415/180.0;
    
  begin[0] = 0;
  begin[1] = 0;

  begin[2] = 0;
  begin[3] = -10.0*r2d;
  begin[4] = 0;
  begin[5] = 0;
  begin[6] = 0;
  begin[7] = 0;
  begin[8] = 0;
  begin[9] = 0;

  begin[10] = 0;
  begin[11] = 10.0*r2d;
  begin[12] = 0;
  begin[13] = 0;
  begin[14] = 0;
  begin[15] = 0;
  begin[16] = 0;
  begin[17] = 0;              
  
  
  end[0] = 0;
  end[1] = 0;
                 
  end[2] = 0;
  end[3] = -10.0*r2d;
  end[4] = 0;
  end[5] = 0;
  end[6] = 0;
  end[7] = 0;
  end[8] = 0;
  end[9] = 0;
                 
  end[10] = 0;
  end[11] = 10.0*r2d;
  end[12] = 0;
  end[13] = 0;
  end[14] = 0;
  end[15] = 0;
  end[16] = 0; 
  end[17] = 0;               

  m_planGraspPath->GraspPlanningStart(mode,begin,end,robotId,objectTagId,resolution,trajectory,state);

       cout << trajectory->length() << endl;
  mode2 = planGraspPath::SETPOS;
  m_planGraspPath->SetStatusObject(mode2, obj);

  m_planGraspPath->ReleasePlanningStart(mode,begin,end,robotId,objectTagId,resolution,trajectory,state);


  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t GraspPathPlanConsumer::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void GraspPathPlanConsumerInit(RTC::Manager* manager)
  {
    coil::Properties profile(grasppathplanconsumer_spec);
    manager->registerFactory(profile,
                             RTC::Create<GraspPathPlanConsumer>,
                             RTC::Delete<GraspPathPlanConsumer>);
  }
  
};



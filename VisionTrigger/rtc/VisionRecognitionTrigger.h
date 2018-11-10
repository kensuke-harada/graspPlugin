// -*- C++ -*-
/*!
 * @file  VisionRecognitionTrigger.h * @brief Trigger recognition module * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef VisionRECOGNITIONTRIGGER_H
#define VisionRECOGNITIONTRIGGER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "RecognitionServiceExtSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "RecognitionServiceStub.h"

// </rtc-template>

using namespace RTC;

#if 0
class VisionRecognitionTrigger;
class cbInPortRecog : public RTC::OnWrite<TimedRecognitionResult> {
  VisionRecognitionTrigger& parent;
public:
  cbInPortRecog(VisionRecognitionTrigger& rt) : parent(rt) {}
  void operator()(const TimedRecognitionResult& result);
};
#endif

namespace grasp{
	class VisionTriggerRtcController;
}

class VisionRecognitionTrigger  : public RTC::DataFlowComponentBase
{
	friend class grasp::VisionTriggerRtcController;
 public:
  VisionRecognitionTrigger(RTC::Manager* manager);
  ~VisionRecognitionTrigger();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
  
  RTC::CorbaConsumer<RecognitionService>* m_recogTrigger_() { return &m_recogTrigger; };
  std::vector <double> results; 

	bool ext_mode_flag;

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedRecognitionResult m_recogResult;
  InPort<TimedRecognitionResult> m_recogResultIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_recogTriggerPort;
  /*!
   */
  RTC::CorbaPort m_recogTriggerExtPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  RecognitionResultServiceExtSVC_impl m_recogResultExt;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  RTC::CorbaConsumer<RecognitionService> m_recogTrigger;
  /*!
   */
  RTC::CorbaConsumer<RecognitionServiceExt> m_recogTriggerExt;
  
  // </rtc-template>

  // </rtc-template>

  //cbInPortRecog m_cbRecog;
  
 private:

};


extern "C"
{
  void VisionRecognitionTriggerInit(RTC::Manager* manager);
};

#endif // VisionRECOGNITIONTRIGGER_H


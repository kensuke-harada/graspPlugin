// -*- C++ -*-
/*!
 * @file  VisionRecognitionTrigger.cpp * @brief Trigger recognition module * $Date$ 
 *
 * $Id$ 
 */

#include <cstdio>

#include "VisionRecognitionTrigger.h"

#include "../ObjectRecognitionResultManipulator.h"

// Module specification
// <rtc-template block="module_spec">
static const char* vvvrecognitiontrigger_spec[] =
  {
    "implementation_id", "VisionRecognitionTrigger",
    "type_name",         "VisionRecognitionTrigger",
    "description",       "Trigger recognition module",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

VisionRecognitionTrigger::VisionRecognitionTrigger(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_recogResultIn("recogResult", m_recogResult),
    m_recogTriggerPort("recogPort"),
	m_recogTriggerExtPort("recogTriggerExt"),
	ext_mode_flag(false)
    //m_cbRecog(*this)

    // </rtc-template>
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  //m_recogResultIn.setOnWrite(&m_cbRecog);
  registerInPort("recogResult", m_recogResultIn);

  // Set OutPort buffer

  // Set service provider to Ports
  m_recogTriggerExtPort.registerProvider("recogResultExt", "RecognitionResultServiceExt", m_recogResultExt);

  // Set service consumers to Ports
  m_recogTriggerPort.registerConsumer("recogPort", "RecognitionService", m_recogTrigger);
  m_recogTriggerExtPort.registerConsumer("recogTriggerExt", "RecognitionServiceExt", m_recogTriggerExt);

  // Set CORBA Service Ports
  registerPort(m_recogTriggerPort);
  registerPort(m_recogTriggerExtPort);
  // </rtc-template>

}

VisionRecognitionTrigger::~VisionRecognitionTrigger()
{
}


RTC::ReturnCode_t VisionRecognitionTrigger::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t VisionRecognitionTrigger::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VisionRecognitionTrigger::onExecute(RTC::UniqueId ec_id)
{
/*
  if (m_recogResultIn.isNew()) {
    m_recogResultIn.read();
    printf("length: %ld\n", m_recogResult.data.length());
	  
	  results.clear();
	  for(unsigned int i=0; i<  m_recogResult.data.length(); i++){
		  results.push_back(m_recogResult.data[i]);
	  }

    for (size_t i = 0; i < m_recogResult.data.length()/4; ++i) {
      for (size_t j = 0; 4*i+j < m_recogResult.data.length() && j < 4; ++j) {
	printf("% 7.4f ", m_recogResult.data[4*i+j]);
      }
      printf("\n");

      if (i % 5 == 4) {
	printf("\n");
      }
    }
    printf("\n");
  }
*/
  /*
  if (::CORBA::is_nil(m_recogTrigger.getObject()) == false) {
    ::CORBA::Long id;

    printf("input object ID: ");
    if (scanf("%ld", &id) < 1) {
      return RTC::RTC_ERROR;
    }
    
    if (::CORBA::is_nil(m_recogTrigger.getObject()) == false) {
      m_recogTrigger->setModelID(id);
    }
  }
*/

	if (ext_mode_flag) {
		grasp::ObjectRecognitionResultManipulator* manip = grasp::ObjectRecognitionResultManipulator::instance();
		if (manip->getStartFlag()) {
			m_recogTrigger->setModelID(manip->getObjectID());

			std::vector<double> param_vec;
			manip->getRecogParamsDblVec(param_vec);

			RecognitionServiceExt::DblSeq param_seq;
			param_seq.length(param_vec.size());
			for (size_t i = 0; i < param_vec.size(); i++) {
				param_seq[i] = param_vec[i];
			}
			m_recogTriggerExt->setParameters(param_seq);

			m_recogTriggerExt->startObjectRecognition();

			manip->changeStartFlag(false);
		} else {
			if (m_recogResultIn.isNew()) {
				m_recogResultIn.read();
				int len = m_recogResult.data.length();
				double res[len];
				for (int i = 0; i < len; i++) {
					res[i] = static_cast<double>(m_recogResult.data[i]);
				}
				manip->pushResult(res, len);
			}
		}
	}

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VisionRecognitionTrigger::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t VisionRecognitionTrigger::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void VisionRecognitionTriggerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(vvvrecognitiontrigger_spec);
    manager->registerFactory(profile,
                             RTC::Create<VisionRecognitionTrigger>,
                             RTC::Delete<VisionRecognitionTrigger>);
  }
  
};


#if 0
void cbInPortRecog::operator()(const TimedRecognitionResult& result)
{
  printf("length: %ld\n", result.data.length());

  for (size_t i = 0; i < result.data.length()/4; ++i) {
    for (size_t j = 0; 4*i+j < result.data.length() && j < 4; ++j) {
      printf("% 7.4f ", result.data[4*i+j]);
    }
    printf("\n");

    if (i % 5 == 4) {
      printf("\n");
    }
  }
  printf("\n");
}
#endif

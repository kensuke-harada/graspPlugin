/**
   c) Tokuo Tsuji (Kyushu univ./AIST)
*/

#ifndef EXCADE_ROBOTICS_RTC_CONTROLLER_H_INCLUDED
#define EXCADE_ROBOTICS_RTC_CONTROLLER_H_INCLUDED

#include "rtc/VisionRecognitionTrigger.h"
#include "../Grasp/GraspController.h"


namespace grasp {


class VisionTriggerRtcController{
	public:
//		VisionTriggerRtcController(){}
//		~VisionTriggerRtcController(){}
		static VisionTriggerRtcController* instance();
		static int RtcStart ();
		VisionRecognitionTrigger* comp_;
		bool VisionRecoginitionStart();

		bool switchExtModeFlag();


		static void VisionTriggerMyModuleInit(RTC::Manager* manager);

		bool stopFlag;

	private:
		cnoid::Vector3 basePos;
		cnoid::Matrix3 baseOri;


};


}


#endif

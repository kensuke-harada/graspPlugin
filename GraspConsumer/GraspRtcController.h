#ifndef EXCADE_ROBOTICS_RTC_CONTROLLER_H_INCLUDED
#define EXCADE_ROBOTICS_RTC_CONTROLLER_H_INCLUDED

#include "rtc/GraspConsumer.h"
#include "../Grasp/GraspController.h"

namespace grasp {


class GraspRtcController{
	public:
//		GraspRtcController()   { }
//		~GraspRtcController(){}
		static GraspRtcController* instance();
		int RtcStart ();
		static void MyModuleInit(RTC::Manager* manager);

		GraspConsumer* comp_;
	
		bool graspPlanStart(int mode);
		bool graspPlanResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, 
												 const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri,
												 const GraspPlanResult::DblSequence& angle);
	
	
	
	private:
		cnoid::Vector3 basePos;
		cnoid::Matrix3 baseOri;
					
};


}

#endif

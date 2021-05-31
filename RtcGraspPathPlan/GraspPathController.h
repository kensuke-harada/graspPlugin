#ifndef _GRASPPATH_RTC_CONTROLLER_H_INCLUDED
#define _GRASPPATH_RTC_CONTROLLER_H_INCLUDED

#include "GraspPathPlanner.h"
#include "../Grasp/PlanBase.h"
#include "GraspPathControllerSVC_impl.h"

namespace grasp {

class pathInfo{
	public:
	std::vector<double> pos;
	int state;
	bool leftArm;
	bool rightArm;
	bool leftGripper;
	bool rightGripper;
	bool waist;
};

class GraspPathController{
	public:
		GraspPathController();
//		~GraspPathController(){}
		static GraspPathController* instance();
		int RtcStart ();
		static void MyModuleInit(RTC::Manager* manager);

		GraspPathPlanner* comp_;

		bool graspPathPlanStart(
			int mode, std::vector<double> begin, std::vector<double> end,
			std::string robotId, std::string objectTagId, double resolution,
			std::vector<pathInfo>* trajectory, int* state);

		bool releasePathPlanStart(
			int mode, std::vector<double> begin, std::vector<double> end,
			std::string robotId, std::string objectTagId, double resolution,
			std::vector<pathInfo>* trajectory, int* state);
//		bool graspPathPlanResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri,
//				const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle);
		bool selectReleasePattern(cnoid::Vector3 objVisPos, cnoid::Matrix3 objVisRot);

		bool createRecord(int objId, std::string tagId);
		bool deleteRecord(std::string tagId);
		bool appear(std::string tagId);
		bool disappear(std::string tagId);
		bool setPos(std::string tagId, cnoid::Vector3 pos, cnoid::Matrix3 ori);
		bool setTolerance(double setTolerance);

		std::string objectBasePath;
		std::map <int, std::string> objId2File;
		std::map <int, std::string> objId2PrePlan;
		std::map <std::string, std::string> objTag2PrePlan;

		std::map <std::string,cnoid::BodyItemPtr>& objTag2Item(){
			return PlanBase::instance()->objTag2Item;
		}
		std::map <std::string,ArmFingers*>& robTag2Arm(){
			return PlanBase::instance()->robTag2Arm;
		}

	private:
		std::ostream& os;
//		cnoid::Vector3 basePos;
//		cnoid::Matrix3 baseOri;

};


}

#endif

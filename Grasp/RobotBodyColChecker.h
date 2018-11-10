#ifndef _GRASP_ROBOTBODYCOLCHECKER_H_
#define _GRASP_ROBOTBODYCOLCHECKER_H_

#include <vector>
#include <string>

#include <cnoid/Body>

#include "ColdetLinkPair.h"
#include "InterObject.h"
#include "RobotBody.h"
#include "PointCloudEnv.h"
#include "InterLink.h"

#include "exportdef.h"

namespace grasp {
	class TargetObject;
	class MultiHandObjectColChecker;
	class ObjectManager;
	class EXCADE_API RobotBodyColChecker {
	public:
		RobotBodyColChecker();
		virtual ~RobotBodyColChecker();

		RobotBodyPtr getRobotBody() const {return body_;}

		bool isColliding();
		double clearance();
		void calcForwardKinematics();

		void setInterLink();

		void setGraspingState(int state);
		void setGraspingState2(int state);
		void setGraspingState(int i, int state);
		int checkGraspingState(int arm_id) const;
		void setObjectContactState(int state) {objectContactState = state;}
		int getObjectContactState() const {return objectContactState;}
		std::list<cnoid::BodyItemPtr>& getBodyItemEnv() {return bodyItemEnv;}

		void SetEnvironment(cnoid::BodyItemPtr& bodyitem);
		void RemoveEnvironment(cnoid::BodyItemPtr& bodyitem);

		std::string colPairName[2];

		TargetObject* targetObject;
		cnoid::BodyPtr obj_body;
		cnoid::Link* object;
		ArmFingers* targetArmFinger;

		int objectContactState;

		bool useRobotSafeBoundingBox;
		bool doCheckCollisionPointCloudFinger;
		double tolerance;

	protected:
		ColdetLinkPairVector robotSelfPairs, robotEnvPairs, robotObjPairs, objEnvPairs, safeRobotEnvPairs, safeRobotObjPairs;
		ColdetLinkPairVector robotObjPairWithoutHand;
		std::vector<ColdetLinkPairVector > handObjPair;
		std::vector<std::pair<cnoid::Link*, grasp::PointCloudEnv*> > robotExcludingFingPointCloudPairs, fingPointCloudPairs, objPointCloudPairs;
		std::vector<InterObject> interObjectList;
		std::list<cnoid::BodyItemPtr> bodyItemEnv;
		std::vector<grasp::InterLink> interLinkList;

	private:
		RobotBodyPtr body_;
		MultiHandObjectColChecker* hand_obj_col_checker_;
		ObjectManager* obj_manager_;

		bool isCollidingTest(const ColdetLinkPairVector& target_pairs, const std::string& debug_msg);
		bool isCollidingTestPointCloud(const std::vector<std::pair<cnoid::Link*, grasp::PointCloudEnv*> >& target_pairs, const std::string& debug_msg);

		void copyTestPairs();
		void copyInterObjectList();
		void copyInterLinkList();
		void copyTargetArmFinger();
	};
}

#endif  /* _GRASP_ROBOTBODYCOLCHECKER_H_ */

/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _TrajectoryPlanner_H
#define _TrajectoryPlanner_H

#include <iostream>

#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/BodyMotion>

#include <cnoid/EigenTypes>


#include "../Grasp/PlanBase.h"

#include "exportdef.h"

namespace grasp{


class EXCADE_API TrajectoryPlanner {

	public :

	TrajectoryPlanner(int id=0);
	virtual ~TrajectoryPlanner(){};
		
	virtual void setStartPos();
	virtual bool doTrajectoryPlanning();
	virtual bool simpleTrajectoryPlanning();
		
	virtual bool updateTrajectoryFromMotion(const cnoid::BodyMotionPtr motionObject, const cnoid::BodyMotionPtr motionRobot, std::vector<MotionState>& motionSeq);
	virtual bool outputTrajectoryForOpenHRP(const cnoid::BodyMotionPtr motionRobot);
	
	cnoid::PoseSeqItem* poseSeqItemRobot;
	cnoid::PoseSeqItem* poseSeqItemObject;
	const std::vector<cnoid::PoseSeqItemPtr>& poseSeqItemAccompanyings() const;

//	hrp::Link *palm;
//	hrp::Link *base;
	cnoid::VectorXd iniJoint, finJoint;
	std::vector<MotionState> motionSeq;
	bool useSafeBB;

	std::vector<bool> isTrajectorySectionSucceed;

	bool verbose;

 protected:
	bool updateTrajectoriesFromMotion(const std::vector<MotionState>& motionSeq, bool has_object);

	std::ostream& os;

	std::vector<cnoid::PoseSeqItemPtr> poseSeqItemAccompanyings_;
	std::vector<ObjectBase*> accompanying_objects_;

	cnoid::PosePtr createPose(ObjectBase* object, const MotionState& target_motion) const;
};


}


#endif

#ifndef _TASKBASE_ACTIONSEQUENCE_H_
#define _TASKBASE_ACTIONSEQUENCE_H_

#include <list>
#include <vector>

#include <cnoid/EigenTypes>
#include "../../../src/PoseSeqPlugin/PoseSeq.h"

#include "exportdef.h"

namespace grasp {

class ActionPose {
 public:
	cnoid::Vector3 p;
	cnoid::Matrix3 R;
};

class ActionPrehension {
 public:
	enum EPrehension {
		NONE,
		FREE,
		PREHENSION,
		FIXED,
		ENVIRONMENT,
	};

	EPrehension type;
	bool has_hand_frame;
	ActionPose hand_frame;
	std::vector<cnoid::Vector3> grasp_points;
};

class EXCADE_API ActionContent {
 public:
	ActionContent();
	virtual ~ActionContent();

	double& time();
	const double& time() const;
	ActionPose& ref_pose();
	const ActionPose& ref_pose() const;
	ActionPrehension& subPrehension();
	const ActionPrehension& subPrehension() const;
	cnoid::PoseSeq::iterator& refPoseIte();

 private:
	double time_;
	ActionPose ref_pose_;
	ActionPrehension sub_obj_prehension_;
	cnoid::PoseSeq::iterator ref_pose_ite_;
};

typedef std::list<ActionContent> ActionSequence;
typedef ActionSequence::iterator ActionSeqIte;
typedef ActionSequence::const_iterator ActionSeqConstIte;
typedef ActionSequence::reverse_iterator ActionSeqRIte;
typedef ActionSequence::const_reverse_iterator  ActionSeqConstRIte;

}

#endif /* _TASKBASE_ACTIONSEQUENCE_H_ */

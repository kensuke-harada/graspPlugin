#ifndef _TASKBASE_ROBOTMOTIONGENERATOR_H_
#define _TASKBASE_ROBOTMOTIONGENERATOR_H_

#include <iostream>
#include <vector>
#include <map>
#include <string>

#include <cnoid/MessageView>
#include <cnoid/BodyItem>

#include "Task.h"
#include "Action.h"

namespace grasp {

	class MotionState;

	class EXCADE_API RobotMotionGenerator {
	public:
		RobotMotionGenerator();
		virtual ~RobotMotionGenerator();

		bool plan(const TaskItemPtr& task) const;
		bool plan(const ActionItemPtr& action) const;

	protected:
		std::ostream& os;

		void registerAssemblyObjects(const TaskItemPtr& task) const;
		void registerAssemblyObjects(const ActionItemPtr& action) const;
		void moveObjectsInitial(const TaskItemPtr& task) const;
		void moveObjectsInitial(const ActionItemPtr& action) const;

		bool doTrajectoryAndRenamePoseSeqs(const std::string& name) const;
	};

	class EXCADE_API ActionGraspMotionConverter {
	public:
		ActionGraspMotionConverter();
		virtual ~ActionGraspMotionConverter();

		bool convert(const ActionItemPtr& action, std::vector<MotionState>& motion_seq) const;

		void setAppVec(const cnoid::Vector3& vec);

		// for develop
		/* bool debug(const TaskItemPtr& task); */

	protected:
		std::ostream& os;

		cnoid::Vector3 app_vec_;

		bool addAppPose(const cnoid::Vector3& grasp_p, const cnoid::Matrix3& grasp_R,
										std::vector<MotionState>& motion_seq) const;
	};

}

#endif /* _TASKBASE_ROBOTMOTIONGENERATOR_H_ */

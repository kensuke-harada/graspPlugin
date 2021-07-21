/**
 * @file   JointSeqDivider.h
 * @author Akira Ohchi
 */

#ifndef _PICKINGTASKPLANNER_JOINTSEQDIVIDER_H_
#define _PICKINGTASKPLANNER_JOINTSEQDIVIDER_H_

#include <vector>

#include <cnoid/EigenTypes>

namespace grasp {
	class JointSeqData;
	class MotionSeqData;

	class JointSeqDivider {
	public:
		JointSeqDivider();
		virtual ~JointSeqDivider();

		enum KeyPose {
			INITIAL_POSE = 0,
			APPROACH_POSE = 1,
			GRASPOPEN_POSE = 2,
			GRASPCLOSE_POSE = 3,
			LIFTUP_POSE = 4,
			PRERELEASE_POSE = 5,
			RELEASECLOSE_POSE = 6,
			RELEASEOPEN_POSE = 7,
			POSTRELEASE_POSE = 8,
			FINAL_POSE = 9
		};

		void setDividingKeyPose(KeyPose keypose);
		void clearDividingKeyPose();
		void divide();
		int size() const;
		void setJointSeq(int num);
		void revertJointSeq();
		void registerMotionSeq(int num);
		void setMotionSeq(int num);

	private:
		JointSeqData* original_data_;
		std::vector<JointSeqData*> divided_data_;
		std::vector<MotionSeqData*> motion_data_;
		std::vector<bool> is_divide_pose_;

		void clear();
	};
}

#endif /* _PICKINGTASKPLANNER_JOINTSEQDIVIDER_H_ */

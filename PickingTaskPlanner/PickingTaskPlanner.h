/**
 * @file   PickingTaskPlanner.h
 * @author Akira Ohchi
 */

#ifndef _PICKINGTASKPLANNER_PICKINGTASKPLANNER_H_
#define _PICKINGTASKPLANNER_PICKINGTASKPLANNER_H_

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <cnoid/EigenTypes>
#include <cnoid/BodyItem>

namespace cnoid {
class PoseSeqItem;
class PoseSeq;
}

class PoseEstimator;

namespace grasp {
	class BoxInfo;
	class ViewPose;
	typedef boost::shared_ptr<ViewPose> ViewPosePtr;
	class MotionState;
	class OccupiedVoxel;

	class PickingTaskPlanner {
	public:
		static PickingTaskPlanner* instance();

		virtual ~PickingTaskPlanner();

		bool startPicking(bool multi_pc, bool try_negative_sol = false);

		void initialize();

		void visionTriggerStart();
		bool visionTriggerStart(const cnoid::VectorXd& init_pose, bool do_merge, bool use_prev_result, int prev_grasped_cluster_id);

		static void mergePoseSeq(const std::vector<cnoid::PoseSeqItem*>& items, cnoid::PoseSeqItem* merged_item);

		static bool isSolInsideBox(const cnoid::Vector3& p, const cnoid::Matrix3& R);

	private:
		std::ostream& os;
		BoxInfo* target_box_;
		BoxInfo* collection_box_;

		cnoid::VectorXd init_q_; ///< joint angles at initial pose
		cnoid::Vector3 init_obj_p_;
		cnoid::Matrix3 init_obj_R_;
		cnoid::VectorXd view_q_; ///< joint angles at view pose
		bool has_sol_; ///< Is a feasible solution founded in previous picking operation?
		bool is_inserted_viewpose_; ///< Is a view pose is inserted in picking motion?
		bool use_prev_cluster_result_;
		bool is_try_nagative_sol_;
		int prev_grasped_cluster_id_;
		std::vector<MotionState> first_motion_; ///< If motion is divided, motion to viewpose is stored in this. Otherwise, whole motion is stored.
		std::vector<MotionState> second_motion_; ///< If motion is divided, motion from viewpose is stored.

		std::vector<OccupiedVoxel> occupied_voxel_grid;

		PickingTaskPlanner();

		bool startPickingProc(bool multi_pc);

		bool confirm() const;

		bool initProc(bool* is_first, bool* prev_succeed);

		bool planViewPoseAndGoBack(bool is_firstview, bool prev_succeed, int view_count);
		bool viewPlan(bool is_firstview, bool prev_succeed, int view_count);
		bool genTrajectory(const cnoid::VectorXd& from, const cnoid::VectorXd& to, std::vector<MotionState>& motion, cnoid::PoseSeqItem** item);

		bool poseEstimationAndPlanning(bool multi_pc, int view_count, bool* is_estimation_succeed);
		void capture(PoseEstimator* estimator, bool do_merge);
		void captureMultiPC(bool do_merge);
		bool poseEstimate(PoseEstimator* estimator);
		bool binPicking();
		bool binPickingMultiPC(bool* is_estimation_succeed);

		bool doMove(std::vector<MotionState> motion) const;

		void storePose(cnoid::VectorXd& q) const;
		void restorePose(const cnoid::VectorXd& q);


		void getObjPosPrevMotionSeq(cnoid::Vector3& p, cnoid::Matrix3& R) const;
		void getBodyItem(int n, cnoid::BodyItemPtr& item);

		bool insertViewPose();

		int readObjID(const cnoid::BodyPtr body);
	};
}

#endif /* _PICKINGTASKPLANNER_PICKINGTASKPLANNER_H_ */

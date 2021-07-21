/**
 * @file   PickingTaskPlanner.cpp
 * @author Akira Ohchi
 */

#include "PickingTaskPlanner.h"

#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>

#include <QMessageBox>

#include <cnoid/MessageView>
#include <cnoid/TimeBar>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>
#include <omniORB4/CORBA.h>

#include "../Grasp/PlanBase.h"
#include "../PickAndPlacePlanner/ManipController.h"
#include "../PRM/TrajectoryPlanner.h"
#include "../PCL/ObjectPoseEstimatorInterface.h"
#include "../RobotInterface/RobotInterface.h"
#include "../ViewPlanner/ViewPlanner.h"
#include "../ViewPlanner/Box.h"
#include "../Grasp/Camera.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"
#include "../VisionTrigger/ObjectRecognitionResultManipulator.h"
#include "../Grasp/OccupiedVoxel.h"

#include "JointSeqDivider.h"
#include "ConfirmDialog.h"
#include "Config.h"
#include "ResultDataManipulator.h"

#ifdef PICKINGTASKPLANNER_WRITELOG_CAPTUREJOINTANGLE
#include <fstream>
#endif

// for measuring the time
// #define NOOUTPUT_TIMELOG
#include "../GraspDataGen/Util/StopWatch.h"

using namespace cnoid;
using namespace grasp;
using grasp::PickAndPlacePlanner::ManipController;
using grasp::RobotInterface;

#ifdef PICKINGTASKPLANNER_WRITELOG_CAPTUREJOINTANGLE
namespace {
	void writeLogJointAngle(const cnoid::VectorXd& angle) {
		static int file_no = 0;
		std::string angle_file = "./capture_angle_" + boost::lexical_cast<std::string>(file_no) + ".txt";
		std::ofstream out(angle_file.c_str());
		for (size_t i = 0; i < angle.size(); i++) {
			out << angle[i] << " ";
		}
		out.close();
		file_no++;
	}
}
#endif

PickingTaskPlanner* PickingTaskPlanner::instance() {
	static PickingTaskPlanner* instance = new PickingTaskPlanner();
	return instance;
}

PickingTaskPlanner::PickingTaskPlanner() :
	os(cnoid::MessageView::mainInstance()->cout()),
	target_box_(NULL),
	collection_box_(NULL),
	has_sol_(false),
	is_inserted_viewpose_(false),
	use_prev_cluster_result_(false),
	is_try_nagative_sol_(false),
	prev_grasped_cluster_id_(-1) {
}

PickingTaskPlanner::~PickingTaskPlanner() {
}

/**
 * @brief start bin picking
 * @param[in] multi_pc true if vision recognition is executed by another PC
 */
bool PickingTaskPlanner::startPicking(bool multi_pc, bool try_negative_sol) {
	is_try_nagative_sol_ = try_negative_sol;
	if (BoxHolder::instance()->getTargetBox() == NULL) {
		os << "Target box is not selected!" << std::endl;
		return false;
	}

	ResultDataExporter::instance()->exportStart();

#ifndef NOOUTPUT_TIMELOG
	std::cout << "[time_log] ====start picking proc====" << std::endl;
#endif
	bool res = startPickingProc(multi_pc);
#ifndef NOOUTPUT_TIMELOG
	if (res) {
	std::cout << "[time_log] ===end picking proc(success)===" << std::endl;
	} else {
		std::cout << "[time_log] ===end picking proc(fail)===" << std::endl;
	}
#endif

	// set robot joints to initail pose if picking planning is failed
	if (!res) {
		has_sol_ = false;
		is_inserted_viewpose_ = false;
		use_prev_cluster_result_ = false;
		restorePose(init_q_);
	}

	ResultDataExporter::instance()->exportEnd(res);

	return  res;
}

/**
 * @brief start bin picking procedure
 * @param[in] multi_pc true if vision recognition is executed by another PC
 */
bool PickingTaskPlanner::startPickingProc(bool multi_pc) {
	ScopeStopWatch timer("picking_total");
	// do planning if planing is not done in previous picking operation
	if (!has_sol_) {
		// initial procedure
		bool is_first = true;
		bool is_prev_succeed = false;

		if (!initProc(&is_first, &is_prev_succeed)) return false;

		bool is_firstview = is_first;
		const int max_view_change = 3;
		int view_change = 0;

		while (!has_sol_) {
			view_change++;
			ResultDataExporter::instance()->setViewCount(view_change);
			if (view_change > max_view_change) {
				os << "the number of view change reaches a limit!" << std::endl;
				return false;
			}

			// search feasible view pose and generate path to view pose and back
			restorePose(init_q_);
			if (!planViewPoseAndGoBack(is_firstview, is_prev_succeed, view_change)) {
				return false;
			}

			timer.stop();
			if (!confirm()) {
				return false;
			}
			timer.start();

			is_firstview = false;
			// pose estiamtion, planning and do move
			bool is_estimation_succeed = false;
			has_sol_ = poseEstimationAndPlanning(multi_pc, view_change, &is_estimation_succeed);
			if ((!has_sol_) && is_estimation_succeed) {
				// planning is failed
				return false;
			} /*else {
					ViewPlanner::instance()->setCurrentPoseUnavailable();
					}*/
		}
	}

	timer.stop();
	if (!confirm()) {
		return false;
	}
	timer.start();

	has_sol_ = false;
	if (is_inserted_viewpose_) {
		// pose estiamtion, planning and do move
		ResultDataExporter::instance()->setInNextPlan();
		bool is_estimation_succeed = false;
		has_sol_ = poseEstimationAndPlanning(multi_pc, 1, &is_estimation_succeed);
	} else {
		doMove(first_motion_);
	}

	return true;
}

bool PickingTaskPlanner::confirm() const {
#ifdef PICKINGTASKPLANNER_DISPLAY_CONFIRM
	ConfirmDialog* confirm = new ConfirmDialog();
	confirm->show();
	while (true) {
		MessageView::instance()->flush();
		if (confirm->getState() == -1) {
			delete confirm;
			return false;
		}
		if (confirm->getState() == 1) {
			delete confirm;
			return true;
		}
	}
#endif
	return true;
}

/**
 * @brief initial procedure
 * @param[out] is_first true if this is the first time operation.
 * @param[out] prev_succed picking success or failure at previous operation
 */
bool PickingTaskPlanner::initProc(bool* is_first, bool* prev_succeed) {
	// If the target box is the same box at last picking, this is not in the first time.
	BoxInfo* tmp_box = BoxHolder::instance()->getTargetBox();
	*prev_succeed = false;
	*is_first = true;
	if (target_box_ == tmp_box) {
#ifdef PICKINGTASKPLANNER_DO_VIEWPLANNER_EACHCYCLE
		// ask last pikcing result
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(NULL, "Picking result", "Did the last picking succeed?",
																	QMessageBox::Yes|QMessageBox::No);
		*prev_succeed = (reply == QMessageBox::Yes);

		ViewPlanner::instance()->clearVisitedFlags();
		*is_first = false;
#endif
	} else {
		// do first procedure
		target_box_ = tmp_box;

		// store initial pose
		storePose(init_q_);

		init_obj_p_ = grasp::PlanBase::instance()->targetObject->object->p();
		init_obj_R_ = grasp::PlanBase::instance()->targetObject->object->R();

		// listup view candidates
		if (!ViewPlanner::instance()->viewPlanning(target_box_)) {
			os << "initial procedure failed!" << std::endl;
			return false;
		}

		// threre is no cluster infomation at first time
		use_prev_cluster_result_ = false;
	}

	return true;
}

/**
 * @brief search a feasible view pose and generate a trajectory to the view pose and back
 * @param[in] is_firstview
 * @param[in] prev_succed
 */
bool PickingTaskPlanner::planViewPoseAndGoBack(bool is_firstview, bool prev_succeed, int view_count) {
	std::vector<cnoid::PoseSeqItem*> items(2);

	grasp::PlanBase::instance()->targetObject->object->p() = init_obj_p_;
	grasp::PlanBase::instance()->targetObject->object->R() = init_obj_R_;

	if (!viewPlan(is_firstview, prev_succeed, view_count)) return false;
	if (!genTrajectory(init_q_, view_q_, first_motion_, &(items[0]))) return false;
	if (!genTrajectory(view_q_, init_q_, second_motion_, &(items[1]))) return false;

	// merge motions
	cnoid::PoseSeqItemPtr seq_item = new cnoid::PoseSeqItem();
	grasp::PlanBase::instance()->bodyItemRobot()->addSubItem(seq_item);
	seq_item->setName("GraspPoseSeqItem");
	mergePoseSeq(items, seq_item.get());
	ItemTreeView::mainInstance()->clearSelection();
	cnoid::ItemTreeView::mainInstance()->selectItem(seq_item->bodyMotionItem());

	// delete the first pose of second_motion since it is the same as the last pose of first_motion_
	if (second_motion_.size() > 1) {
		second_motion_.erase(second_motion_.begin());
	}

	ResultDataExporter::instance()->exportViewPlannerPoseSeqItem(items[0], items[1]);

	return true;
}

/**
 * @brief search a feasible view pose
 * @param[in] is_firstview
 * @param[in] prev_succed
 */
bool PickingTaskPlanner::viewPlan(bool is_firstview, bool prev_succeed, int view_count) {
	bool has_view = false;
	bool do_remove_obj = prev_succeed && (view_count == 1);

	if (is_firstview) {
		has_view = ViewPlanner::instance()->goFirstFeasibleViewPose();
	} else {
		has_view = ViewPlanner::instance()->goNextFeasibleViewPoseUsingGrid(do_remove_obj);
	}

	if (has_view) {
		storePose(view_q_);
	}

	return has_view;
}

/**
 * @brief generate a trajectory
 * @param[in] from
 * @param[in] to
 * @param[out] motion
 * @param[out] item
 */
bool PickingTaskPlanner::genTrajectory(const cnoid::VectorXd& from, const cnoid::VectorXd& to, std::vector<MotionState>& motion, cnoid::PoseSeqItem** item) {
	bool ret = false;
	grasp::PlanBase* pb = grasp::PlanBase::instance();

	pb->jointSeq.clear();
	pb->graspMotionSeq.clear();

	grasp::MotionState state;
	state.jointSeq = from;
	state.pathPlanDOF = pb->pathPlanDOF;
	state.graspingState = PlanBase::NOT_GRASPING;
	state.graspingState2 = PlanBase::NOT_GRASPING;
	state.objectContactState = PlanBase::ON_ENVIRONMENT;
	state.motionTime = 2.5;
	state.pos = pb->body()->rootLink()->p();
	state.rpy.setZero();
	pb->graspMotionSeq.push_back(state);

	state.jointSeq = to;
	pb->graspMotionSeq.push_back(state);

	TrajectoryPlanner tp;
	ret = tp.doTrajectoryPlanning();

	*item = tp.poseSeqItemRobot;

	if (ret) {
		motion = pb->graspMotionSeq;
	}

	return ret;
}

/**
 * @brief pose estiamtion, pikcing planning  and  command the robot
 * @param[in] multi_pc
 * @param[in] view_count
 * @param[out] is_estimation_succeed
 */
bool PickingTaskPlanner::poseEstimationAndPlanning(bool multi_pc, int view_count, bool* is_estimation_succeed) {
	bool ret = false;

	TimeBar::instance()->setTime(0);
	ManipController::instance()->clearTrajectories();
	ItemTreeView::mainInstance()->clearSelection();
	grasp::PlanBase::instance()->targetObject->object->p() = init_obj_p_;
	grasp::PlanBase::instance()->targetObject->object->R() = init_obj_R_;

	// command the robot to move to view pose
	doMove(first_motion_);

	// set the robot to view pose
	restorePose(view_q_);

	// capture
	PoseEstimator* estimator = NULL;  // only used in single pc
#ifdef PICKINGTASKPLANNER_WRITELOG_CAPTUREJOINTANGLE
	writeLogJointAngle(view_q_);
#endif
	bool do_merge = (view_count != 1);
	StopWatch timer("total_recogplan");
	timer.start();
	if (multi_pc) {
		captureMultiPC(do_merge);
	} else {
		estimator = new PoseEstimator();
		capture(estimator, do_merge);
	}

	timer.stopAndPrintTime("recogplan(capture)");
	timer.start();
	prev_grasped_cluster_id_ = -1;

#if EIGEN_VERSION_AT_LEAST(3,2,0)
	Eigen::initParallel();
#endif

	// a process which command the robot to move from view pose executes in another thread
	boost::thread thr_move(&PickingTaskPlanner::doMove, this, second_motion_);

	if (multi_pc) {
		// pose estimation and planning
		ret = binPickingMultiPC(is_estimation_succeed);
	} else {
		// pose estimation
		StopWatch timer2;
		timer2.start();
		(*is_estimation_succeed) = poseEstimate(estimator);
		timer2.stopAndPrintTime("recog(total)");
		// bin picking planning
		if (*is_estimation_succeed) {
			timer2.start();
			ret = binPicking();
			timer2.stopAndPrintTime("planning(total)");
		}

		delete estimator;
	}
	timer.stopAndPrintTime("recogplan(recog+planing)");
	timer.printTotalTime();

	thr_move.join();

	if (ret) {
		init_obj_p_ = grasp::PlanBase::instance()->targetObject->object->p();
		init_obj_R_ = grasp::PlanBase::instance()->targetObject->object->R();
	}

	use_prev_cluster_result_ = *is_estimation_succeed;
	return ret;
}

/**
 * @brief capture point cloud method for sigle PC
 * @param[in,out] estiamtor
 * @param[in] do_merge
 */
void PickingTaskPlanner::capture(PoseEstimator* estimator, bool do_merge) {
	CameraPtr camera = CameraHandler::instance()->getTargetCamera();
	estimator->setCameraMat(camera->getCameraPosition(), camera->getCameraRotation());
	ObjectPoseEstimateParams param;
	PoseEstimator::readParams(&param);
	param.is_mergecloud = do_merge;
	param.use_prev_result = use_prev_cluster_result_;
	param.prev_grasped_cluster_id = prev_grasped_cluster_id_;
	estimator->init(param);
	estimator->readObject();
	estimator->capture();
}

/**
 * @brief capture point cloud method for multi PC
 * @param[in] do_merge
 */
void PickingTaskPlanner::captureMultiPC(bool do_merge) {
	ObjectRecognitionResultManipulator* manip = ObjectRecognitionResultManipulator::instance();
	grasp::PlanBase* pb = grasp::PlanBase::instance();

	// set paramters
	CameraPtr camera = CameraHandler::instance()->getTargetCamera();
	manip->setObjectID(readObjID(pb->targetObject->bodyItemObject->body()));
	ConvertData::RecognitionParam param;
	param.camera_p = camera->getCameraPosition();
	param.camera_R = camera->getCameraRotation();
	param.box_model_id = readObjID(BoxHolder::instance()->getTargetBox()->body());
	param.box_p = BoxHolder::instance()->getTargetBox()->center();
	param.box_R = BoxHolder::instance()->getTargetBox()->R();
	param.do_merge = do_merge;
	param.use_prev_result = use_prev_cluster_result_;
	param.prev_grasped_cluster_id = prev_grasped_cluster_id_;
	manip->setRecogParams(param);

	// start recogniton
	manip->startRecognition();

	while (manip->getStartFlag()) {
		usleep(1000);
	}
}

/**
 * @brief objet pose estiamtion method for single PC
 * @param[in] estiamtor
 */
bool PickingTaskPlanner::poseEstimate(PoseEstimator* estimator) {
	estimator->boxRegistration(BoxHolder::instance()->getTargetBox()->body());
	BoxHolder::instance()->flush();

	if (!estimator->segmentation()) {
		return false;
	}

	bool ret = estimator->estimation();

	ResultDataExporter::instance()->exportBox();
	ResultDataExporter::instance()->exportPointClouds(estimator);

	if (ret) {
		ret = false;
		grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
		for (size_t i = 0; i < opes->size(); i++) {
			bool is_feasible = isSolInsideBox(opes->at(i).p, opes->at(i).R);
			if (is_feasible) ret = true;
			opes->at(i).is_feasible = is_feasible;
		}
		ResultDataExporter::instance()->exportObjectEstimationSols();
	}
	return ret;
}

/**
 * @brief bin picking planning
 */
bool PickingTaskPlanner::binPicking() {
	bool ret = false;

	grasp::PlanBase* pb = grasp::PlanBase::instance();

	grasp::TargetObject* obj = pb->targetObject;
	if (!obj) return false;

	// compute occupied voxel grid
	restorePose(view_q_);
	ViewPlanner::instance()->calcCurrentOccupiedVoxelGrid(occupied_voxel_grid);

	// init pose
	restorePose(init_q_);

	JointSeqDivider jsd;
	// jsd.setDividingKeyPose(JointSeqDivider::LIFTUP_POSE);

	// clear trajectories
	// Vector3 ori_p = pb->targetObject->object->p();
	// Matrix3 ori_R = pb->targetObject->object->R();
	// ManipController::instance()->clearTrajectories();
	// pb->targetObject->object->p() = ori_p;
	// pb->targetObject->object->R() = ori_R;

	bool col_check_flag = pb->doCheckCollisionPointCloudFinger;
	pb->doCheckCollisionPointCloudFinger = false;

	// planning
	int putting_posid = 0;
	bool success_binpicking = false;
	if (collection_box_ != NULL) {
		putting_posid = collection_box_->getFreePuttingPosID();
		if (putting_posid < 0) {
			os << "collection box has no space to put" << std::endl;
			return false;
		}
		cnoid::Vector3 put_pos = collection_box_->getPuttingPos(putting_posid);
		success_binpicking = ManipController::instance()->doBinPickingGraspPlanning(put_pos);
	} else {
		success_binpicking = ManipController::instance()->doBinPickingGraspPlanning(&occupied_voxel_grid, is_try_nagative_sol_);
	}

	if (success_binpicking) {
		ScopeStopWatch timer("planning(trajectory)");
		// insert view pose
		is_inserted_viewpose_ = insertViewPose();
		if (is_inserted_viewpose_) {
			jsd.setDividingKeyPose(JointSeqDivider::PRERELEASE_POSE);
		}

		ret = true;
		jsd.divide();
		obj = pb->targetObject;
		cnoid::Vector3 ori_p = obj->bodyItemObject->body()->link(0)->p();
		cnoid::Matrix3 ori_R = obj->bodyItemObject->body()->link(0)->R();
		obj->objVisPos = obj->object->p();
		obj->objVisRot = obj->object->R();
		cnoid::Vector3 ori_vis_p = obj->objVisPos;
		cnoid::Matrix3 ori_vis_R = obj->objVisRot;
		std::vector<cnoid::PoseSeqItem*> robot_seqs, obj_seqs;

		for (int i = 0; i < jsd.size(); i++) {
			jsd.setJointSeq(i);
			for (int j = 0; j < pb->bodyItemRobot()->body()->numJoints(); j++) {
						pb->bodyItemRobot()->body()->joint(j)->q() = pb->jointSeq[0](j);
			}
			pb->bodyItemRobot()->body()->calcForwardKinematics();

			// velocity check
			for (int j = 0; j < pb->jointSeq.size() - 1; j++) {
				for (int k = 0; k < pb->body()->numJoints(); k++) {
					double velocity = fabs(pb->jointSeq[j](k) - pb->jointSeq[j+1](k)) / pb->motionTimeSeq[j+1];
					if (velocity > M_PI / 3.0) {
						os << "!!!!WARINGIN!!! joint: " << k << ":" << velocity * 180.0 / M_PI <<  std::endl;
					}
				}
			}
			TrajectoryPlanner tp;
			ret &= tp.doTrajectoryPlanning();
			robot_seqs.push_back(tp.poseSeqItemRobot);
			obj_seqs.push_back(tp.poseSeqItemObject);
			cnoid::Vector3 p;
			cnoid::Matrix3 R;
			getObjPosPrevMotionSeq(p, R);
			obj->bodyItemObject->body()->link(0)->p() = p;
			obj->bodyItemObject->body()->link(0)->R() = R;
			obj->objVisPos = p;
			obj->objVisRot = R;

			jsd.registerMotionSeq(i);
			if (i == 0) {
				first_motion_ = pb->graspMotionSeq;
			} else if (i == 1) {
				second_motion_ = pb->graspMotionSeq;
				if (second_motion_.size() > 1) {
					second_motion_.erase(second_motion_.begin());
				}
			}
		}

		ResultDataExporter::instance()->exportPickingSeqItem(robot_seqs, obj_seqs);

		// merge motions
		cnoid::PoseSeqItemPtr roboseq_item = new cnoid::PoseSeqItem();
		pb->bodyItemRobot()->addSubItem(roboseq_item);
		roboseq_item->setName("GraspPoseSeqItem");
		mergePoseSeq(robot_seqs, roboseq_item.get());
		cnoid::PoseSeqItemPtr objseq_item = new cnoid::PoseSeqItem();
		obj->bodyItemObject->addSubItem(objseq_item);
		objseq_item->setName("GraspPoseSeqItem");
		mergePoseSeq(obj_seqs, objseq_item.get());
		ItemTreeView::mainInstance()->clearSelection();
		cnoid::ItemTreeView::mainInstance()->selectItem(roboseq_item->bodyMotionItem());
		cnoid::ItemTreeView::mainInstance()->selectItem(objseq_item->bodyMotionItem());

		jsd.revertJointSeq();

		obj->bodyItemObject->body()->link(0)->p() = ori_p;
		obj->bodyItemObject->body()->link(0)->R() = ori_R;
		obj->objVisPos = ori_vis_p;
		obj->objVisRot = ori_vis_R;
	}
	pb->doCheckCollisionPointCloudFinger = col_check_flag;
	ManipController::instance()->strategy = ManipController::NOT_SELECTED;

	if (ret) {
		if (collection_box_ != NULL) {
			collection_box_->setPuttingPosFill(putting_posid);
		}
	}

	// restore pose
	restorePose(init_q_);

	return ret;
}

/**
 * @brief bin picking planning for mutli PC
 * @param[out] is_estimation_succeed
 */
bool PickingTaskPlanner::binPickingMultiPC(bool* is_estimation_succeed) {
	bool ret = true;
	ObjectRecognitionResultManipulator* manip = ObjectRecognitionResultManipulator::instance();
	grasp::PlanBase* pb = grasp::PlanBase::instance();

	int count = 0;
	bool is_first = true;
	bool has_pick_sol = false;
	cnoid::Vector3 best_p_grasp, best_p_put, best_p_obj;
	cnoid::Matrix3 best_R_grasp, best_R_put, best_R_obj;
	cnoid::VectorXd best_theta;
	int best_id = 0;
	double best_score = -std::numeric_limits<double>::max();
	int best_sec = std::numeric_limits<int>::max();
	cnoid::BodyItemPtr best_bodyitem;
	ConvertData::RecognitionResultExt recog_res;
	std::vector<int> outlier_idx;
	bool is_finish = false;
	bool col_check_flag = pb->doCheckCollisionPointCloudFinger;
	pb->doCheckCollisionPointCloudFinger = false;

	ObjPoseEstimateSolHolder* opes = ObjPoseEstimateSolHolder::instance();
	opes->clear();

	restorePose(init_q_);

	// clear trajectories
	// Vector3 ori_p = pb->targetObject->object->p();
	// Matrix3 ori_R = pb->targetObject->object->R();
	// ManipController::instance()->clearTrajectories();
	// pb->targetObject->object->p() = ori_p;
	// pb->targetObject->object->R() = ori_R;

	std::vector<BodyItemPtr> target_bodyitems;

	while (true) {
		// cnoid::MessageView::instance()->flush();

		if (manip->popResult(recog_res)) {
			if (is_first) {
				ScopeStopWatch timer("planning(init)");
				ConvertData::RecognitionEnvResult env_res;
				manip->getEnvResult(env_res);
				// set env point cloud
				PoseEstimator::showEnvCloud(env_res.point_cloud);
				pb->RemoveAllPointCloudEnvironment();
				pb->pointCloudEnv.clear();
				std::vector<cnoid::Vector3> normal_vec;
				pb->SetPointCloudEnvironment(env_res.point_cloud, normal_vec);

				ManipController::instance()->strategy = ManipController::RIGHT_RIGHT;
				ManipController::instance()->binPickingInitProc();

				// set box position
				BoxInfo* target_box = BoxHolder::instance()->getTargetBox();
				target_box->body()->rootLink()->p() = env_res.box_p;
				target_box->body()->rootLink()->R() = env_res.box_R;
				BoxHolder::instance()->flush();
				is_first = false;
			}

			ScopeStopWatch timer("planning(main)");

			if (!isSolInsideBox(recog_res.p, recog_res.R)) continue;

			cnoid:: BodyItemPtr bodyitem;
			getBodyItem(count, bodyitem);
			bodyitem->body()->rootLink()->p() = recog_res.p;
			bodyitem->body()->rootLink()->R() = recog_res.R;
			bodyitem->notifyKinematicStateChange();
			count++;

			grasp::ObjPoseEstimateSol sol;
			sol.p = recog_res.p;
			sol.R = recog_res.R;
			sol.outlier_indices = recog_res.outlier_indices;
			sol.env_point_cloud = pb->pointCloudEnv.back();
			target_bodyitems.push_back(bodyitem);
			opes->push_back(sol);

			cnoid::Vector3 p_grasp, p_put;
			cnoid::Matrix3 R_grasp, R_put;
			cnoid::VectorXd theta;
			double score = best_score;;
			int sec = best_sec;
			pb->targetObject->object->p() = sol.p;
			pb->targetObject->object->R() = sol.R;

			if (ManipController::instance()->binPickingMainProc(sol, sec, score, p_grasp, R_grasp, p_put, R_put, theta)) {
				if (sec <= best_sec && best_score < score) {
					best_score = score;
					best_sec = sec;
					best_p_grasp = p_grasp;
					best_R_grasp = R_grasp;
					best_p_put = p_put;
					best_R_put = R_put;
					best_p_obj = sol.p;
					best_R_obj = sol.R;
					best_theta = theta;
					best_bodyitem = bodyitem;
					best_id = opes->size() - 1;
					has_pick_sol = true;
				}
			}
		} else {
			if (manip->isFinish()) {
				if (count != 0) {
					ScopeStopWatch timer("planning(trajectory)");
					*is_estimation_succeed = has_pick_sol;
					if (has_pick_sol) {
						// pb->SetGraspedObject(best_bodyitem);
						pb->targetObject->object->p() = best_p_obj;
						pb->targetObject->object->R() = best_R_obj;
					} else {
						ret = false;
					}
					ManipController::instance()->binPickingFinalProc(has_pick_sol, best_p_grasp, best_R_grasp, best_p_put, best_R_put, best_theta);
					if (has_pick_sol) {
						for (size_t i = 0; i < target_bodyitems.size(); i++) {
							grasp::TargetObject* target_object = new grasp::TargetObject(target_bodyitems[i]);
							opes->at(i).targetObject = target_object;
						}
						JointSeqDivider jsd;
						is_inserted_viewpose_ = insertViewPose();
						if (is_inserted_viewpose_) {
							jsd.setDividingKeyPose(JointSeqDivider::PRERELEASE_POSE);
						}
						jsd.divide();
						grasp::TargetObject* obj = pb->targetObject;
						cnoid::Vector3 ori_p = obj->bodyItemObject->body()->link(0)->p();
						cnoid::Matrix3 ori_R = obj->bodyItemObject->body()->link(0)->R();
						cnoid::Vector3 ori_vis_p = obj->objVisPos;
						cnoid::Matrix3 ori_vis_R = obj->objVisRot;
						std::vector<cnoid::PoseSeqItem*> robot_seqs, obj_seqs;

						for (int i = 0; i < jsd.size(); i++) {
							jsd.setJointSeq(i);
							// velocity check
							for (int j = 0; j < pb->jointSeq.size() - 1; j++) {
								for (int k = 0; k < pb->body()->numJoints(); k++) {
									double velocity = fabs(pb->jointSeq[j](k) - pb->jointSeq[j+1](k)) / pb->motionTimeSeq[j+1];
									if (velocity > M_PI / 3.0) {
										os << "!!!!WARINGIN!!! joint: " << k << ":" << velocity * 180.0 / M_PI <<  std::endl;
									}
								}
							}
							TrajectoryPlanner tp;
							ret &= tp.doTrajectoryPlanning();
							robot_seqs.push_back(tp.poseSeqItemRobot);
							obj_seqs.push_back(tp.poseSeqItemObject);
							cnoid::Vector3 p;
							cnoid::Matrix3 R;
							getObjPosPrevMotionSeq(p, R);
							obj->bodyItemObject->body()->link(0)->p() = p;
							obj->bodyItemObject->body()->link(0)->R() = R;
							obj->objVisPos = p;
							obj->objVisRot = R;
							jsd.registerMotionSeq(i);
							if (i == 0) {
								first_motion_ = pb->graspMotionSeq;
							} else if (i == 1) {
								second_motion_ = pb->graspMotionSeq;
								if (second_motion_.size() > 1) {
									second_motion_.erase(second_motion_.begin());
								}
							}
						}
						// merge motions
						cnoid::PoseSeqItemPtr roboseq_item = new cnoid::PoseSeqItem();
						pb->bodyItemRobot()->addSubItem(roboseq_item);
						roboseq_item->setName("GraspPoseSeqItem");
						mergePoseSeq(robot_seqs, roboseq_item.get());
						cnoid::PoseSeqItemPtr objseq_item = new cnoid::PoseSeqItem();
						obj->bodyItemObject->addSubItem(objseq_item);
						objseq_item->setName("GraspPoseSeqItem");
						mergePoseSeq(obj_seqs, objseq_item.get());
						ItemTreeView::mainInstance()->clearSelection();
						cnoid::ItemTreeView::mainInstance()->selectItem(roboseq_item->bodyMotionItem());
						cnoid::ItemTreeView::mainInstance()->selectItem(objseq_item->bodyMotionItem());

						jsd.revertJointSeq();

						obj->bodyItemObject->body()->link(0)->p() = ori_p;
						obj->bodyItemObject->body()->link(0)->R() = ori_R;
						obj->objVisPos = ori_vis_p;
						obj->objVisRot = ori_vis_R;

						for (size_t i = 0; i < target_bodyitems.size(); i++) {
							opes->at(i).is_target = (i == best_id);
						}
						prev_grasped_cluster_id_ = best_id;
					}
				} else {
					ret = false;
					*is_estimation_succeed = false;
                }
                ManipController::instance()->strategy = ManipController::NOT_SELECTED;
				pb->doCheckCollisionPointCloudFinger = col_check_flag;
				std::cout << "finish" << std::endl;
				break;
			}
		}
		usleep(100);
	}
	return ret;
}

bool PickingTaskPlanner::isSolInsideBox(const cnoid::Vector3& p, const cnoid::Matrix3& R) {
	// target object's center of mass
	cnoid::Vector3 obj_com = R * PlanBase::instance()->targetObject->objCoM_ + p;
	BoxInfo* target_box = BoxHolder::instance()->getTargetBox();
	cnoid::Vector3 box_p = target_box->center();
	cnoid::Matrix3 box_R = target_box->R();
	// center of mass in box coordinate
	cnoid::Vector3 obj_com_b = box_R.transpose() * (obj_com - box_p);
	if (fabs(obj_com_b.x()) - target_box->halfX() > 0) return false;
	if (fabs(obj_com_b.y()) - target_box->halfY() > 0) return false;
	if (obj_com_b.z() < 0) return false;
	return true;
}

bool PickingTaskPlanner::doMove(std::vector<MotionState> motion) const {
#ifdef PICKINGTASKPLANNER_REPLACE_DOMOVE_DUMMY
		os << "  Begin move (stub)" << std::endl;
		const int size = motion.size();
		const int angle_size = motion[0].jointSeq.size();

		for (int i = 0; i < size; i++) {
			os << "   motion time(" << i << "):" << motion[i].motionTime << std::endl;
			os << "   angle:";
			for (int j = 0; j < angle_size; j++) {
				os << " " << (180.0 / M_PI) * motion[i].jointSeq(j);
			}
			os << std::endl;
		}
		os << "  End move (stub)" << std::endl;
		return true;
#endif

	bool ret = false;
	try {
		RobotInterface::instance()->doMove(motion);
		ret = true;
	}catch (CORBA::INV_OBJREF e) {
		showWarningDialog("CORBA::INV_OBJREF 例外: ポートは接続されていますか?");
	} catch (CORBA::OBJECT_NOT_EXIST e) {
		showWarningDialog("CORBA::OBJECT_NOT_EXIST 例外: RTコンポーネントはアクティベートされていますか?");
	} catch (CORBA::NO_IMPLEMENT e) {
		showWarningDialog("CORBA::NO_IMPLEMENT 例外: メソッドは実装されていますか？(エラー出力を確認)");
	} catch (CORBA::UNKNOWN e) {
		showWarningDialog("CORBA::UNKNOWN 例外: ロボットはセットアップされていますか?");
	} catch (CORBA::SystemException& e) {
		char msg[1024];
		sprintf(msg, "%s(%lu) 例外: 未対応のシステム例外が発生しました", e._name(), (unsigned long)e.minor());
		showWarningDialog(msg);
	} catch (CORBA::Exception& e) {
		char msg[1024];
		sprintf(msg, "%s 例外: 未対応の例外が発生しました", e._name());
		showWarningDialog(msg);
	}
	return ret;
}

void PickingTaskPlanner::storePose(cnoid::VectorXd& q) const {
	PlanBase* pb = PlanBase::instance();
	q.resize(pb->body()->numJoints());
	for (int i = 0; i < pb->body()->numJoints(); i++) {
		q(i) = (pb->body()->joint(i)->q());
	}
}

void PickingTaskPlanner::restorePose(const cnoid::VectorXd& q) {
	PlanBase* pb = PlanBase::instance();
	for (int i = 0; i < pb->body()->numJoints(); i++) {
		pb->body()->joint(i)->q() = q(i);
	}
	pb->body()->calcForwardKinematics();
	pb->flush();
}


void PickingTaskPlanner::getObjPosPrevMotionSeq(cnoid::Vector3& p, cnoid::Matrix3& R) const {
	grasp::TargetObject* obj = grasp::PlanBase::instance()->targetObject;
	if (!obj) return;
	cnoid::BodyItemPtr obj_item = obj->bodyItemObject;
	cnoid::Item* child_item = obj_item->childItem();
	cnoid::PoseSeqItem* last_pose_seq_item = NULL;
	int max_id = -1;
	while (child_item != NULL) {
		cnoid::PoseSeqItem* pose_seq_item = dynamic_cast<cnoid::PoseSeqItem*>(child_item);
		child_item = child_item->nextItem();
		if (pose_seq_item != NULL) {
			//  We assume that PoseSeqItem's name format is "GraspPoseSeqItemX"
			std::string name = pose_seq_item->name();
			if (name.size() <= std::string("GraspPoseSeqItem").size()) continue;
			int id = boost::lexical_cast<int>(name.substr(16));
			if (id > max_id) {
				max_id = id;
				last_pose_seq_item = pose_seq_item;
			}
		}
	}
	if (!last_pose_seq_item) return;
	cnoid::BodyMotionPtr motion = last_pose_seq_item->bodyMotionItem()->motion();
	cnoid::MultiSE3Seq::Frame last_frame = motion->linkPosSeq()->frame(motion->linkPosSeq()->numFrames()-1);
	p = last_frame[0].translation();
	R = last_frame[0].rotation().toRotationMatrix();
}

void PickingTaskPlanner::initialize() {
	target_box_ = NULL;
	has_sol_ = false;
	is_inserted_viewpose_ = false;
	use_prev_cluster_result_ = false;
	prev_grasped_cluster_id_ = -1;
	PoseEstimator::clearPointClouds();
}

void PickingTaskPlanner::visionTriggerStart() {
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	cnoid::VectorXd init_q(pb->body()->numJoints());
	for (int j = 0; j < pb->body()->numJoints(); j++) {
		init_q(j) = pb->body()->joint(j)->q();
	}
	visionTriggerStart(init_q, false, false, -1);
}

bool PickingTaskPlanner::visionTriggerStart(const cnoid::VectorXd& init_pose, bool do_merge, bool use_prev_result, int prev_grasped_cluster_id) {
	bool ret = true;
	ObjectRecognitionResultManipulator* manip = ObjectRecognitionResultManipulator::instance();
	grasp::PlanBase* pb = grasp::PlanBase::instance();

	// set paramters
	CameraPtr camera = CameraHandler::instance()->getTargetCamera();
	manip->setObjectID(readObjID(pb->targetObject->bodyItemObject->body()));
	ConvertData::RecognitionParam param;
	param.camera_p = camera->getCameraPosition();
	param.camera_R = camera->getCameraRotation();
	param.box_model_id = readObjID(BoxHolder::instance()->getTargetBox()->body());
	param.box_p = BoxHolder::instance()->getTargetBox()->center();
	param.box_R = BoxHolder::instance()->getTargetBox()->R();
	param.do_merge = do_merge;
	param.use_prev_result = use_prev_result;
	param.prev_grasped_cluster_id = prev_grasped_cluster_id;
	manip->setRecogParams(param);

	// start recogniton
	manip->startRecognition();

	int count = 0;
	bool has_pick_sol = false;
	cnoid::Vector3 best_p_grasp, best_p_put;
	cnoid::Matrix3 best_R_grasp, best_R_put;
	cnoid::VectorXd best_theta;
	int best_id = 0;
	double best_score = -std::numeric_limits<double>::max();
	int best_sec = std::numeric_limits<int>::max();
	cnoid::BodyItemPtr best_bodyitem;
	ConvertData::RecognitionResultExt recog_res;
	std::vector<int> outlier_idx;
	bool is_finish = false;
	bool col_check_flag = pb->doCheckCollisionPointCloudFinger;
	pb->doCheckCollisionPointCloudFinger = false;

	ObjPoseEstimateSolHolder* opes = ObjPoseEstimateSolHolder::instance();
	opes->clear();

	for (int i = 0; i < pb->body()->numJoints(); i++) {
		pb->body()->joint(i)->q() = init_pose(i);
	}
	pb->body()->calcForwardKinematics();

	std::vector<BodyItemPtr> target_bodyitems;

	while (true) {
		// cnoid::MessageView::instance()->flush();

		if (manip->popResult(recog_res)) {
			cnoid::BodyItemPtr bodyitem;
			getBodyItem(count, bodyitem);
			bodyitem->body()->rootLink()->p() = recog_res.p;
			bodyitem->body()->rootLink()->R() = recog_res.R;
			bodyitem->notifyKinematicStateChange();
			count++;

			if (count == 1) {
				ConvertData::RecognitionEnvResult env_res;
				manip->getEnvResult(env_res);
				// set env point cloud
				PoseEstimator::showEnvCloud(env_res.point_cloud);
				pb->RemoveAllPointCloudEnvironment();
				pb->pointCloudEnv.clear();
				std::vector<Vector3> normal_vec;
				pb->SetPointCloudEnvironment(env_res.point_cloud, normal_vec);

				ManipController::instance()->strategy = ManipController::RIGHT_RIGHT;
				ManipController::instance()->binPickingInitProc();

				// set box position
				BoxInfo* target_box = BoxHolder::instance()->getTargetBox();
				target_box->body()->rootLink()->p() = env_res.box_p;
				target_box->body()->rootLink()->R() = env_res.box_R;
				BoxHolder::instance()->flush();
			}

			grasp::ObjPoseEstimateSol sol;
			sol.p = recog_res.p;
			sol.R = recog_res.R;
			sol.outlier_indices = recog_res.outlier_indices;
			sol.env_point_cloud = pb->pointCloudEnv.back();
			target_bodyitems.push_back(bodyitem);
			opes->push_back(sol);
			opes->back().outlier_indices = recog_res.outlier_indices;

			cnoid::Vector3 p_grasp, p_put;
			cnoid::Matrix3 R_grasp, R_put;
			cnoid::VectorXd theta;
			double score = best_score;;
			int sec = best_sec;
			pb->SetGraspedObject(bodyitem);

			if (ManipController::instance()->binPickingMainProc(sol, sec, score, p_grasp, R_grasp, p_put, R_put, theta)) {
				std::cout << "sol:" << sec << ":" << score << std::endl;
				if (sec <= best_sec && best_score < score) {
					best_score = score;
					best_sec = sec;
					best_p_grasp = p_grasp;
					best_R_grasp = R_grasp;
					best_p_put = p_put;
					best_R_put = R_put;
					best_theta = theta;
					best_bodyitem = bodyitem;
					best_id = opes->size() - 1;
					has_pick_sol = true;
				}
			}
			ManipController::instance()->strategy = ManipController::NOT_SELECTED;
		} else {
			if (manip->isFinish()) {
				if (count != 0) {
					if (has_pick_sol) {
						pb->SetGraspedObject(best_bodyitem);
					} else {
						ret = false;
					}
					ManipController::instance()->binPickingFinalProc(has_pick_sol, best_p_grasp, best_R_grasp, best_p_put, best_R_put, best_theta);
					if (has_pick_sol) {
						insertViewPose();
						TrajectoryPlanner tp;
						tp.doTrajectoryPlanning();
						for (size_t i = 0; i < target_bodyitems.size(); i++) {
							grasp::TargetObject* target_object = new grasp::TargetObject(target_bodyitems[i]);
							opes->at(i).targetObject = target_object;
							opes->at(i).is_target = (i == best_id);
						}
					}
				} else {
					ret = false;
				}
				pb->doCheckCollisionPointCloudFinger = col_check_flag;
				std::cout << "finish" << std::endl;
				break;
			}
		}
		usleep(100);
	}
	return ret;
}

void PickingTaskPlanner::mergePoseSeq(const std::vector<cnoid::PoseSeqItem*>& items, cnoid::PoseSeqItem* merged_item) {
	if (items.empty()) return;
	cnoid::PoseSeq::const_iterator it;


	int num_frames = 0;
	int num_joints = items[0]->bodyMotionItem()->motion()->jointPosSeq()->getNumParts();
	int num_links = items[0]->bodyMotionItem()->motion()->linkPosSeq()->getNumParts();
	int num_rate = items[0]->bodyMotionItem()->motion()->frameRate();
	for (size_t i = 0; i < items.size(); i++) {
		num_frames += items[i]->bodyMotionItem()->motion()->getNumFrames();
	}

	BodyMotionPtr motion = merged_item->bodyMotionItem()->motion();
	motion->setDimension(num_frames, num_joints, num_links);
	motion->setFrameRate(num_rate);

	MultiSE3SeqPtr lseq = motion->linkPosSeq();
	MultiValueSeqPtr jseq = motion->jointPosSeq();
	int cur_f = 0;
	for (size_t i = 0; i < items.size(); i++) {
		BodyMotionPtr base_motion = items[i]->bodyMotionItem()->motion();
		MultiSE3SeqPtr base_lseq = base_motion->linkPosSeq();
		MultiValueSeqPtr base_jseq = base_motion->jointPosSeq();
		for (int f = 0; f < base_motion->getNumFrames(); f++) {
			for (int k = 0; k < num_links; k++) {
				lseq->at(cur_f, k) = base_lseq->at(f, k);
			}
			for (int k = 0; k < num_joints; k++) {
				jseq->at(cur_f, k) = base_jseq->at(f, k);
			}
			cur_f++;
		}
	}

	merged_item->bodyMotionItem()->notifyUpdate();
}

void PickingTaskPlanner::getBodyItem(int n, cnoid::BodyItemPtr& item) {
	ItemPtr ro_item = RootItem::mainInstance()->findItem<Item>("RecognizedObjects");
	if (ro_item == NULL) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ro_item = new Item();
#else
		ro_item = new BodyItem();
#endif
		ro_item->setName("RecognizedObjects");
		RootItem::mainInstance()->addChildItem(ro_item);
	}

	grasp::PlanBase* pb = grasp::PlanBase::instance();

	Item* target_item = ro_item->childItem();
	int count = -1;
	while (target_item != NULL) {
		if (target_item->name() == pb->targetObject->bodyItemObject->name()) {
			BodyItem* bodyitem = dynamic_cast<BodyItem*>(target_item);
			if (bodyitem != NULL) {
				count++;
				if (count == n) {
					ItemTreeView::mainInstance()->checkItem(target_item, true);
					item = bodyitem;
				}
			}
		}
		if (count > n) {
			ItemTreeView::mainInstance()->checkItem(target_item, false);
		}
		target_item = target_item->nextItem();
	}

	for (int i = count; i < n; i++) {
		BodyItemPtr bodyitem(new BodyItem(*(grasp::PlanBase::instance()->targetObject->bodyItemObject)));
		ro_item->addChildItem(bodyitem);
		ItemTreeView::mainInstance()->checkItem(bodyitem, true);
		item = bodyitem;
	}
}

bool PickingTaskPlanner::insertViewPose() {
#ifdef PICKINGTAKSPLANNER_SKIP_INSERTVIEWPOSE
	return false;
#endif
	bool ret = false;
	grasp::PlanBase* pb = grasp::PlanBase::instance();

	if (pb->jointSeq.size() < 9) return false;

	int target_keypose_id = static_cast<int>(JointSeqDivider::PRERELEASE_POSE);

	cnoid::VectorXd joint_seq = pb->jointSeq[target_keypose_id];

	if (ViewPlanner::instance()->insertViewPose(joint_seq)) {
		pb->jointSeq[target_keypose_id] = joint_seq;
		view_q_ = joint_seq;
		ret = true;
	}
	return ret;
}

int PickingTaskPlanner::readObjID(const cnoid::BodyPtr body) {
	const Mapping& info = *(body->info()->toMapping());

	int id;
	if (info.read("objID", id)) {
		return id;
	}
	return -1;
}

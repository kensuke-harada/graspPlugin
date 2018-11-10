#include "WaistPositionSearcher.h"

#include <math.h>
#include <stack>

#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>

#include "../Grasp/Parallelizer.h"
#include "../Grasp/DrawUtility.h"
#include "PathSearcher.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

using namespace std;
using namespace cnoid;

using namespace grasp;

VirtualObjectHandler::VirtualObjectHandler() {
	tc = PlanBase::instance();
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 4; j++) {
			body[i][j] = NULL;
		}
	}
}

void VirtualObjectHandler::setObject(const cnoid::BodyItem& orig_obj, CORNER corner, LOCATION location, const cnoid::Vector3& p) {
	body[corner][location] = new BodyItem(orig_obj);
	body[corner][location]->body()->link(0)->p() = p;
}

void VirtualObjectHandler::enable(RobotBodyPtr body_, CORNER corner) {
	for (int i = 0; i < 4; i++) {
		if (body[corner][i] == NULL) continue;
		tc->SetEnvironment(body_, body[corner][i]);
	}
}

void VirtualObjectHandler::disable(RobotBodyPtr body_, CORNER corner) {
	for (int i = 0; i < 4; i++) {
		if (body[corner][i] == NULL) continue;
		tc->RemoveEnvironment(body_, body[corner][i]);
	}
}

WaistPositionSearcher::WaistPositionSearcher() {
	x_trans_joint_id = 23;
	y_trans_joint_id = 24;
  tc = PlanBase::instance();
	infeasible_rid = vector<vector<int> >(4, vector<int>());
}

WaistPositionSearcher* WaistPositionSearcher::instance(WaistPositionSearcher *wps) {
  static WaistPositionSearcher* instance = (wps) ? wps : new WaistPositionSearcher();
  if(wps) instance = wps;
  return instance;
}

void WaistPositionSearcher::setXYtransJoint(unsigned int x_joint_id, unsigned int y_joint_id) {
	x_trans_joint_id = x_joint_id;
	y_trans_joint_id = y_joint_id;
}


void PoseHolder::store(const BodyPtr body) {
	q.clear();
	for (int i = 0; i < body->numJoints(); i++) {
		 q.push_back(body->joint(i)->q());
  }
}

void PoseHolder::restore(const BodyPtr body) {
	for (int i = 0; i < body->numJoints(); i++) {
		body->joint(i)->q() = q[i];
  }
	body->calcForwardKinematics();
}

WaistPosSearchResult* WaistPosSearchResult::instance(WaistPosSearchResult *wpsr) {
	static WaistPosSearchResult* instance = (wpsr) ? wpsr : new WaistPosSearchResult();
	if(wpsr) instance = wpsr;
	return instance;
}

void WaistPosSearchResult::showResult(vector<bool> right, vector<bool> left, bool route) {
	BoxInfoArray boxes = getBoxes();
	region.displayPoints(boxes, right, left, true, false);

	DrawUtility* draw = DrawUtility::instance();

	double radius = 0.003;
	double tip_radius = 0.01;
	double tip_len = 0.015;
	Vector3 color(1.0, 0.0, 0.0);
	if (route) {
		vector<Vector3> route_path;
		if (!route_coords.empty()) {
			route_path.push_back(start);
			for (size_t i = 0; i < route_coords.size(); i++) {
				route_path.push_back(Vector3(region.getCoord(route_coords[i].x(), route_coords[i].y())));
			}
			route_path.push_back(start);
		}

		for (size_t i = 0; i < route_path.size()-1; i++) {
			Vector3 diff = route_path[i+1] - route_path[i];
			Vector3 pos = route_path[i] + (diff/2.0);
			Vector3 dir = unit(diff);
			draw->cylinders.push_back(Cylinders(pos, dir, radius, diff.norm(), color, 1.0));

			Vector3 tip_pos = route_path[i+1] - (dir * tip_len);
			Vector3 z_vec(0, 0, 1);
			Matrix3 tip_R = rotFromTwoVecs(z_vec, dir);
			draw->cones.push_back(Cones(tip_pos, tip_R, tip_radius, tip_len, color, 10));
		}
	}

	draw->displayShapes();
}

void WaistPositionSearcher::graspCornerObj(const BoxInfoArray& box_infos, const WaistSearchParam& param, int corner) {

	params = param;

	initial(tc->targetObject, tc->targetArmFinger);

	// set the prehension paramter so that GRCmax paramter can be used
	PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
	settingPrehension(target_prehension);
	target_arm_id = getTargetArmID();

// #if defined(CNOID_10_11_12_13) || defined(CNOID_14)
// 	CameraHandler::instance()->init(tc->body(), tc->bodyItemRobot()->lastAccessedFilePath());
// #else
// 	CameraHandler::instance()->init(tc->body(), tc->bodyItemRobot()->filePath());
// #endif

	BodyItemPtr target_object_item = new BodyItem();
	target_box_info = box_infos[0];
	loadObject(box_infos[0], target_object_item);

	setObjCornerPos();

	readGraspDB();

	double x = tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q();
	double y = tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q();
	BodyPtr body = tc->bodyItemRobot()->body();
	voh.enable(tc->robotBody(), VirtualObjectHandler::CORNER(corner));
		 //set ik solver
	const int num_step = 20;
	PoseHolder state;
	state.store(body);
	ConstraintIKSolver* cik = new ConstraintIKSolver(body, tc->arm(target_arm_id));
	CameraIKPath* camera_ik = NULL;
	if (params.with_camera) {
		AttachedCameraPtr target_camera = boost::dynamic_pointer_cast<AttachedCamera>(CameraHandler::instance()->getTargetCamera())->copy(body);
		if (target_camera->getType() == Camera::HANDEYE_CAMERA) {
			camera_ik = new HandCameraIKPath();
		} else {
			camera_ik = new Camera2DimensionIKPath();
		}
		camera_ik->setCamera(target_camera);

		if (params.gaze_object) {
			camera_ik->setTargetPoint(obj_corner_pos[corner]);
		} else {
			camera_ik->setTargetPoint(params.gaze_point);
		}
		cik->addIKArm(camera_ik);
	}
	ArmIKPath arm_ik;
	arm_ik.setArm(tc->arm( target_arm_id));
	cik->addIKArm(&arm_ik);
	cik->init();

	unsigned int m;
	int sstep;
	bool first_ik = true;
	for (m = 0; m < grasp_postures.size(); m++) {
		if (first_ik) {
			state.restore(body);
		}
		//tc->object(body)->p() = obj_corner_pos[corner];

		int ik_step = (first_ik) ? num_step : num_step - sstep;
		cik->setStep(ik_step);
		cik->setTranslationalJointCoeff(1);
		cik->setFixJoint(x_trans_joint_id);
		cik->setFixJoint(y_trans_joint_id);

		int success_step;
		cik->setMaxIteration(10);
		bool t_ik;

		arm_ik.setGoal(target_object->R() * grasp_postures[m].p +  obj_corner_pos[corner], target_object->R() * grasp_postures[m].R);
		t_ik = cik->ik(success_step, !first_ik);

		if(first_ik){
			sstep = success_step;
		}

		if (!t_ik) {
			first_ik  = false;
			continue;
		} else {
			int k = 0;
			for (int l = 0; l < tc->nFing(target_arm_id); l++) {
				for (int j = 0; j < tc->fingers(target_arm_id, l)->fing_path->numJoints(); j++) {
					tc->fingers(target_arm_id, l)->fing_path->joint(j)->q() = grasp_postures[m].finger_q(k++);
				}
			}
			body->calcForwardKinematics();

			if (isColliding() || !withinJointLimit(body)) {
				if(m==0) first_ik = true;
				continue;
			}
#ifdef DEBUG_MODE
			//cout << "grasp id:" <<  grasp_candidate[corner][m]->id << endl;
			//cout << "x y :" << body->joint(x_trans_joint_id)->q() << " " << body->joint(y_trans_joint_id)->q() << endl;

			//os << "grasp id:" <<  grasp_candidate[corner][m]->id << endl;
			//os << "x y :" << tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q() << " " << tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q() << endl;
#endif
			break;
		}
	}

	if(camera_ik != NULL) {
		delete camera_ik;
		camera_ik = NULL;
	}
	delete cik;
	voh.disable(tc->robotBody(), VirtualObjectHandler::CORNER(corner));
}

int WaistPositionSearcher::getTargetArmID() {
	int armid = 0;
	for (size_t i = 0; i < PlanBase::instance()->armsList.size(); i++) {
		if (PlanBase::instance()->targetArmFinger->arm == PlanBase::instance()->arm(i)) {
			armid = i;
			break;
		}
	}
	return armid;
}

bool WaistPositionSearcher::searchWaistPos(const BoxInfoArray& box_infos, const WaistSearchParam& param) {
	if (tc == NULL) {
    os << "implmentation error: you have to call initial()" << endl;
    return false;
  }

  if (!tc->targetObject || !tc->targetArmFinger) {
    os << "Please select Grasped Object and Grasping Robot" << endl;
    return false;
  }

	if (box_infos.empty()) {
		os << "Please select box-obect pair(s)" << endl;
		return false;
	}

#ifdef THREAD
#if EIGEN_VERSION_AT_LEAST(3,2,0)
	Eigen::initParallel();
#endif
#endif

	params = param;

	initial(tc->targetObject, tc->targetArmFinger);

	// set the prehension paramter so that GRCmax paramter can be used
	PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
	settingPrehension(target_prehension);
	target_arm_id = getTargetArmID();

// #if defined(CNOID_10_11_12_13) || defined(CNOID_14)
// 	CameraHandler::instance()->init(tc->body(), tc->bodyItemRobot()->lastAccessedFilePath());
// #else
// 	CameraHandler::instance()->init(tc->body(), tc->bodyItemRobot()->filePath());
// #endif

	bool has_sol = false;
	if (box_infos.size() == 1) {
		// single target
		Vector3 pos;
		has_sol = searchWaistPosSingleBox(box_infos[0], pos);
		if (has_sol) {
			os << "graspable position: " << pos.transpose() << endl;
		}
	} else {
		// mulit target
		has_sol = searchWaistPosMultiBox(box_infos);			
	}

#ifdef THREAD
	RobotParallelizer::clearColChecker();
#endif

	//timer.end();

	//os << "time2:" << timer.getLastTime() << endl;

	return has_sol;
}

bool WaistPositionSearcher::searchWaistPosSingleBox(const BoxInfoPtr box_info, Vector3& pos) {
	bool ret;
	BodyItemPtr target_object_item = new BodyItem();

	loadObject(box_info, target_object_item);

	ret = searchWaistPosSub(box_info, pos);

	unloadObject(target_object_item);

	return ret;
}

bool WaistPositionSearcher::searchWaistPosMultiBox(const BoxInfoArray& box_infos) {
	bool ret = true;
	vector<vector<int> > boxids;
	vector<GraspableRegion> regions;
	
	GraspableRegion::grid_interval = params.grid_interval;

	for (size_t i = 0; i < box_infos.size(); i++) {
		Vector3 pos;

		// load target object
		BodyItemPtr target_object_item = new BodyItem();
		loadObject(box_infos[i], target_object_item);

		GraspableRegion region;
		for (size_t j = 0; j < params.target_arm_ids.size(); j++) {
			target_arm_id = params.target_arm_ids[j];
			infeasible_rid = vector<vector<int> >(4, vector<int>());
			PrehensionPtr target_prehension = tc->armsList[target_arm_id]->getTargetPrehension()[0];
			settingPrehension(target_prehension);
			// obtain graspable region
			bool has_sol = searchWaistPosSub(box_infos[i], pos);
			graspable_pose.restore(tc->body());
			if (has_sol) {
				GraspableRegion tmp_region(tc->armsList.size());
				getGraspableRegion(pos, tmp_region);
				region |= tmp_region;
			}
			original_pose.restore(tc->body());
		}
		ret &= !region.empty();
		regions.push_back(region);

		// unload target object
		unloadObject(target_object_item);
	}

	if (!ret) {
		return false;
	}

	// get the solution with min number of moves
	SetCoveringSolver scs;
	scs.solve(regions, boxids);

	// make result
	vector<GraspableRegion> results;
	GraspableRegion region_for_display;
	for (size_t i = 0; i < boxids.size(); i++) {
		GraspableRegion overlap = regions[boxids[i][0]];
		region_for_display |= regions[boxids[i][0]];
		for (size_t j = 1; j < boxids[i].size(); j++) {
			overlap &= regions[boxids[i][j]];
			region_for_display |= regions[boxids[i][j]];
		}
		if (overlap.empty()) {
			continue;
		}
		results.push_back(overlap);
	}

	PathSearcher path_searcher;
	vector<int> route;
	vector<Eigen::Vector2i> coords_id;
	path_searcher.getPath(results, Vector3(0,0,0), route, coords_id);

	displayResult(results, route, coords_id);

	WaistPosSearchResult* res = WaistPosSearchResult::instance();
	res->start = Vector3(0, 0, 0);
	res->route_coords = coords_id;
	res->region = region_for_display;

	if (params.display_graspable_points) {
		region_for_display.displayPoints();
	}

	return ret;
}

void WaistPositionSearcher::loadObject(BoxInfoPtr box_info, BodyItemPtr& target_object_item) {
	if(target_object_item->load(box_info->getObjInfo()->object_model_path, "OpenHRP-VRML-MODEL")){
		RootItem::mainInstance()->addChildItem(target_object_item);
		ItemTreeView::mainInstance()->checkItem(target_object_item, true);
	}

	target_object = target_object_item->body()->link(0);
	Link* target_box_link = box_info->body()->link(0);

	target_object->p() = target_box_link->p() + target_box_link->R() * box_info->objRelPos();
	target_object->R() = target_box_link->R() * box_info->objRelR();
	target_object_item->notifyKinematicStateChange();

	box_info->setObjectBodyItem(target_object_item);
}

void WaistPositionSearcher::unloadObject(BodyItemPtr& target_object_item) {
	target_object_item->detachFromParentItem();
	target_object = NULL;
}

bool WaistPositionSearcher::searchWaistPosSub(const BoxInfoPtr box_info, Vector3& pos) {
	bool ret = false;
	target_box_info = box_info;

	// Set object positions
	setObjCornerPos();

	original_pose.store(tc->body());

	readGraspDB();

	ret = searchWaistPosProc(pos);

	graspable_pose.store(tc->body());
	original_pose.restore(tc->body());
	tc->bodyItemRobot()->notifyKinematicStateChange();

	return ret;
}

bool WaistPositionSearcher::searchWaistPosProc(Vector3& pos) {
	// search initial position
	if (!initialPosSearch()) {
		return false;
	}

	// find graspable position for each corner
	if (!searchCornerGraspablePos()) {
		return false;
	}

	initial_pose.restore(tc->body());

	Vector3 cur_pos;
	obtainAveragePos(cur_pos);
  tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q() = cur_pos(0);
  tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q() = cur_pos(1);
  tc->bodyItemRobot()->body()->calcForwardKinematics();
	tc->bodyItemRobot()->notifyKinematicStateChange();

	makeGraspPostureCandidate();

	vector<bool> is_grasp;
	 // check
  if(isGraspable(is_grasp)){
#ifdef DEBUG_MODE
    os << "waist pos:" << endl << cur_pos << endl;
    os << "success!" << endl;
#endif
		pos = cur_pos;
    return true;
  }

	// if it cannot grasp an object placed in each corner, there is no solution 
	bool is_all_fail = true;
	for (unsigned int i = 0; i < 4; i++) {
		is_all_fail &= !is_grasp[i];
	}
	if (is_all_fail) {
		os << "grasping failed at all corners" << endl;
		return false;
	}

#ifdef DEBUG_MODE
	os << "average pos:" << endl << cur_pos << endl;
  os << "avearage position is infeasible" << endl;
	for (unsigned int i = 0; i < 4; i++) {
		os << getCornerString(i) << ":" << (is_grasp[i] ? "OK" : "NG") << endl;
	}
  os << "second stage" << endl;
#endif

	Vector3 prev_pos;
	bool found_sol;
	if (!searchGraspableEdgePos(cur_pos, prev_pos, is_grasp, &found_sol)) {
		pos = cur_pos;
		return found_sol;
	}

	//binary search
	if (!searchBinary(cur_pos, prev_pos, is_grasp)) {
#ifdef DEBUG_MODE
		os << "pos:" << endl << prev_pos << endl;
		os << "fail!" << endl;
#endif
		return false;
	}

	pos = cur_pos;

	return true;
}

bool WaistPositionSearcher::initialPosSearch() {
	bool ret = true;

	const double coeff = 10.0;

	voh.enable(tc->robotBody(), VirtualObjectHandler::CENTER);

	//move to initial position
	Vector3 pos;
	Vector3 dir = target_box_info->body()->link(0)->R() * target_box_info->initSearchPos();
	dir(2) = 0.0;
	dir.normalize();

	pos = target_box_info->body()->link(0)->p() + dir * norm2(target_box_info->initSearchPos());
	pos(2) = tc->bodyItemRobot()->body()->link(0)->p()(2);
	tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q() = pos(0);
	tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q() = pos(1);
	tc->bodyItemRobot()->body()->calcForwardKinematics();
	tc->bodyItemRobot()->notifyKinematicStateChange();

#ifdef DEBUG_MODE
	os << "pos(init):" << endl <<  pos << endl;
#endif

	//move the robot to the postion where it can grasp the object placed in the center of the box
	if (searchGraspPosition(pos, coeff, false)) {
#ifdef DEBUG_MODE
		os << "pos(center grasp):" << endl << pos << endl;
#endif
	} else {
		os << "error: cannot find feasible position at searching intial position." << endl;
		ret = false;
	}

	voh.disable(tc->robotBody(), VirtualObjectHandler::CENTER);

	initial_pose.store(tc->body());

	return ret;
}

bool WaistPositionSearcher::searchCornerGraspablePos() {
	const double coeff = 100.0;
#ifdef THREAD
	initial_pose.restore(tc->body());

	RobotParallelizer pal;
	pal.initialize(4);

	vector<int> has_sol(4, 0);
	for (unsigned int i = 0; i < 4; i++) {
		pal.addTask(boost::bind(&WaistPositionSearcher::searchGraspablePosParallel, this, _1, coeff, false, i, &has_sol[i], &graspable_pos[i]));
	}
	pal.doTasks();
	pal.join();
	for (unsigned int i = 0; i < 4; i++) {
		if(has_sol[i] == 0) return false;
	}
#else
	for (unsigned int i = 0; i < 4; i++) {
		initial_pose.restore(tc->body());
		voh.enable(tc->robotBody(), VirtualObjectHandler::CORNER(i));

		Vector3 pos = Vector3::Zero();
		if (!searchGraspPosition(pos, coeff, false, i)) {
			os << getCornerString(i) << ":failed" << endl;
			voh.disable(tc->robotBody(), VirtualObjectHandler::CORNER(i));
			return false;
		}
#ifdef DEBUG_MODE
		os << "corner pos(" << getCornerString(i)  << "):" << endl <<  pos << endl;
#endif
		graspable_pos[i] = pos;
		voh.disable(tc->robotBody(), VirtualObjectHandler::CORNER(i));
	}
#endif
	return true;
}

bool WaistPositionSearcher::searchGraspablePosParallel(RobotBodyPtr body, double coeff, bool is_fix, int corner, int* has_sol, Vector3* pos) {
	{
#ifdef THREAD
		boost::mutex::scoped_lock lock(RobotParallelizer::g_mutex);
#endif
		voh.enable(body, VirtualObjectHandler::CORNER(corner));
	}

	bool ret = searchGraspPosition(body, coeff, is_fix, corner, false, *pos);

	*has_sol = (ret ? 1 : 0);
	
	{
#ifdef THREAD
		boost::mutex::scoped_lock lock(RobotParallelizer::g_mutex);
#endif
		voh.disable(body, VirtualObjectHandler::CORNER(corner));
	}
	return true;
}


/**
* Check if object which is placed in each corner can be grasped.
* @param[out] is_graspable graspable check result of each case
* @return True if graspable in all cases, False if not
*/
bool WaistPositionSearcher::isGraspable(vector<bool>& is_graspable) {
	bool ret = true;
	is_graspable = vector<bool>(4, true);

	PoseHolder state;
	state.store(tc->body());
	
	double x = tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q();
	double y = tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q();

#ifdef THREAD
	RobotParallelizer pal;
	pal.initialize(4);

	vector<int> has_sol(4, 0);
	for (unsigned int i = 0; i < 4; i++) {
		pal.addTask(boost::bind(&WaistPositionSearcher::isGraspableSub, this, _1, i, x, y, &has_sol[i]));
	}
	pal.doTasks();
	pal.join();
	for (unsigned int i = 0; i < 4; i++) {
		if(has_sol[i] == 0) ret = false;
		is_graspable[i] = (has_sol[i]== 1);
	}
#else
	for (unsigned int i = 0; i < 4; i++) {
		state.restore(tc->body());

		int has_sol = 0;
		isGraspableSub(tc->robotBody(), i, x, y, &has_sol);
		if(has_sol == 0) ret = false;
		is_graspable[i] = (has_sol == 1);
	}
#endif

	state.restore(tc->body());

	return ret;
}

bool WaistPositionSearcher::isGraspable() {
	vector<bool> dummy;
	return isGraspable(dummy);
}

bool WaistPositionSearcher::isGraspableSub(RobotBodyPtr body, int corner, double x, double y, int* has_sol) {
	*has_sol = 0;

	body->bodyItem()->body()->joint(x_trans_joint_id)->q() = x;
	body->bodyItem()->body()->joint(y_trans_joint_id)->q() = y;
	body->bodyItem()->body()->calcForwardKinematics();

	PoseHolder state;
	state.store(body->bodyItem()->body());

	const int num_step = 20;

	//tc->object(body)->p() = obj_corner_pos[corner];

	{
#ifdef THREAD
	boost::mutex::scoped_lock lock(RobotParallelizer::g_mutex);
#endif
	voh.enable(body, VirtualObjectHandler::CORNER(corner));
	}
	// set ik solver
	ConstraintIKSolver* cik = new ConstraintIKSolver(body->bodyItem()->body(), body->getArmFingers(target_arm_id)->arm);
	CameraIKPath* camera_ik = NULL;
	if (params.with_camera) {
		AttachedCameraPtr target_camera = boost::dynamic_pointer_cast<AttachedCamera>(CameraHandler::instance()->getTargetCamera())->copy(body->bodyItem()->body());
		if (target_camera->getType() == Camera::HANDEYE_CAMERA) {
			camera_ik = new HandCameraIKPath();
		} else {
			camera_ik = new Camera2DimensionIKPath();
		}
		camera_ik->setCamera(target_camera);

		if (params.gaze_object) {
			camera_ik->setTargetPoint(obj_corner_pos[corner]);
		} else {
			camera_ik->setTargetPoint(params.gaze_point);
		}
		cik->addIKArm(camera_ik);
	}
	ArmIKPath arm_ik;
	arm_ik.setArm(body->getArmFingers(target_arm_id)->arm);
	cik->addIKArm(&arm_ik);
	cik->init();

	unsigned int m;
	int sstep;
	bool first_ik = true;
	for (m = 0; m < grasp_candidate[corner].size(); m++) {
		if (first_ik) {
			state.restore(body->bodyItem()->body());
		}
		//tc->object(body)->p() = obj_corner_pos[corner];

		int ik_step = (first_ik) ? num_step : num_step - sstep;
		cik->setStep(ik_step);
		cik->setTranslationalJointCoeff(1);
		cik->setFixJoint(x_trans_joint_id);
		cik->setFixJoint(y_trans_joint_id);

		int success_step;
		cik->setMaxIteration(10);
		bool t_ik;

		arm_ik.setGoal(target_object->R() * grasp_candidate[corner][m]->p +  obj_corner_pos[corner], target_object->R() * grasp_candidate[corner][m]->R);
		t_ik = cik->ik(success_step, !first_ik);

		if(first_ik){
			sstep = success_step;
		}

		if (!t_ik) {
			first_ik  = false;
			continue;
		} else {
			int k = 0;
			for (int l = 0; l < tc->nFing(target_arm_id); l++) {
				for (int j = 0; j < body->getArmFingers(target_arm_id)->fingers[l]->fing_path->numJoints(); j++) {
					body->getArmFingers(target_arm_id)->fingers[l]->fing_path->joint(j)->q() = grasp_candidate[corner][m]->finger_q(k++);
				}
			}
			body->calcForwardKinematics();

			if (isColliding(body) || !withinJointLimit(body->bodyItem()->body())) {
				if(m==0) first_ik = true;
				continue;
			}
			*has_sol = 1;
#ifdef DEBUG_MODE
			//cout << "grasp id:" <<  grasp_candidate[corner][m]->id << endl;
			//cout << "x y :" << body->joint(x_trans_joint_id)->q() << " " << body->joint(y_trans_joint_id)->q() << endl;

			//os << "grasp id:" <<  grasp_candidate[corner][m]->id << endl;
			//os << "x y :" << tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q() << " " << tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q() << endl;
#endif
			break;
		}
	}

	if(camera_ik != NULL) {
		delete camera_ik;
		camera_ik = NULL;
	}
	delete cik;

	{
#ifdef THREAD
	boost::mutex::scoped_lock lock(RobotParallelizer::g_mutex);
#endif
	voh.disable(body, VirtualObjectHandler::CORNER(corner));
	}
	state.restore(body->bodyItem()->body());
	return true;

}

bool WaistPositionSearcher::searchGraspableEdgePos(Vector3& curr_pos, Vector3& prev_pos, vector<bool>& is_graspable, bool* found_sol) {
  prev_pos = curr_pos;
	*found_sol = false;
  while (true) {
    curr_pos = Vector3::Zero();
    int count =0;
    for (unsigned int i = 0; i < 4; i++) {
      if (!is_graspable[i]) {
        curr_pos = curr_pos + graspable_pos[i];
        count++;
      }
    }
    curr_pos = curr_pos / static_cast<double>(count);
		original_pose.restore(tc->body());
    tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q() = curr_pos(0);
    tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q() = curr_pos(1);
    tc->bodyItemRobot()->body()->calcForwardKinematics();

#ifdef DEBUG_MODE
		os << "curr_pos" << curr_pos << endl;
    os << "pos(second_stage):" << endl << curr_pos << endl;
#endif
    vector<bool> is_grasp_new;
    if (isGraspable(is_grasp_new)) {
#ifdef DEBUG_MODE
      os << "waist pos:" << endl << curr_pos << endl;
      os << "success2!" << endl;
#endif
			*found_sol = true;
      return false;
    }
#ifdef DEBUG_MODE
		for (unsigned int i = 0; i < 4; i++) {
			os << getCornerString(i) << ":" << (is_grasp_new[i] ? "OK" : "NG") << endl;
		}
#endif

    bool condition_break = true;
    bool is_same = true;
    for (unsigned int i = 0; i < 4; i++) {
      if (!is_graspable[i] && is_grasp_new[i]) {
        condition_break = false;
        is_same = false;
      }
      if(is_graspable[i] && !is_grasp_new[i]){
        condition_break = true;
        is_same = false;
        break;
      }
    }

    if(is_same){
#ifdef DEBUG_MODE
      os << "pos:" << endl << prev_pos << endl;
      os << "fail!" << endl;
#endif
      return false;
    }
#ifdef DEBUG_MODE
		os << "curr_pos" << curr_pos << endl;
		os << "prev_pos" << prev_pos << endl;
#endif
    if(condition_break) break;
    is_graspable = is_grasp_new;
    prev_pos = curr_pos;
  }
	return true;
}

bool WaistPositionSearcher::searchBinary(Vector3& curr_pos, Vector3& prev_pos, const vector<bool>& is_graspable) {
	const int max_ite = 5;

	vector<bool> is_grasp = is_graspable;
  for (unsigned int i = 0; i < max_ite; i++) {
    Vector3 target_pos = (curr_pos + prev_pos) / 2.0;
    tc->bodyItemRobot()->body()->joint(x_trans_joint_id)->q() = target_pos(0);
    tc->bodyItemRobot()->body()->joint(y_trans_joint_id)->q() = target_pos(1);
    tc->bodyItemRobot()->body()->calcForwardKinematics();
#ifdef DEBUG_MODE
    os << "pos:" << endl << target_pos << endl;
#endif
    vector<bool> is_grasp_new;
    if (isGraspable(is_grasp_new)) {
#ifdef DEBUG_MODE
      os << "pos:" << endl << target_pos << endl;
      os << "success3!" << endl;
#endif
			curr_pos = target_pos;
			return true;
    }
#ifdef DEBUG_MODE
    for (unsigned int i = 0; i < 4; i++) {
			os << getCornerString(i) << ":" << (is_grasp_new[i] ? "OK" : "NG") << endl;
		}
#endif

    bool condition_break = false;
    int count_changetrue=0;
    int count_changefalse=0;
    for (unsigned int i = 0; i < is_grasp.size(); i++) {
      if (is_grasp[i] && !is_grasp_new[i]) {
        count_changefalse++;
      }
      if (!is_grasp[i] && is_grasp_new[i]) {
        count_changetrue++;
      }
    }

    if(count_changefalse > 0 && count_changetrue <= 0)
      return false;

    if(count_changefalse > 0){
      curr_pos  = target_pos;
    }else{
      prev_pos = target_pos;
    }
  }
	return false;
}



bool WaistPositionSearcher::searchGraspPosition(Vector3& pos, double coeff, bool is_fix) {
	return searchGraspPosition(tc->robotBody(), coeff, is_fix, 0, true, pos);
}

bool WaistPositionSearcher::searchGraspPosition(Vector3& pos, double coeff, bool is_fix, unsigned int corner) {
	return searchGraspPosition(tc->robotBody(), coeff, is_fix, corner, false, pos);
}

bool WaistPositionSearcher::searchGraspPosition(RobotBodyPtr body, double coeff, bool is_fix, unsigned int corner, bool is_center, cnoid::Vector3& pos) {
	bool ret = false;

	PoseHolder state;
	state.store(tc->body());

	Vector3 target_p = (is_center) ? target_object->p() : obj_corner_pos[corner];

	// setting IK solver
	CameraIKPath* camera_ik = NULL;
	if (params.with_camera) {
		AttachedCameraPtr target_camera = (tc->body() == body->bodyItem()->body()) ? boost::dynamic_pointer_cast<AttachedCamera>(CameraHandler::instance()->getTargetCamera()) : boost::dynamic_pointer_cast<AttachedCamera>(CameraHandler::instance()->getTargetCamera())->copy(body->bodyItem()->body());
		if (target_camera->getType() == Camera::HANDEYE_CAMERA) {
			camera_ik = new HandCameraIKPath();
		} else {
			camera_ik = new Camera2DimensionIKPath();
		}

		camera_ik->setCamera(target_camera);
	}
	ArmIKPath arm_ik;
	arm_ik.setArm(body->getArmFingers(target_arm_id)->arm);

	ConstraintIKSolver* cik = new ConstraintIKSolver(body->bodyItem()->body(), body->getArmFingers(target_arm_id)->arm);
	cik->addIKArm(&arm_ik);
	if(params.with_camera){
		cik->addIKArm(camera_ik);
	}
	cik->init();
  cik->setTranslationalJointCoeff(coeff);
  if (is_fix) {
		cik->setFixJoint(x_trans_joint_id);
    cik->setFixJoint(y_trans_joint_id);
  }

	for (unsigned int id = 0; id < grasp_postures.size(); id++) {
		// go next if target pose collide box
		if (!is_center && included(id, infeasible_rid[corner])) continue;
		
		GraspPosture hp = grasp_postures[id];
		state.restore(body->bodyItem()->body());

		cik->setMaxIteration(10);
		if(params.with_camera){
			if(params.gaze_object){
				camera_ik->setTargetPoint(target_p);
			} else {
				camera_ik->setTargetPoint(params.gaze_point);
			}
		}
		arm_ik.setGoal(target_object->R() * hp.p + target_p, target_object->R() * hp.R);
		if (!cik->ik()) continue;

		int k = 0;
    for (int l = 0; l < tc->nFing(target_arm_id); l++) {
      for (int j = 0; j < body->getArmFingers(target_arm_id)->fingers[l]->fing_path->numJoints(); j++) {
        body->getArmFingers(target_arm_id)->fingers[l]->fing_path->joint(j)->q() = hp.finger_q(k++);
      }
    }
    body->calcForwardKinematics();
      
		if (!withinJointLimit(body->bodyItem()->body())) {
			continue;
		}

		if (isColliding(body)) {
			for (int i = 0; i < tc->nFing(target_arm_id); i++) {
				for (int j = 0; j < body->getArmFingers(target_arm_id)->fingers[i]->fing_path->numJoints(); j++) {
					string link_name = body->getArmFingers(target_arm_id)->fingers[i]->fing_path->joint(j)->name();
					if (tc->getColPairName(body, 0) == link_name || tc->getColPairName(body, 1) == link_name) {
#ifdef DEBUG_MODE
						os << tc->colPairName[0] << ":" << tc->colPairName[1] << endl;
#endif
						if (!is_center) {
							infeasible_rid[corner].push_back(id);
						}
						break;
					}
				}
			}
			continue;
		}

		ret = true;
		if (!is_center) {
			grasp_candidate[corner].clear();
			grasp_candidate[corner].push_back(&grasp_postures[id]);
		}
#ifdef DEBUG_MODE
		os << "grasp id:" <<  id << endl;
#endif
		break;
	}

	body->bodyItem()->body()->calcForwardKinematics();
  pos.x() = body->bodyItem()->body()->joint(x_trans_joint_id)->q();
	pos.y() = body->bodyItem()->body()->joint(y_trans_joint_id)->q();
	pos.z() = 0.0;
  
  delete cik;
	if(camera_ik != NULL) {
		delete camera_ik;
		camera_ik = NULL;
	}
  return ret;
}


/**
* Set object positions which are placed in the corners of a box.
*/
void WaistPositionSearcher::setObjCornerPos() {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  ColdetModelPtr c = target_object->coldetModel();
#else
  ColdetModelPtr c = ColdetConverter::ConvertFrom(target_object->collisionShape());
#endif
  Vector3 edge, center, com;
  Matrix3 rot;
  tc->calcBoundingBox(c, edge, center, com, rot);
	Matrix3 boxR = target_box_info->body()->link(0)->R();
  vector<double> x_offset, y_offset;
  Vector3 r_edge = edge/2.0;
	
  const unsigned int x_bit = 0x0001;
  const unsigned int y_bit = 0x0002;
  const unsigned int z_bit = 0x0004;

	// calculate the vertices points of the target object bouding box
  for (unsigned int i = 0; i < 8; i++) {
    int sign_x = (i & x_bit) ? 1 : -1;
    int sign_y = (i & y_bit) ? 1 : -1;
    int sign_z = (i & z_bit) ? 1 : -1;
    Vector3 temp_r_edge = Vector3(sign_x * r_edge(0), sign_y * r_edge(1), sign_z * r_edge(2));
		Vector3 offset_vec = trans(boxR) * (target_object->R() * (rot * temp_r_edge));
		x_offset.push_back(offset_vec(0));
		y_offset.push_back(offset_vec(1));
  }

	Vector3 obj_p = target_object->p();
	double obj_x_len_p =  vmax(x_offset);
	double obj_x_len_n = -vmin(x_offset);
	double obj_y_len_p =  vmax(y_offset);
	double obj_y_len_n = -vmin(y_offset);
	double obj_x_len = obj_x_len_p + obj_x_len_n;
	double obj_y_len = obj_y_len_p + obj_y_len_p;
	double back_offset  =  target_box_info->size()->x - (obj_x_len_p + target_box_info->marginBoxObj()->x);
	double front_offset = -target_box_info->size()->x +  obj_x_len_n + target_box_info->marginBoxObj()->x;
	double left_offset  =  target_box_info->size()->y - (obj_y_len_p + target_box_info->marginBoxObj()->y);
	double right_offset = -target_box_info->size()->y +  obj_y_len_n + target_box_info->marginBoxObj()->y;

	// set object positions.
	obj_corner_pos[BACKLEFT]   = obj_p + boxR * Vector3(back_offset , left_offset , 0);
	obj_corner_pos[FRONTLEFT]  = obj_p + boxR * Vector3(front_offset, left_offset , 0);
	obj_corner_pos[FRONTRIGHT] = obj_p + boxR * Vector3(front_offset, right_offset, 0);
	obj_corner_pos[BACKRIGHT]  = obj_p + boxR * Vector3(back_offset , right_offset, 0);

	// set virtual obstacles offset positions
	Vector3 offset[4];
	offset[VirtualObjectHandler::FRONT] = boxR * Vector3(-(obj_x_len+target_box_info->marginObjObj()->x), 0, 0);
	offset[VirtualObjectHandler::BACK]  = boxR * Vector3(  obj_x_len+target_box_info->marginObjObj()->x , 0, 0);
	offset[VirtualObjectHandler::LEFT]  = boxR * Vector3(0,  obj_y_len+target_box_info->marginObjObj()->y , 0);
	offset[VirtualObjectHandler::RIGHT] = boxR * Vector3(0,-(obj_y_len+target_box_info->marginObjObj()->y), 0);

	if (target_box_info->size()->x > target_box_info->marginObjObj()->x + obj_x_len + obj_x_len_n) {
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::CENTER,
			VirtualObjectHandler::FRONT,
			obj_p + offset[VirtualObjectHandler::FRONT]);
	}
	if (target_box_info->size()->x > target_box_info->marginObjObj()->x + obj_x_len + obj_x_len_p) {
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::CENTER,
			VirtualObjectHandler::BACK,
			obj_p + offset[VirtualObjectHandler::BACK]);
	}
	if (target_box_info->size()->y > target_box_info->marginObjObj()->y + obj_y_len - vmin(y_offset)) {
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::CENTER,
			VirtualObjectHandler::RIGHT,
			obj_p + offset[VirtualObjectHandler::RIGHT]);
	}
	if (target_box_info->size()->y > target_box_info->marginObjObj()->y + obj_y_len + vmax(y_offset)) {
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::CENTER,
			VirtualObjectHandler::LEFT,
			obj_p + offset[VirtualObjectHandler::LEFT]);
	}

	if (target_box_info->size()->x > (target_box_info->marginBoxObj()->x + target_box_info->marginObjObj()->x)/2.0 + obj_x_len) {
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::BACKLEFT,
			VirtualObjectHandler::FRONT,
			obj_corner_pos[BACKLEFT] + offset[VirtualObjectHandler::FRONT]);
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::FRONTLEFT,
			VirtualObjectHandler::BACK,
			obj_corner_pos[FRONTLEFT] + offset[VirtualObjectHandler::BACK]);
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::FRONTRIGHT,
			VirtualObjectHandler::BACK,
			obj_corner_pos[FRONTRIGHT] + offset[VirtualObjectHandler::BACK]);
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::BACKRIGHT,
			VirtualObjectHandler::FRONT,
			obj_corner_pos[BACKRIGHT] + offset[VirtualObjectHandler::FRONT]);

	}
	if (target_box_info->size()->y > (target_box_info->marginBoxObj()->y + target_box_info->marginObjObj()->y)/2.0 + obj_y_len) {
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::BACKLEFT,
			VirtualObjectHandler::RIGHT,
			obj_corner_pos[BACKLEFT] + offset[VirtualObjectHandler::RIGHT]);
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::FRONTLEFT,
			VirtualObjectHandler::RIGHT,
			obj_corner_pos[FRONTLEFT] + offset[VirtualObjectHandler::RIGHT]);
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::FRONTRIGHT,
			VirtualObjectHandler::LEFT,
			obj_corner_pos[FRONTRIGHT] + offset[VirtualObjectHandler::LEFT]);
		voh.setObject(*target_box_info->getObjInfo()->body_item,
			VirtualObjectHandler::BACKRIGHT,
			VirtualObjectHandler::LEFT,
			obj_corner_pos[BACKRIGHT] + offset[VirtualObjectHandler::LEFT]);
	}
}

/**
* Get grasp posutres from the grasp DB.
*/
void WaistPositionSearcher::readGraspDB() {
	GraspDatabaseManipulator gdm;
	string db_path =  tc->dataFilePath(target_arm_id) + "preplanning_" + target_box_info->getObjInfo()->object_name + ".txt";
	gdm.readFile(db_path);
	grasp_postures = gdm.getRecords();
}


/**
* Get average position of the robot wasit positions where the robot can grasp the object placed on the corner.
* @param[out] average position
*/
void WaistPositionSearcher::obtainAveragePos(Vector3& pos) const {
	pos = Vector3::Zero();

	for (unsigned int i = 0; i < 4; i++) {
		pos += graspable_pos[i];
	}
	pos = pos / 4.0;
}

void WaistPositionSearcher::makeGraspPostureCandidate(unsigned int max_num) {
	const double angle_th = 0.7; // 45 degree

	for (unsigned int i = 0; i < 4; i++) {
		int start_id = grasp_candidate[i][0]->id;
		Vector3 base_app_vec = grasp_candidate[i][0]->R * GRCmax.R * Vector3(0.0, 1.0, 0.0);
		for (unsigned int j = start_id + 1; j < grasp_postures.size(); j++) {
			Vector3 app_vec = grasp_postures[j].R * GRCmax.R * Vector3(0.0, 1.0, 0.0);
			if (dot(base_app_vec, app_vec) < angle_th) continue;
			grasp_candidate[i].push_back(&grasp_postures[j]);
			if (grasp_candidate[i].size() >= max_num) break;
		}
	}
}

/**
* Collision detection.
*/
bool WaistPositionSearcher::isColliding() const {
  tc->bodyItemRobot()->body()->calcForwardKinematics();
	
	if (tc->targetArmFinger == tc->armsList[0]) {
		tc->setGraspingState(PlanBase::GRASPING);
		tc->setGraspingState2(PlanBase::NOT_GRASPING);
	} else {
		tc->setGraspingState(PlanBase::NOT_GRASPING);
		tc->setGraspingState2(PlanBase::GRASPING);
	}
  tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);

  tc->calcForwardKinematics();

  bool ret = tc->isColliding();

  return ret;
}

/**
* Check if angle of each joint is within limit.
*/
bool WaistPositionSearcher::withinJointLimit() const {
  for (int i = 0; i < tc->bodyItemRobot()->body()->numJoints(); i++) {
    if (tc->bodyItemRobot()->body()->joint(i)->q_upper() < tc->bodyItemRobot()->body()->joint(i)->q() 
            ||  tc->bodyItemRobot()->body()->joint(i)->q_lower() > tc->bodyItemRobot()->body()->joint(i)->q()) {
      return false;
    }
  }
  return true;
}

/**
* Collision detection.
*/
bool WaistPositionSearcher::isColliding(const RobotBodyPtr body) const {
	body->bodyItem()->body()->calcForwardKinematics();

	if (tc->targetArmFinger == tc->armsList[0]) {
		tc->setGraspingState(body, PlanBase::GRASPING);
		tc->setGraspingState2(body, PlanBase::NOT_GRASPING);
	} else {
		tc->setGraspingState(body, PlanBase::NOT_GRASPING);
		tc->setGraspingState2(body, PlanBase::GRASPING);
	}

	tc->setObjectContactState(body, PlanBase::ON_ENVIRONMENT);

	tc->calcForwardKinematics(body);

	bool ret = tc->isColliding(body);

	return ret;
}

/**
* Check if angle of each joint is within limit.
*/
bool WaistPositionSearcher::withinJointLimit(const BodyPtr body) const {
	for (int i = 0; i < body->numJoints(); i++) {
		if (body->joint(i)->q_upper() < body->joint(i)->q() 
			||  body->joint(i)->q_lower() > body->joint(i)->q()) {
				return false;
		}
	}
	return true;
}

string WaistPositionSearcher::getCornerString(unsigned int i) const {
	switch (i) {
	case 0: return "Left back ";
	case 1: return "Left front";
	case 2: return "Right front ";
	case 3: return "Right back  ";
	}
	return "";
}

void WaistPositionSearcher::displayResult(const std::vector<GraspableRegion>& regions, const std::vector<int>& route, const std::vector<Eigen::Vector2i>& coords_id) const {
	for (size_t i = 0; i < route.size(); i++) {
		int x_id = coords_id[i][0];
		int y_id = coords_id[i][1];
		int region_id = route[i];
		os << "position" << i << ":";
		os << regions[region_id].getCoord(x_id, y_id).transpose() << endl;
		os << "target box: ";
		BoxInfoArray boxes = regions[region_id].getBoxes(x_id, y_id);
		vector<vector<int> > arms = regions[region_id].getArmIDs(x_id, y_id);
		for (size_t j = 0; j < boxes.size() ; j++) {
			os << boxes[j]->bodyItem()->name();
			if (tc->armsList.size() > 1){
				os << "(";
				for (size_t k = 0; k < arms[j].size(); k++) {
					if (arms[j][k] == 0) {
						os << " right";
					} else if (arms[j][k] == 1) {
						os << " left";
					}
				}
				os << " ) ";
			}
		}
		os << endl;	
	}
}

void WaistPositionSearcher::getGraspableRegion(const cnoid::Vector3& pos, GraspableRegion& region) {
	double dist = GraspableRegion::grid_interval;
	map<int, map<int, int> > done;
	stack<pair<int,int> > stack_search;

	int x, y;
	x = ceil(pos(0) / dist);
	y = ceil(pos(1) / dist);

	stack_search.push(make_pair(x,y));
	stack_search.push(make_pair(x,y-1));
	stack_search.push(make_pair(x-1,y));
	stack_search.push(make_pair(x-1,y-1));
	vector<Vector3> graspable_points;

#ifdef THREAD
	RobotParallelizer pal;
	pal.initialize();
#endif
	vector<int> has_sol;
	vector<pair<int, int> > coord;

	while(!stack_search.empty()) {
		has_sol.clear();
		coord.clear();
		while(!stack_search.empty()) {
			pair<int,int> target = stack_search.top();
			stack_search.pop();
			if(done[target.first].count(target.second) > 0) continue;
			coord.push_back(target);
			has_sol.push_back(0);
			done[target.first][target.second] = 1;
		}
		if (has_sol.empty()) break;
		for (size_t i = 0; i < has_sol.size(); i++) {
#ifdef THREAD
			pal.addTask(boost::bind(&WaistPositionSearcher::isGraspableParallel, this, _1, coord[i].first*dist, coord[i].second*dist, &has_sol[i]));
#else
			has_sol[i] = 1;
			for (unsigned int j = 0; j < 4; j++) {
				int has_sol_corner = 0;
				isGraspableSub(tc->robotBody(), j, coord[i].first*dist, coord[i].second*dist, &has_sol_corner);
				if (has_sol_corner == 0) {
					has_sol[i] = 0;
					break;
				}
			}
#endif
		}

#ifdef THREAD
		pal.doTasks();
		pal.join();
#endif

		for (size_t i = 0; i < has_sol.size(); i++) {
			if (has_sol[i] == 1) {
				if(done[coord[i].first].count(coord[i].second+1) == 0) {stack_search.push(make_pair(coord[i].first  ,coord[i].second+1));}
				if(done[coord[i].first+1].count(coord[i].second) == 0) {stack_search.push(make_pair(coord[i].first+1,coord[i].second  ));}
				if(done[coord[i].first].count(coord[i].second-1) == 0) {stack_search.push(make_pair(coord[i].first  ,coord[i].second-1));}
				if(done[coord[i].first-1].count(coord[i].second) == 0) {stack_search.push(make_pair(coord[i].first-1,coord[i].second  ));}
				region.addGraspablePoint(coord[i].first, coord[i].second, target_box_info, target_arm_id);
				graspable_points.push_back(Vector3(coord[i].first*dist, coord[i].second*dist,0));
			}
		}
	}
}

bool WaistPositionSearcher::isGraspableParallel(RobotBodyPtr body, double x, double y, int* has_sol) {
	bool ret = true;

	*has_sol = 1;

	PoseHolder state;

	state.store(tc->body());

	for (unsigned int i = 0; i < 4; i++) {
		state.restore(body->bodyItem()->body());

		int has_sol_corner = 0;
		isGraspableSub(body, i, x, y, &has_sol_corner);
		if(has_sol_corner == 0) {
			*has_sol = 0;
			state.restore(body->bodyItem()->body());
			return true;
		}
	}

	state.restore(body->bodyItem()->body());

	return true;
}


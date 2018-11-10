#include "ManipFailureAnalyzer.h"

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

void ManipFailureAnalyzer::Result::addCollisions(const vector<PlanBase::ColPair>& collisions) {
	for (size_t i = 0; i < collisions.size(); i++) {
		COLLISION_PART col[2];
		for (int j = 0; j < 2; j++) {
			switch(collisions[i].type[j]) {
			case PlanBase::ColPair::ROBO:
				col[j] = BODY;
				for (int k = 0; k < PlanBase::instance()->targetArmFinger->nFing; k++) {
					for (int l = 0; l < PlanBase::instance()->fingers(k)->nJoints; l++) {
						if (collisions[i].link_name[j] == PlanBase::instance()->fingers(k)->fing_path->joint(l)->name()) {
							col[j] = FING;
							break;
						}
					}
				}
				for (int k = 0; k < PlanBase::instance()->targetArmFinger->arm->nJoints; k++) {
					if (collisions[i].link_name[j] == PlanBase::instance()->targetArmFinger->arm->arm_path->joint(k)->name()) {
						col[j] = ARM;
						break;
					}
				}
				break;
			case PlanBase::ColPair::ENV:
				col[j] = ENV;
				break;
			case PlanBase::ColPair::OBJ:
				col[j] = OBJ;
				break;
			case PlanBase::ColPair::CLOUD:
				col[j] = ENV;
				break;
			}
		}
#ifdef CNOID_GE_16
		colpair.push_back(std::make_pair(col[0], col[1]));
#else
		colpair.push_back(std::make_pair<COLLISION_PART, COLLISION_PART>(col[0], col[1]));
#endif
	}
}

bool ManipFailureAnalyzer::Result::hasCollision(COLLISION_PART_FLAG col1, COLLISION_PART_FLAG col2) const {
	for (size_t i = 0; i < colpair.size(); i++) {
		if (((colpair[i].first & col1) && (colpair[i].second & col2)) ||
			((colpair[i].first & col2) && (colpair[i].second & col1))) {
				return true;
		}
	}
	return false;
}


ManipFailureAnalyzer* ManipFailureAnalyzer::instance() {
	static ManipFailureAnalyzer* instance = new ManipFailureAnalyzer();
	return instance;
}

void ManipFailureAnalyzer::analyze() {
	PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
	settingPrehension(target_prehension);

	GraspDatabaseManipulator gdm;
	string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	gdm.readFile(filepath);
	grasp_poses = gdm.getRecords();

	setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

	bool is_change_po_des = !(searchPutPos());

	failuareCheck();

	bool has_feasible_sol = false;

	if (is_change_po_des) {
		has_feasible_sol = sortPutPos();
		PlacePlanner::instance()->writePutPos(Po_des, Ro_des);
		if (has_feasible_sol) {
			os << "Putting pos mat file is updated." << endl;
		}
	}

	if (!has_feasible_sol) {
		GraspDatabaseManipulator::GraspPoses inserted_poses;
		interpolateGraspPose(inserted_poses);

		if (!inserted_poses.empty()) {
			gdm.appendRecords(inserted_poses);
			gdm.unique();
			gdm.writeFile(filepath);
			os << "Grasping posture database is updated." << endl;
		} else {
			displayAnalysisResult();
		}
	}

	for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
		bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
	}
	bodyItemRobot()->body()->calcForwardKinematics();

	tc->jointSeq.clear();


}

bool ManipFailureAnalyzer::searchPutPos() {
	pf = new ParameterFileData();
	pf->readObjClusterParameters();
	Vector3 pressPos = envItem->body()->link(0)->p() + envItem->body()->link(0)->attitude()*(PlanBase::instance()->objPressPos);
	PlacePlanner::instance()->clusterIncludeTest = pf->doInclusionTest;
	PlacePlanner::instance()->stabilityTest = pf->doStabilityTest;
	PlacePlanner::instance()->collisionTest = pf->doCollisionTest;
	PlacePlanner::instance()->colTestMargin = pf->colTestMargin;


	if (!PlacePlanner::instance()->calcPutPos(pressPos, objVisRot(), Po_des, Ro_des) || Po_des.empty()) {
		Po_des.clear();
		Ro_des.clear();
		PlacePlanner::instance()->searchPutPos(pressPos, Po_des, Ro_des);
		return false;
	}
	return true;
}

void ManipFailureAnalyzer::failuareCheck() {
	grasp_result = vector<GraspResult>(grasp_poses.size(), GraspResult());
	place_result = vector<vector<PlaceResult> >(grasp_poses.size(), vector<PlaceResult>());

	VectorXd jointSeq(bodyItemRobot()->body()->numJoints());

	for (size_t i = 0; i < grasp_poses.size(); i++) {
		// store state
		Vector3 orig_obj_p = tc->targetObject->object->p();
		Matrix3 orig_obj_R = tc->targetObject->object->R();

		// init pose
		for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
      bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
		}
		int k=0;
		for (int m = 0; m < arm_g_n_fing; m++) {
			for (int n = 0; n < fingers_g[m]->fing_path->numJoints(); n++) {
				fingers_g[m]->fing_path->joint(n)->q() = grasp_poses[i].finger_q(k++);
			}
		}
		bodyItemRobot()->body()->calcForwardKinematics();

		// check if grasp poses are feasible
		checkGraspPoses(i);

		// store  state
		Vector3 tmp_obj_p = tc->targetObject->object->p();
		Matrix3 tmp_obj_R = tc->targetObject->object->R();

		for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
      jointSeq(n) = bodyItemRobot()->body()->joint(n)->q();
		}

		place_result[i] = vector<PlaceResult>(Po_des.size(), PlaceResult());
		for(size_t j = 0; j < Po_des.size(); j++){
			// restore state
			tc->targetObject->object->p() = tmp_obj_p;
			tc->targetObject->object->R() = tmp_obj_R;

			for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
			 bodyItemRobot()->body()->joint(n)->q() = jointSeq(n);
			}
			bodyItemRobot()->body()->calcForwardKinematics();

			// check if place poses are feasible
			checkPlacePoses(i, j);
		}
		// restore object state
		tc->targetObject->object->p() = orig_obj_p;
		tc->targetObject->object->R() = orig_obj_R;
	}
}

void ManipFailureAnalyzer::checkGraspPoses(GraspID gid) {
	VectorXd jointSeq(bodyItemRobot()->body()->numJoints());

	Vector3 target_p = Po_ini + Ro_ini * grasp_poses[gid].p;
	Matrix3 target_R = Ro_ini * grasp_poses[gid].R;

	grasp_result[gid].approach.result = SUCCESS;
	grasp_result[gid].pregrasp.result = SUCCESS;
	grasp_result[gid].grasp.result = SUCCESS;
	grasp_result[gid].lift.result = SUCCESS;

	int nj = arm_g->arm_path->numJoints()-1;
	Vector3 appVec = target_R * GRCmax.R * Vector3(0.0, 1.0, 0.0);
  //==== Approach Point
 	double app_length=pf->appLength;
	if (!searchLiftUpKeyPose(target_p, target_R, jointSeq, arm_g, fingers_g, -appVec, 0, &app_length, 0.01, PlanBase::NOT_GRASPING)) {
		grasp_result[gid].approach.result = COLLISION;
	}

	//==== PreGrasp and Grasp Point
	if (arm_g->IK_arm(target_p, target_R)) {
		// pre-grasp
		setJointAngle(PlanBase::UNDER_GRASPING, jointSeq, arm_g, fingers_g);
		if (arm_g_n_fing > 0 && isCollidingAllCheck(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) {
			grasp_result[gid].pregrasp.result = COLLISION;
			grasp_result[gid].pregrasp.addCollisions(tc->colpair);
		}

		// grasp
		setJointAngle(PlanBase::GRASPING, jointSeq, arm_g, fingers_g);
		if (arm_g_n_fing > 0 && isCollidingAllCheck(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) {
			grasp_result[gid].grasp.result = COLLISION;
			grasp_result[gid].grasp.addCollisions(tc->colpair);
			if (grasp_result[gid].grasp.hasCollision((FING | ARM), ENV)) {
				getColPoints(grasp_result[gid].grasp.col_p);
			}
		}
	} else {
		grasp_result[gid].grasp.result = IKFAIL;
		grasp_result[gid].pregrasp.result = IKFAIL;

		// move object to palm
		tc->targetObject->object->p() = palm()->p() - palm()->R() * grasp_poses[gid].R.transpose() * grasp_poses[gid].p;
    tc->targetObject->object->R() = palm()->R() * grasp_poses[gid].R.transpose();

		setJointAngle(PlanBase::GRASPING, jointSeq, arm_g, fingers_g);

		bodyItemRobot()->body()->calcForwardKinematics();

		if (graspingHand == RIGHT) {
			tc->setGraspingState(PlanBase::GRASPING);
			tc->setGraspingState2(PlanBase::NOT_GRASPING);
		}
		else{
			tc->setGraspingState2(PlanBase::GRASPING);
			tc->setGraspingState(PlanBase::NOT_GRASPING);
		}
	}

	//==== LiftUp Point
	Vector3 z(0.0, 0.0, 1.0);
	lift_height = pf->liftHeight;
	if (!searchLiftUpKeyPose(target_p, target_R, jointSeq,arm_g, fingers_g, z, 0, &lift_height, min_lift_height, PlanBase::GRASPING)) {
		grasp_result[gid].lift.result = COLLISION;
	}
}

void ManipFailureAnalyzer::checkPlacePoses(GraspID gid, PutID pid) {
	VectorXd jointSeq(bodyItemRobot()->body()->numJoints());

	Vector3 target_p = Po_des[pid] + Ro_des[pid] * grasp_poses[gid].p;
	Matrix3 target_R = Ro_des[pid] * grasp_poses[gid].R;

	place_result[gid][pid].approach.result = SUCCESS;
	place_result[gid][pid].place.result = SUCCESS;
	place_result[gid][pid].release.result = SUCCESS;

	//=== Top of release point
	Vector3 z(0.0, 0.0, 1.0);
	release_height = pf->releaseHeight;
	if (!searchLiftUpKeyPose(target_p, target_R, jointSeq, arm_g, fingers_g, z, 0, &release_height, min_relase_height, PlanBase::GRASPING)) {
		place_result[gid][pid].approach.result = COLLISION;
	}

  //== Release point
	if (arm_g->IK_arm(target_p, target_R)) {
		// place
		setJointAngle(PlanBase::GRASPING, jointSeq, arm_g, fingers_g);
		if (arm_g_n_fing > 0 && isCollidingAllCheck(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) {
			place_result[gid][pid].place.result = COLLISION;
			place_result[gid][pid].place.addCollisions(tc->colpair);
			if (place_result[gid][pid].place.hasCollision((FING | ARM), ENV)) {
				getColPoints(place_result[gid][pid].place.col_p);
			}
		}

		// release
		setJointAngle(PlanBase::NOT_GRASPING, jointSeq, arm_g, fingers_g);
		if (arm_g_n_fing > 0 && isCollidingAllCheck(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) {
			place_result[gid][pid].release.result = COLLISION;
			place_result[gid][pid].release.addCollisions(tc->colpair);
		}
	} else {
		place_result[gid][pid].place.result = IKFAIL;
		place_result[gid][pid].release.result = IKFAIL;
	}
}

bool ManipFailureAnalyzer::isCollidingAllCheck(int graspingState, int graspingState2, int contactState) {
	bodyItemRobot()->body()->calcForwardKinematics();
	if (graspingHand == RIGHT) {
		tc->setGraspingState(graspingState);
		tc->setGraspingState2(graspingState2);
	}
	else{
		tc->setGraspingState2(graspingState);
		tc->setGraspingState(graspingState2);
	}

#ifdef OBJ_ENV_CONTACT
	tc->setObjectContactState(contactState);
#endif

	tc->calcForwardKinematics();

	bool ret = tc->isCollidingAllCheck();

	return ret;
}

void ManipFailureAnalyzer::displayAnalysisResult() {
	os << endl;
	os << "============================================" << endl;
	os << " Failure analysis report" << endl;

	if (Po_des.empty()) {
		os << " *Cannot find feasible putting object patterns." << endl;
		return;
	}

	if (isGraspAllIKFail()) {
		os << " *The object position is far away from the robot. Please move the object." << endl;
	} else if (isGraspAllCollision()) {
		os << " *Collisions occur at grasp poses." << endl;
	}

	GraspIDVec gids = getGraspSuccessIDs();

	if (gids.empty()) return;

	if (isApproachAllFail(gids)) {
		os << " *Cannot find feasible approach poses for grasping." << endl;
	}

	GraspIDVec grasp_col_ids;
	if (isPreGraspAllCollision(gids, grasp_col_ids)) {
		os << " *Collisions between the robot and environment occur at pre-grasp poses." << endl;
		vector<double> grasp_open_offset;
		searchFeasiblePreGraspFingerAngle(grasp_col_ids, grasp_open_offset);
		os << "  To avoid collisions, please set \"fingerOpenPoseOffset\" to [";
		for(int i=0;i<grasp_open_offset.size();i++){
			os << grasp_open_offset[i] << " ";
		}
		os << "]." << endl;
	}

	if (isLiftAllFail(gids)) {
		os << " *Cannot find feasible lift-up poses." << endl;
	}

	if (isPlaceAllIKFail()) {
		os << " *The object release point is far away from the robot. Please change the release point." << endl;
	}

	IDPairVec pids = getPlaceSuccessIDs();

	if (pids.empty()) return;

	if (isPlaceApproachAllFail(pids)) {
		os << " *Cannot find feasible approach poses for placing." << endl;
	}

	IDPairVec palce_col_ids;
	if (isReleaseAllCollision(pids, palce_col_ids)) {
		os << " *Collisions between the robot and environment occur at release poses." << endl;
		vector<double> relase_open_offset;
		searchFeasibleReleaseFingerAngle(palce_col_ids, relase_open_offset);
		os << "  To avoid collisions, please set \"fingerOpenPoseOffset\" to [";
		for(int i=0;i<relase_open_offset.size();i++){
			os << relase_open_offset[i] << " ";
		}
		os << "]." << endl;
	}


	os << "============================================" << endl;
	os << endl;
}

bool ManipFailureAnalyzer::isGraspAllIKFail() const {
	for (size_t i = 0; i < grasp_result.size(); i++) {
		if (grasp_result[i].grasp.result != IKFAIL) {
			return false;
		}
	}
	return true;
}

bool ManipFailureAnalyzer::isGraspAllCollision() const {
	for (size_t i = 0; i < grasp_result.size(); i++) {
		if (grasp_result[i].grasp.result != COLLISION) {
			return false;
		}
	}
	return true;
}

ManipFailureAnalyzer::GraspIDVec ManipFailureAnalyzer::getGraspSuccessIDs() const {
	GraspIDVec ret;
	for (size_t i = 0; i < grasp_result.size(); i++) {
		if (grasp_result[i].grasp.result == SUCCESS) {
			ret.push_back(i);
		}
	}
	return ret;
}


bool ManipFailureAnalyzer::isApproachAllFail(const GraspIDVec& gids) const {
	for (size_t i = 0; i < gids.size(); i++) {
		if (grasp_result[gids[i]].approach.result == SUCCESS) {
			return false;
		}
	}
	return true;
}

bool ManipFailureAnalyzer::isPreGraspAllCollision(const GraspIDVec& gids, GraspIDVec& col_ids) const {
	col_ids.clear();
	for (size_t i = 0; i < gids.size(); i++) {
		if (grasp_result[gids[i]].pregrasp.result != COLLISION) {
			return false;
		}
		if (!grasp_result[gids[i]].pregrasp.hasCollision(FING, ENV)) {
			return false;
		}
		col_ids.push_back(gids[i]);
	}
	return true;
}

bool ManipFailureAnalyzer::isLiftAllFail(const GraspIDVec& gids) const {
	for (size_t i = 0; i < gids.size(); i++) {
		if (grasp_result[gids[i]].lift.result == SUCCESS) {
			return false;
		}
	}
	return true;
}

bool ManipFailureAnalyzer::isPlaceAllIKFail() const {
	for (size_t i = 0; i < place_result.size(); i++) {
		for (size_t j = 0; j < place_result[i].size(); j++) {
			if (place_result[i][j].place.result == NOT_TRY) {
				continue;
			}
			if (place_result[i][j].place.result != IKFAIL) {
				return false;
			}
		}
	}
	return true;
}

ManipFailureAnalyzer::IDPairVec ManipFailureAnalyzer::getPlaceSuccessIDs() const {
	IDPairVec ret;
	for (size_t i = 0; i < place_result.size(); i++) {
		for (size_t j = 0; j < place_result[i].size(); j++) {
			if (place_result[i][j].place.result == SUCCESS) {
				ret.push_back(make_pair(i,j));
			}
		}
	}
	return ret;
}

bool ManipFailureAnalyzer::isReleaseAllCollision(const IDPairVec& pids, IDPairVec& col_ids) const {
	col_ids.clear();
	for (size_t i = 0; i < pids.size(); i++) {
		if (place_result[pids[i].first][pids[i].second].release.result != COLLISION) {
			return false;
		}
		if (!place_result[pids[i].first][pids[i].second].release.hasCollision(FING, ENV)) {
			return false;
		}
		col_ids.push_back(make_pair(pids[i].first, pids[i].second));
	}
	return true;
}

bool ManipFailureAnalyzer::isPlaceApproachAllFail(const IDPairVec& pids) const {
	for (size_t i = 0; i < pids.size(); i++) {
		if (place_result[pids[i].first][pids[i].second].approach.result == SUCCESS) {
			return false;
		}
	}
	return true;
}

void ManipFailureAnalyzer::getColPoints(vector<Vector3>& colpoints) {
	// collision check between meshes
	for (vector<ColdetLinkPairPtr>::const_iterator it = tc->robotEnvPairs.begin(); it != tc->robotEnvPairs.end(); it++) {
		ColdetLinkPairPtr col = *it;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		col->model(0)->setPosition(col->link(0)->R(), col->link(0)->p());
		col->model(1)->setPosition(col->link(1)->R(), col->link(1)->p());
#else
		col->model(0)->setPosition(col->link(0)->T());
		col->model(1)->setPosition(col->link(1)->T());
#endif
		vector<collision_data> data = col->detectCollisions();
		for (size_t k = 0; k < data.size(); k++) {
			for (int i = 0; i < data[k].num_of_i_points; i++) {
				colpoints.push_back(data[k].i_points[i]);
			}
		}
	}

	// collision check between robot and point cloud
	for (vector<pair<Link*, PointCloudEnv*> >::const_iterator it = tc->robotExcludingFingPointCloudPairs.begin(); it != tc->robotExcludingFingPointCloudPairs.end(); ++it) {
			Link* target_link = (*it).first;
			PointCloudEnv* target_env = (*it).second;
			if (tc->isCollidingPointCloudSub(target_env, target_link)) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
					ColdetModelPtr c = target_link->coldetModel();
					Vector3 center = Vector3::Zero();
					for (int v = 0; v < c->getNumVertices(); v++) {
							float x, y, z;
							c->getVertex(v, x, y, z);
							center += Vector3(x, y, z);
					}
					center /= c->getNumVertices();
#else
					SgMeshPtr mesh = ColdetConverter::ExtractMesh(target_link->collisionShape());
        			SgVertexArrayPtr vertices = mesh->vertices();
					Vector3 center = Vector3::Zero();
					for (int v = 0; v < vertices->size(); v++) {
							cnoid::Vector3f vec = vertices->at(v);
							center += Vector3(vec[0], vec[1], vec[2]);
					}
					center /= vertices->size();
#endif
					colpoints.push_back(center);
			}
	}
}

void ManipFailureAnalyzer::searchFeasiblePreGraspFingerAngle(const GraspIDVec& target_gids, std::vector<double>& feasible_offset) {
	vector<double> base_offset;
	for (int i = 0; i < arm_g_n_fing; i++) {
		for (int j = 0; j < fingers_g[i]->fing_path->numJoints(); j++) {
#ifdef FINGER_OPEN_OFFSET_MODE
			double offset = (fingers_g[i]->fingerOpenPoseOffset.size() > j) ?  fingers_g[i]->fingerOpenPoseOffset[j] : 0;
			base_offset.push_back(offset);
#else
      base_offset.push_back(fingers_g[i]->fingerOpenPose[j]);
#endif
		}
	}

	feasible_offset = vector<double>(base_offset.size(), 0.0);

#ifndef FINGER_OPEN_OFFSET_MODE
	int target_gid = 0;
#endif

	const int max_count = 10;
	int feasible_count = 10;
	for (size_t i = 0; i < target_gids.size(); i++) {
		Vector3 target_p = Po_ini + Ro_ini * grasp_poses[target_gids[i]].p;
		Matrix3 target_R = Ro_ini * grasp_poses[target_gids[i]].R;

		arm_g->IK_arm(target_p, target_R);

		for (int j = 0; j <= max_count; j++) {
			int k=0;
			for (int m = 0; m < arm_g_n_fing; m++) {
				for (int n = 0; n < fingers_g[m]->fing_path->numJoints(); n++) {
#ifdef FINGER_OPEN_OFFSET_MODE
					fingers_g[m]->fing_path->joint(n)->q() = grasp_poses[target_gids[i]].finger_q(k) + (base_offset[k] * (1-(double)j/(double)max_count));
#else
					fingers_g[m]->fing_path->joint(n)->q() = fingers_g[m]->fingerOpenPose[n] - ((fingers_g[m]->fingerOpenPose[n] - grasp_poses[target_gids[i]].finger_q(k)) * (double)j/(double)max_count);
#endif
					k++;
				}
			}
			bodyItemRobot()->body()->calcForwardKinematics();
			if (!isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) {
				if (feasible_count > j) feasible_count = j;
#ifndef FINGER_OPEN_OFFSET_MODE
				target_gid = target_gids[i];
#endif
				break;
			}
		}
	}

	for (size_t j = 0; j < feasible_offset.size(); j++) {
#ifdef FINGER_OPEN_OFFSET_MODE
		feasible_offset[j] = base_offset[j] * (1-(double)feasible_count/(double)max_count);
#else
		feasible_offset[j] = base_offset[j] - ((base_offset[j] - grasp_poses[target_gid].finger_q(j)) * (double)feasible_count/(double)max_count);
#endif
	}
}

void ManipFailureAnalyzer::searchFeasibleReleaseFingerAngle(const IDPairVec& target_pids, std::vector<double>& feasible_offset) {
	vector<double> base_offset;
	for (int i = 0; i < arm_g_n_fing; i++) {
		for (int j = 0; j < fingers_g[i]->fing_path->numJoints(); j++) {
#ifdef FINGER_OPEN_OFFSET_MODE
			double offset = (fingers_g[i]->fingerOpenPoseOffset.size() > j) ?  fingers_g[i]->fingerOpenPoseOffset[j] : 0;
			base_offset.push_back(offset);
#else
      base_offset.push_back(fingers_g[i]->fingerOpenPose[j]);
#endif
		}
	}

	feasible_offset = vector<double>(base_offset.size(), 0.0);

#ifndef FINGER_OPEN_OFFSET_MODE
	int target_gid = 0;
#endif

	const int max_count = 10;
	int feasible_count = 10;
	for (size_t i = 0; i < target_pids.size(); i++) {
		Vector3 target_p = Po_des[target_pids[i].second] + Ro_des[target_pids[i].second] * grasp_poses[target_pids[i].first].p;
		Matrix3 target_R = Ro_des[target_pids[i].second] * grasp_poses[target_pids[i].first].R;

		arm_g->IK_arm(target_p, target_R);

		for (int j = 0; j <= max_count; j++) {
			int k=0;
			for (int m = 0; m < arm_g_n_fing; m++) {
				for (int n = 0; n < fingers_g[m]->fing_path->numJoints(); n++) {
#ifdef FINGER_OPEN_OFFSET_MODE
					fingers_g[m]->fing_path->joint(n)->q() = grasp_poses[target_pids[i].first].finger_q(k) + (base_offset[k] * (1-(double)j/(double)max_count));
#else
					fingers_g[m]->fing_path->joint(n)->q() = fingers_g[m]->fingerOpenPose[n] - ((fingers_g[m]->fingerOpenPose[n] - grasp_poses[target_pids[i].first].finger_q(k)) * (double)j/(double)max_count);
#endif
					k++;
				}
			}
			bodyItemRobot()->body()->calcForwardKinematics();
			if (!isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) {
				if (feasible_count > j) feasible_count = j;
#ifndef FINGER_OPEN_OFFSET_MODE
				target_gid = target_pids[i].first;
#endif
				break;
			}
		}
	}

	for (size_t j = 0; j < feasible_offset.size(); j++) {
#ifdef FINGER_OPEN_OFFSET_MODE
		feasible_offset[j] = base_offset[j] * (1-(double)feasible_count/(double)max_count);
#else
		feasible_offset[j] = base_offset[j] - ((base_offset[j] - grasp_poses[target_gid].finger_q(j)) * (double)feasible_count/(double)max_count);
#endif
	}
}

bool ManipFailureAnalyzer::interpolateGraspPose(GraspDatabaseManipulator::GraspPoses& inserted_poses) {
	GraspDatabaseManipulator::GraspPoses poses;
	interpolateGraspPoseSub(true, 0, poses);
	for (size_t i = 0; i < Po_des.size(); i++) {
		interpolateGraspPoseSub(false, i, poses);
	}

	if (poses.empty()) return false;

	pickAndPlaceTest(poses, inserted_poses);

	for (int i = 0; i < bodyItemRobot()->body()->numJoints(); i++)
		bodyItemRobot()->body()->joint(i)->q() = tc->jointSeq[0](i);
	bodyItemRobot()->body()->calcForwardKinematics();


	if (inserted_poses.empty()) return false;

	os << "feasible pick and place motion is found." << endl;

	return true;
}

void ManipFailureAnalyzer::interpolateGraspPoseSub(bool is_grasp, PutID pid, GraspDatabaseManipulator::GraspPoses& inserted_poses) {
	Vector3 orig_obj_p = tc->targetObject->object->p();
	Matrix3 orig_obj_R = tc->targetObject->object->R();

	vector<vector<int> > gids;

	int nContact = 0;
	for(int i=0;i<tc->nFing();i++) for(int j=0;j<tc->fingers(i)->nJoints;j++) if(tc->fingers(i)->contact[j]) nContact++;

	// classified by palmR
	for (size_t i = 0; i < grasp_poses.size(); i++) {
		bool has_same_r = false;
		for (size_t j = 0; j < gids.size(); j++) {
			if (grasp_poses[i].R == grasp_poses[gids[j][0]].R) {
				gids[j].push_back(i);
				has_same_r = true;
				break;
			}
		}
		if (!has_same_r) {
			gids.push_back(vector<int>(1, i));
		}
	}

	bool is_move_object = false;

	for (size_t i = 0; i < gids.size(); i++) {
		double min_dist = 0.0;
		Matrix3 relR = GRCmax.R.transpose() * grasp_poses[gids[i][0]].R.transpose();

		// calcurate minimum distance between grasp points
		for (size_t j = 0; j < gids[i].size(); j++) {
			Vector3 t1 = relR * grasp_poses[gids[i][j]].p;
			t1(1) = 0.0;
			double min_len = DBL_MAX;
			for (size_t k = 0; k < gids[i].size(); k++) {
				if (j == k) continue;
				Vector3 t2 = relR * grasp_poses[gids[i][k]].p;
				t2(1) = 0.0;
				if (norm2(t1 - t2) < min_len) {
					min_len = norm2(t1 - t2);
				}
			}
			min_dist += min_len;
		}
		min_dist /= gids[i].size();

		for (size_t j = 0; j < gids[i].size() - 1; j++) {

			// move object
			if (!is_move_object){
				Vector3 target_p;
				Matrix3 target_R;
				if (is_grasp) {
					target_p = Po_ini + Ro_ini * grasp_poses[gids[i][j]].p;
					target_R = Ro_ini * grasp_poses[gids[i][j]].R;
				} else {
					target_p = Po_des[pid] + Ro_des[pid] * grasp_poses[gids[i][j]].p;
					target_R = Ro_des[pid] * grasp_poses[gids[i][j]].R;
				}

				if(!arm_g->IK_arm(target_p, target_R)) continue;

				tc->targetObject->object->p() = palm()->p() - palm()->R() * grasp_poses[gids[i][j]].R.transpose() * grasp_poses[gids[i][j]].p;
				tc->targetObject->object->R() = palm()->R() * grasp_poses[gids[i][j]].R.transpose();
				is_move_object = true;
			}

			Vector3 obj_p = tc->targetObject->object->p();
			Matrix3 obj_R = tc->targetObject->object->R();

			std::vector<cnoid::Vector3> col_p1;
			if (is_grasp) {
				col_p1 = grasp_result[gids[i][j]].grasp.col_p;
			} else {
				col_p1 = place_result[gids[i][j]][pid].place.col_p;
			}
			if (col_p1.empty()) {
				continue;
			}

			Vector3 pp1 = grasp_poses[gids[i][j]].p;
			Vector3 t1 = relR * pp1;
			t1(1) = 0.0;

			for (size_t k = j + 1; k < gids[i].size(); k++) {
				std::vector<cnoid::Vector3> col_p2;
				if (is_grasp) {
					col_p2 = grasp_result[gids[i][k]].grasp.col_p;
				} else {
					col_p2 = place_result[gids[i][k]][pid].place.col_p;
				}
				if (col_p2.empty()) {
					continue;
				}

				Vector3 pp2 = grasp_poses[gids[i][k]].p;
				Vector3 t2 = relR * pp2;
				t2(1) = 0.0;

				if (norm2(t1 - t2) > 1.8 * min_dist) {
					continue;
				}

				// compute collision point
				double min_len_far = DBL_MAX;
				double min_len_near = DBL_MAX;
				Vector3 col1far, col1near, col2far, col2near;
				for (size_t m = 0; m < col_p1.size(); m++) {
					double len = dot((t1- t2), (relR* (obj_R.transpose() * (col_p1[m] - obj_p)) - t2));
					if (len < min_len_near) {
						min_len_near = len;
						col1near = relR * (obj_R.transpose() * (col_p1[m] - obj_p));
					}
					if (-len < min_len_far) {
						min_len_far = -len;
						col1far = relR * (obj_R.transpose() * (col_p1[m] - obj_p));
					}
				}

				min_len_far = DBL_MAX;
				min_len_near = DBL_MAX;
				for (size_t m = 0; m < col_p2.size(); m++) {
					double len = dot((t2 - t1), (relR * (obj_R.transpose() * (col_p2[m] - obj_p)) - t1));
					if (len < min_len_near) {
						min_len_near = len;
						col2near = relR * (obj_R.transpose() * (col_p2[m] - obj_p));
					}
					if (-len < min_len_far) {
						min_len_far = -len;
						col2far = relR * (obj_R.transpose() * (col_p2[m] - obj_p));
					}
				}

				double  dx = norm2(t1-t2);
				double dc1 = dot((t1-t2), ((col1far-col2near))) / dx;
				double dc2 = dot((t1-t2), ((col1near-col2far))) / dx;
				double d1 = dot((t1-t2), ((col1far-col1near))) / dx;
				double d2 = dot((t1-t2), ((col2near-col2far))) / dx;

				double delta = 0.001;
				vector<double> candidate_ratio;


				if (!((dc1 - dx > delta) || (dc2 - dx > delta))) continue;
				if ((d1) > (dx - d2)) continue;

				candidate_ratio.push_back(((dx-d2+d1)*0.5)/ dx);
				const int num_div = 2;
				for (int d = 0; d < num_div; d++) {
					candidate_ratio.push_back((d1 + (dx- d2-d1)*((double)(num_div+(d+1))/(double)(num_div*2)) )/ dx);
					candidate_ratio.push_back((d1 + (dx- d2-d1)*((double)(num_div-(d+1))/(double)(num_div*2)) )/ dx);
				}

				for (size_t r = 0; r < candidate_ratio.size(); r++) {
					GraspDatabaseManipulator::GraspPosture pose;
					pose.p  = (1.0 - candidate_ratio[r]) * pp1 + candidate_ratio[r] * pp2;
					pose.R = grasp_poses[gids[i][j]].R;

					Vector3 target_p;
					Matrix3 target_R;
					if (is_grasp) {
						target_p = Po_ini + Ro_ini * pose.p;
						target_R = Ro_ini * pose.R;
					} else {
						target_p = Po_des[pid] + Ro_des[pid] * pose.p;
						target_R = Ro_des[pid] * pose.R;
					}

					arm_g->IK_arm(target_p, target_R);

					int l = 0;
					for (int m = 0; m < arm_g_n_fing; m++) {
						for (int n = 0; n < fingers_g[m]->fing_path->numJoints(); n++) {
							fingers_g[m]->joint(n)->q() = grasp_poses[gids[i][j]].finger_q[l++];
						}
					}

					bodyItemRobot()->body()->calcForwardKinematics();

					int cnt = 0;
					vector <Vector3> objPos(nContact), objN(nContact);
					for(int m=0;m<nFing();m++){
						if(!fingers(m)->contactSearch(cnt,2000,&objPos[0],&objN[0])){
							break;
						}
					}

					if(cnt < nContact) continue;

					if(graspingHand==RIGHT){
						tc->setGraspingState(PlanBase::GRASPING);
						tc->setGraspingState2(PlanBase::NOT_GRASPING);
					}
					else{
						tc->setGraspingState2(PlanBase::GRASPING);
						tc->setGraspingState(PlanBase::NOT_GRASPING);
					}
#ifdef OBJ_ENV_CONTACT
					tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);
#endif

					if(tc->isColliding()) continue;

					pose.finger_q = grasp_poses[gids[i][j]].finger_q;
					l = 0;
					for (int m = 0; m < arm_g_n_fing; m++) {
						for (int n = 0; n < fingers_g[m]->fing_path->numJoints(); n++) {
							 pose.finger_q[l++] =fingers_g[m]->joint(n)->q();
						}
					}
					pose.q = (1.0 - candidate_ratio[r]) * grasp_poses[gids[i][j]].q + candidate_ratio[r] * grasp_poses[gids[i][k]].q;

					inserted_poses.push_back(pose);

					break;
				}
			}
		}
	}
	tc->targetObject->object->p() = orig_obj_p;
	tc->targetObject->object->R() = orig_obj_R;
}

void ManipFailureAnalyzer::pickAndPlaceTest(const GraspDatabaseManipulator::GraspPoses& poses, GraspDatabaseManipulator::GraspPoses& feasible_poses) {
	int nj = arm_g->arm_path->numJoints()-1;

	for (size_t i = 0; i < poses.size(); i++) {
		Vector3 target_p = Po_ini + Ro_ini * poses[i].p;
		Matrix3 target_R = Ro_ini * poses[i].R;

		for(size_t env = 0; env < Po_des.size(); env++){
			Vector3 des_p = Po_des[env] + Ro_des[env] * poses[i].p;
			Matrix3 des_R = Ro_des[env] * poses[i].R;

			Matrix3 rel_R_target = target_R * arm_g->arm_path->joint(nj)->Rs();
      Matrix3 rel_R_des = des_R * arm_p->arm_path->joint(nj)->Rs();
			Vector3 eco1(0.0, 1.0, 0.0);
			Vector3 app_vec =  target_R * GRCmax.R * eco1;

			int k=0;
			for (int m = 0; m < arm_g_n_fing; m++) {
				for (int n = 0; n < fingers_g[m]->fing_path->numJoints(); n++) {
					fingers_g[m]->fing_path->joint(n)->q() = poses[i].finger_q[k++];
				}
			}

			bodyItemRobot()->body()->calcForwardKinematics();

			if(calcJointSeq(target_p,rel_R_target , des_p, rel_R_des, app_vec)){
				feasible_poses.push_back(poses[i]);
				break;
			}
		}
	}
}

bool ManipFailureAnalyzer::sortPutPos() {
	bool ret = false;
	vector<bool> is_put_success(Po_des.size());
	vector<bool> is_pickandplace_success(Po_des.size());

	for (size_t pid = 0; pid < Po_des.size(); pid++){
		is_put_success[pid] = false;
		is_pickandplace_success[pid] = false;
		for (size_t gid = 0; gid < grasp_result.size(); gid++) {
			if (place_result[gid][pid].approach.result == SUCCESS &&
				place_result[gid][pid].place.result == SUCCESS &&
				place_result[gid][pid].release.result == SUCCESS) {
					is_put_success[pid] = true;
					if(grasp_result[gid].approach.result == SUCCESS &&
						grasp_result[gid].grasp.result == SUCCESS &&
						grasp_result[gid].lift.result == SUCCESS &&
						grasp_result[gid].pregrasp.result == SUCCESS) {
							is_pickandplace_success[pid] = true;
							ret = true;
							break;
					}
			}
		}
	}

	vector<Vector3> Po_tmp1, Po_tmp2, Po_tmp3;
	vector<Matrix3> Ro_tmp1, Ro_tmp2, Ro_tmp3;

	for (size_t pid = 0; pid < Po_des.size(); pid++) {
		if (is_pickandplace_success[pid]) {
			Po_tmp1.push_back(Po_des[pid]);
			Ro_tmp1.push_back(Ro_des[pid]);
		} else if (is_put_success[pid]) {
			Po_tmp2.push_back(Po_des[pid]);
			Ro_tmp2.push_back(Ro_des[pid]);
		} else {
			Po_tmp3.push_back(Po_des[pid]);
			Ro_tmp3.push_back(Ro_des[pid]);
		}
	}
	Po_des.clear();
	Ro_des.clear();
	Po_des.insert(Po_des.end(), Po_tmp1.begin(), Po_tmp1.end());
	Ro_des.insert(Ro_des.end(), Ro_tmp1.begin(), Ro_tmp1.end());
	Po_des.insert(Po_des.end(), Po_tmp2.begin(), Po_tmp2.end());
	Ro_des.insert(Ro_des.end(), Ro_tmp2.begin(), Ro_tmp2.end());
	Po_des.insert(Po_des.end(), Po_tmp3.begin(), Po_tmp3.end());
	Ro_des.insert(Ro_des.end(), Ro_tmp3.begin(), Ro_tmp3.end());

	return ret;
}


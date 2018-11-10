// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "ManipController.h"
#include "IKParallerizer.h"
#include "SweptVolume.h"
#include "PenaltyFunction.h"

#include <boost/lexical_cast.hpp>

#include <cnoid/RootItem>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

#define m_pi 3.141592
//#define DEBUG_MODE
//#define PUT_PATTERN
#define OBJ_ENV_CONTACT
//#define FINGER_OPEN_OFFSET_MODE

// for measuring time
// #define NOOUTPUT_TIMELOG
#include "../GraspDataGen/Util/StopWatch.h"

#define THREAD

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

double drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
	}

ManipController::ManipController()  : 	os (MessageView::mainInstance()->cout() ) {
		initPalm = true;
		intention = 0;
		envItem = NULL;
		strategy = NOT_SELECTED;
		rb = NULL;
		pf = NULL;
		PlacePlanner::instance()->putPos.Index = 0;
}

ManipController::~ManipController() {
}

ManipController* ManipController::instance() {
		static ManipController* instance = new ManipController();
		return instance;
}

bool ManipController::initial(TargetObject* targetObject, ArmFingers* targetArmFinger)
{
		if (rb != NULL) delete rb;
		if (pf != NULL) delete pf;
		rb = new RobotLocalFunctions();
		pf = new ParameterFileData();

		if( grasp::GraspController::initial(targetObject, targetArmFinger) == false) return false;

		chooseArmToGrasp();
		if(arm_g_n_fing>0)
				offset_g = fingers_g[0]->fingerCloseOffset;

		if(initPalm){
                Pinit =  arm_g->arm_path->joint(arm_g->arm_path->numJoints()-1)->p();
				initPalm=false;
		}

		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		if(tc->jointSeq.size()>0){
				jointSeq_ini = tc->jointSeq.front();
				tc->jointSeq.clear();
		}
		else{
				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
                        jointSeq_ini(i) = bodyItemRobot()->body()->joint(i)->q();
		}
		tc->jointSeq.push_back(jointSeq_ini);

		pf->readObjClusterParameters();

		for(list<BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it != tc->bodyItemEnv.end(); ++it)
				if(PlanBase::instance()->objPressName == (*it)->body()->name())
						envItem = *it;

		string filename = CHOREONOID_TOP_PATH + "extplugin/graspPlugin/ObjectPlacePlanner/data/data_put.mat";
		struct stat st;
		Vector3 pressPos = Vector3::Zero();

		if(envItem != NULL || stat(filename.c_str(), &st)==0 )
				putObject=true;
		else
				putObject=false;

#ifdef DEBUG_MODE
		if (!fout.is_open())
			fout.open("ManipController.log");
		if(envItem != NULL)                 fout << "Env item is set to place an object." << endl;
		if(stat(filename.c_str(), &st)==0)  fout << "Found data_put.mat to place an object." << endl;
		if(arm_g->armFinalPose.size()>0)    fout << "armFinalPose is set to place an object." << endl;
#endif

		if(envItem != NULL)
                pressPos = envItem->body()->link(0)->p() + envItem->body()->link(0)->attitude()*(PlanBase::instance()->objPressPos);

		if(putObject){
				PlacePlanner::instance()->clusterIncludeTest = pf->doInclusionTest;
				PlacePlanner::instance()->stabilityTest = pf->doStabilityTest;
				PlacePlanner::instance()->collisionTest = pf->doCollisionTest;
				PlacePlanner::instance()->colTestMargin = pf->colTestMargin;
				PlacePlanner::instance()->calcPutPos(pressPos, objVisRot(), Po_put, Ro_put);
#ifdef DEBUG_MODE
				fout << "Object place mode ( Inclusion Test:" << pf->doInclusionTest << ", Stability Test:" << pf->doStabilityTest << ", Collision Test:" << pf->doCollisionTest << ")" << endl;
				fout << "Collision Test Margin:" << pf->colTestMargin << endl;
				fout << "Number of generated object putting posture: " << Po_put.size() << endl;
#endif
		}

		if(putObject || arm_g->armFinalPose.size()>0 ){
				chooseArmToPut();
				if(arm_p_n_fing>0)
						offset_p = fingers_p[0]->fingerCloseOffset;
				selectManipStrategy();
		}

		pf->calcIDPair(intention);

		if     (bodyItemRobot()->body()->name() == "PA10" ||
			 bodyItemRobot()->body()->name() == "PA10_VVV" ) robot=PA10;
		else if(bodyItemRobot()->body()->name() == "HIRO"      ) robot = HIRO;
		else if(bodyItemRobot()->body()->name() == "HRP2"      ) robot = HRP2;
		else                                                     robot = OTHER;

		firstPick = true;

		os << "Environments: ";
		for(list<BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it != tc->bodyItemEnv.end(); ++it)
				os << (*it)->body()->name() << " ";	os << endl;
		os << "Object: " << targetObject->bodyItemObject->body()->name() << endl;

		lift_height=pf->liftHeight;
		release_height=pf->releaseHeight;
		min_lift_height = 0.1 * lift_height;
		min_relase_height = 0.1 * release_height;

		return true;
}

void ManipController::selectManipStrategy()
{
		strategy = RIGHT_RIGHT;

		if     (tc->arm(0)==arm_g && tc->arm(0)==arm_p) strategy = RIGHT_RIGHT;
		else if(tc->arm(1)==arm_g && tc->arm(1)==arm_p) strategy = LEFT_LEFT;
		else if(tc->arm(0)==arm_g && tc->arm(1)==arm_p) strategy = RIGHT_LEFT;
		else if(tc->arm(1)==arm_g && tc->arm(0)==arm_p) strategy = LEFT_RIGHT;
}

void ManipController::setGraspArm(int j)
{
	arm_g = tc->arm(j);
	arm_g_n_fing = tc->nFing(j);
	for(int i=0;i<arm_g_n_fing;i++)
			fingers_g[i] = tc->fingers(j,i);
}

void ManipController::chooseArmToGrasp()
{
        Vector3 P = bodyItemRobot()->body()->link(0)->attitude().transpose()*(objVisPos() - bodyItemRobot()->body()->link(0)->p());

		if(strategy == NOT_SELECTED){
				if(tc->armsList.size() < 2 || tc->arm(1)==NULL || P(1) <0.05 )
						setGraspArm(0);
				else
						setGraspArm(1);
		}
		else if(strategy == RIGHT_RIGHT || strategy == RIGHT_LEFT)
			setGraspArm(0);
		else
			setGraspArm(1);

		for(int i=0;i<arm_g_n_fing;i++)
			fingers_g[i]->fingerGraspPose.resize(fingers_g[i]->fing_path->numJoints());
}

void ManipController::setPutArm(int j)
{
	arm_p = tc->arm(j);
	arm_p_n_fing = tc->nFing(j);
	for(int i=0;i<arm_p_n_fing;i++)
			fingers_p[i] = tc->fingers(j,i);
}

void ManipController::chooseArmToPut()
{
		Vector3 P = Vector3::Zero();
		if(Po_put.size()>0)
                P = bodyItemRobot()->body()->link(0)->attitude().transpose()*(Po_put[0] - bodyItemRobot()->body()->link(0)->p());

		if(strategy == NOT_SELECTED){
				if(tc->armsList.size() < 2 || tc->arm(1)==NULL || P(1) <0 )
						setPutArm(0);
				else
						setPutArm(1);
		}
		else if(strategy == RIGHT_RIGHT || strategy == LEFT_RIGHT)
			setPutArm(0);
		else
			setPutArm(1);

		for(int i=0;i<arm_p_n_fing;i++){
			fingers_p[i]->fingerGraspPose.resize(fingers_p[i]->fing_path->numJoints());
		}
}

bool ManipController::withinJointLimit(){

		for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
				if (bodyItemRobot()->body()->joint(i)->q_upper() < bodyItemRobot()->body()->joint(i)->q()  ||  bodyItemRobot()->body()->joint(i)->q_lower() > bodyItemRobot()->body()->joint(i)->q()){
#ifdef DEBUG_MODE
						fout << "Limit over: " << i << "-th joint" << endl;
#endif
						return false;
				}

		return true;
}

void ManipController::setTargetObjPos(Vector3& P_ini, Matrix3& R_ini, vector<Vector3>& P_des, vector<Matrix3>& R_des, int j){

		P_des.clear();
		R_des.clear();

		if(strategy==RIGHT_RIGHT || strategy==LEFT_LEFT || strategy==RIGHT_LEFT || strategy==LEFT_RIGHT ){
				P_ini = objVisPos();
				R_ini = objVisRot();
				for(unsigned int i=0; i<Po_put.size(); i++){
						P_des.push_back( Po_put[i] );
						R_des.push_back( Ro_put[i] );
				}
		}
		else if(firstPick && (strategy==RIGHT_PUT_LEFT || strategy==LEFT_PUT_RIGHT)){
				P_ini = objVisPos();
				R_ini = objVisRot();
				P_des.push_back( Po_tmp[j] );
				R_des.push_back( Ro_tmp[j] );
		}
		else if(!firstPick && (strategy==RIGHT_PUT_LEFT || strategy==LEFT_PUT_RIGHT)){
				P_ini = Po_tmp[j];
				R_ini = Ro_tmp[j];
				for(unsigned int i=0; i<Po_put.size(); i++){
						P_des.push_back( Po_put[i] );
						R_des.push_back( Ro_put[i] );
				}
		}

		if(strategy==RIGHT_RIGHT || (firstPick && (strategy==RIGHT_PUT_LEFT || strategy == RIGHT_LEFT)) || (!firstPick && (strategy==LEFT_PUT_RIGHT || strategy == LEFT_RIGHT)) )
				graspingHand = RIGHT;
		else if(strategy==LEFT_LEFT || (!firstPick && (strategy==RIGHT_PUT_LEFT || strategy == RIGHT_LEFT)) || (firstPick && (strategy==LEFT_PUT_RIGHT || strategy == LEFT_RIGHT)) )
				graspingHand = LEFT;


		//Heuristic Rule (to replace)
		if(strategy == RIGHT_LEFT){
				Po_tmp.clear();
				Ro_tmp.clear();
				Vector3 rpyObj = rpyFromRot(R_ini);
				if((fabs(rpyObj(0))<0.001 || fabs(rpyObj(0)-m_pi)<0.001 || fabs(rpyObj(0)+m_pi)<0.001) && (fabs(rpyObj(1))<0.001 || fabs(rpyObj(1)-m_pi)<0.001 || fabs(rpyObj(1)+m_pi)<0.001 )){
                        Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p()(0)+0.4, arm_g->arm_path->joint(0)->p()(1), arm_g->arm_path->joint(1)->p()(2)) );
						Ro_tmp.push_back( rotFromRpy(M_PI/2.0, 0, 0)*R_ini );
				}
				else{
						Vector3 rpyObj2 = rpyFromRot(rotFromRpy(M_PI/2.0, 0, 0)*R_ini);
                        Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p()(0)+0.43, arm_g->arm_path->joint(0)->p()(1), arm_g->arm_path->joint(1)->p()(2)-0.23) );
						Ro_tmp.push_back( rotFromRpy(M_PI, 0, rpyObj2(2)));
				}

		}
		else if(strategy == LEFT_RIGHT){
				Po_tmp.clear();
				Ro_tmp.clear();
				Vector3 rpyObj = rpyFromRot(R_ini);
				if((fabs(rpyObj(0))<0.001 || fabs(rpyObj(0)-m_pi)<0.001 || fabs(rpyObj(0)+m_pi)<0.001) && (fabs(rpyObj(1))<0.001 || fabs(rpyObj(1)-m_pi)<0.001 || fabs(rpyObj(1)+m_pi)<0.001 )){
                        Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p()(0)+0.4, arm_g->arm_path->joint(0)->p()(1), arm_g->arm_path->joint(1)->p()(2)) );
						Ro_tmp.push_back( rotFromRpy(-M_PI/2.0, 0, 0)*R_ini );
				}
				else{
						Vector3 rpyObj2 = rpyFromRot(rotFromRpy(-M_PI/2.0, 0, 0)*R_ini);
                        Po_tmp.push_back( Vector3(arm_g->arm_path->joint(0)->p()(0)+0.43, arm_g->arm_path->joint(0)->p()(1), arm_g->arm_path->joint(1)->p()(2)-0.23) );
						Ro_tmp.push_back( rotFromRpy(M_PI, 0, rpyObj2(2)));
				}
		}
}

void ManipController::
calcFingerOpenPose(const Vector3& Pco1, const Vector3& Pco2, const Vector3& Nco1, const Vector3& Nco2, const Matrix3& Rp_grasp, const Vector3& pPcr1, const Matrix3& pRcr1, double margin)
{
	int nJ=0;
	for(int i=0; i<arm_g_n_fing; i++)
		nJ += fingers_g[i]->fing_path->numJoints();

	VectorXd th(nJ);
	Vector3 P;

	Vector3 P1 = Pco1 + margin*Nco1;
	Vector3 P2 = Pco2 + margin*Nco2;

	arm_g->getPalmPos(P1, P2, Rp_grasp, pPcr1, pRcr1, P, th);

	int k=0;
	for(int i=0; i<arm_g_n_fing; i++)
			for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
					fingers_g[i]->fingerOpenPose[j] = th(k++);

}

bool ManipController::getPosture(string data,Vector3& p,Matrix3& R,VectorXd& finger_angle){
	stringstream is(data);
	vector<double> x;
	double tmp;
	while(!is.eof()){
			is >> tmp;
			x.push_back(tmp);
	};
	if(x.size() < 13) return false;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			R(i,j) = x[i*3+j+1];
		}
	}
	for(int i=0;i<3;i++)
		p(i) = x[10+i];

	finger_angle.resize(x.size()-13);

	int k=0;
	for(int i=13;i<x.size();i++)
		finger_angle(k++) = x[i];

	return true;
}

void ManipController::displaySweptVolume(int sol_id) {
		double len = 0.1;
		ManipController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);

		PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
		settingPrehension(target_prehension);

		SweptVolume sv;
		sv.setGRCR(GRCmax.R);
		sv.setNumGraspStep(3);
		sv.setAppLength(len);
		sv.makeSweptVolume();


		ObjPoseEstimateSolHolder* opes = ObjPoseEstimateSolHolder::instance();
		Vector3 app_vec = tc->palm()->R() * GRCmax.R * Vector3(0, len, 0);
		if (!opes->empty()) {
				SweptVolumeChecker svc(&sv);
				svc.setObjName(tc->targetObject->name());
				svc.setMargin(0.005);
				svc.setObjPoseEstimateSol(&(opes->at(sol_id)));

				svc.check(tc->palm()->p()-app_vec, tc->palm()->R());
				SweptVolumeDrawer sv_drawer(&sv);

				std::vector<int> indices;
				svc.getInlierIndices(indices);

				sv_drawer.addPointsPtr(&(tc->pointCloudEnv.front()->p()), &indices);
				sv_drawer.clear();
				sv_drawer.draw(-app_vec);
		} else {
				SweptVolumeDrawer sv_drawer(&sv);
				sv_drawer.draw(-app_vec);
		}
}

void ManipController::clearSweptVolume() {
		SweptVolumeDrawer sv_drawer(NULL);
		sv_drawer.clear();
}

bool ManipController::searchPickAndPlaceMotionFromDB(Vector3& Pp_grasp2, Matrix3& Rp_grasp2, Vector3& Pp_put2, Matrix3& Rp_put2, dvector& th_grasp2, bool edge_grasp,string& db_path){

		bool ret = false;
		clock_t start,end;
		start = clock();

		Vector3 Po_ini;
		Matrix3 Ro_ini;
		vector<Vector3> Po_des;
		vector<Matrix3> Ro_des;

		setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

		PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
		settingPrehension(target_prehension);

		GraspDatabaseManipulator* gdm_grasp = new GraspDatabaseManipulator();
		gdm_grasp->readFile(db_path);
		GraspDatabaseManipulator::GraspPoses g_poses = gdm_grasp->getRecords();
		delete gdm_grasp;

#ifdef DEBUG_MODE
		fout << "searchPickAndPlaceMotionFromDB" << endl;
		fout << "Database file name: " << db_path << endl;
		fout << "Database size: " << g_poses.size() << endl;
#endif

		int nj = arm_g->arm_path->numJoints()-1;
		VectorXd finger_angle;

#ifdef DEBUG_MODE
		fout << "Checking grasping pose.." << endl;
#endif
		if(arm_g->armFinalPose.size()>0){
			for(size_t m=0; m<g_poses.size(); m++){
				for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
						bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
				bodyItemRobot()->body()->calcForwardKinematics();

				Vector3 Pp_grasp = Po_ini + Ro_ini * g_poses[m].p;
				Matrix3 Rp_grasp = Ro_ini * g_poses[m].R;

				if(!arm_g->IK_arm(Pp_grasp,Rp_grasp)){
#ifdef DEBUG_MODE
						fout << "IK(grasping posture) not solvable in " << m << "-th grasp data" << endl;
#endif
						continue;
				}
				finger_angle = g_poses[m].finger_q;
				int k=0;
				for(int i=0; i<arm_g_n_fing; i++){
						for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
								double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
								finger_angle(k) += offset;
								fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
						}
				}
				bodyItemRobot()->body()->calcForwardKinematics();
				bodyItemRobot()->notifyKinematicStateChange();

				Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
				Vector3 app_vec =  Rp_grasp * GRCmax.R * Vector3(0,1,0);

				if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
						fout << "Collision(grasping posture) in " << m << "-th grasp data" << endl;
#endif
						continue;
				}
#ifdef DEBUG_MODE
						fout << "arm Final pose found"<< endl;
#endif
						for(size_t i=0; i<arm_g->armFinalPose.size(); i++)
								arm_g->arm_path->joint(i)->q() = arm_g->armFinalPose[i];
						arm_g->arm_path->calcForwardKinematics();

						if(calcJointSeq2(Pp_grasp,Rp_grasp_rel , app_vec)){
#ifdef DEBUG_MODE
								fout << "Found solution in " << m << "-th grasp data" << endl;
#endif
					Pp_grasp2 = Pp_grasp;
					Rp_grasp2 = Rp_grasp_rel;
					th_grasp2 = finger_angle;
					approachVec = app_vec;
					tc->flush();
					ret = true;
					break;
				}
			}
		} else if(putObject) {
			enum GRASPABLE {NOT_CHECK, FEASIBLE, NOT_FEASIBLE};
			bool dist = false;
			vector<int> graspable(g_poses.size(), NOT_CHECK);
			for(unsigned int env=0;env<Po_des.size(); env++){
				for(size_t m=0; m<g_poses.size(); m++){
					// check grasping pose
					if (graspable[m] == NOT_CHECK) {
						for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
						bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
						bodyItemRobot()->body()->calcForwardKinematics();

						Vector3 Pp_grasp = Po_ini + Ro_ini * g_poses[m].p;
						Matrix3 Rp_grasp = Ro_ini * g_poses[m].R;

						if(!arm_g->IK_arm(Pp_grasp,Rp_grasp)){
#ifdef DEBUG_MODE
								fout << "IK(grasping posture) not solvable in " << m << "-th grasp data" << endl;
#endif
								graspable[m] = NOT_FEASIBLE;
								continue;
						}
						finger_angle = g_poses[m].finger_q;
						int k=0;
						for(int i=0; i<arm_g_n_fing; i++){
								for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
										double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
										finger_angle(k) += offset;
										fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
								}
						}
						bodyItemRobot()->body()->calcForwardKinematics();
						bodyItemRobot()->notifyKinematicStateChange();

						if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
		#ifdef DEBUG_MODE
								fout << "Collision(grasping posture) in " << m << "-th grasp data" << endl;
		#endif
								graspable[m] = NOT_FEASIBLE;
								continue;
						}
						graspable[m] = FEASIBLE;
					} else if (graspable[m] == NOT_FEASIBLE) {
						continue;
					}

					// check putting pose
					for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
						bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
					bodyItemRobot()->body()->calcForwardKinematics();

					Vector3 Pp_put = Po_des[env] + Ro_des[env] * g_poses[m].p;
					Matrix3 Rp_put = Ro_des[env] * g_poses[m].R;

					if(!arm_p->IK_arm(Pp_put,Rp_put)){
#ifdef DEBUG_MODE
							fout << "IK (putting posture) not solvable in " << m << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
							continue;
					}
					finger_angle = g_poses[m].finger_q;
					int k=0;
					for(int i=0; i<arm_g_n_fing; i++){
							for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
									fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
							}
					}

					Vector3 Pp_grasp = Po_ini + Ro_ini * g_poses[m].p;
					Matrix3 Rp_grasp = Ro_ini * g_poses[m].R;
					Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
					Vector3 app_vec =  Rp_grasp * GRCmax.R * Vector3(0,1,0);
					bodyItemRobot()->body()->calcForwardKinematics();
					if(!isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) && withinJointLimit()){

							dist=true;

							Matrix3 Rp_put_rel = Rp_put * arm_p->arm_path->joint(nj)->Rs();

							if(!calcJointSeq(Pp_grasp,Rp_grasp_rel , Pp_put, Rp_put_rel, app_vec)){
									dist = false;
									continue;
							}
#ifdef DEBUG_MODE
										fout << "Found solution in " << m << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif

										Pp_grasp2 = Pp_grasp;
										Rp_grasp2 = Rp_grasp_rel;
										Pp_put2 = Pp_put;
										Rp_put2 = Rp_put_rel;
										th_grasp2 = finger_angle;
										approachVec = app_vec;
										tc->flush();
										break;
								}
								else{
									tc->flush();
#ifdef DEBUG_MODE
									fout << "Collision(putting posture) in " << m << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
					}
				}
				if(!dist) continue;
				ret = true;
				break;
			}
		}

		if(ret){
				arm_g->IK_arm(Pp_grasp2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp2));

				int k=0;
				for(int i=0; i<arm_g_n_fing; i++)
						for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
                                fingers_g[i]->fing_path->joint(j)->q() = th_grasp2(k++);

				bodyItemRobot()->body()->calcForwardKinematics();
		} else {
				for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
					bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
				}
				bodyItemRobot()->body()->calcForwardKinematics();
		}
		end = clock();
		cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
		tc->flush();

		 return ret;
}

bool ManipController::searchPickAndPlaceMotion(Vector3& Pp_grasp2, Matrix3& Rp_grasp2, Vector3& Pp_put2, Matrix3& Rp_put2, dvector& th_grasp2, bool edge_grasp)
{
                clock_t start,end;
                start = clock();

		int nj = arm_g->arm_path->numJoints()-1;
		bool ret=false;

		int numG=0;
		if(edge_grasp)
				numG=1;

		double Quality_o=1.0e+100;

		Vector3 Po_ini, Pp_grasp, Pp_put;
		Matrix3 Ro_ini, Rp_grasp, Rp_put;
		vector<Vector3> Po_des;
		vector<Matrix3> Ro_des;

		setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

		for(unsigned int id=0; id<pf->idPairs.size(); id++){

				int id0 = pf->idPairs[id][0];
				int id1 = pf->idPairs[id][1];

				for(unsigned int c0=0; c0<pf->objClusters[id0].controlPoints.size(); c0++){
				for(unsigned int c1=0; c1<pf->objClusters[id1].controlPoints.size(); c1++){

						Vector3 oPco1 = pf->objClusters[id0].controlPoints[c0];
						Vector3 oPco2 = pf->objClusters[id1].controlPoints[c1];
						Vector3 oNco1 = pf->objClusters[id0].normal;
						Vector3 oNco2 = pf->objClusters[id1].normal;

						Vector3 pPcr1 = pf->handClusters[0].controlPoints[numG] ;
						Vector3 pNcr1 = pf->handClusters[0].normal;
						Vector3 pTcr1 = pf->handClusters[0].approachVec[0];
						Matrix3 pRcr1 = v3(pNcr1, pTcr1, cross(pNcr1,pTcr1) );

						if(norm2(normalPoint(oPco2, oPco1, oNco1)-oPco2)>1.0e-3) continue;

						for(unsigned int ap=0; ap<pf->objClusters[id0].approachVec.size(); ap++){

								Vector3 oTco1 =  pf->objClusters[id0].approachVec[ap];
								Vector3 oTco2 =  pf->objClusters[id1].approachVec[ap];

								if(norm2(oTco1)<1.0e-3) continue;
								if(norm2(oTco2)<1.0e-3) continue;
								if(!included(ap, pf->graspPostureCandidate)) continue;
#ifdef DEBUG_MODE
								fout << "Trying. (Cluster= " << id0 << "/" << id1 << ", ControlPoint= " << c0 << "/" << c1 << ", Approach= " << ap << ")" << endl;
#endif

								Matrix3 oRco1 = v3(oNco1, oTco1, cross(oNco1,oTco1) );
								Matrix3 oRco2 = v3(oNco2, oTco2, cross(oNco2,oTco2) );

								Vector3 eco1 =  Ro_ini*oTco1;
								Vector3 ncr1 = -Ro_ini*oNco1;
								Matrix3 R_psi = rodrigues(eco1, m_pi);

								Rp_grasp = R_psi*Ro_ini*oRco1*trans(pRcr1);

								Vector3 Pco1 = Po_ini + Ro_ini*oPco1;
								Vector3 Pco2 = Po_ini + Ro_ini*oPco2;

								Vector3 nco1 = -ncr1;
								Pco2 = Pco1 + nco1 * dot(nco1, Pco2-Pco1);

								int nJ=0;
								for(int i=0; i<arm_g_n_fing; i++)
									nJ += fingers_g[i]->fing_path->numJoints();

								VectorXd th_grasp(nJ);
								VectorXd th_put(nJ);

								arm_g->getPalmPos(Pco1, Pco2, Rp_grasp, pPcr1, pRcr1, Pp_grasp, th_grasp, offset_g);

								//if( ! arm_p->IK_arm(Pp_p,     arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_p),  0.01) ) continue;
								if( ! arm_g->IK_arm(Pp_grasp, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp)) ){
#ifdef DEBUG_MODE
										fout << "IK(grasping posture) not solvable" << endl;
#endif
										continue;}

								int k=0;
								for(int i=0; i<arm_g_n_fing; i++)
										for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
                                                fingers_g[i]->fing_path->joint(j)->q() = th_grasp(k++);

								double nom = 1.0-fabs(dot(Rp_grasp*Vector3(0.0,0.0,1.0), Vector3(Pp_grasp - Pinit)/norm2(Pp_grasp - Pinit)));

								double Quality = pf->gainParameter[0]*((c0+1) + (c1+1) + id0 + id1) + pf->gainParameter[1]*nom;
								//(c0+1):ctrlPtsP is ordered by the distance from the center. surf?: cluster is ordered by the area.

								if( !( Quality < Quality_o && withinJointLimit() )) continue;

								if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
										fout << "Collision(grasping posture)" << endl;
#endif
										continue;}

								//Collision of putting posture
								if(putObject){

										bool dist = false;
										for(unsigned int env=0; env<Po_des.size(); env++){

												Vector3 eco2 =  Ro_des[env]*oTco1;
												R_psi = rodrigues(eco2, m_pi);
												Rp_put = R_psi*Ro_des[env]*oRco1*trans(pRcr1);

												Pco1 = Po_des[env] + Ro_des[env]*oPco1;
												Pco2 = Po_des[env] + Ro_des[env]*oPco2;

												arm_p->getPalmPos(Pco1, Pco2, Rp_put, pPcr1, pRcr1, Pp_put, th_put, offset_p);
												if( ! arm_p->IK_arm(Pp_put,  arm_p->arm_path->joint(nj)->calcRfromAttitude(Rp_put)) ) continue;

												//k=0;
												//for(int i=0; i<2; i++)
												//		for(int j=0; j<fingers_p[i]->fing_path->numJoints(); j++)
												//				fingers_p[i]->fing_path->joint(j)->q = th_put(k++);

												if(!isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT) && withinJointLimit()){
														dist=true;
														break;
												}
										}

										if(!dist){
#ifdef DEBUG_MODE
												fout << "Collision(putting posture)" << endl;
#endif
												continue;}

										if(!calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put, eco1)) continue;
								}


#ifdef DEBUG_MODE
								fout << "Solution found (Cluster= " << id0 << "/" << id1 << ", ControlPoint= " << c0 << "/" << c1 << ", Approach= " << ap << ")" << endl;
#endif
								Quality_o = Quality;
								Pp_grasp2 = Pp_grasp;
								Rp_grasp2 = Rp_grasp;
								Pp_put2 = Pp_put;
								Rp_put2 = Rp_put;
								th_grasp2 = th_grasp;
								approachVec = eco1;
								ret = true;
								tc->flush();
						}
						//tc->flush();
						//usleep(1000000);
				}
				}
		}

		if(ret){
				arm_g->IK_arm(Pp_grasp2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp2));

				int k=0;
				for(int i=0; i<arm_g_n_fing; i++)
						for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
                                fingers_g[i]->fing_path->joint(j)->q() = th_grasp2(k++);

				bodyItemRobot()->body()->calcForwardKinematics();
		}


		end = clock();
                cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
		tc->flush();

		return ret;
}

bool ManipController::isColliding(int graspingState, int graspingState2, int contactState)
{
		bodyItemRobot()->body()->calcForwardKinematics();
		if(graspingHand==RIGHT){
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

#ifdef DEBUG_MODE
		tc->flush();
		usleep(100000);
#endif

		bool ret = tc->isColliding();

#ifdef DEBUG_MODE
		if(ret)fout << "Collision between " << tc->colPairName[0] << " " << tc->colPairName[1] << endl;
#endif

		if(!ret){
				return false;
		}
		else
				return true;

}

bool ManipController::isColliding2()
{
		bool ret = tc->isColliding();

#ifdef DEBUG_MODE
		if(ret)fout << "Collision between " << tc->colPairName[0] << " " << tc->colPairName[1] << endl;
		tc->flush();
		usleep(100000);
#endif
		return ret;
}


bool ManipController::searchLiftUpKeyPose(const Vector3& P, const Matrix3& R,VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], const Vector3& appV, double time, double* height, double min_height, int graspingState)
{
	VectorXd armJointSeq(arm_->arm_path->numJoints());

	calcArmJointSeq(armJointSeq, arm_);

	arm_->IK_arm(P,  R, armJointSeq);

	if(graspingHand==RIGHT){
			tc->setGraspingState(graspingState);
			tc->setGraspingState2(PlanBase::NOT_GRASPING);
	}
	else{
			tc->setGraspingState2(graspingState);
			tc->setGraspingState(PlanBase::NOT_GRASPING);
	}

#ifdef OBJ_ENV_CONTACT
	if (graspingState == PlanBase::GRASPING) {
			tc->setObjectContactState(PlanBase::OFF_ENVIRONMENT);
	} else {
			tc->setObjectContactState(PlanBase::ON_ENVIRONMENT);
	}
#endif

	bool is_solved = false;
	do{
		if(arm_->IK_arm(P + *height *appV,  R, armJointSeq)){
			tc->calcForwardKinematics();
			if(!isColliding2()){
				is_solved = true;
			}
		}
		if(!is_solved){
			*height *= 0.5;
			if(*height < min_height){*height *= 2; return false;}
		}
	}while(!is_solved);

	setJointAngle(graspingState, jointSeq, arm_, fingers_);
	tc->jointSeq.push_back(jointSeq);


	vector<int> dof;
	for(int j=0;j<tc->armsList.size();j++){
		for(int i=0;i<tc->arm(j)->nJoints;i++)
            dof.push_back(tc->arm(j)->arm_path->joint(i)->jointId());
		}
	tc->pathPlanDOFSeq.push_back(dof);

	setGraspingStateSeq(graspingState, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT);
	tc->motionTimeSeq.push_back(time);

	return true;
}

void ManipController::calcArmJointSeq(VectorXd& armJointSeq, ArmPtr& arm_)
{
		for(int i=0; i<arm_->arm_path->numJoints(); i++)
                armJointSeq(i) = tc->jointSeq.back()(arm_->arm_path->joint(i)->jointId());
}

void ManipController::setGraspingStateSeq(int graspingState, int graspingState2, int contactState)
{
		bodyItemRobot()->body()->calcForwardKinematics();
		if(graspingHand==RIGHT){
				if(tc->graspingStateSeq.size() > 0){
					if(tc->graspingStateSeq.back() == PlanBase::GRASPING){
						tc->armsList[0]->objectPalmPos = tc->objectPalmPosSeq.back();
				 		tc->armsList[0]->objectPalmRot = tc->objectPalmRotSeq.back();
				 		tc->calcForwardKinematics();
					}
				}
				tc->setGraspingState(graspingState);
				tc->setGraspingState2(graspingState2);
				if (graspingState2 == PlanBase::GRASPING) {
					tc->objectPalmPosSeq.push_back(tc->armsList[1]->objectPalmPos);
					tc->objectPalmRotSeq.push_back(tc->armsList[1]->objectPalmRot);
				} else {
					tc->objectPalmPosSeq.push_back(tc->armsList[0]->objectPalmPos);
					tc->objectPalmRotSeq.push_back(tc->armsList[0]->objectPalmRot);
				}
		}
		else{
			  if(tc->graspingStateSeq2.size() > 0){
					if(tc->graspingStateSeq2.back() == PlanBase::GRASPING){
						tc->armsList[1]->objectPalmPos = tc->objectPalmPosSeq.back();
				 		tc->armsList[1]->objectPalmRot = tc->objectPalmRotSeq.back();
				 		tc->calcForwardKinematics();
					}
				}
				tc->setGraspingState2(graspingState);
				tc->setGraspingState(graspingState2);
				if (graspingState2 == PlanBase::GRASPING) {
					tc->objectPalmPosSeq.push_back(tc->armsList[0]->objectPalmPos);
					tc->objectPalmRotSeq.push_back(tc->armsList[0]->objectPalmRot);
				} else {
					tc->objectPalmPosSeq.push_back(tc->armsList[1]->objectPalmPos);
					tc->objectPalmRotSeq.push_back(tc->armsList[1]->objectPalmRot);
				}
		}

		tc->graspingStateSeq.push_back(tc->getGraspingState());
		tc->graspingStateSeq2.push_back(tc->getGraspingState2());

#ifdef OBJ_ENV_CONTACT
		tc->setObjectContactState(contactState);
		tc->objectContactStateSeq.push_back(tc->getObjectContactState());
#endif
}

void ManipController::setJointAngle(int graspingState, VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[],bool fingerCloseUnderGrasp)
{
		int nFing = 2;
		if(arm_ == arm_g){ nFing = arm_g_n_fing;}
		else if(arm_ == arm_p){ nFing = arm_p_n_fing;}

		for(int i=0; i<arm_->arm_path->numJoints(); i++)
                jointSeq(arm_->arm_path->joint(i)->jointId()) = arm_->arm_path->joint(i)->q();

		for(int i=0;i<nFing;i++)
				for(int j=0; j<fingers_[i]->fing_path->numJoints(); j++){
						if(graspingState == PlanBase::GRASPING || (graspingState == PlanBase::UNDER_GRASPING && fingerCloseUnderGrasp)){
                                jointSeq(fingers_[i]->fing_path->joint(j)->jointId()) = fingers_[i]->fingerGraspPose[j];
						}else{
#ifdef FINGER_OPEN_OFFSET_MODE
								double offset = (fingers_[i]->fingerOpenPoseOffset.size() > j) ?  fingers_[i]->fingerOpenPoseOffset[j] : 0;
								double close_offset = (fingers_[i]->fingerCloseOffset.size() > j) ?  fingers_[i]->fingerCloseOffset[j] : 0;
								jointSeq(fingers_[i]->fing_path->joint(j)->jointId()) = fingers_[i]->fingerGraspPose[j] + offset - close_offset;
#else
                                jointSeq(fingers_[i]->fing_path->joint(j)->jointId()) = fingers_[i]->fingerOpenPose[j];
#endif
						}
                        fingers_[i]->fing_path->joint(j)->q() = jointSeq(fingers_[i]->fing_path->joint(j)->jointId());
				}
}

bool ManipController::setMotionSeq(int graspingState, int contactState, const Vector3& P, const Matrix3& R, VectorXd& jointSeq, ArmPtr& arm_, FingerPtr fingers_[], double time,bool fingerCloseUnderGrasp, bool is_fixwaist, double waist_q, int graspingState2)
{
		VectorXd armJointSeq(arm_->arm_path->numJoints());

		calcArmJointSeq(armJointSeq, arm_);

		if (is_fixwaist) {
			if(!arm_->IK_arm(P, R, waist_q, armJointSeq)) return false;
		} else {
			if(!arm_->IK_arm(P,  R, armJointSeq)) return false;
		}

		setJointAngle(graspingState, jointSeq, arm_, fingers_,fingerCloseUnderGrasp);
		tc->jointSeq.push_back(jointSeq);

		if(graspingState == PlanBase::GRASPING){
			vector<int> dof;
			for(int j=0;j<tc->armsList.size();j++){
				for(int i=0;i<tc->arm(j)->nJoints;i++)
					if(!included(tc->arm(j)->arm_path->joint(i)->jointId(), dof))
                        dof.push_back(tc->arm(j)->arm_path->joint(i)->jointId());
			}
			tc->pathPlanDOFSeq.push_back(dof);
		}else{
			tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		}

		setGraspingStateSeq(graspingState, graspingState2, contactState);
		//cout << "motionTimeSeq << " << time << endl;
		tc->motionTimeSeq.push_back(time);

		return true;
}

bool ManipController::setMotionSeqDual(int graspingState, int graspingState2, int contactState, const Vector3& P, const Matrix3& R, const Vector3& P2, const Matrix3& R2, VectorXd& jointSeq, ArmPtr& arm, FingerPtr fingers[], double time,bool fingerCloseUnderGrasp,bool fingerCloseUnderGrasp2, bool is_fixwaist, double waist_q)
{
		ArmPtr arm2;
		if(arm == arm_g)	arm2 = arm_p;
		else			    arm2 = arm_g;

		FingerPtr fingers2[3];

		if(fingers[0] == fingers_g[0]) for(int i=0; i<arm_p_n_fing; i++) fingers2[i] = fingers_p[i];
		else                           for(int i=0; i<arm_g_n_fing; i++) fingers2[i] = fingers_g[i];

		VectorXd armJointSeq(arm->arm_path->numJoints()), armJointSeq2(arm2->arm_path->numJoints());
		calcArmJointSeq(armJointSeq, arm);
		calcArmJointSeq(armJointSeq2, arm2);


		if(is_fixwaist){
			if(!arm->IK_arm(P, R, waist_q, armJointSeq)) return false;
		  if(!arm2->IK_arm(P2, R2, waist_q, armJointSeq2)) return false;
		} else {
			if(!arm->IK_arm(P, R, armJointSeq)) return false;
			if(!arm2->IK_arm(P2, R2, arm->arm_path->joint(0)->q(), armJointSeq2)) return false;
		}

		setJointAngle(graspingState, jointSeq, arm, fingers, fingerCloseUnderGrasp);
		setJointAngle(graspingState2, jointSeq, arm2, fingers2, fingerCloseUnderGrasp2);
		tc->jointSeq.push_back(jointSeq);

		if (graspingState == PlanBase::GRASPING || graspingState2 == PlanBase::GRASPING) {
				vector<int> fing_ids;
				if (graspingState == PlanBase::GRASPING) {
						int fing_n = (fingers[0] == fingers_g[0]) ? arm_g_n_fing : arm_p_n_fing;
						for (int i = 0; i < fing_n; i++) {
								for (int j = 0; j < fingers[i]->nJoints; j++) {
										fing_ids.push_back(fingers[i]->fing_path->joint(j)->jointId());
								}
						}
				}
				if (graspingState2 == PlanBase::GRASPING) {
						int fing_n = (fingers[0] == fingers_g[0]) ? arm_p_n_fing : arm_g_n_fing;
						for (int i = 0; i < fing_n; i++) {
								for (int j = 0; j < fingers2[i]->nJoints; j++) {
										fing_ids.push_back(fingers2[i]->fing_path->joint(j)->jointId());
								}
						}
				}

				vector<int> dof;
				for (size_t i = 0; i < tc->pathPlanDOF.size(); i++) {
						if (!included(tc->pathPlanDOF[i], fing_ids)) {
								dof.push_back(tc->pathPlanDOF[i]);
						}
				}
				tc->pathPlanDOFSeq.push_back(dof);
		} else {
				tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		}

		setGraspingStateSeq(graspingState, graspingState2, contactState);

		tc->motionTimeSeq.push_back(time);

		return true;
}

// This method replaces the last element of pathPlanDOFSeq to the DOF contains only target arm.
// You should call this method before setMotionSeq, if you want to fix the links except target
// arm links in path planning when target arm is different between at previous key pose and
// current key pose.
void ManipController::replaceLastPathDOFSeqOneArm(int graspingState, ArmPtr& arm_, FingerPtr fingers_[])
{
	int nFing = 2;
	if(arm_ == arm_g){ nFing = arm_g_n_fing;}
	else if(arm_ == arm_p){ nFing = arm_p_n_fing;}

	vector<int> dof;
	for(int i=0;i<arm_->nJoints;i++)
		if(!included(arm_->arm_path->joint(i)->jointId(), dof))
      dof.push_back(arm_->arm_path->joint(i)->jointId());

//	if(graspingState != PlanBase::GRASPING){
//		for(int i=0;i<nFing;i++){
//			for(int j=0;j<fingers_[i]->nJoints;j++)
//				if(!included(fingers_[i]->fing_path->joint(j)->jointId(), dof))
//					dof.push_back(fingers_[i]->fing_path->joint(j)->jointId());
//		}
//	}

	tc->pathPlanDOFSeq.back() = dof;
}

void ManipController::clearSequence()
{
	tc->jointSeq.clear();
	tc->graspingStateSeq.clear();
	tc->graspingStateSeq2.clear();
	tc->pathPlanDOFSeq.clear();
	tc->objectContactStateSeq.clear();
	tc->setTrajectoryPlanDOF(graspingHand);
	tc->objectPalmPosSeq.clear();
	tc->objectPalmRotSeq.clear();
	tc->motionTimeSeq.clear();
}

bool ManipController::calcJointSeqTest(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& Pp_put, Matrix3& Rp_put, Vector3& appVec){

#ifdef DEBUG_MODE
				fout << "Calculating joint sequence (calcJointSeq2)" << endl;
#endif

		bool cond = true;
		Vector3 z(0.0, 0.0, 1.0);
		double app_length=pf->appLength;
		double v_margin=pf->appHeight;
		double release_height = pf->releaseHeight;
		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);

		for(int i=0; i<arm_g_n_fing; i++){
				for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
                        fingers_g[i]->fingerGraspPose[j] = fingers_g[i]->joint(j)->q();
		}

		int nj = arm_g->arm_path->numJoints()-1;

		VectorXd jointSeq     = tc->jointSeq[0];
		VectorXd jointSeq_ini = tc->jointSeq[0];

		clearSequence();

		tc->pathPlanDOF.clear();
		for(int j=0;j<tc->armsList.size();j++){
			for(int i=0;i<tc->arm(j)->nJoints;i++)
				if(!included(tc->arm(j)->arm_path->joint(i)->jointId(), tc->pathPlanDOF))
                    tc->pathPlanDOF.push_back(tc->arm(j)->arm_path->joint(i)->jointId());
		}

		//===== Initial Point (already included in armJointSeq)
		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		//==== Mid Point
		if(robot==PA10){
				for(int i=0; i<arm_g->arm_path->numJoints(); i++)
                        arm_g->arm_path->joint(i)->q() = jointSeq(arm_g->arm_path->joint(i)->jointId());
				arm_g->arm_path->calcForwardKinematics();

                Vector3 Pp_mid = (arm_g->arm_path->joint(nj)->p() + Vector3(Pp_grasp - app_length*appVec + v_margin*z))*0.5;
				Matrix3 Att_mid = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp);

				if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_mid, Att_mid, jointSeq, arm_g, fingers_g, mtime[0]))
						return false;
		}

		//==== Approach Point
 		Matrix3 Att_grasp = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp);
 		app_length=pf->appLength;
		if (arm_g_n_fing == 0) {
			cond = searchLiftUpKeyPose(Pp_grasp,Att_grasp,jointSeq,arm_g,fingers_g, -appVec, mtime[0], &app_length,0.01,PlanBase::UNDER_GRASPING);
		} else {
			cond = searchLiftUpKeyPose(Pp_grasp,Att_grasp,jointSeq,arm_g,fingers_g, -appVec, mtime[0], &app_length,0.01,PlanBase::NOT_GRASPING);
		}
		if(!cond){
#ifdef DEBUG_MODE
				fout << "Approach point not solvable" << endl;
#endif
				return false;}

		//==== Grasping Point
		if(!setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(arm_g_n_fing>0 && isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) cond = false;
		if(!setMotionSeq(PlanBase::GRASPING,       PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				fout << "Grasping point not solvable" << endl;
#endif
				return false;}

		//==== LiftUp Point
		Vector3 Pp_lift;
		cond = searchLiftUpKeyPose(Pp_grasp,Att_grasp,jointSeq,arm_g,fingers_g, z, mtime[1], &lift_height,min_lift_height,PlanBase::GRASPING);
		Pp_lift = Pp_grasp+lift_height*z;

		if(!cond){
#ifdef DEBUG_MODE
				fout << "Liftup point not solvable" << endl;
#endif
				return false;}

		//=== Top of release point
		Vector3 Pp_rel;
		Matrix3 Att_put = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_put);
		cond = searchLiftUpKeyPose(Pp_put,Att_put,jointSeq,arm_g,fingers_g, z, mtime[1], &release_height,min_relase_height,PlanBase::GRASPING);
		Pp_rel = Pp_put+release_height*z;
		if(!cond){
#ifdef DEBUG_MODE
				fout << "Top of release point not solvable" << endl;
#endif
				return false;}

		//== Release point
		if(!setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_put, Att_put, jointSeq, arm_g, fingers_g, mtime[1],true)) cond = false;
		if (arm_g_n_fing == 0) {
			if(!setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_put, Att_put, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		} else {
			if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_put, Att_put, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		}
		if(arm_p_n_fing>0 && isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				fout << "Release point not solvable" << endl;
#endif
				return false;}

		//== Return to original position
		if(!setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_rel, Att_put, jointSeq, arm_g, fingers_g, mtime[1])) return false;
		tc->jointSeq.push_back(jointSeq_ini);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//==Just for Graphics
		tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
		arm_g->IK_arm(Pp_grasp, Att_grasp);
		bodyItemRobot()->body()->calcForwardKinematics();
		bodyItemRobot()->notifyKinematicStateChange();

		bool limit = true;
		if(robot==PA10)
				for(unsigned int i=1; i<tc->jointSeq.size(); i++)
						if( tc->jointSeq[i](arm_g->arm_path->joint(4)->jointId())+m_pi > arm_g->arm_path->joint(4)->q_upper())
								limit=false;
		return limit;
}


bool ManipController::calcJointSeqTest2(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& appVec){

#ifdef DEBUG_MODE
				fout << "Calculating joint sequence (calcJointSeq2)" << endl;
#endif

		bool cond = true;
		Vector3 z(0.0, 0.0, 1.0);
		double app_length=pf->appLength, v_margin=pf->appHeight;
		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);

		for(int i=0; i<arm_g_n_fing; i++){
				for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
						fingers_g[i]->fingerGraspPose[j] = fingers_g[i]->joint(j)->q();
		}

		int nj = arm_g->arm_path->numJoints()-1;

		VectorXd jointSeq(bodyItemRobot()->body()->numJoints());
		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		jointSeq     = tc->jointSeq[0];
		jointSeq_ini = tc->jointSeq[0];

		clearSequence();

		tc->pathPlanDOF.clear();
		for(int j=0;j<tc->armsList.size();j++){
			for(int i=0;i<tc->arm(j)->nJoints;i++)
					if(!included(tc->arm(j)->arm_path->joint(i)->jointId(), tc->pathPlanDOF))
							tc->pathPlanDOF.push_back(tc->arm(j)->arm_path->joint(i)->jointId());
		}

		//===== Initial Point (already included in armJointSeq)

		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		//==== Approach Point
		Vector3 Pp_app = Pp_grasp - app_length*appVec + v_margin*z;
 		Matrix3 Att_grasp = arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp);
		if(arm_g_n_fing>0  && !setMotionSeq(PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_app, Att_grasp, jointSeq, arm_g, fingers_g, mtime[0])) cond = false;
		if(arm_g_n_fing==0 && !setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_app, Att_grasp, jointSeq, arm_g, fingers_g, mtime[0])) cond = false;
		if(isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				fout << "Approach point not solvable" << endl;
#endif
				return false;}

		//==== Grasping Point
		if(arm_g_n_fing>0  && !setMotionSeq(PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(arm_g_n_fing==0 && !setMotionSeq(PlanBase::GRASPING, PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(arm_g_n_fing>0 && isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) cond = false;
		if(!setMotionSeq(PlanBase::GRASPING,       PlanBase::ON_ENVIRONMENT, Pp_grasp, Att_grasp, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				fout << "Grasping point not solvable" << endl;
#endif
				return false;}

		//==== LiftUp Point
		Vector3 Pp_lift;
		cond = searchLiftUpKeyPose(Pp_grasp,Att_grasp,jointSeq,arm_g,fingers_g, z, mtime[1], &lift_height,min_lift_height,PlanBase::GRASPING);
		Pp_lift = Pp_grasp+lift_height*z;

		if(!cond){
#ifdef DEBUG_MODE
				fout << "Liftup point not solvable" << endl;
#endif
				return false;}

		//== Return to original position
		VectorXd jointSeq_fin = jointSeq_ini;
		for(int i=0; i<arm_g->arm_path->numJoints(); i++)
				jointSeq_fin(arm_g->arm_path->joint(i)->jointId()) = arm_g->armFinalPose[i];

		tc->jointSeq.push_back(jointSeq_fin);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//==Just for Graphics
		tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
		arm_g->IK_arm(Pp_grasp, Att_grasp);
		bodyItemRobot()->body()->calcForwardKinematics();
		bodyItemRobot()->notifyKinematicStateChange();

		bool limit = true;
		if(robot==PA10)
				for(unsigned int i=1; i<tc->jointSeq.size(); i++)
                        if( tc->jointSeq[i](arm_g->arm_path->joint(4)->jointId())+m_pi > arm_g->arm_path->joint(4)->q_upper())
								limit=false;
		return limit;
}


Matrix3 upright(const Matrix3& R)
{
		if(dot(col(R,2), Vector3(0,0,1))>0)
				return R;
		else
				return v3(col(R,0), -col(R,1), -col(R,2));
}

void ManipController::calcJointSeq(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& Pp_put, Matrix3& Rp_put){

		calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put, approachVec);

		return;
}

bool ManipController::calcJointSeq(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& Pp_put, Matrix3& Rp_put, Vector3& appVec){

		Vector3 orig_obj_p = tc->targetObject->object->p();
		Matrix3 orig_obj_R = tc->targetObject->object->R();

		bool ret = false;
		if(calcJointSeqTest(Pp_grasp, Rp_grasp, Pp_put, Rp_put, appVec))
				ret = true;

		tc->targetObject->object->p() = orig_obj_p;
		tc->targetObject->object->R() = orig_obj_R;

		return ret;
}

bool ManipController::calcJointSeq2(Vector3& Pp_grasp, Matrix3& Rp_grasp, Vector3& appVec){

		Vector3 orig_obj_p = tc->targetObject->object->p();
		Matrix3 orig_obj_R = tc->targetObject->object->R();

		bool ret = false;
		if(calcJointSeqTest2(Pp_grasp, Rp_grasp, appVec))
			ret = true;

		tc->targetObject->object->p() = orig_obj_p;
		tc->targetObject->object->R() = orig_obj_R;

		return ret;
}

bool ManipController::doGraspPlanning()
{

		//os << "Gripper Manipulation"  << endl;
		bool success=true;

		Vector3 Pp_put, Pp_grasp; //Palm putting/grasping position
		Matrix3 Rp_put, Rp_grasp;
		VectorXd theta;

		if(strategy==RIGHT_RIGHT || strategy==LEFT_LEFT) {

#ifdef GRASPPOSTURE_FROM_DB
				string dataFilePath_g = (arm_g == tc->arm(0)) ? tc->dataFilePath(0) : tc->dataFilePath(1);
				string db_path =  dataFilePath_g + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";

				if(searchPickAndPlaceMotionFromDB(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, false,db_path)){
#else
				if(searchPickAndPlaceMotion(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, false)){
#endif
						palmPos = Pp_grasp;
						palmRot = Rp_grasp;

						if(arm_g->armFinalPose.size()>0)
								calcJointSeq2(Pp_grasp, Rp_grasp, approachVec);
						else if(putObject)
								calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put);
				}
				else 	success=false;
		}

		strategy = NOT_SELECTED;

		if(success)
				os << "Success: " << PlacePlanner::instance()->putPos.Index << endl;
		else{
				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
                        bodyItemRobot()->body()->joint(i)->q() = tc->jointSeq[0](i);
				bodyItemRobot()->body()->calcForwardKinematics();
				os << "Fail (grasp plan): " << PlacePlanner::instance()->putPos.Index << endl;
				tc->flush();
				return false;
		}
#endif
		return true;
}


#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
 bool ManipController::doBinPickingGraspPlanning(int obj_id, OccupiedVoxelVec* voxel, bool include_negative)
#else
		 bool ManipController::doBinPickingGraspPlanning(OccupiedVoxelVec* voxel, bool include_negative)
#endif
{
		clock_t start,end;
		start = clock();

		StopWatch timer;
		timer.start();

		tc = PlanBase::instance();

		if ( !tc->targetObject || !tc->targetArmFinger) {
				os << "Please select Grasped Object and Grasping Robot" << endl;
				return false;
		}

		strategy = RIGHT_RIGHT;
		if (tc->armsList.size() >= 2 && tc->armsList[1] == tc->targetArmFinger) {
				strategy = LEFT_LEFT;
		}

		tc->initial();

		initial(tc->targetObject,  tc->targetArmFinger);

		timer.stopAndPrintTime("planning(init)");
		ScopeStopWatch timer2("planning(main_total)");

		Vector3 Pp_put, Pp_grasp; //Palm putting/grasping position
		Matrix3 Rp_put, Rp_grasp;
		VectorXd theta;

		ObjPoseEstimateSolHolder* opes = ObjPoseEstimateSolHolder::instance();
		if (opes->empty()) {
				os << "there no recognized objects" << std::endl;
				return false;
		}
		bool is_ml_mode = PenaltyFunctionUpdater::instance()->isExistModelFile(tc->targetArmFinger->handName, tc->targetObject->name());
#ifdef TRAINING_MODE
		is_ml_mode = true;
#endif
		bool is_succeed;
		if (is_ml_mode) {
				os << "Start learning based bin picking planning...." << std::endl;
#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
				is_succeed = searchBinPicking(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, obj_id, include_negative, voxel);
#else
				is_succeed = searchBinPicking(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, include_negative, voxel);
#endif
		} else {
				os << "Start bin picking planning...." << std::endl;
				is_succeed = searchBinPickingNoML(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta);
		}

		if (is_succeed) {
				palmPos = Pp_grasp;
				palmRot = Rp_grasp;

				tc->targetObject->objVisPos = tc->targetObject->object->p();
				tc->targetObject->objVisRot = tc->targetObject->object->R();
				if(putObject)
						calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put);

				os << "Success: " << endl;
				tc->flush();
				PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
				end = clock();
				os << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
				return true;
		}
		else{
				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
						bodyItemRobot()->body()->joint(i)->q() = tc->jointSeq[0](i);

				bodyItemRobot()->body()->calcForwardKinematics();
				bodyItemRobot()->notifyKinematicStateChange();
				os << "Fail (grasp plan): " << endl;
		}

		tc->flush();
		PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
		end = clock();
		os << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
		return false;
}

bool ManipController::doBinPickingGraspPlanning(Vector3& put_pos)
{
		clock_t start,end;
		start = clock();

		tc = PlanBase::instance();

		if ( !tc->targetObject || !tc->targetArmFinger) {
				os << "Please select Grasped Object and Grasping Robot" << endl;
				return false;
		}

		strategy = RIGHT_RIGHT;
		if (tc->armsList.size() >= 2 && tc->armsList[1] == tc->targetArmFinger) {
				strategy = LEFT_LEFT;
		}

		tc->initial();

		initial(tc->targetObject,  tc->targetArmFinger);
		if (putObject) {
				PlacePlanner::instance()->calcPutPos(put_pos, objVisRot(), Po_put, Ro_put);
		}

		Vector3 Pp_put, Pp_grasp; //Palm putting/grasping position
		Matrix3 Rp_put, Rp_grasp;
		VectorXd theta;
		if(searchBinPicking(Pp_grasp, Rp_grasp, Pp_put, Rp_put, theta, false)){

				palmPos = Pp_grasp;
				palmRot = Rp_grasp;

				if(putObject)
						calcJointSeq(Pp_grasp, Rp_grasp, Pp_put, Rp_put);

				os << "Success: " << endl;
				tc->flush();
				PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
				end = clock();
				os << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
				return true;
		}
		else{
				for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
						bodyItemRobot()->body()->joint(i)->q() = tc->jointSeq[0](i);

				bodyItemRobot()->body()->calcForwardKinematics();
				bodyItemRobot()->notifyKinematicStateChange();
				os << "Fail (grasp plan): " << endl;
		}

		tc->flush();
		PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
		end = clock();
		os << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
		return false;
}

#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
 bool ManipController::searchBinPicking(Vector3& Pp_grasp2, Matrix3& Rp_grasp2, Vector3& Pp_put2, Matrix3& Rp_put2, VectorXd& th_grasp2, int obj_id, bool calc_negative, OccupiedVoxelVec* voxel) {
#else
		 bool ManipController::searchBinPicking(Vector3& Pp_grasp2, Matrix3& Rp_grasp2, Vector3& Pp_put2, Matrix3& Rp_put2, VectorXd& th_grasp2, bool calc_negative, OccupiedVoxelVec* voxel) {
#endif
		Vector3 Po_ini;
		Matrix3 Ro_ini;
		vector<Vector3> Po_des;
		vector<Matrix3> Ro_des;
		bool ret = false;

		StopWatch timer2;
		timer2.start();
		setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

		PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
		settingPrehension(target_prehension);

		GraspDatabaseManipulator* gdm_grasp = new GraspDatabaseManipulator();
		string dataFilePath_g = (arm_g == tc->arm(0)) ? tc->dataFilePath(0) : tc->dataFilePath(1);
		string db_path =  dataFilePath_g + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
		gdm_grasp->readFile(db_path);
		GraspDatabaseManipulator::GraspPoses g_poses = gdm_grasp->getRecords();
		delete gdm_grasp;

		double app_length=pf->appLength;
		SweptVolume sv;
		sv.setGRCR(GRCmax.R);
		sv.setAppLength(app_length);
		sv.setNumGraspStep(3);
		sv.makeSweptVolume();

		SweptVolumeChecker svc(&sv);
		svc.setObjName(tc->targetObject->name());
		svc.setMargin(0.002);
		if (voxel != NULL) {
				svc.setUnknownRegionCheckMode(true);
				svc.setOccupiedVoxelVec(voxel);
		}

		timer2.stopAndPrintTime("planning(init_in_main)");

		int nj = arm_g->arm_path->numJoints()-1;
		VectorXd finger_angle;

		std::vector<double> best_feature;
#ifdef EXPERIMENT_DATA_OUTPUT
		std::vector<double> best_grasp_q;
#endif
		const int num_sec = 3;
		int best_id = 0;
		ObjPoseEstimateSolHolder* opes = ObjPoseEstimateSolHolder::instance();
		double best_prob = 0;
		for (int sec = 0; sec < num_sec; sec++) {
				double best_score = -std::numeric_limits<double>::max();
				bool found_pos_sol = false;
				bool found_neg_sol = false;

				// bool found_sol = false;
				for (size_t s = 0; s < opes->size(); s++) {
						if (!opes->at(s).is_feasible) continue;
				ScopeStopWatch timer("planning(proc_in_main)");
#ifdef EXPERIMENT_DATA_OUTPUT
#ifndef LEARNING_DATA_OUTPUT
				if (s != obj_id) continue;
#endif
				std::map<int, std::vector<double> > grasp_q_map;
				std::map<int, int> point_size_map;
#endif
#ifdef DEBUG_MODE
				fout << "Object : " << s << endl;
#endif
				std::map<int, std::vector<double> > feature_map;
				GraspDatabaseManipulator::GraspPoses feasible_poses;
				svc.setObjPoseEstimateSol(&opes->at(s));
				// save object pose
				tc->targetObject->object->p() = opes->at(s).targetObject->object->p();
				tc->targetObject->object->R() = opes->at(s).targetObject->object->R();
                Vector3 orig_obj_p = tc->targetObject->object->p();
                Matrix3 orig_obj_R = tc->targetObject->object->R();

				map<int, Matrix3> Rp_grasp_rel_map;
				map<int, Vector3> app_vec_map;
				std::map<int, double> probability_map;

#ifdef THREAD
#if EIGEN_VERSION_AT_LEAST(3,2,0)
				Eigen::initParallel();
#endif
				// calculate destination positions
				vector<Vector3> Pp_g;
				vector<Matrix3> Rp_g;
				for (size_t m = round(g_poses.size() * (static_cast<double>(sec)/num_sec)); m <  round(g_poses.size() * (static_cast<double>(sec+1)/num_sec)); m++) {
						Pp_g.push_back(opes->at(s).p + opes->at(s).R * g_poses[m].p);
						Rp_g.push_back(opes->at(s).R * g_poses[m].R);
				}

				// set arm generation function
				GrasplotEntry getGrasplotFuncGrasp = NULL;
				for (size_t i = 0; i < PlanBase::instance()->armsList.size(); i++) {
						if (PlanBase::instance()->armsList[i]->arm == arm_g){
								getGrasplotFuncGrasp = PlanBase::instance()->armsList[i]->getGrasplotFunc();
								break;
						}
				}

				vector<bool> has_sol_g;
				vector<VectorXd> sol_q_g;

				IKParallerizer ik_grasp;
				ik_grasp.initialize(bodyItemRobot()->body(), arm_g, getGrasplotFuncGrasp);
				ik_grasp.addTasks(Pp_g, Rp_g);
				ik_grasp.solveIKs();
				ik_grasp.join();
				ik_grasp.getResults(has_sol_g, sol_q_g);
#endif
				// for (size_t m = 0; m < g_poses.size(); m++) {
#ifdef THREAD
				for (size_t sol_id = 0; sol_id < has_sol_g.size(); sol_id++) {
						size_t m = round(g_poses.size() * (static_cast<double>(sec)/num_sec)) + sol_id;
						if (!has_sol_g[sol_id]) continue;
#else
				for (size_t m = round(g_poses.size() * (static_cast<double>(sec)/num_sec)); m <  round(g_poses.size() * (static_cast<double>(sec+1)/num_sec)); m++) {
#endif
						// if (g_poses[m].q < best_score) {
						// 		continue;
						// }

                        tc->targetObject->object->p() = orig_obj_p;
                        tc->targetObject->object->R() = orig_obj_R;

						for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
								bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
						bodyItemRobot()->body()->calcForwardKinematics();

						Vector3 Pp_grasp = opes->at(s).p + opes->at(s).R * g_poses[m].p;
						Matrix3 Rp_grasp = opes->at(s).R * g_poses[m].R;
#ifdef THREAD
						for (int n = 0; n <arm_g->arm_path->numJoints(); n++)
								arm_g->arm_path->joint(n)->q() = sol_q_g[sol_id][n];

#else
						// IK check
						if (!arm_g->IK_arm(Pp_grasp, Rp_grasp)) {
#ifdef DEBUG_MODE
								fout << "IK(grasping posture) not solvable in grasp data id " << g_poses[m].id << endl;
#endif
								continue;
						}
#endif
#ifdef EXPERIMENT_DATA_OUTPUT
						for (int i = 0; i < arm_g_n_fing; i++) {
								for (int j = 0; j < fingers_g[i]->fing_path->numJoints(); j++) {
										fingers_g[i]->fing_path->joint(j)->q() = fingers_g[i]->fingerOpenPose[j];
								}
						}
						grasp_q_map[g_poses[m].id].clear();
						for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
								grasp_q_map[g_poses[m].id].push_back(bodyItemRobot()->body()->joint(n)->q());
#endif

						// collision check (among models)
						finger_angle = g_poses[m].finger_q;
						int k = 0;
						for (int i = 0; i < arm_g_n_fing; i++) {
								for (int j = 0; j < fingers_g[i]->fing_path->numJoints(); j++) {
										double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
										finger_angle(k) += offset;
										fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
								}
						}
						bodyItemRobot()->body()->calcForwardKinematics();
						bodyItemRobot()->notifyKinematicStateChange();
						if (isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()) {
#ifdef DEBUG_MODE
								fout << "Collision(grasping posture) in grasp data id " << g_poses[m].id << endl;
#endif
								continue;
						}

						Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
						Vector3 app_vec = Rp_grasp * GRCmax.R * Vector3(0, app_length, 0);
						Rp_grasp_rel_map[g_poses[m].id] = Rp_grasp_rel;
						app_vec_map[g_poses[m].id] = app_vec;
						// compute score
						// double penalty = svc.check(tc->palm()->p() - app_vec, tc->palm()->R());
						// double score = g_poses[m].q + penalty;
						// double score = svc.check(tc->palm()->p() - app_vec, tc->palm()->R());
						double prob = svc.check(tc->palm()->p() - app_vec, tc->palm()->R());
						svc.getFeatureVector(feature_map[g_poses[m].id]);
						probability_map[g_poses[m].id] = prob;
						double score = svc.computeScore();
#ifdef TRAINING_MODE
						score = 1;
						prob = 1;
#endif
#ifdef EXPERIMENT_DATA_OUTPUT
						std::vector<int> indices;
						svc.getInlierIndices(indices);
						point_size_map[g_poses[m].id] = indices.size();
#endif
#ifdef DEBUG_MODE
						// fout << "Score (id:" << g_poses[m].id << ") : " << score << " (q:" << g_poses[m].q << ", penalty" << penalty << ")" << endl;
						fout << "Score (id:" << g_poses[m].id << ") : " << score << endl;
#endif
						bool is_feasible_pose = false;
						if (calc_negative) {
								is_feasible_pose = (found_pos_sol && score >= best_score && prob > 0.5) || (!found_pos_sol && ((prob > 0.5) || score >= best_score));
						} else {
								is_feasible_pose = (found_pos_sol && score >= best_score) || (!found_pos_sol && (prob > 0.5));
						}
						if (is_feasible_pose) {
						// if (score >= best_score) {
								feasible_poses.push_back(g_poses[m]);
								feasible_poses.back().q = score;
						}
				}

                tc->targetObject->object->p() = orig_obj_p;
                tc->targetObject->object->R() = orig_obj_R;
				std::sort(feasible_poses.begin(), feasible_poses.end(), GraspDatabaseManipulator::GraspPosture::sortData);

				if (arm_g->armFinalPose.size() > 0) {
#ifdef DEBUG_MODE
						fout << "arm Final pose found"<< endl;
#endif
						for (size_t m = 0; m < feasible_poses.size(); m++) {
								for (size_t i = 0; i < arm_g->armFinalPose.size(); i++)
										arm_g->arm_path->joint(i)->q() = arm_g->armFinalPose[i];
								arm_g->arm_path->calcForwardKinematics();
								Vector3 Pp_grasp = opes->at(s).p + opes->at(s).R * feasible_poses[m].p;

								if(calcJointSeq2(Pp_grasp,Rp_grasp_rel_map[feasible_poses[m].id] , app_vec_map[feasible_poses[m].id])){
#ifdef DEBUG_MODE
										fout << "Found solution in  grasp data id" << feasible_poses[m].id << endl;
#endif
										Pp_grasp2 = Pp_grasp;
										Rp_grasp2 = Rp_grasp_rel_map[feasible_poses[m].id];
										th_grasp2 = feasible_poses[m].finger_q;
										approachVec = app_vec_map[feasible_poses[m].id];
										tc->flush();
										best_score = feasible_poses[m].q;
										ret = true;
										break;
								}
						}
				} else if(putObject) {
						for (size_t m = 0; m < feasible_poses.size(); m++) {
								if (found_neg_sol && probability_map[feasible_poses[m].id] <= 0.5) continue;
								bool has_sol = false;
								Vector3 Pp_grasp = opes->at(s).p + opes->at(s).R * feasible_poses[m].p;
								Matrix3 Rp_grasp = opes->at(s).R * feasible_poses[m].R;
								Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
								Vector3 app_vec =  Rp_grasp * GRCmax.R * Vector3(0,1,0);
								for(unsigned int env=0;env<Po_des.size(); env++){
										tc->targetObject->object->p() = Po_des[env];
                                        tc->targetObject->object->R() = Ro_des[env];
										for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
												bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
										bodyItemRobot()->body()->calcForwardKinematics();

										Vector3 Pp_put = Po_des[env] + Ro_des[env] * feasible_poses[m].p;
										Matrix3 Rp_put = Ro_des[env] * feasible_poses[m].R;
										if(!arm_p->IK_arm(Pp_put,Rp_put)){
#ifdef DEBUG_MODE
												fout << "IK (putting posture) not solvable in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
												continue;
										}
										finger_angle = feasible_poses[m].finger_q;
										int k=0;
										for(int i=0; i<arm_g_n_fing; i++){
												for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
														double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
														finger_angle(k) += offset;
														fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
												}
										}
										bodyItemRobot()->body()->calcForwardKinematics();
										if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
												fout << "Collision(putting posture) in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
												continue;
										}

										Matrix3 Rp_put_rel = Rp_put * arm_p->arm_path->joint(nj)->Rs();
                                        tc->targetObject->object->p() = orig_obj_p;
                                        tc->targetObject->object->R() = orig_obj_R;
										if(!calcJointSeq(Pp_grasp,Rp_grasp_rel , Pp_put, Rp_put_rel, app_vec)){
												continue;
										}

#ifdef DEBUG_MODE
										fout << "Found solution in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
										Pp_grasp2 = Pp_grasp;
										Rp_grasp2 = Rp_grasp_rel;
										Pp_put2 = Pp_put;
										Rp_put2 = Rp_put_rel;
										th_grasp2 = finger_angle;
                                        approachVec = app_vec;
                                        best_score = feasible_poses[m].q;
										best_feature = feature_map[feasible_poses[m].id];
                                        best_id = s;
										found_neg_sol = true;
										has_sol = true;
										best_prob = probability_map[feasible_poses[m].id];
										if (probability_map[feasible_poses[m].id] > 0.5) {
												found_pos_sol = true;
										}
										ret = true;
#ifdef EXPERIMENT_DATA_OUTPUT
										best_grasp_q.clear();
										for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
												best_grasp_q.push_back(grasp_q_map[feasible_poses[m].id][n]);
										os << "points: " << point_size_map[feasible_poses[m].id] << endl;
#endif
										break;
								}
                                if (found_pos_sol && has_sol) {
										break;
								}
						}
				}
                tc->targetObject->object->p() = orig_obj_p;
                tc->targetObject->object->R() = orig_obj_R;
		}
		if (found_pos_sol || found_neg_sol) {
				os << "score:" << best_score << std::endl;
				os << "prob:" << best_prob << std::endl;
				break;
		}
		}

#ifdef EXPERIMENT_DATA_OUTPUT
				std::string cur_id_file = "./data/cur_id.txt";
				std::ifstream id_in(cur_id_file.c_str());
				int id;
				if (id_in) {
						id_in >> id;
						id_in.close();
				} else {
						id = 0;
				}
				os << "Current output id: " << id << std::endl;
				id++;
				grasp::ObjPoseEstimateSol::datafile_id = boost::lexical_cast<std::string>(id);
				std::ofstream id_out(cur_id_file.c_str());
				id_out << id;

				std::string prob_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_prob.txt";

				std::ofstream prob_out(prob_file.c_str());
				prob_out << best_prob;
#endif
		if(ret){
				ScopeStopWatch timer("planning(end_in_main)");
				PenaltyFunctionUpdater::instance()->setHandName(tc->targetArmFinger->handName);
				PenaltyFunctionUpdater::instance()->setObjectName(opes->at(best_id).targetObject->name());
				PenaltyFunctionUpdater::instance()->addData(best_feature);

				arm_g->IK_arm(Pp_grasp2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp2));

				int k=0;
				for(int i=0; i<arm_g_n_fing; i++)
						for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
                                fingers_g[i]->fing_path->joint(j)->q() = th_grasp2(k++);

				bodyItemRobot()->body()->calcForwardKinematics();

				opes->at(best_id).is_target = true;
#ifdef DEBUG_MODE
                fout << "Solution id:" << best_id << endl;
#endif
				tc->targetObject->object->p() = opes->at(best_id).targetObject->object->p();
				tc->targetObject->object->R() = opes->at(best_id).targetObject->object->R();
				ItemTreeView::mainInstance()->checkItem(opes->at(best_id).targetObject->bodyItemObject, false);
#ifdef EXPERIMENT_DATA_OUTPUT
				std::string angle_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_angle.txt";
				std::string object_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_object.txt";
				std::string pcd_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_pcd.txt";
				std::string idx_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_idx.txt";
				std::string feature_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_feature.txt";
				std::string recogscore_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_score.txt";

				std::ofstream score_out (recogscore_file.c_str());
				score_out << opes->at(best_id).score;

				std::ofstream feature_out(feature_file.c_str());
				for (int n = 0; n < best_feature.size(); n++) {
						feature_out << best_feature[n];
						feature_out << " ";
				}

				std::ofstream angle_out(angle_file.c_str());
				for (int n = 0; n < best_grasp_q.size(); n++) {
						angle_out << best_grasp_q[n];
						if (n != best_grasp_q.size()-1) angle_out << " ";
				}
				angle_out.close();
				std::ofstream object_out(object_file.c_str());
				for (int i = 0; i < 3; i++) {
						object_out << tc->targetObject->object->p()(i);
						if (i!=2) object_out << " ";
				}
				object_out << std::endl;
				for (size_t i = 0; i < 3; i++) {
						for (size_t j = 0; j < 3; j++) {
								object_out << tc->targetObject->object->R()(i, j);
								if (!(i == 2 && j == 2))
										object_out << " ";
						}
				}
				object_out.close();

				std::ofstream pcd_out(pcd_file.c_str());
				for (size_t n = 0; n < opes->at(best_id).env_point_cloud->p().size(); n++) {
						pcd_out << opes->at(best_id).env_point_cloud->p()[n].transpose() << std::endl;
				}
				pcd_out.close();

				std::ofstream idx_out(idx_file.c_str());
				for (size_t n = 0; n < opes->at(best_id).outlier_indices.size(); n++) {
						idx_out << opes->at(best_id).outlier_indices[n] << " ";
				}
				idx_out.close();
#endif
		} else {
				for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
					bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
				}
				bodyItemRobot()->body()->calcForwardKinematics();
#ifdef EXPERIMENT_DATA_OUTPUT
				std::string pcd_file = "./data/" + grasp::ObjPoseEstimateSol::datafile_id + "_pcd.txt";
				std::ofstream pcd_out(pcd_file.c_str());
				for (size_t n = 0; n < opes->at(0).env_point_cloud->p().size(); n++) {
						pcd_out << opes->at(0).env_point_cloud->p()[n].transpose() << std::endl;
				}
				pcd_out.close();
#endif
		}
		tc->flush();

		return ret;
 }

		bool ManipController::searchBinPickingNoML(cnoid::Vector3& Pp_grasp2, cnoid::Matrix3& Rp_grasp2, cnoid::Vector3& Pp_put2, cnoid::Matrix3& Rp_put2, cnoid::VectorXd& th_grasp2) {
				Vector3 Po_ini;
				Matrix3 Ro_ini;
				vector<Vector3> Po_des;
				vector<Matrix3> Ro_des;
				bool ret = false;

				setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);

				PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
				settingPrehension(target_prehension);

				GraspDatabaseManipulator* gdm_grasp = new GraspDatabaseManipulator();
				string dataFilePath_g = (arm_g == tc->arm(0)) ? tc->dataFilePath(0) : tc->dataFilePath(1);
				string db_path =  dataFilePath_g + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
				gdm_grasp->readFile(db_path);
				GraspDatabaseManipulator::GraspPoses g_poses = gdm_grasp->getRecords();
				delete gdm_grasp;

				double app_length=pf->appLength;

				int nj = arm_g->arm_path->numJoints()-1;
				VectorXd finger_angle;

				const int num_sec = 3;
				int best_id = 0;
				ObjPoseEstimateSolHolder* opes = ObjPoseEstimateSolHolder::instance();

				for (int sec = 0; sec < num_sec; sec++) {
						double best_score = -std::numeric_limits<double>::max();
						for (size_t s = 0; s < opes->size(); s++) {
								if (!opes->at(s).is_feasible) continue;

								GraspDatabaseManipulator::GraspPoses feasible_poses;
								// save object pose
								tc->targetObject->object->p() = opes->at(s).targetObject->object->p();
								tc->targetObject->object->R() = opes->at(s).targetObject->object->R();
								Vector3 orig_obj_p = tc->targetObject->object->p();
								Matrix3 orig_obj_R = tc->targetObject->object->R();

								map<int, Matrix3> Rp_grasp_rel_map;
								map<int, Vector3> app_vec_map;

#ifdef THREAD
#if EIGEN_VERSION_AT_LEAST(3,2,0)
								Eigen::initParallel();
#endif
								// calculate destination positions
								vector<Vector3> Pp_g;
								vector<Matrix3> Rp_g;
								for (size_t m = round(g_poses.size() * (static_cast<double>(sec)/num_sec)); m <  round(g_poses.size() * (static_cast<double>(sec+1)/num_sec)); m++) {
										Pp_g.push_back(opes->at(s).p + opes->at(s).R * g_poses[m].p);
										Rp_g.push_back(opes->at(s).R * g_poses[m].R);
								}

								// set arm generation function
								GrasplotEntry getGrasplotFuncGrasp = NULL;
								for (size_t i = 0; i < PlanBase::instance()->armsList.size(); i++) {
										if (PlanBase::instance()->armsList[i]->arm == arm_g){
												getGrasplotFuncGrasp = PlanBase::instance()->armsList[i]->getGrasplotFunc();
												break;
										}
								}

								vector<bool> has_sol_g;
								vector<VectorXd> sol_q_g;

								IKParallerizer ik_grasp;
								ik_grasp.initialize(bodyItemRobot()->body(), arm_g, getGrasplotFuncGrasp);
								ik_grasp.addTasks(Pp_g, Rp_g);
								ik_grasp.solveIKs();
								ik_grasp.join();
								ik_grasp.getResults(has_sol_g, sol_q_g);
#endif

#ifdef THREAD
								for (size_t sol_id = 0; sol_id < has_sol_g.size(); sol_id++) {
										size_t m = round(g_poses.size() * (static_cast<double>(sec)/num_sec)) + sol_id;
										if (!has_sol_g[sol_id]) continue;
#else
								for (size_t m = round(g_poses.size() * (static_cast<double>(sec)/num_sec)); m <  round(g_poses.size() * (static_cast<double>(sec+1)/num_sec)); m++) {
#endif

										if (g_poses[m].q < best_score) {
												continue;
										}
										tc->targetObject->object->p() = orig_obj_p;
										tc->targetObject->object->R() = orig_obj_R;

										for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
												bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
										bodyItemRobot()->body()->calcForwardKinematics();

										Vector3 Pp_grasp = opes->at(s).p + opes->at(s).R * g_poses[m].p;
										Matrix3 Rp_grasp = opes->at(s).R * g_poses[m].R;
#ifdef THREAD
										for (int n = 0; n <arm_g->arm_path->numJoints(); n++)
												arm_g->arm_path->joint(n)->q() = sol_q_g[sol_id][n];

#else
										// IK check
										if (!arm_g->IK_arm(Pp_grasp, Rp_grasp)) {
#ifdef DEBUG_MODE
												fout << "IK(grasping posture) not solvable in grasp data id " << g_poses[m].id << endl;
#endif
												continue;
										}
#endif
										// collision check (among models)
										finger_angle = g_poses[m].finger_q;
										int k = 0;
										for (int i = 0; i < arm_g_n_fing; i++) {
												for (int j = 0; j < fingers_g[i]->fing_path->numJoints(); j++) {
														double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
														finger_angle(k) += offset;
														fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
												}
										}
										bodyItemRobot()->body()->calcForwardKinematics();
										bodyItemRobot()->notifyKinematicStateChange();
										if (isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()) {
#ifdef DEBUG_MODE
												fout << "Collision(grasping posture) in grasp data id " << g_poses[m].id << endl;
#endif
												continue;
										}

										Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
										Vector3 app_vec = Rp_grasp * GRCmax.R * Vector3(0, app_length, 0);
										Rp_grasp_rel_map[g_poses[m].id] = Rp_grasp_rel;
										app_vec_map[g_poses[m].id] = app_vec;
										feasible_poses.push_back(g_poses[m]);
								}

								tc->targetObject->object->p() = orig_obj_p;
								tc->targetObject->object->R() = orig_obj_R;

								if (arm_g->armFinalPose.size() > 0) {
#ifdef DEBUG_MODE
										fout << "arm Final pose found"<< endl;
#endif
										for (size_t m = 0; m < feasible_poses.size(); m++) {
												for (size_t i = 0; i < arm_g->armFinalPose.size(); i++)
														arm_g->arm_path->joint(i)->q() = arm_g->armFinalPose[i];
												arm_g->arm_path->calcForwardKinematics();
												Vector3 Pp_grasp = opes->at(s).p + opes->at(s).R * feasible_poses[m].p;

												if(calcJointSeq2(Pp_grasp,Rp_grasp_rel_map[feasible_poses[m].id] , app_vec_map[feasible_poses[m].id])){
#ifdef DEBUG_MODE
														fout << "Found solution in  grasp data id" << feasible_poses[m].id << endl;
#endif
														Pp_grasp2 = Pp_grasp;
														Rp_grasp2 = Rp_grasp_rel_map[feasible_poses[m].id];
														th_grasp2 = feasible_poses[m].finger_q;
														approachVec = app_vec_map[feasible_poses[m].id];
														tc->flush();
														best_score = feasible_poses[m].q;
														ret = true;
														break;
												}
										}
								} else if (putObject) {
										for (size_t m = 0; m < feasible_poses.size(); m++) {
												bool has_sol = false;
												Vector3 Pp_grasp = opes->at(s).p + opes->at(s).R * feasible_poses[m].p;
												Matrix3 Rp_grasp = opes->at(s).R * feasible_poses[m].R;
												Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
												Vector3 app_vec =  Rp_grasp * GRCmax.R * Vector3(0,1,0);
												for(unsigned int env=0;env<Po_des.size(); env++){
														tc->targetObject->object->p() = Po_des[env];
														tc->targetObject->object->R() = Ro_des[env];
														for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
																bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
														bodyItemRobot()->body()->calcForwardKinematics();

														Vector3 Pp_put = Po_des[env] + Ro_des[env] * feasible_poses[m].p;
														Matrix3 Rp_put = Ro_des[env] * feasible_poses[m].R;
														if(!arm_p->IK_arm(Pp_put,Rp_put)){
#ifdef DEBUG_MODE
																fout << "IK (putting posture) not solvable in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
																continue;
														}
														finger_angle = feasible_poses[m].finger_q;
														int k=0;
														for(int i=0; i<arm_g_n_fing; i++){
																for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
																		double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
																		finger_angle(k) += offset;
																		fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
																}
														}
														bodyItemRobot()->body()->calcForwardKinematics();
														if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
																fout << "Collision(putting posture) in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
																continue;
														}
														Matrix3 Rp_put_rel = Rp_put * arm_p->arm_path->joint(nj)->Rs();
														tc->targetObject->object->p() = orig_obj_p;
														tc->targetObject->object->R() = orig_obj_R;
														if(!calcJointSeq(Pp_grasp,Rp_grasp_rel , Pp_put, Rp_put_rel, app_vec)){
																continue;
														}

#ifdef DEBUG_MODE
														fout << "Found solution in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
														Pp_grasp2 = Pp_grasp;
														Rp_grasp2 = Rp_grasp_rel;
														Pp_put2 = Pp_put;
														Rp_put2 = Rp_put_rel;
														th_grasp2 = finger_angle;
														approachVec = app_vec;
														best_score = feasible_poses[m].q;
														best_id = s;
														has_sol = true;
														ret = true;
														break;
												}
												if (has_sol) {
														break;
												}
										}
								}
								tc->targetObject->object->p() = orig_obj_p;
								tc->targetObject->object->R() = orig_obj_R;
						}
						if (ret) {
								break;
						}
				}

				if(ret){
						arm_g->IK_arm(Pp_grasp2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp2));

						int k=0;
						for(int i=0; i<arm_g_n_fing; i++)
								for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
										fingers_g[i]->fing_path->joint(j)->q() = th_grasp2(k++);

						bodyItemRobot()->body()->calcForwardKinematics();

						opes->at(best_id).is_target = true;
						tc->targetObject->object->p() = opes->at(best_id).targetObject->object->p();
						tc->targetObject->object->R() = opes->at(best_id).targetObject->object->R();
						ItemTreeView::mainInstance()->checkItem(opes->at(best_id).targetObject->bodyItemObject, false);
				} else {
						for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
								bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
						}
						bodyItemRobot()->body()->calcForwardKinematics();
				}
				tc->flush();

				return ret;
		}

		void ManipController::binPickingInitProc() {
				tc = PlanBase::instance();

				strategy = RIGHT_RIGHT;
				if (tc->armsList.size() >= 2 && tc->armsList[1] == tc->targetArmFinger) {
						strategy = LEFT_LEFT;
				}

				tc->initial();

				initial(tc->targetObject,  tc->targetArmFinger);

				PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
				settingPrehension(target_prehension);

				GraspDatabaseManipulator* gdm_grasp = new GraspDatabaseManipulator();
				string dataFilePath_g = (arm_g == tc->arm(0)) ? tc->dataFilePath(0) : tc->dataFilePath(1);
				string db_path =  dataFilePath_g + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
				gdm_grasp->readFile(db_path);
				grasp_poses = gdm_grasp->getRecords();
				delete gdm_grasp;

				double app_length=pf->appLength;

				sv_ = new SweptVolume;
				sv_->setGRCR(GRCmax.R);
				sv_->setAppLength(app_length);
				sv_->setNumGraspStep(3);
				sv_->makeSweptVolume();
		}

		bool ManipController::binPickingMainProc(const grasp::ObjPoseEstimateSol& sol, int& sec_c, double& score_c,
												 cnoid::Vector3& Pp_grasp2, cnoid::Matrix3& Rp_grasp2,
												 cnoid::Vector3& Pp_put2, cnoid::Matrix3& Rp_put2,
												 cnoid::VectorXd& th_grasp2) {
				Vector3 Po_ini;
				Matrix3 Ro_ini;
				vector<Vector3> Po_des;
				vector<Matrix3> Ro_des;
				bool ret = false;

				setTargetObjPos(Po_ini, Ro_ini, Po_des, Ro_des, 0);


				double app_length=pf->appLength;


				SweptVolumeChecker svc(sv_);
				svc.setObjName(tc->targetObject->name());
				svc.setMargin(0.002);

				int nj = arm_g->arm_path->numJoints()-1;
				VectorXd finger_angle;

				std::vector<double> best_feature;

				const int num_sec = 3;
				int max_c = (num_sec < (sec_c + 1)) ? num_sec : sec_c + 1;

				std::cout << "solp" << sol.p.transpose() << std::endl;

				for (int sec = 0; sec < max_c; sec++) {
						double best_score = -std::numeric_limits<double>::max();
						if (sec == sec_c) best_score = score_c;
						bool found_pos_sol = false;
						bool found_neg_sol = false;
						double best_prob = 0;
						std::map<int, std::vector<double> > feature_map;
						GraspDatabaseManipulator::GraspPoses feasible_poses;
						svc.setObjPoseEstimateSol(&sol);
						// save object pose
						Vector3 orig_obj_p = sol.p;
						Matrix3 orig_obj_R = sol.R;

						for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
								bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
						bodyItemRobot()->body()->calcForwardKinematics();

						map<int, Matrix3> Rp_grasp_rel_map;
						map<int, Vector3> app_vec_map;
						std::map<int, double> probability_map;
#ifdef THREAD
#if EIGEN_VERSION_AT_LEAST(3,2,0)
						Eigen::initParallel();
#endif
						// calculate destination positions
						vector<Vector3> Pp_g;
						vector<Matrix3> Rp_g;
						for (size_t m = round(grasp_poses.size() * (static_cast<double>(sec)/num_sec)); m <  round(grasp_poses.size() * (static_cast<double>(sec+1)/num_sec)); m++) {
								Pp_g.push_back(sol.p + sol.R * grasp_poses[m].p);
								Rp_g.push_back(sol.R * grasp_poses[m].R);
						}

						// set arm generation function
						GrasplotEntry getGrasplotFuncGrasp = NULL;
						for (size_t i = 0; i < PlanBase::instance()->armsList.size(); i++) {
								if (PlanBase::instance()->armsList[i]->arm == arm_g){
										getGrasplotFuncGrasp = PlanBase::instance()->armsList[i]->getGrasplotFunc();
										break;
								}
						}

						vector<bool> has_sol_g;
						vector<VectorXd> sol_q_g;

						IKParallerizer ik_grasp;
						ik_grasp.initialize(bodyItemRobot()->body(), arm_g, getGrasplotFuncGrasp);
						ik_grasp.addTasks(Pp_g, Rp_g);
						ik_grasp.solveIKs();
						ik_grasp.join();
						ik_grasp.getResults(has_sol_g, sol_q_g);
#endif
#ifdef THREAD
						int ik_suc = 0;
						int prob_suc = 0;
						int sol_num = 0;
						for (size_t sol_id = 0; sol_id < has_sol_g.size(); sol_id++) {
								size_t m = round(grasp_poses.size() * (static_cast<double>(sec)/num_sec)) + sol_id;
								if (!has_sol_g[sol_id]) continue;
								sol_num++;
#else
								for (size_t m = round(grasp_poses.size() * (static_cast<double>(sec)/num_sec)); m <  round(grasp_poses.size() * (static_cast<double>(sec+1)/num_sec)); m++) {
										int sol_id = m;
#endif
								tc->targetObject->object->p() = orig_obj_p;
								tc->targetObject->object->R() = orig_obj_R;

								for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
										bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
								bodyItemRobot()->body()->calcForwardKinematics();

								Vector3 Pp_grasp = sol.p + sol.R * grasp_poses[m].p;
								Matrix3 Rp_grasp = sol.R * grasp_poses[m].R;
#ifdef THREAD
								for (int n = 0; n <arm_g->arm_path->numJoints(); n++)
										arm_g->arm_path->joint(n)->q() = sol_q_g[sol_id][n];

#else
								// IK check
								if (!arm_g->IK_arm(Pp_grasp, Rp_grasp)) {
#ifdef DEBUG_MODE
										fout << "IK(grasping posture) not solvable in grasp data id " << grasp_poses[m].id << endl;
#endif
										continue;
								}
#endif
								// collision check (among models)
								finger_angle = grasp_poses[m].finger_q;
								int k = 0;
								for (int i = 0; i < arm_g_n_fing; i++) {
										for (int j = 0; j < fingers_g[i]->fing_path->numJoints(); j++) {
												double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
												finger_angle(k) += offset;
												fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
										}
								}
								bodyItemRobot()->body()->calcForwardKinematics();
								bodyItemRobot()->notifyKinematicStateChange();

								if (isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()) {
#ifdef DEBUG_MODE
										fout << "Collision(grasping posture) in grasp data id " << grasp_poses[m].id << endl;
#endif
										continue;
								}
								ik_suc++;
								Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
								Vector3 app_vec = Rp_grasp * GRCmax.R * Vector3(0, app_length, 0);
								Rp_grasp_rel_map[grasp_poses[m].id] = Rp_grasp_rel;
								app_vec_map[grasp_poses[m].id] = app_vec;
								// compute score
								// double penalty = svc.check(tc->palm()->p() - app_vec, tc->palm()->R());
								// double score = grasp_poses[m].q + penalty;
								// double score = svc.check(tc->palm()->p() - app_vec, tc->palm()->R());
								double prob = svc.check(tc->palm()->p() - app_vec, tc->palm()->R());
								svc.getFeatureVector(feature_map[grasp_poses[m].id]);
								probability_map[grasp_poses[m].id] = prob;
								double score = svc.computeScore();

								if (prob > 0.5) prob_suc++;

#ifdef DEBUG_MODE
								// fout << "Score (id:" << grasp_poses[m].id << ") : " << score << " (q:" << grasp_poses[m].q << ", penalty" << penalty << ")" << endl;
								fout << "Score (id:" << grasp_poses[m].id << ") : " << score << endl;
#endif
								bool is_feasible_pose = false;
								is_feasible_pose = (score >= best_score && prob > 0.5);
								if (is_feasible_pose) {
										// if (score >= best_score) {
										feasible_poses.push_back(grasp_poses[m]);
										feasible_poses.back().q = score;
								}
								}

								std::cout << "ik : "  << ik_suc << " prob_suc" << prob_suc << " sol_num" << feasible_poses.size() << std::endl;
								tc->targetObject->object->p() = orig_obj_p;
								tc->targetObject->object->R() = orig_obj_R;
								std::sort(feasible_poses.begin(), feasible_poses.end(), GraspDatabaseManipulator::GraspPosture::sortData);

								if (arm_g->armFinalPose.size() > 0) {
#ifdef DEBUG_MODE
										fout << "arm Final pose found"<< endl;
#endif
										for (size_t m = 0; m < feasible_poses.size(); m++) {
												for (size_t i = 0; i < arm_g->armFinalPose.size(); i++)
														arm_g->arm_path->joint(i)->q() = arm_g->armFinalPose[i];
												arm_g->arm_path->calcForwardKinematics();
												Vector3 Pp_grasp = sol.p + sol.R * feasible_poses[m].p;

												if(calcJointSeq2(Pp_grasp,Rp_grasp_rel_map[feasible_poses[m].id] , app_vec_map[feasible_poses[m].id])){
#ifdef DEBUG_MODE
														fout << "Found solution in  grasp data id" << feasible_poses[m].id << endl;
#endif
														Pp_grasp2 = Pp_grasp;
														Rp_grasp2 = Rp_grasp_rel_map[feasible_poses[m].id];
														th_grasp2 = feasible_poses[m].finger_q;
														approachVec = app_vec_map[feasible_poses[m].id];
														tc->flush();
														best_score = feasible_poses[m].q;
														sec_c = sec;
														score_c = best_score;
														ret = true;
														break;
												}
										}
								} else if(putObject) {
										for (size_t m = 0; m < feasible_poses.size(); m++) {
												if (found_neg_sol && probability_map[feasible_poses[m].id] <= 0.5) continue;
												bool has_sol = false;
												Vector3 Pp_grasp = sol.p + sol.R * feasible_poses[m].p;
												Matrix3 Rp_grasp = sol.R * feasible_poses[m].R;
												Matrix3 Rp_grasp_rel = Rp_grasp * arm_g->arm_path->joint(nj)->Rs();
												Vector3 app_vec =  Rp_grasp * GRCmax.R * Vector3(0,1,0);
												for(unsigned int env=0;env<Po_des.size(); env++){
														tc->targetObject->object->p() = Po_des[env];
														tc->targetObject->object->R() = Ro_des[env];
														for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
																bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
														bodyItemRobot()->body()->calcForwardKinematics();

														Vector3 Pp_put = Po_des[env] + Ro_des[env] * feasible_poses[m].p;
														Matrix3 Rp_put = Ro_des[env] * feasible_poses[m].R;
														if(!arm_p->IK_arm(Pp_put,Rp_put)){
#ifdef DEBUG_MODE
																fout << "IK (putting posture) not solvable in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
																continue;
														}
														finger_angle = feasible_poses[m].finger_q;
														int k=0;
														for(int i=0; i<arm_g_n_fing; i++){
																for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
																		double offset = (fingers_g[i]->fingerCloseOffset.size() > j) ?  fingers_g[i]->fingerCloseOffset[j] : 0;
																		finger_angle(k) += offset;
																		fingers_g[i]->fing_path->joint(j)->q() = finger_angle(k++);
																}
														}
														bodyItemRobot()->body()->calcForwardKinematics();
														if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
																fout << "Collision(putting posture) in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
																continue;
														}

														Matrix3 Rp_put_rel = Rp_put * arm_p->arm_path->joint(nj)->Rs();
														tc->targetObject->object->p() = orig_obj_p;
														tc->targetObject->object->R() = orig_obj_R;
														if(!calcJointSeq(Pp_grasp,Rp_grasp_rel , Pp_put, Rp_put_rel, app_vec)){
																continue;
														}

#ifdef DEBUG_MODE
														fout << "Found solution in " << feasible_poses[m].id << "-th grasp data and " << env << "-th object placing posture"<< endl;
#endif
														Pp_grasp2 = Pp_grasp;
														Rp_grasp2 = Rp_grasp_rel;
														Pp_put2 = Pp_put;
														Rp_put2 = Rp_put_rel;
														th_grasp2 = finger_angle;
														approachVec = app_vec;
														best_score = feasible_poses[m].q;
														best_feature = feature_map[feasible_poses[m].id];
														found_neg_sol = true;
														has_sol = true;
														sec_c = sec;
														score_c = best_score;
														best_prob = probability_map[feasible_poses[m].id];
														if (probability_map[feasible_poses[m].id] > 0.5) {
																found_pos_sol = true;
														}
														ret = true;
														std::cout << "best_score" << best_score << std::endl;
#ifdef EXPERIMENT_DATA_OUTPUT
														// best_grasp_q.clear();
														// for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++)
														// 		best_grasp_q.push_back(grasp_q_map[feasible_poses[m].id][n]);
														// os << "points: " << point_size_map[feasible_poses[m].id] << endl;
#endif
														break;
												}
												if (found_pos_sol && has_sol) {
														break;
												}
										}
								}
								tc->targetObject->object->p() = orig_obj_p;
								tc->targetObject->object->R() = orig_obj_R;
								if (found_pos_sol || found_neg_sol) {
										os << "score:" << best_score << std::endl;
										os << "prob:" << best_prob << std::endl;
										break;
								}

						}
						if (ret) {
								arm_g->IK_arm(Pp_grasp2, arm_g->arm_path->joint(nj)->calcRfromAttitude(Rp_grasp2));

								int k = 0;
								for(int i=0; i<arm_g_n_fing; i++)
										for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
												fingers_g[i]->fing_path->joint(j)->q() = th_grasp2(k++);

								bodyItemRobot()->body()->calcForwardKinematics();
						} else {
								for (int n = 0; n < bodyItemRobot()->body()->numJoints(); n++) {
										bodyItemRobot()->body()->joint(n)->q() = tc->jointSeq[0](n);
								}
								bodyItemRobot()->body()->calcForwardKinematics();
						}
				return ret;
		}

		void ManipController::binPickingFinalProc(bool is_succeed,
												  const cnoid::Vector3& Pp_grasp, const cnoid::Matrix3& Rp_grasp,
												  const cnoid::Vector3& Pp_put, const cnoid::Matrix3& Rp_put,
												  const cnoid::VectorXd& theta) {
				if (is_succeed) {
						palmPos = Pp_grasp;
						palmRot = Rp_grasp;

						cnoid::Vector3 p_grasp = Pp_grasp;
						cnoid::Vector3 p_put = Pp_put;
						cnoid::Matrix3 R_grasp = Rp_grasp;
						cnoid::Matrix3 R_put = Rp_put;
						if(putObject) {
								int k = 0;
								for(int i=0; i<arm_g_n_fing; i++)
										for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
												fingers_g[i]->fing_path->joint(j)->q() = theta(k++);

								bodyItemRobot()->body()->calcForwardKinematics();
								calcJointSeq(p_grasp, R_grasp, p_put, R_put);
						}

						os << "Success: " << endl;

				}
				else{
						for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
								bodyItemRobot()->body()->joint(i)->q() = tc->jointSeq[0](i);

						bodyItemRobot()->body()->calcForwardKinematics();
						bodyItemRobot()->notifyKinematicStateChange();
						os << "Fail (grasp plan): " << endl;
				}
				tc->flush();
				PlanBase::instance()->graspMotionState = PlanBase::instance()->getMotionState();
				if (sv_ != NULL) delete sv_;
		}

		void ManipController::clearTrajectories() {
				// clear robot's trajectories
				if (!PlanBase::instance()->bodyItemRobot()) return;
				Item* target_item = PlanBase::instance()->bodyItemRobot()->childItem();
				while (target_item != NULL) {
						PoseSeqItem* pose_seq_item = dynamic_cast<PoseSeqItem*>(target_item);
						target_item = target_item->nextItem();
						if (pose_seq_item != NULL) {
								pose_seq_item->detachFromParentItem();
						}
				}

				// clear recognized object's trajectories (unnecessary?)
				ItemPtr ro_item = cnoid::RootItem::mainInstance()->findItem<Item>("RecognizedObjects");
				if (ro_item == NULL) return;
				target_item = ro_item->childItem();
				while (target_item != NULL) {
						Item* child_item = target_item->childItem();
						while (child_item != NULL) {
								PoseSeqItem* pose_seq_item = dynamic_cast<PoseSeqItem*>(child_item);
								child_item = child_item->nextItem();
								if (pose_seq_item != NULL) {
										pose_seq_item->detachFromParentItem();
								}
						}
						target_item = target_item->nextItem();
				}

				// clear object's trajectories
				if (!PlanBase::instance()->targetObject) return;
				target_item = PlanBase::instance()->targetObject->bodyItemObject->childItem();
				while (target_item != NULL) {
						PoseSeqItem* pose_seq_item = dynamic_cast<PoseSeqItem*>(target_item);
						target_item = target_item->nextItem();
						if (pose_seq_item != NULL) {
								pose_seq_item->detachFromParentItem();
						}
				}
		}


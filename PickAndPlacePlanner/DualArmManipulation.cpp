// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "DualArmManipulation.h"

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

DualArmManipulation::DualArmManipulation() {
}

DualArmManipulation::~DualArmManipulation(){
}

DualArmManipulation* DualArmManipulation::instance() {
		static DualArmManipulation* instance = new DualArmManipulation();
		return instance;
}

bool DualArmManipulation::graspWithBothHand(bool same_direction,int keyposes){
	tc = PlanBase::instance();

	if ( !tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}

	tc->initial();
	initial(tc->targetObject,tc->targetArmFinger);

	PrehensionPtr target_prehension_right = tc->armsList[0]->getTargetPrehension()[0];
	settingPrehension(target_prehension_right);
	r_GRCR = GRCmax.R;
	PrehensionPtr target_prehension_left = tc->armsList[1]->getTargetPrehension()[0];
	settingPrehension(target_prehension_left);
	l_GRCR= GRCmax.R;

	bool ret = false;

	// set arm and fingers
	arm_g = tc->arm(0);
	arm_p = tc->arm(1);
	fingers_g[0] = tc->fingers(0,0);
	fingers_g[1] = tc->fingers(0,1);
	fingers_p[0] = tc->fingers(1,0);
	fingers_p[1] = tc->fingers(1,1);
	fingers_g[0]->fingerGraspPose.resize(fingers_g[0]->fing_path->numJoints());
	fingers_g[1]->fingerGraspPose.resize(fingers_g[1]->fing_path->numJoints());
	fingers_p[0]->fingerGraspPose.resize(fingers_p[0]->fing_path->numJoints());
	fingers_p[1]->fingerGraspPose.resize(fingers_p[1]->fing_path->numJoints());

	//set initial and destination position and pose
	Vector3 Po_ini, Pr_grasp, Pl_grasp;
	Matrix3 Ro_ini, Rr_grasp, Rl_grasp;
	vector<Vector3> Po_des;
	vector<Matrix3> Ro_des;
	
	Po_ini = objVisPos();
	Ro_ini = objVisRot();
	for(unsigned int i=0; i<Po_put.size(); i++){
		Po_des.push_back( Po_put[i] );
		Ro_des.push_back( Ro_put[i] );
	}

	//check db files
	struct stat st;
	string db_right =  tc->dataFilePath(0) + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	string db_left =  tc->dataFilePath(1) + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	if(stat(db_right.c_str(), &st)!=0){
		os << db_right << " does not exist." << endl;
		return false;
	}
	if(stat(db_left.c_str(), &st)!=0){
		os << db_left << " does not exist." << endl;
		return false;
	}

	ifstream ifs_right,ifs_left;
	ifs_right.open(db_right.c_str());
	ifs_left.open(db_left.c_str());

	if(!ifs_right || !ifs_left){
		os << "cannot open preplanning files" << endl;
	}

	//save joint angles
	vector<double> org_q;
	for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++)
        org_q.push_back(bodyItemRobot()->body()->joint(i)->q());

    Vector3 org_obj_p = tc->targetObject->object->p();
    Matrix3 org_obj_R = tc->targetObject->object->R();

	while(!ifs_right.eof()){
		ret = false;
		string rline;
		getline(ifs_right,rline);
		VectorXd r_finger_angle(fingers_g[0]->fing_path->numJoints() + fingers_g[1]->fing_path->numJoints());

		if(!getPosture(rline,Pr_grasp,Rr_grasp,r_finger_angle))break;
		Vector3 target_pr = Po_ini + Ro_ini * Pr_grasp;
		Matrix3 target_Rr = Ro_ini * Rr_grasp;

		if(!arm_g->IK_arm(target_pr,target_Rr)){
#ifdef DEBUG_MODE
			os << "IK(right arm) not solvable" << endl;
#endif
			continue;
		}
		int k=0;
		for(int i=0; i<2; i++){
			for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++){
                fingers_g[i]->fing_path->joint(j)->q() = r_finger_angle(k++);
			}
		}
		bodyItemRobot()->body()->calcForwardKinematics();

		if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
			os << "Collision(right hand)" << endl;
#endif
			continue;
		}

		//save joint angles of left arm.
		int n = arm_p->arm_path->numJoints();
		vector<double> org_leftarm_q(n);
		for(int i=0;i<n;i++){
            org_leftarm_q[i] = arm_p->arm_path->joint(i)->q();
		}

		ifs_left.clear();
		ifs_left.seekg(0,std::ios::beg);
		while(!ifs_left.eof()){
			ret = false;
			string lline;
			getline(ifs_left,lline);
			VectorXd l_finger_angle(fingers_p[0]->fing_path->numJoints() + fingers_p[1]->fing_path->numJoints());

			if(!getPosture(lline,Pl_grasp,Rl_grasp,l_finger_angle))break;
			Vector3 target_pl = Po_ini + Ro_ini * Pl_grasp;
			Matrix3 target_Rl = Ro_ini * Rl_grasp;
			
			if(!arm_p->IK_arm(target_pl,target_Rl,arm_g->arm_path->joint(0)->q())){
#ifdef DEBUG_MODE
				os << "IK(left arm) not solvable" << endl;
#endif
				continue;
			}
			k=0;
			for(int i=0; i<2; i++){
				for(int j=0; j<fingers_p[i]->fing_path->numJoints(); j++){
                    fingers_p[i]->fing_path->joint(j)->q() = l_finger_angle(k++);
				}
			}
			bodyItemRobot()->body()->calcForwardKinematics();

			if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
				os << "Collision(left hand)" << endl;
#endif
				continue;
			}

			if(envItem != NULL){
				for(unsigned int env=0;env<Po_des.size(); env++){
					Vector3 des_pr = Po_des[env] + Ro_des[env] * Pr_grasp;
					Matrix3 des_Rr = Ro_des[env] * Rr_grasp;
					Vector3 des_pl = Po_des[env] + Ro_des[env] * Pl_grasp;
					Matrix3 des_Rl = Ro_des[env] * Rl_grasp;
					if(!arm_g->IK_arm(des_pr,des_Rr)) continue;
                    if(!arm_p->IK_arm(des_pl,des_Rl,arm_g->arm_path->joint(0)->q()))continue;
					if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT) || !withinJointLimit()){
#ifdef DEBUG_MODE
						os << "Collision(put)" << endl;
#endif
						continue;
					}

					int nj = arm_g->arm_path->numJoints()-1;
                    Matrix3 rel_R_r = target_Rr * arm_g->arm_path->joint(nj)->Rs();
                    Matrix3 rel_R_rdes = des_Rr * arm_g->arm_path->joint(nj)->Rs();
					nj = arm_p->arm_path->numJoints()-1;
                    Matrix3 rel_R_l = target_Rl * arm_p->arm_path->joint(nj)->Rs();
                    Matrix3 rel_R_ldes = des_Rl * arm_p->arm_path->joint(nj)->Rs();

					Vector3 eco1(0.0, 1.0, 0.0);
					Vector3 app_vec_r =  rel_R_r * r_GRCR * eco1;
					Vector3 app_vec_l =  rel_R_l * l_GRCR * eco1;
					if (app_vec_r.dot(app_vec_l) < 0.9 && same_direction) continue;

					if(!calcJointSeqBothHand(Po_ini,Ro_ini,target_pr,rel_R_r,target_pl,rel_R_l,des_pr,rel_R_rdes,des_pl,rel_R_ldes,keyposes)){
                        tc->targetObject->object->p() = org_obj_p;
                        tc->targetObject->object->R() = org_obj_R;
						tc->targetObject->bodyItemObject->body()->calcForwardKinematics();
						continue;
					}
					ret = true;
					break;
				}
			}
			break;
		}
		if(ret)break;
		for(int i=0;i<n;i++){
            arm_p->arm_path->joint(i)->q()=org_leftarm_q[i];
		}
		bodyItemRobot()->body()->calcForwardKinematics();
	}
	
	if(ret){
		os << "success" <<  endl;
	}else{
		os << "failed" << endl;
		for(int i=0; i<bodyItemRobot()->body()->numJoints(); i++){
            bodyItemRobot()->body()->joint(i)->q() = org_q[i];
			bodyItemRobot()->body()->calcForwardKinematics();
		}
	}

	tc->flush();
	return  ret;
}

bool DualArmManipulation::calcJointSeqBothHand(Vector3& Po_ini,Matrix3& Ro_ini,Vector3& P_grasp1,Matrix3& R_grasp1,Vector3& P_grasp2,Matrix3& R_grasp2,Vector3& P_put1,Matrix3& R_put1,Vector3& P_put2,Matrix3& R_put2,int keyposes){
		Vector3 orig_obj_p = tc->targetObject->object->p();
		Matrix3 orig_obj_R = tc->targetObject->object->R();
	
		bool cond = true;
		Vector3 z(0.0, 0.0, 1.0);
		double app_length=pf->appLength, v_margin=pf->appHeight, lift_height=pf->liftHeight, release_height=pf->releaseHeight;

		Vector3 eco1(0.0, 1.0, 0.0);

		vector<double> mtime;
		for(unsigned int i=0; i<pf->motionTime.size(); i++)
				mtime.push_back(pf->motionTime[i]);

		for(int i=0; i<2; i++){
				for(int j=0; j<fingers_g[i]->fing_path->numJoints(); j++)
                        fingers_g[i]->fingerGraspPose[j] = fingers_g[i]->joint(j)->q();
		}
		for(int i=0; i<2; i++){
				for(int j=0; j<fingers_p[i]->fing_path->numJoints(); j++)
                        fingers_p[i]->fingerGraspPose[j] = fingers_p[i]->joint(j)->q();
		}

		int ngj = arm_g->arm_path->numJoints()-1;
		int npj = arm_p->arm_path->numJoints()-1;

		VectorXd jointSeq(bodyItemRobot()->body()->numJoints());
		VectorXd jointSeq_ini(bodyItemRobot()->body()->numJoints());

		jointSeq     = tc->jointSeq[0];
		jointSeq_ini = tc->jointSeq[0];

		tc->jointSeq.clear();
		tc->graspingStateSeq.clear();
		tc->graspingStateSeq2.clear();
		tc->pathPlanDOFSeq.clear();
		tc->objectContactStateSeq.clear();
		tc->pathPlanDOF.clear();
		for(int i=0; i<tc->body()->numJoints(); i++)
            tc->pathPlanDOF.push_back(tc->body()->joint(i)->jointId());
		tc->motionTimeSeq.clear();


		tc->jointSeq.push_back(jointSeq);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[1]);

		//==== Approach Point
		Vector3 appVec1 =  R_grasp1 * r_GRCR * eco1;
		Vector3 appVec2 =  R_grasp2 * l_GRCR * eco1;
		Vector3 Pr_app = P_grasp1 - app_length*appVec1;
		Vector3 Pl_app = P_grasp2 - app_length*appVec2;
 		Matrix3 Attr_grasp = arm_g->arm_path->joint(ngj)->calcRfromAttitude(R_grasp1);
		Matrix3 Attl_grasp = arm_p->arm_path->joint(npj)->calcRfromAttitude(R_grasp2);
		if(!setMotionSeqDual(PlanBase::NOT_GRASPING,PlanBase::NOT_GRASPING,PlanBase::ON_ENVIRONMENT,Pr_app,Attr_grasp,Pl_app,Attl_grasp,jointSeq,arm_g,fingers_g,mtime[0])) cond = false;
		if(isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Approach point not solvable" << endl;
#endif
				tc->targetObject->object->p() = orig_obj_p;
				tc->targetObject->object->R() = orig_obj_R;
				return false;}

		//==== Grasping Point
		if(!setMotionSeqDual(PlanBase::UNDER_GRASPING,PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, P_grasp1, Attr_grasp,P_grasp2, Attl_grasp, jointSeq, arm_g, fingers_g, mtime[1])) cond = false;
		if(isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) cond = false;
		if(!setMotionSeqDual(PlanBase::GRASPING      ,PlanBase::GRASPING,       PlanBase::ON_ENVIRONMENT, P_grasp1, Attr_grasp,P_grasp2, Attl_grasp, jointSeq, arm_g, fingers_g, mtime[2])) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Grasping point not solvable" << endl;
#endif
				tc->targetObject->object->p() = orig_obj_p;
				tc->targetObject->object->R() = orig_obj_R;
				return false;}

		//==== LiftUp Point
		Vector3 Pr_lift = P_grasp1+lift_height*z;
		Vector3 Pl_lift = P_grasp2+lift_height*z;


		//=== Top of release point
		Vector3 Pr_rel = P_put1+release_height*z;
		Vector3 Pl_rel = P_put2+release_height*z;
		Matrix3 Attr_put = arm_g->arm_path->joint(ngj)->calcRfromAttitude(R_put1);
		Matrix3 Attl_put = arm_p->arm_path->joint(npj)->calcRfromAttitude(R_put2);


		Vector3 Po = Po_ini + lift_height*z;
		vector<Vector3> Pr_inter;
		vector<Matrix3> Rr_inter;
		vector<Vector3> Pl_inter;
		vector<Matrix3> Rl_inter;

		poseInterpolation(Po,Ro_ini,Pr_lift,R_grasp1,Pl_lift,R_grasp2,Pr_rel,R_put1,Pl_rel,R_put2,keyposes,Pr_inter,Rr_inter,Pl_inter,Rl_inter);

		for(int i=0;i<Pr_inter.size();i++){
			Matrix3 Attr_inter = arm_g->arm_path->joint(ngj)->calcRfromAttitude(Rr_inter[i]);
			Matrix3 Attl_inter = arm_p->arm_path->joint(npj)->calcRfromAttitude(Rl_inter[i]);
			if(!setMotionSeqDual(PlanBase::GRASPING,PlanBase::GRASPING, PlanBase::OFF_ENVIRONMENT, Pr_inter[i], Attr_inter,Pl_inter[i], Attl_inter, jointSeq, arm_g, fingers_g, mtime[1]/(keyposes+1))) cond = false;
			if(isColliding(PlanBase::GRASPING, PlanBase::NOT_GRASPING, PlanBase::OFF_ENVIRONMENT)) cond = false;
			if(!cond){
#ifdef DEBUG_MODE
					cout << "during move not solvable" << endl;
#endif
					tc->targetObject->object->p() = orig_obj_p;
					tc->targetObject->object->R() = orig_obj_R;
					return false;}
		}


		//== Release point
		if(!setMotionSeqDual(PlanBase::UNDER_GRASPING,PlanBase::UNDER_GRASPING, PlanBase::ON_ENVIRONMENT, P_put1, Attr_put,P_put2, Attl_put, jointSeq, arm_g, fingers_g, mtime[1],true,true)) cond = false;
		if(!setMotionSeqDual(PlanBase::NOT_GRASPING,  PlanBase::NOT_GRASPING,   PlanBase::ON_ENVIRONMENT, P_put1, Attr_put,P_put2, Attl_put, jointSeq, arm_g, fingers_g, mtime[2]))           cond = false;
		if(isColliding(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT)) cond = false;
		if(!cond){
#ifdef DEBUG_MODE
				cout << "Release point not solvable" << endl;
#endif
				tc->targetObject->object->p() = orig_obj_p;
				tc->targetObject->object->R() = orig_obj_R;
				return false;}


		appVec1 =  R_put1 * r_GRCR * eco1;
		appVec2 =  R_put2 * l_GRCR * eco1;
		Pr_rel = P_put1 - app_length*appVec1;
		Pl_rel = P_put2 - app_length*appVec2;
		////== Return to original position
		if(!setMotionSeqDual(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT, Pr_rel, Attr_put, Pl_rel,Attl_put, jointSeq, arm_g, fingers_g, mtime[1])) return false;
		tc->jointSeq.push_back(jointSeq_ini);
		tc->pathPlanDOFSeq.push_back(tc->pathPlanDOF);
		setGraspingStateSeq(PlanBase::NOT_GRASPING, PlanBase::NOT_GRASPING, PlanBase::ON_ENVIRONMENT);
		tc->motionTimeSeq.push_back(mtime[0]);

		//==Just for Graphics
		tc->setGraspingStateMainArm(PlanBase::NOT_GRASPING);
		arm_g->IK_arm(P_grasp1, Attr_grasp);
        arm_p->IK_arm(P_grasp2, Attl_grasp,arm_g->arm_path->joint(0)->q());
		bodyItemRobot()->body()->calcForwardKinematics();
		bodyItemRobot()->notifyKinematicStateChange();

		tc->targetObject->object->p() = orig_obj_p;
		tc->targetObject->object->R() = orig_obj_R;

		return true;
}


void DualArmManipulation::poseInterpolation(Vector3& Po_start,Matrix3& Ro_start,Vector3& Pr_start,Matrix3& Rr_start,Vector3& Pl_start,Matrix3& Rl_start,
	Vector3& Pr_end,Matrix3& Rr_end,Vector3& Pl_end,Matrix3& Rl_end,int keyposes,
	vector<Vector3>& Pr,vector<Matrix3>& Rr,vector<Vector3>& Pl,vector<Matrix3>& Rl){
		Matrix3 Rr_o = Rr_start.transpose() * Ro_start;
		Vector3 Pr_o = Ro_start.transpose() * (Pr_start-Po_start);
		Matrix3 Rl_o = Rl_start.transpose() * Ro_start;
		Vector3 Pl_o = Ro_start.transpose() * (Pl_start-Po_start);

		Matrix3 Ro_end = Rr_end * Rr_o;
		Vector3 Po_end = Pr_end - Ro_end * Pr_o;
		
		Vector3 rpyo_start = rpyFromRot(Ro_start);
		Vector3 rpyo_end = rpyFromRot(Ro_end);
		for(int i=0;i<=keyposes+1;i++){
			Vector3 Po = Po_start + (Po_end - Po_start)*i/(keyposes+1);
			Matrix3 Ro = rotFromRpy(rpyo_start + (rpyo_end - rpyo_start)*i/(keyposes+1));
			Pr.push_back(Po+ Ro * Pr_o);
			Rr.push_back(Ro * Rr_o.transpose());
			Pl.push_back(Po+ Ro * Pl_o);
			Rl.push_back(Ro * Rl_o.transpose());
		}
}

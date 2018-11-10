// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "TrajectoryPlanner.h"

#include<stdio.h>

#include <cnoid/LinkPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/TimeBar>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/

#ifndef WIN32
#include <dirent.h>
#endif

#include "mpkInterface.h"
#include "RRTPathPlanner.h"
#include "PRM.h"
#include "RRT.h"
#include "RRTStar.h"
#include "ParamDialog.h"

#include "../Grasp/GraspController.h"
#include "../Grasp/AssemblyObject.h"
#include "../Grasp/ObjectBase.h"
#include "../Grasp/ObjectManager.h"

//#define USE_EXACT_TIME

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

/*
bool PoseSeqItemGrasp::interpolationGrasp(){
	return false;
}
*/

bool  TrajectoryPlanner::updateTrajectoryFromMotion(const BodyMotionPtr motionObject, const BodyMotionPtr motionRobot, vector<MotionState>& motionSeq){

		PlanBase* gc= PlanBase::instance();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		BodyPtr objectBody = gc->targetObject->bodyItemObject->body();
		const int numJoints = objectBody->numJoints();
		const int numLinksToPut = objectBody->numLinks() ;
		motionObject->setDimension(numFrames, numJoints, numLinksToPut);
		motionObject->setFrameRate(frameRate);

#ifdef CNOID_10_11
		MultiAffine3Seq& pseq = *motionObject->linkPosSeq();
#else
		MultiSE3Seq& pseq = *motionObject->linkPosSeq();
#endif
		MultiValueSeq& qseqRobot = *motionRobot->jointPosSeq();
		BodyPtr robotBody = gc->bodyItemRobot()->body();

		const int numJointsRobot = robotBody->numJoints();

		Link* rootLink = objectBody->rootLink();

		// store the original state
		//Se3 orgp;
		//orgp.p = rootLink->p;
		//orgp.R = rootLink->R;

		vector<MotionState>::iterator itg = motionSeq.begin();
		gc->setGraspingState( itg->graspingState );
		gc->setGraspingState2( itg->graspingState2 );
		itg++;

        gc->object()->p() = gc->objVisPos();
        gc->object()->R() = gc->objVisRot();
		gc->setMotionState(*itg);

		for(int frame = 0; frame < numFrames; ++frame){
			if( (itg != motionSeq.end()) && ( itg->time <= ((double)frame)/frameRate) ){
				gc->setGraspingState( itg->graspingState );
				gc->setGraspingState2( itg->graspingState2 );
				itg++;
			}

#ifdef CNOID_10_11
			MultiValueSeq::View qs = qseqRobot.frame(frame);
			MultiAffine3Seq& pseqRobot = *motionRobot->linkPosSeq();
#else
			MultiValueSeq::Frame qs = qseqRobot.frame(frame);
			MultiSE3Seq& pseqRobot= *motionRobot->linkPosSeq();
#endif
			SE3& p2 = pseqRobot.at(frame, 0);
            robotBody->link(0)->p() = p2.translation();
            robotBody->link(0)->R() = Matrix3(p2.rotation()) ;

			for(int i=0; i < numJointsRobot; ++i){
                robotBody->joint(i)->q() = qs[i];
			}
			gc->calcForwardKinematics();
#ifdef CNOID_10_11
			Affine3& p = pseq.at(frame, 0);
            p.translation() = gc->object()->p;
            p.linear() = gc->object()->R;
#else
			SE3& p = pseq.at(frame, 0);
            p.set(gc->object()->p(), gc->object()->R());
#endif
		}

		// store the moved state
        rootLink->p() = gc->object()->p();
        rootLink->R() = gc->object()->R();
		objectBody->calcForwardKinematics();
		gc->flush();

		return true;
}


bool  TrajectoryPlanner::outputTrajectoryForOpenHRP(const BodyMotionPtr motionRobot){
	BodyMotionPtr motionObject;
	vector<MotionState> motionSeq;

		PlanBase* gc= PlanBase::instance();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		BodyPtr robotBody = gc->body();
		const int numJoints = robotBody->numJoints();
		const int numLinksToPut = robotBody->numLinks() ;

#ifdef CNOID_10_11
		MultiAffine3Seq& pseq = *motionRobot->linkPosSeq();
#else
		MultiSE3Seq& pseq = *motionRobot->linkPosSeq();
#endif
		MultiValueSeq& qseqRobot = *motionRobot->jointPosSeq();
//		BodyPtr robotBody = gc->body();

		const int numJointsRobot = robotBody->numJoints();

		Link* rootLink = robotBody->rootLink();

		// store the original state
		//Se3 orgp;
		//orgp.p = rootLink->p;
		//orgp.R = rootLink->R;
		Link* rfoot = robotBody->link("RLEG_JOINT5");
        Vector3 rfootp= rfoot->p();
        Matrix3 rfootR= rfoot->R();


        gc->object()->p() = gc->objVisPos();
        gc->object()->R() = gc->objVisRot();

		ofstream pfile ( "grasp.pos") ;
		ofstream zfile ( "grasp.zmp") ;
		ofstream rfile ( "grasp.rpy") ;

		for(int frame = 0; frame < numFrames; ++frame){
#ifdef CNOID_10_11
			MultiValueSeq::View qs = qseqRobot.frame(frame);
#else
			MultiValueSeq::Frame qs = qseqRobot.frame(frame);
#endif
			for(int i=0; i < numJointsRobot; ++i){
                robotBody->joint(i)->q() = qs[i];
			}
			robotBody->calcForwardKinematics();
            Vector3 waistp = rfootR*rfoot->R().transpose()*(rootLink->p() - rfoot->p())+rfootp;
            Matrix3 waistR = rfootR*rfoot->R().transpose()*rootLink->R();

#ifdef CNOID_10_11
			Affine3& p = pseq.at(frame, 0);
            p.translation() = gc->object()->p;
            p.linear() = gc->object()->R;
#else
			SE3& p = pseq.at(frame, 0);
			p.set(waistp, waistR);
#endif

            Vector3 wporg = rootLink->p();
            Matrix3 wRorg = rootLink->R();

            rootLink->p() = waistp;
            rootLink->R() = waistR;
			robotBody->calcForwardKinematics();
			double time = (double)frame/frameRate;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			Vector3 cm = robotBody->calcCM();
#else
			Vector3 cm = robotBody->calcCenterOfMass();
#endif
			cm[2]=0;
            Vector3 zmp = (rootLink->R().transpose()*(cm-rootLink->p()));
            Vector3 rpy = (rpyFromRot(rootLink->R()));

			pfile << time ;
			for(int i=0; i < numJointsRobot; ++i){
                pfile << " " <<robotBody->joint(i)->q()*180.0/M_PI;
			}
			pfile << endl;

			zfile << time << " " << zmp[0] << " " << zmp[1] << " " << zmp[2] << endl;
			rfile << time << " " << rpy[0] << " " << rpy[1] << " " << rpy[2] << endl;

            rootLink->p() = wporg;
            rootLink->R() = wRorg;
		}
		return true;
}

TrajectoryPlanner::TrajectoryPlanner(int id) :
		os (MessageView::instance()->cout())
{

		PlanBase* gc = PlanBase::instance();
		static int id_ =0;
		if(id==0){
			id = id_;
			id_ ++;
		}
		stringstream name ;
		name  << "GraspPoseSeqItem" << id;

		poseSeqItemRobot = new PoseSeqItem();
		poseSeqItemRobot->setName(name.str());
		gc->bodyItemRobot()->addSubItem(poseSeqItemRobot);	/* modified by qtconv.rb 4th rule*/

		// create PoseSeqItems
		gc->getObjectManager()->extract(accompanying_objects_, (ObjectBase::Type)(ObjectBase::TYPE_HAND | ObjectBase::TYPE_OBJECT));
		poseSeqItemAccompanyings_.resize(accompanying_objects_.size());
		for (size_t i = 0; i < accompanying_objects_.size(); i++) {
				PoseSeqItemPtr pose_item = PoseSeqItemPtr(new PoseSeqItem());
				pose_item->setName(name.str());
				accompanying_objects_[i]->bodyItem()->addSubItem(pose_item);
				poseSeqItemAccompanyings_[i] = pose_item;
		}

		if(gc->targetObject){
				poseSeqItemObject = new PoseSeqItem();
				poseSeqItemObject->setName(name.str());
				gc->targetObject->bodyItemObject->addSubItem(poseSeqItemObject);	/* modified by qtconv.rb 4th rule*/
		}
		
		useSafeBB = false;

		verbose = true;
}

void TrajectoryPlanner::setStartPos() {
		PlanBase* gc = PlanBase::instance();
}

bool TrajectoryPlanner::doTrajectoryPlanning() {
		ItemTreeView::mainInstance()->clearSelection();
		PlanBase* gc = PlanBase::instance();

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		MotionState tempState = gc->getMotionState();
		gc->initialCollision();

		vector<MotionState> inputMotionSeq;
		bool outputGraspMotionSeq = true;
		isTrajectorySectionSucceed.clear();

		if(gc->jointSeq.size()>1){
			for(unsigned int i=0; i<gc->jointSeq.size(); i++){
				if (gc->handObjectStateSeq.size()>0) gc->setHandObjectState(gc->handObjectStateSeq[i]);
				gc->setGraspingState(gc->graspingStateSeq[i]);
				gc->setGraspingState2(gc->graspingStateSeq2[i]);
				if(gc->objectContactStateSeq.size()>0) gc->setObjectContactState(gc->objectContactStateSeq[i]);
				if(gc->pathPlanDOFSeq.size()>0) gc->pathPlanDOF = gc->pathPlanDOFSeq[i];
				MotionState temp = gc->getMotionState();
				temp.jointSeq = gc->jointSeq[i];
				temp.motionTime = gc->motionTimeSeq[i];
				temp.objectPalmPos = gc->objectPalmPosSeq[i];
				temp.objectPalmRot = gc->objectPalmRotSeq[i];
				inputMotionSeq.push_back(temp);
			}
		}
		else if ( gc->graspMotionSeq.size() > 1){
			inputMotionSeq = gc->graspMotionSeq;
		}
		else {
			outputGraspMotionSeq = false;
			//gc->graspMotionSeq.clear();
			MotionState startMotionState, endMotionState;

			if(gc->startMotionState.id > 0){
				startMotionState = gc->startMotionState;
			}
			else{
				MotionState temp = gc->getMotionState();
				for(int i=0;i<temp.jointSeq.size();i++) temp.jointSeq[i]=0;
				temp.id = 1;
				startMotionState = temp;
			}
			if(gc->endMotionState.id > 0){
				endMotionState = gc->endMotionState;
			}
			else{
				MotionState temp = gc->getMotionState();
				temp.id = 2;
				endMotionState = temp;
			}

			if( (startMotionState.pos - endMotionState.pos).norm() > 1.e-10){
				gc->setTrajectoryPlanMapDOF();
			}else{
				gc->setTrajectoryPlanDOF();
			}
			startMotionState.pathPlanDOF = gc->pathPlanDOF;
			endMotionState.pathPlanDOF = gc->pathPlanDOF;
			inputMotionSeq.push_back(startMotionState);
			inputMotionSeq.push_back(endMotionState);
		}

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);

		// grasp::mpkInterface planner(gc->bodyItemRobot(), gc->bodyItemEnv);
		grasp::mpkInterface planner(gc->robotBody(), gc->bodyItemEnv);

		vector<VectorXd> config, config_tmp;

		bool successAll=true;

/*		if(gc->jointSeq.size()>1){
			useGraspMotionSeq = false;
			gc->graspMotionSeq.clear();
			for(unsigned int i=0; i<gc->jointSeq.size()-1; i++){
				config_tmp.clear();

				gc->setGraspingState(gc->graspingStateSeq[i]);
				gc->setGraspingState2(gc->graspingStateSeq2[i]);
				if(gc->objectContactStateSeq.size()>0)
						gc->setObjectContactState(gc->objectContactStateSeq[i]);
				if(gc->pathPlanDOFSeq.size()>0)
						gc->pathPlanDOF = gc->pathPlanDOFSeq[i];

				VectorXd cfull(gc->jointSeq[i].size()+6);
				cfull << gc->jointSeq[i], gc->body()->link(0)->p, rpyFromRot(gc->body()->link(0)->R);
				config_tmp.push_back(cfull);

				cfull << gc->jointSeq[i+1], gc->body()->link(0)->p, rpyFromRot(gc->body()->link(0)->R);
				config_tmp.push_back(cfull);

				bool success = planner.call_planner(config_tmp, gc->pathPlanDOF);
				if(success) planner.call_smoother(config_tmp);
				else        successAll = false;

				for(unsigned int j=0; j<config_tmp.size(); j++){
					int k=i, l=i+1;
					if(j==0)                   l=i;
					if(j==config_tmp.size()-1) k=i+1;

					motionSeq.push_back( MotionState( config_tmp[j], gc->graspingStateSeq[k], gc->graspingStateSeq2[k]) );

					if(gc->motionTimeSeq.size()>0)
								(motionSeq.back()).motionTime = gc->motionTimeSeq[l];

					if(j<config_tmp.size()-1 || i==gc->jointSeq.size()-2)
							gc->graspMotionSeq.push_back(motionSeq.back());
				}

			}
		}else */

		// register IK link index
		vector<int> baseindex_list;
		vector<int> eeindex_list;
		const Mapping& info = *gc->body()->info();
		const Listing& defaultIkLinks = *info.findListing("defaultIkInterpolationLinks");
		const Mapping& setupMap = *info.findMapping("defaultIKsetup");
		if(defaultIkLinks.isValid()){
			for(int i=0; i< defaultIkLinks.size(); ++i){
				if(setupMap.isValid()){
					const Listing& setup = *setupMap.findListing(gc->body()->link(defaultIkLinks[i])->name());
					if(setup.isValid() && !setup.empty()){
						baseindex_list.push_back(gc->body()->link(defaultIkLinks[i])->index());
						eeindex_list.push_back(gc->body()->link(setup[0].toString())->index());
					}
				}
			}
		}
		vector<vector<Vector3> > basepos;
		vector<vector<Vector3> > eepos;
		vector<vector<Matrix3> > baserpy;
		vector<vector<Matrix3> > eerpy;
		basepos.resize(baseindex_list.size());
		eepos.resize(eeindex_list.size());
		baserpy.resize(baseindex_list.size());
		eerpy.resize(eeindex_list.size());

		if(inputMotionSeq.size() > 1){
			gc->graspMotionSeq.clear();
			PRMParams params;
			params.loadParams();
			for(unsigned int i=0; i<inputMotionSeq.size()-1; i++){
#if DEBUG_MOTIONFILE_INPUT
				cout <<"PRM: "<<  i <<" th input motion" << endl;
#endif
				config_tmp.clear();

				gc->setMotionState(inputMotionSeq[i]);
				//if(gc->graspMotionSeq[i].tolerance >=0) gc->setTolerance(gc->graspMotionSeq[i].tolerance);
				VectorXd cfull(inputMotionSeq[i].jointSeq.size()+6);
				cfull << inputMotionSeq[i].jointSeq, inputMotionSeq[i].pos, inputMotionSeq[i].rpy;
				config_tmp.push_back(cfull);

				cfull << inputMotionSeq[i+1].jointSeq, inputMotionSeq[i+1].pos, inputMotionSeq[i+1].rpy;
				config_tmp.push_back(cfull);
				bool success;
				std::string error_message = "";
				if (params.param.algo == PRMParams::ALGO_SBL) {
						success = planner.call_planner(config_tmp, inputMotionSeq[i].pathPlanDOF);
						if (!success) error_message = planner.error_message();
				} else if (params.param.algo == PRMParams::ALGO_RRT) {
						PathEngine::PathPlanner rrtplanner(gc->robotBody(), gc->robotBody()->numJoints()+6, inputMotionSeq[i].pathPlanDOF);
						PathEngine::RRT rrt(&rrtplanner);
						rrtplanner.setAlgorithm(&rrt);
						success = rrtplanner.calcPath(config_tmp);
						if (!success) error_message = rrtplanner.error_message();
				} else if (params.param.algo == PRMParams::ALGO_RRTSTAR) {
						PathEngine::PathPlanner rrtplanner(gc->robotBody(), gc->robotBody()->numJoints()+6, inputMotionSeq[i].pathPlanDOF);
						PathEngine::RRTStar rrtstar(&rrtplanner);
						rrtplanner.setAlgorithm(&rrtstar);
						success = rrtplanner.calcPath(config_tmp);
						if (!success) error_message = rrtplanner.error_message();
				} else if (params.param.algo == PRMParams::ALGO_PRM) {
						PathEngine::PathPlanner rrtplanner(gc->robotBody(), gc->robotBody()->numJoints()+6, inputMotionSeq[i].pathPlanDOF);
						PathEngine::PRM prm(&rrtplanner);
						rrtplanner.setAlgorithm(&prm);
						success = rrtplanner.calcPath(config_tmp);
						if (!success) error_message = rrtplanner.error_message();
				} else {
						std::cerr << "algorithm is not selected!!" << std::endl;
				}

				if (verbose) {
						if (!success) {
								os << " PRM: fail " << i << " th input motion (" << error_message << ")" << endl;
						}
				}

				isTrajectorySectionSucceed.push_back(success);

				if(!success) successAll=false;

				if(useSafeBB) grasp::PlanBase::instance()->useRobotSafeBoundingBox=true;
				if (params.param.algo == PRMParams::ALGO_SBL || params.param.algo == PRMParams::ALGO_RRT || params.param.algo == PRMParams::ALGO_PRM) {
						planner.call_smoother(config_tmp);
				}
				if(useSafeBB) grasp::PlanBase::instance()->useRobotSafeBoundingBox=false;

				for(int j=0;j<gc->robotBody()->numJoints();j++){
						gc->robotBody()->joint(j)->q() = inputMotionSeq[i+1].jointSeq[j];
				}
				gc->body()->link(0)->p() = inputMotionSeq[i+1].pos;
				gc->body()->link(0)->R() = rotFromRpy(inputMotionSeq[i+1].rpy);
				gc->calcForwardKinematics();

				vector<VectorXd> config_tmp2;

				for(unsigned int j=0; j<config_tmp.size()-1; j++){
					double dyaw = fabs( config_tmp[j][gc->robotBody()->numJoints()+5] - config_tmp[j+1][gc->robotBody()->numJoints()+5] ) ;
					int div = dyaw/3.0;
					div +=1;
					for(int i=0;i<div;i++){
						config_tmp2.push_back( config_tmp[j+1]*i/div + config_tmp[j]*(div-i)/div);
					}
				}
				config_tmp2.push_back(config_tmp.back());

				for(unsigned int j=0; j<config_tmp2.size(); j++){
					int l=i+1;
					if(j==0) l=i;

					gc->body()->link(0)->p() = config_tmp2[j].segment<3>(gc->robotBody()->numJoints());
					gc->body()->link(0)->R() = rotFromRpy( config_tmp2[j].segment<3>(gc->robotBody()->numJoints()+3) );
					for(int k=0;k<gc->robotBody()->numJoints();k++) gc->robotBody()->joint(k)->q() = config_tmp2[j][k];
					gc->setInterLink();
					gc->calcForwardKinematics();
					MotionState temp = gc->getMotionState();
					motionSeq.push_back( temp );

					if(outputGraspMotionSeq){
						if( l < inputMotionSeq.size()  )  temp.motionTime = inputMotionSeq[l].motionTime;
//						if( l < gc->motionTimeSeq.size()  )  temp.motionTime = gc->motionTimeSeq[l];
						if(j < config_tmp2.size()-1 || i==inputMotionSeq.size()-2){
							gc->graspMotionSeq.push_back(temp);
						}
					}
				}
			}
		}
		else{
			return false;
		}

		if(baseindex_list.size()>0){
			for(unsigned int i=0; i<inputMotionSeq.size(); i++){
				for(int j=0;j<gc->robotBody()->numJoints();j++){
					gc->robotBody()->joint(j)->q() = inputMotionSeq[i].jointSeq[j];
				}
				gc->calcForwardKinematics();
				if(i > 0){   // initial pos/attitude is recorded only once
					// motion start position and attitude
					for(int k=0; k<baseindex_list.size(); ++k){
						basepos[k].push_back(gc->robotBody()->link(baseindex_list[k])->p());
						eepos[k].push_back(gc->robotBody()->link(eeindex_list[k])->p());
						baserpy[k].push_back(gc->robotBody()->link(baseindex_list[k])->R());
						eerpy[k].push_back(gc->robotBody()->link(eeindex_list[k])->R());
					}
				}
				// motion end position and attitude
				for(int k=0; k<baseindex_list.size(); ++k){
					basepos[k].push_back(gc->robotBody()->link(baseindex_list[k])->p());
					eepos[k].push_back(gc->robotBody()->link(eeindex_list[k])->p());
					baserpy[k].push_back(gc->robotBody()->link(baseindex_list[k])->R());
					eerpy[k].push_back(gc->robotBody()->link(eeindex_list[k])->R());
				}
			}
		}

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		if(gc->targetObject){
			gc->object()->p() = gc->objVisPos();
			gc->object()->R() = gc->objVisRot();
		}

		gc->setMotionState(tempState);
		gc->robotBody()->calcForwardKinematics();


		PosePtr pose = new Pose(gc->bodyItemRobot()->body()->numJoints());
		for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
                pose->setJointPosition(gc->bodyItemRobot()->body()->joint(i)->jointId(), gc->bodyItemRobot()->body()->joint(i)->q());
		}

		PosePtr poseObject = new Pose(1);
		if(gc->targetObject){
			poseObject->setBaseLink(0, gc->objVisPos(), gc->objVisRot());
			poseSeqItemObject->poseSeq()->insert(poseSeqItemObject->poseSeq()->end(), 0 , poseObject);
		}

		vector<VectorXd>::iterator it = config.begin();
		double time = 0;
		vector<double> key_time;       // times specified in motion.dat
		for(int j=1;j<inputMotionSeq.size(); ++j){
			key_time.push_back(inputMotionSeq[j].startTime);   // command start time
			key_time.push_back(inputMotionSeq[j].endTime);         // command end time
		}
		for(int j=0;j<motionSeq.size();j++){
			PosePtr pose_ = new Pose(*pose);
			for(int k=0; k<baseindex_list.size(); ++k){
				pose_->setBaseLink(baseindex_list[k], basepos[k][j], baserpy[k][j]);
				pose_->setBaseLink(eeindex_list[k], eepos[k][j], eerpy[k][j]);
			}
			pose_->setBaseLink(0, motionSeq[j].pos, rotFromRpy(motionSeq[j].rpy) );
			for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
				pose_->setJointPosition(i, motionSeq[j].jointSeq[i]);
			}
			motionSeq[j].endTime = time;
			motionSeq[j].time = time;
			poseSeqItemRobot->poseSeq()->insert(poseSeqItemRobot->poseSeq()->end(), time , pose_);

			// generate pose sequence
			for (size_t pi = 0; pi < poseSeqItemAccompanyings_.size(); pi++) {
					PosePtr pose = createPose(accompanying_objects_[pi], motionSeq[j]);
					poseSeqItemAccompanyings_[pi]->poseSeq()->insert(poseSeqItemAccompanyings_[pi]->poseSeq()->end(), time, pose);
			}

#ifdef USE_EXACT_TIME
			time += key_time[j+1] - key_time[j];    // command run time or interval between commands
#else
			time += 1.0;
#endif
		}
		poseSeqItemRobot->updateInterpolation();
		poseSeqItemRobot->updateTrajectory();
		ItemTreeView::mainInstance()->selectItem( poseSeqItemRobot->bodyMotionItem() );

		// make trajectories
		for (size_t pi = 0; pi < poseSeqItemAccompanyings_.size(); pi++) {
				poseSeqItemAccompanyings_[pi]->updateInterpolation();
				poseSeqItemAccompanyings_[pi]->updateTrajectory();
				ItemTreeView::instance()->selectItem(poseSeqItemAccompanyings_[pi]->bodyMotionItem());
		}

		//outputTrajectoryForOpenHRP(poseSeqItemRobot->bodyMotionItem()->motion());

		if(gc->targetObject){
            gc->object()->p() = gc->objVisPos();
            gc->object()->R() = gc->objVisRot();
			poseSeqItemObject->updateInterpolation();
			// updateTrajectoryFromMotion(poseSeqItemObject->bodyMotionItem()->motion(), poseSeqItemRobot->bodyMotionItem()->motion(),motionSeq);
			poseSeqItemObject->bodyMotionItem()->notifyUpdate();
			ItemTreeView::mainInstance()->selectItem( poseSeqItemObject->bodyMotionItem() );
		}

		updateTrajectoriesFromMotion(motionSeq, (gc->targetObject != NULL));
/*
		DIR *pDir;
		pDir = opendir("extplugin/graspPlugin/RobotInterface");

		if(pDir != NULL){

				ofstream gout("extplugin/graspPlugin/RobotInterface/data/grasp.mat");

				double p = p=180.0/3.1415;
				gout <<"6 7";
				for(unsigned int i=1; i<motionSeq.size(); i++){
						gout << endl;
						for(int j=0;j<gc->bodyItemRobot()->body()->numJoints();j++)
								gout << motionSeq[i].jointSeq[j]*p << " ";
						gout << motionSeq[i].graspingState;

				}
		}
		*/
		return successAll;
}

bool TrajectoryPlanner::simpleTrajectoryPlanning() {
		PlanBase* gc = PlanBase::instance();
	
		if(gc->graspMotionSeq.size()< 2){
			cout << "push to graspMOtionSeq" << endl;
			return false;
		}
		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		MotionState tempState = gc->getMotionState();
		gc->initialCollision();

		vector<MotionState> inputMotionSeq;
		bool outputGraspMotionSeq = true;
		isTrajectorySectionSucceed.clear();

		inputMotionSeq = gc->graspMotionSeq;

		grasp::mpkInterface planner(gc->robotBody(), gc->bodyItemEnv);

		vector<VectorXd> config, config_tmp;

		bool successAll=true;

		// register IK link index
		vector<int> baseindex_list;
		vector<int> eeindex_list;
		const Mapping& info = *gc->body()->info();
		const Listing& defaultIkLinks = *info.findListing("defaultIkInterpolationLinks");
		const Mapping& setupMap = *info.findMapping("defaultIKsetup");
		if(defaultIkLinks.isValid()){
			for(int i=0; i< defaultIkLinks.size(); ++i){
				if(setupMap.isValid()){
					const Listing& setup = *setupMap.findListing(gc->body()->link(defaultIkLinks[i])->name());
					if(setup.isValid() && !setup.empty()){
						baseindex_list.push_back(gc->body()->link(defaultIkLinks[i])->index());
						eeindex_list.push_back(gc->body()->link(setup[0].toString())->index());
					}
				}
			}
		}
		vector<vector<Vector3> > basepos;
		vector<vector<Vector3> > eepos;
		vector<vector<Matrix3> > baserpy;
		vector<vector<Matrix3> > eerpy;
		basepos.resize(baseindex_list.size());
		eepos.resize(eeindex_list.size());
		baserpy.resize(baseindex_list.size());
		eerpy.resize(eeindex_list.size());

		if(inputMotionSeq.size() > 1){
			gc->graspMotionSeq.clear();
			PRMParams params;
			params.loadParams();
			for(unsigned int i=0; i<inputMotionSeq.size()-1; i++){
#if DEBUG_MOTIONFILE_INPUT
				cout <<"PRM: "<<  i <<" th input motion" << endl;
#endif
				config_tmp.clear();

				gc->setMotionState(inputMotionSeq[i]);
				//if(gc->graspMotionSeq[i].tolerance >=0) gc->setTolerance(gc->graspMotionSeq[i].tolerance);
				VectorXd cfull(inputMotionSeq[i].jointSeq.size()+6);
				cfull << inputMotionSeq[i].jointSeq, inputMotionSeq[i].pos, inputMotionSeq[i].rpy;
				config_tmp.push_back(cfull);

				cfull << inputMotionSeq[i+1].jointSeq, inputMotionSeq[i+1].pos, inputMotionSeq[i+1].rpy;
				config_tmp.push_back(cfull);
				bool success;
				std::string error_message = "";
				if (params.param.algo == PRMParams::ALGO_SBL) {
						success = planner.call_planner(config_tmp, inputMotionSeq[i].pathPlanDOF);
						if (!success) error_message = planner.error_message();
				} else if (params.param.algo == PRMParams::ALGO_RRT) {
						PathEngine::PathPlanner rrtplanner(gc->robotBody(), gc->robotBody()->numJoints()+6, inputMotionSeq[i].pathPlanDOF);
						PathEngine::RRT rrt(&rrtplanner);
						rrtplanner.setAlgorithm(&rrt);
						success = rrtplanner.calcPath(config_tmp);
						if (!success) error_message = rrtplanner.error_message();
				} else if (params.param.algo == PRMParams::ALGO_RRTSTAR) {
						PathEngine::PathPlanner rrtplanner(gc->robotBody(), gc->robotBody()->numJoints()+6, inputMotionSeq[i].pathPlanDOF);
						PathEngine::RRTStar rrtstar(&rrtplanner);
						rrtplanner.setAlgorithm(&rrtstar);
						success = rrtplanner.calcPath(config_tmp);
						if (!success) error_message = rrtplanner.error_message();
				} else {
						std::cerr << "algorithm is not selected!!" << std::endl;
				}

				if (verbose) {
						if (!success) {
								os << " PRM: fail " << i << " th input motion (" << error_message << ")" << endl;
						}
				}

				isTrajectorySectionSucceed.push_back(success);

				if(!success) successAll=false;

				if(useSafeBB) grasp::PlanBase::instance()->useRobotSafeBoundingBox=true;
				if (params.param.algo == PRMParams::ALGO_SBL || params.param.algo == PRMParams::ALGO_RRT || params.param.algo == PRMParams::ALGO_PRM) {
						planner.call_smoother(config_tmp);
				}
				if(useSafeBB) grasp::PlanBase::instance()->useRobotSafeBoundingBox=false;

				for(int j=0;j<gc->robotBody()->numJoints();j++){
						gc->robotBody()->joint(j)->q() = inputMotionSeq[i+1].jointSeq[j];
				}
				gc->body()->link(0)->p() = inputMotionSeq[i+1].pos;
				gc->body()->link(0)->R() = rotFromRpy(inputMotionSeq[i+1].rpy);
				gc->calcForwardKinematics();

				vector<VectorXd> config_tmp2;

				for(unsigned int j=0; j<config_tmp.size()-1; j++){
					double dyaw = fabs( config_tmp[j][gc->robotBody()->numJoints()+5] - config_tmp[j+1][gc->robotBody()->numJoints()+5] ) ;
					int div = dyaw/3.0;
					div +=1;
					for(int i=0;i<div;i++){
						config_tmp2.push_back( config_tmp[j+1]*i/div + config_tmp[j]*(div-i)/div);
					}
				}
				config_tmp2.push_back(config_tmp.back());

				for(unsigned int j=0; j<config_tmp2.size(); j++){
					int l=i+1;
					if(j==0) l=i;

					gc->body()->link(0)->p() = config_tmp2[j].segment<3>(gc->robotBody()->numJoints());
					gc->body()->link(0)->R() = rotFromRpy( config_tmp2[j].segment<3>(gc->robotBody()->numJoints()+3) );
					for(int k=0;k<gc->robotBody()->numJoints();k++) gc->robotBody()->joint(k)->q() = config_tmp2[j][k];
					gc->setInterLink();
					gc->calcForwardKinematics();
					MotionState temp = gc->getMotionState();
					motionSeq.push_back( temp );

					if(outputGraspMotionSeq){
						if( l < inputMotionSeq.size()  )  temp.motionTime = inputMotionSeq[l].motionTime;
//						if( l < gc->motionTimeSeq.size()  )  temp.motionTime = gc->motionTimeSeq[l];
						if(j < config_tmp2.size()-1 || i==inputMotionSeq.size()-2){
							gc->graspMotionSeq.push_back(temp);
						}
					}
				}
			}
		}
		else{
			return false;
		}

		return successAll;
}


const std::vector<cnoid::PoseSeqItemPtr>& TrajectoryPlanner::poseSeqItemAccompanyings() const {
		return poseSeqItemAccompanyings_;
}

bool TrajectoryPlanner::updateTrajectoriesFromMotion(const std::vector<MotionState>& motionSeq, bool has_object) {
		PlanBase* gc= PlanBase::instance();
		BodyPtr robotBody = gc->bodyItemRobot()->body();
		const int numJointsRobot = robotBody->numJoints();

		BodyMotionPtr motionRobot = poseSeqItemRobot->bodyMotionItem()->motion();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		MultiValueSeqPtr qseqRobot = motionRobot->jointPosSeq();
		MultiSE3SeqPtr pseqRobot = motionRobot->linkPosSeq();

		// initial settings
		const int num_accompanyings = poseSeqItemAccompanyings_.size();
		std::vector<MultiValueSeqPtr> qseqAccompanyings(num_accompanyings);
		std::vector<MultiSE3SeqPtr> pseqAccompanyings(num_accompanyings);
		std::vector<int> numJointsAccompanyings(num_accompanyings);
		std::vector<BodyPtr> AccompanyingsBody(num_accompanyings);
		for (int i = 0; i < num_accompanyings; i++) {
				AccompanyingsBody[i] = poseSeqItemAccompanyings_[i]->findOwnerItem<BodyItem>()->body();
				BodyMotionPtr motion = poseSeqItemAccompanyings_[i]->bodyMotionItem()->motion();
				motion->setDimension(numFrames, AccompanyingsBody[i]->numJoints(), AccompanyingsBody[i]->numLinks());
				motion->setFrameRate(frameRate);
				qseqAccompanyings[i] = motion->jointPosSeq();
				pseqAccompanyings[i] = motion->linkPosSeq();
				numJointsAccompanyings[i] = qseqAccompanyings[i]->getNumParts();
		}

		// initial setting for object motion
		BodyPtr objectBody;
		MultiSE3SeqPtr pseqObj;
		if (has_object) {
				objectBody = gc->targetObject->bodyItemObject->body();
				BodyMotionPtr motionObject = poseSeqItemObject->bodyMotionItem()->motion();
				motionObject->setDimension(numFrames, objectBody->numJoints(), objectBody->numLinks());
				motionObject->setFrameRate(frameRate);

				pseqObj = motionObject->linkPosSeq();
		}

		// set robot to inital state
		vector<MotionState>::const_iterator itg = motionSeq.begin();
		gc->setHandObjectState( itg->handObjectState );
		gc->setGraspingState( itg->graspingState );
		gc->setGraspingState2( itg->graspingState2 );
		itg++;
		if (has_object) {
				gc->object()->p() = gc->objVisPos();
				gc->object()->R() = gc->objVisRot();
		}

		gc->setMotionState(*itg);

		for (int frame = 0; frame < numFrames; ++frame) {
			if( (itg != motionSeq.end()) && ( itg->time <= ((double)frame)/frameRate) ){
				gc->setHandObjectState( itg->handObjectState );
				gc->robotBody()->calcForwardKinematics();
				gc->setGraspingState( itg->graspingState );
				gc->setGraspingState2( itg->graspingState2 );
				itg++;
			}

			// set robot
			SE3& p_robot = pseqRobot->at(frame, 0);
			robotBody->link(0)->p() = p_robot.translation();
			robotBody->link(0)->R() = Matrix3(p_robot.rotation());

			for (int i = 0; i < numJointsRobot; ++i) {
					robotBody->joint(i)->q() = qseqRobot->at(frame, i);
			}

			// update joints
			for (int i = 0; i < num_accompanyings; ++i) {
					for (int j = 0; j < numJointsAccompanyings[i]; ++j) {
							AccompanyingsBody[i]->joint(j)->q() = qseqAccompanyings[i]->at(frame, j);
					}
			}

			gc->calcForwardKinematics();

			// set root link position at current frame
			for (int i = 0; i < num_accompanyings; ++i) {
					SE3& p_accompanyings = pseqAccompanyings[i]->at(frame, 0);
					p_accompanyings.set(AccompanyingsBody[i]->rootLink()->p(), AccompanyingsBody[i]->rootLink()->R());
			}

			if (has_object) {
					SE3& p_obj = pseqObj->at(frame, 0);
					p_obj.set(gc->object()->p(), gc->object()->R());
			}
		}

		if (has_object) {
				// store the moved state
				objectBody->rootLink()->p() = gc->object()->p();
				objectBody->rootLink()->R() = gc->object()->R();
				objectBody->calcForwardKinematics();
		}
		for (int i = 0; i < num_accompanyings; ++i) {
				AccompanyingsBody[i]->calcForwardKinematics();
		}

		return true;
}

PosePtr TrajectoryPlanner::createPose(ObjectBase* object, const MotionState& target_motion) const {
		const cnoid::BodyPtr target_body = object->bodyItem()->body();
		PosePtr pose = new Pose(target_body->numJoints());
		pose->setBaseLink(0, target_body->rootLink()->p(), target_body->rootLink()->R());
		if (object->isHand()) {
				// robot hand
				for (int i = 0; i < target_body->numJoints(); i++) {
						pose->setJointPosition(i, target_motion.jointSeq[target_body->joint(i)->jointId()]);
				}
		} else {
				for (int i = 0; i < target_body->numJoints(); i++) {
						pose->setJointPosition(target_body->joint(i)->jointId(), target_body->joint(i)->q());
				}
		}
		return pose;
}

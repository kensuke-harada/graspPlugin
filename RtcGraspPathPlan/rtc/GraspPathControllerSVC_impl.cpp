// -*-C++-*-
/*!
 * @file  GraspPathControllerSVC_impl.cpp
 * @brief Service implementation code of GraspPathController.idl
 *
 */
#include <boost/bind.hpp>
#include <cnoid/EigenTypes>
#include <cnoid/LazyCaller>

#include "GraspPathControllerSVC_impl.h"
#include "../GraspPathController.h"

using namespace std;
using namespace cnoid;
using namespace boost;
using namespace grasp;
/*
 * Example implementational code for IDL interface planGraspPath
 */
planGraspPathSVC_impl::planGraspPathSVC_impl()
{
  // Please add extra constructor code here.
}


planGraspPathSVC_impl::~planGraspPathSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */

void planGraspPathSVC_impl::GraspPlanningStart(
	planGraspPath::ULONG mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end,
	const char* robotId, const char* objectTagId, ::CORBA::Double resolution,
	planGraspPath::ManipInfoSeq_out trajectory, planGraspPath::ULONG& state)
{
	int mode_ = mode;
	string robotId_ (robotId);
	string objectTagId_ (objectTagId);
	double resolution_ = resolution;

	vector <double> begin_;
	vector <double> end_;
	for(unsigned int i=0; i<begin.length(); i++){
		begin_.push_back(begin[i]);
	}
	for(unsigned int i=0; i<end.length(); i++){
		end_.push_back(end[i]);
	}

	int state_=0;
	vector <grasp::pathInfo> trajectory_;

//	bool success = grasp::GraspPathController::instance()->graspPathPlanStart(mode_,begin_,end_,robotId_,objectTagId_,resolution_,trajectory_,state_);

	callSynchronously(bind(&GraspPathController::graspPathPlanStart,GraspPathController::instance(),mode_,begin_,end_,robotId_,objectTagId_,resolution_,&trajectory_,&state_));
	bool success = false;
	if(state_ == 0) success = true;


	state = state_;
	trajectory = new planGraspPath::ManipInfoSeq ;
	if(!success){ // if planner return fail
		trajectory->length(1);
		(*trajectory)[0].state = planGraspPath::APROACH;
		(*trajectory)[0].pos.length(begin.length());
		for(int j=0; j< (int)begin.length();j++){
			(*trajectory)[0].pos[j] =  begin[j];
		}
		return;
	}

	trajectory->length(trajectory_.size());
	for(int i=0; i< (int)trajectory_.size();i++){
		planGraspPath::ManipInfo temp;
		(*trajectory)[i].state = (planGraspPath::MotionState)trajectory_[i].state;
		(*trajectory)[i].pos.length(begin.length());
		for(int j=0; j< (int)begin.length();j++){
			(*trajectory)[i].pos[j] =  trajectory_[i].pos[j];
		}
	}
	return;
}

void planGraspPathSVC_impl::ReleasePlanningStart(
	planGraspPath::ULONG mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end,
	const char* robotId, const char* objectTagId, ::CORBA::Double resolution,
	planGraspPath::ManipInfoSeq_out trajectory, planGraspPath::ULONG& state)
{
	int mode_ = mode;
	string robotId_ (robotId);
	string objectTagId_ (objectTagId);
	double resolution_ = resolution;

	vector <double> begin_;
	vector <double> end_;
	for(unsigned int i=0; i<begin.length(); i++){
		begin_.push_back(begin[i]);
	}
	for(unsigned int i=0; i<end.length(); i++){
		end_.push_back(end[i]);
	}

	int state_=0;
	vector <grasp::pathInfo> trajectory_;

//	bool success = grasp::GraspPathController::instance()->releasePathPlanStart(mode_,begin_,end_,robotId_,objectTagId_,resolution_,trajectory_,state_);

	callSynchronously(bind(&GraspPathController::releasePathPlanStart,GraspPathController::instance(),mode_,begin_,end_,robotId_,objectTagId_,resolution_,&trajectory_,&state_));
	bool success = false;
	if(state_ == 0) success = true;

	state = state_;
	trajectory = new planGraspPath::ManipInfoSeq ;
	if(!success){ // if planner return fail
		trajectory->length(1);
		(*trajectory)[0].state = planGraspPath::APROACH;
		(*trajectory)[0].pos.length(begin.length());
		for(int j=0; j< (int)begin.length();j++){
			(*trajectory)[0].pos[j] =  begin[j];
		}
		return;
	}

	trajectory->length(trajectory_.size());
	for(int i=0; i< (int)trajectory_.size();i++){
		planGraspPath::ManipInfo temp;
		(*trajectory)[i].state = (planGraspPath::MotionState)trajectory_[i].state;
		(*trajectory)[i].pos.length(begin.length());
		for(int j=0; j< (int)begin.length();j++){
			(*trajectory)[i].pos[j] =  trajectory_[i].pos[j];
		}
	}
	return;
}

void planGraspPathSVC_impl::SetStatusObject(planGraspPath::ObjectInputMode mode, const planGraspPath::ObjectInfo& obj)
{
//	grasp::GraspPathController *gp= grasp::GraspPathController::instance();

	int objId = obj.objId;
	string tagId ( obj.tagId);

	if(mode == planGraspPath::SET_TOLERANCE){
		callSynchronously(bind(&GraspPathController::setTolerance,GraspPathController::instance(),obj.tolerance));
		return;
	}

	if(mode == planGraspPath::CREATE_RECORD){
		callSynchronously(bind(&GraspPathController::createRecord,GraspPathController::instance(),objId,tagId));
//		grasp::GraspPathController::instance()->createRecord(objId,tagId);
	}
	if(mode == planGraspPath::REMOVE_RECORD){
		callSynchronously(bind(&GraspPathController::deleteRecord,GraspPathController::instance(),tagId));
//		grasp::GraspPathController::instance()->deleteRecord(tagId);
		return;
	}
	if(mode == planGraspPath::APPEAR){
		callSynchronously(bind(&GraspPathController::appear,GraspPathController::instance(),tagId));
//		grasp::GraspPathController::instance()->appear(tagId);
	}
	if(mode == planGraspPath::DISAPPEAR){
		callSynchronously(bind(&GraspPathController::disappear,GraspPathController::instance(),tagId));
//		grasp::GraspPathController::instance()->disappear(tagId);
	}
	if(tagId == "ALLID"){
		return;
	}

	cnoid::Vector3 pos (obj.pos[0],obj.pos[1],obj.pos[2]);
	Matrix3 ori;
	ori << obj.ori[0] , obj.ori[1] , obj.ori[2] , obj.ori[3] , obj.ori[4] , obj.ori[5] , obj.ori[6] , obj.ori[7] ,  obj.ori[8] ;

	callSynchronously(bind(&GraspPathController::setPos,GraspPathController::instance(),tagId,pos,ori));
//	grasp::GraspPathController::instance()->setPos(tagId,pos,ori);

//	cout << "Set status finished"  << gp->objTag2Item().size() <<endl;
	//map<string,BodyItemPtr>::iterator it = gp->objTag2Item().begin();
	//for(int i=0;i< gp->objTag2Item().size();i++){
	//	cout << it->first << endl;
	//	it++;
	//}

}



// End of example implementational code

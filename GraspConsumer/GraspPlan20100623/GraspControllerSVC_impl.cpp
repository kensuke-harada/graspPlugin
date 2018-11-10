// -*-C++-*-
/*!
 * @file  GraspControllerSVC_impl.cpp
 * @brief Service implementation code of GraspController.idl
 *
 */
#include <iostream>

#include "GraspControllerSVC_impl.h"
#include "GraspPlanningImpl.h"

using namespace dmatrix;

/*
 * Example implementational code for IDL interface GraspPlanStart
 */
GraspPlanStartSVC_impl::GraspPlanStartSVC_impl()
{
  // Please add extra constructor code here.



}


GraspPlanStartSVC_impl::~GraspPlanStartSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void GraspPlanStartSVC_impl::GraspPlanningStart(CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, CORBA::ULong& sstate)
{
	int objId_ = ObjId;

	dVector3 objPos_;
	COPY_DVECTOR3(objPos_, objPos);

	dMatrix33 objOri_;
	COPY_DMATRIX33(objOri_, objOri);

//	objId_ = 1;
	if( !GraspPlanning_impl::instance()->SortGraspPosition(objId_, objPos_,objOri_) ){
		std::cout << " ERROR: Load Object Information" << std::endl;
		sstate = 1;
		return;
	}
//	selectgraspposition
	GraspPlanResult::DblSequence3 GraspPos,ApproachPos;
	GraspPlanResult::DblSequence9 GraspOri,ApproachOri;

	GraspPos.length(3);
	GraspOri.length(9);
	ApproachPos.length(3);
	ApproachOri.length(9);
	CORBA::Double angle=0;
	CORBA::ULong rstate=0;
	CORBA::ULong isContinue=1;

	int rstate_;
	while(isContinue){
		dVector3 GraspPos_, ApproachPos_;
		dMatrix33 GraspOri_, ApproachOri_;
		double angle_;
		GraspPlanning_impl::instance()->returnGraspPosition(GraspPos_,GraspOri_,ApproachPos_,ApproachOri_,angle_,rstate_);

		for(int i=0; i<3; i++){
			GraspPos[i] = GraspPos_[i];
			ApproachPos[i] = ApproachPos_[i];
			for(int j=0; j<3; j++){
				GraspOri[3*i+j] = GraspOri_(i,j);
				ApproachOri[3*i+j] = ApproachOri_(i,j);
			}
		}
		angle = angle_;
		rstate = rstate_;

 		dVector3 offsetPos;
 		for (int i = 0; i < 3; i++) {
 			offsetPos[i] = c_Position[i];
 		}
 		dMatrix33 offsetOri ;
		COPY_DMATRIX33(offsetOri, c_Posture);

		GraspPos_ =  GraspPos_ +  prod(GraspOri_ , offsetPos);
		ApproachPos_ =  ApproachPos_ +  prod(ApproachOri_, offsetPos);

		GraspOri_ = prod(offsetOri, GraspOri_);
 		ApproachOri_ = prod(offsetOri, ApproachOri_);

		(*c_Result)->GraspPlanningResult(GraspPos, GraspOri, ApproachPos, ApproachOri, angle, rstate, isContinue);

		if(rstate_ > 0) break;
	}

	if(rstate_ == 0) sstate=0; //success
	else sstate=1; //false;

	return;
}

void GraspPlanStartSVC_impl::ReleasePlanningStart(CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, CORBA::ULong& state)
{
  // Please insert your code here and remove the following warning pragma
	std::cout << "NOT implemented " << std::endl;
}



// End of example implementational code

/*
 * Example implementational code for IDL interface GraspPlanResult
 */
GraspPlanResultSVC_impl::GraspPlanResultSVC_impl()
{
  // Please add extra constructor code here.
}


GraspPlanResultSVC_impl::~GraspPlanResultSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void GraspPlanResultSVC_impl::GraspPlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue)
{
  // Please insert your code here and remove the following warning pragma

}


void GraspPlanResultSVC_impl::ReleasePlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue)
{
  // Please insert your code here and remove the following warning pragma

}

// End of example implementational code




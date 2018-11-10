// -*-C++-*-
/*!
 * @file  GraspControllerSVC_impl.cpp
 * @brief Service implementation code of GraspController.idl
 *
 */
#include <iostream>

#include "GraspControllerSVC_impl.h"
#include "GraspRtcController.h"

using namespace std;
using namespace grasp;

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
void GraspPlanStartSVC_impl::GraspPlanningStart(::CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, ::CORBA::ULong& state)
{
  // Please insert your code here and remove the following warning pragma
}

void GraspPlanStartSVC_impl::ReleasePlanningStart(::CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, ::CORBA::ULong& state)
{
  // Please insert your code here and remove the following warning pragma
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
void GraspPlanResultSVC_impl::GraspPlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, const GraspPlanResult::DblSequence& angle, ::CORBA::ULong state, ::CORBA::ULong& isContinue)
{
  // Please insert your code here and remove the following warning pragma
	if(state > 0){
		isContinue=0;
		std::cout << "Grasp Plan failed" << std::endl;	
		return;
	}
	cout << GraspPos[0] << endl;
	
	bool flag;
	flag= grasp::GraspRtcController::instance()->graspPlanResult(GraspPos, GraspOri, ApproachPos, ApproachOri, angle);
	//flag= RtcController::graspPlanResult(GraspPos, GraspOri, ApproachPos, ApproachOri, angle);
	if(flag){
		isContinue=0;
		std::cout << "success" << endl;
	}else{
		isContinue=1;
		std::cout << "fail" << endl;
	}
	return;
}

void GraspPlanResultSVC_impl::ReleasePlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, const GraspPlanResult::DblSequence& angle, ::CORBA::ULong state, ::CORBA::ULong& isContinue)
{
  // Please insert your code here and remove the following warning pragma
	std::cout << "Release Plan" << std::endl;	
	GraspPlanningResult(GraspPos, GraspOri, ApproachPos, ApproachOri, angle, state, isContinue);
	return ;
}



// End of example implementational code




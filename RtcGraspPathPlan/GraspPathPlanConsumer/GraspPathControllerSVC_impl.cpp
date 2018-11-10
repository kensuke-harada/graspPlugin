// -*-C++-*-
/*!
 * @file  GraspPathControllerSVC_impl.cpp
 * @brief Service implementation code of GraspPathController.idl
 *
 */

#include "GraspPathControllerSVC_impl.h"

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
void planGraspPathSVC_impl::GraspPlanningStart(planGraspPath::ULONG mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end, const char* robotId, const char* objectTagId, ::CORBA::Double resolution, planGraspPath::ManipInfoSeq_out trajectory, planGraspPath::ULONG& state)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void planGraspPathSVC_impl::GraspPlanningStart(const planGraspPath::ULONG& mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end, const char* robotId, const char* objectTagId, CORBA::Double resolution, const planGraspPath::ManipInfoSeq&& trajectory, const planGraspPath::ULONG&& state)>"
#endif
}


void planGraspPathSVC_impl::ReleasePlanningStart(planGraspPath::ULONG mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end, const char* robotId, const char* objectTagId, ::CORBA::Double resolution, planGraspPath::ManipInfoSeq_out trajectory, planGraspPath::ULONG& state)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void planGraspPathSVC_impl::ReleasePlanningStart(const planGraspPath::ULONG& mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end, const char* robotId, const char* objectTagId, CORBA::Double resolution, const planGraspPath::ManipInfoSeq&& trajectory, const planGraspPath::ULONG&& state)>"
#endif
}

void planGraspPathSVC_impl::SetStatusObject(planGraspPath::ObjectInputMode mode, const planGraspPath::ObjectInfo& obj)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void planGraspPathSVC_impl::SetStatusObject(ObjectInputMode mode, ObjectInfo obj)>"
#endif
}



// End of example implementational code




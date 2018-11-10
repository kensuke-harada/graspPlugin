// -*-C++-*-
/*!
 * @file  GraspPathControllerSVC_impl.h
 * @brief Service implementation header of GraspPathController.idl
 *
 */

#include "GraspPathControllerSkel.h"

#ifndef GRASPPATHCONTROLLERSVC_IMPL_H
#define GRASPPATHCONTROLLERSVC_IMPL_H
 
/*!
 * @class planGraspPathSVC_impl
 * Example class implementing IDL interface planGraspPath
 */
class planGraspPathSVC_impl
 : public virtual POA_planGraspPath,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~planGraspPathSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   planGraspPathSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~planGraspPathSVC_impl();

   // attributes and operations
   void GraspPlanningStart(planGraspPath::ULONG mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end, const char* robotId, const char* objectTagId, ::CORBA::Double resolution, planGraspPath::ManipInfoSeq_out trajectory, planGraspPath::ULONG& state);
   void ReleasePlanningStart(planGraspPath::ULONG mode, const planGraspPath::DblSeq& begin, const planGraspPath::DblSeq& end, const char* robotId, const char* objectTagId, ::CORBA::Double resolution, planGraspPath::ManipInfoSeq_out trajectory, planGraspPath::ULONG& state);
   void SetStatusObject(planGraspPath::ObjectInputMode mode, const planGraspPath::ObjectInfo& obj);

};



#endif // GRASPPATHCONTROLLERSVC_IMPL_H



// -*-C++-*-
/*!
 * @file  GraspControllerSVC_impl.h
 * @brief Service implementation header of GraspController.idl
 *
 */

#include "GraspControllerSkel.h"

#include <rtm/CorbaPort.h>

#ifndef GRASPCONTROLLERSVC_IMPL_H
#define GRASPCONTROLLERSVC_IMPL_H
 
/*!
 * @class GraspPlanStartSVC_impl
 * Example class implementing IDL interface GraspPlanStart
 */
class GraspPlanStartSVC_impl
 : public virtual POA_GraspPlanStart,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~GraspPlanStartSVC_impl();
  RTC::CorbaConsumer<GraspPlanResult>* c_Result;

 public:
  /*!
   * @brief standard constructor
   */
   GraspPlanStartSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~GraspPlanStartSVC_impl();

   // attributes and operations
   void GraspPlanningStart(CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, CORBA::ULong& state);
   void ReleasePlanningStart(CORBA::ULong ObjId, const GraspPlanStart::DblSequence3& objPos, const GraspPlanStart::DblSequence9& objOri, CORBA::ULong& state);
 

   double* c_Position;
   double* c_Posture;

    void GraspPlanConsumer(RTC::CorbaConsumer<GraspPlanResult> *m_Result , double* m_Position, double* m_Posture){
 		c_Result = m_Result;
 		c_Position = m_Position;
 		c_Posture = m_Posture;
	}

};

/*!
 * @class GraspPlanResultSVC_impl
 * Example class implementing IDL interface GraspPlanResult
 */
class GraspPlanResultSVC_impl
 : public virtual POA_GraspPlanResult,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~GraspPlanResultSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   GraspPlanResultSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~GraspPlanResultSVC_impl();

   // attributes and operations//
//   void GraspPlanningResult(const GraspPlanStart::DblSequence3& GraspPos, const GraspPlanStart::DblSequence9& GraspOri, const GraspPlanStart::DblSequence3& ApproachPos, const GraspPlanStart::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue);
   void GraspPlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue);
   void ReleasePlanningResult(const GraspPlanResult::DblSequence3& GraspPos, const GraspPlanResult::DblSequence9& GraspOri, const GraspPlanResult::DblSequence3& ApproachPos, const GraspPlanResult::DblSequence9& ApproachOri, CORBA::Double angle, CORBA::ULong state, CORBA::ULong& isContinue);
};



#endif // GRASPCONTROLLERSVC_IMPL_H



// -*-C++-*-
/*!
 * @file  RecognitionServiceExtSVC_impl.h
 * @brief Service implementation header of RecognitionServiceExt.idl
 *
 */

#include "RecognitionServiceExtSkel.h"

#ifndef RECOGNITIONSERVICEEXTSVC_IMPL_H
#define RECOGNITIONSERVICEEXTSVC_IMPL_H
 
/*!
 * @class RecognitionServiceExtSVC_impl
 * Example class implementing IDL interface RecognitionServiceExt
 */
class RecognitionServiceExtSVC_impl
 : public virtual POA_RecognitionServiceExt,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RecognitionServiceExtSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RecognitionServiceExtSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RecognitionServiceExtSVC_impl();

   // attributes and operations
   CORBA::Boolean startObjectRecognition();
   void setParameters(const RecognitionServiceExt::DblSeq& param);

};

/*!
 * @class RecognitionResultServiceExtSVC_impl
 * Example class implementing IDL interface RecognitionResultServiceExt
 */
class RecognitionResultServiceExtSVC_impl
 : public virtual POA_RecognitionResultServiceExt,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~RecognitionResultServiceExtSVC_impl();

 public:
  /*!
   * @brief standard constructor
   */
   RecognitionResultServiceExtSVC_impl();
  /*!
   * @brief destructor
   */
   virtual ~RecognitionResultServiceExtSVC_impl();

   // attributes and operations
   void setEnvResults(const RecognitionResultServiceExt::DblSeqResult& res);

};



#endif // RECOGNITIONSERVICEEXTSVC_IMPL_H



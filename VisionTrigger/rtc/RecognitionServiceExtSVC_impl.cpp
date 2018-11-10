// -*-C++-*-
/*!
 * @file  RecognitionServiceExtSVC_impl.cpp
 * @brief Service implementation code of RecognitionServiceExt.idl
 *
 */

#include "RecognitionServiceExtSVC_impl.h"

#include "ObjectRecognitionResultManipulator.h"

/*
 * Example implementational code for IDL interface RecognitionServiceExt
 */
RecognitionServiceExtSVC_impl::RecognitionServiceExtSVC_impl()
{
  // Please add extra constructor code here.
}


RecognitionServiceExtSVC_impl::~RecognitionServiceExtSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
CORBA::Boolean RecognitionServiceExtSVC_impl::startObjectRecognition()
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <CORBA::Boolean RecognitionServiceExtSVC_impl::startObjectRecognition()>"
#endif
  return 0;
}

void RecognitionServiceExtSVC_impl::setParameters(const RecognitionServiceExt::DblSeq& param)
{
  // Please insert your code here and remove the following warning pragma
#ifndef WIN32
  #warning "Code missing in function <void RecognitionServiceExtSVC_impl::setParameters(const RecognitionServiceExt::DblSeq& param)>"
#endif
}



// End of example implementational code

/*
 * Example implementational code for IDL interface RecognitionResultServiceExt
 */
RecognitionResultServiceExtSVC_impl::RecognitionResultServiceExtSVC_impl()
{
  // Please add extra constructor code here.
}


RecognitionResultServiceExtSVC_impl::~RecognitionResultServiceExtSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void RecognitionResultServiceExtSVC_impl::setEnvResults(const RecognitionResultServiceExt::DblSeqResult& res)
{
	int len = res.length();
	double res_dbl[len];
	for (int i = 0; i < len; i++) {
		res_dbl[i] = static_cast<double>(res[i]);
	}
	grasp::ObjectRecognitionResultManipulator::instance()->setEnvResult(res_dbl, len);
}



// End of example implementational code




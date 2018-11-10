// -*- C++ -*-
/*!
 *
 * THIS FILE IS GENERATED AUTOMATICALLY!! DO NOT EDIT!!
 *
 * @file GraspControllerSkel.h 
 * @brief GraspController server skeleton header wrapper code
 * @date Mon Mar 13 13:26:00 2017 
 *
 */

#ifndef _GRASPCONTROLLERSKEL_H
#define _GRASPCONTROLLERSKEL_H



#include <rtm/config_rtc.h>
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION

#if   defined ORB_IS_TAO
#  include "GraspControllerC.h"
#  include "GraspControllerS.h"
#elif defined ORB_IS_OMNIORB
#  if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#    undef USE_stub_in_nt_dll
#  endif
#  include "GraspController.hh"
#elif defined ORB_IS_MICO
#  include "GraspController.h"
#elif defined ORB_IS_ORBIT2
#  include "/GraspController-cpp-stubs.h"
#  include "/GraspController-cpp-skels.h"
#elif defined ORB_IS_RTORB
#  include "GraspController.h"
#else
#  error "NO ORB defined"
#endif

#endif // _GRASPCONTROLLERSKEL_H

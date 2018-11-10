// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#ifndef ROBOTLOCALFUNCTIONS_H
#define ROBOTLOCALFUNCTIONS_H

#include <iostream>
#include <fstream>
#if !(defined(WIN32) || defined(_WIN32))
#include <dirent.h>
#endif
#include <sys/types.h>

#include "../Grasp/VectorMath.h"
#include "../Grasp/PlanBase.h"
#include "ObjectPosReader.h"
#include "../ObjectPlacePlanner/PlacePlanner.h"
#include "exportdef.h"

#ifdef EXCADE_ROBOTINTERFACE_MAKE_DLL
#undef EXCADE_API
#define EXCADE_API
#endif

namespace grasp{
namespace PickAndPlacePlanner{

class EXCADE_API RobotLocalFunctions
{

public :
		static RobotLocalFunctions* instance();
		RobotLocalFunctions();
		virtual ~RobotLocalFunctions();

		void Calibration(int robot);
		void clearSeq();
		void setSeq(const cnoid::VectorXd& jointSeq, int graspingState, int graspingState2, int contactState, double mtime);
#ifndef EXCADE_ROBOTINTERFACE_MAKE_DLL
		void insertPipe();
#endif
		enum Robots {PA10, HIRO, HRP2};

//private:

};

}
}

#endif

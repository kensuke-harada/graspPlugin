/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _PlaceController_H
#define _PlaceController_H

#include <list>
#include <vector>

#include "PlanBase.h"

namespace grasp{

class EXCADE_API PlaceController
{
public:
    static PlaceController* instance( PlaceController* pc=NULL );
	PlaceController();
	~PlaceController();
    
    virtual void setTargetPose( cnoid::Vector3 pos, cnoid::Vector3 nvec );
    virtual bool findFinalPose();

    // variables initialized in setTargetPose
    cnoid::Vector3 targetPoint, targetNormVec;
    cnoid::Matrix3 targetOrient;
    bool targetSet;
    
    // variables initialized in findFinalPose
    cnoid::Vector3 finalGripperPos;
    cnoid::Matrix3 finalGripperOri;
    bool finalGripperSet;
    
	PlanBase* tc;
};

}

#endif

/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <stdlib.h>
#include <iostream>
#include "exportdef.h"

namespace grasp{

class EXCADE_API PlanInterface 
{
	
public :
	PlanInterface();
	~PlanInterface();

    static PlanInterface* instance(PlanInterface *gc=NULL);
	virtual void doGraspPlanning();
	virtual bool doPlacePlanning();
	virtual void doPickAndPlacePlanning();
	virtual bool pickAndPlaceMotionPlanning();

protected:
	std::ostream& os;
};


}

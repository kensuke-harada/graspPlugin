#ifndef _GraspController_H
#define _GraspController_H


#include <stdlib.h>
#include <time.h>

#include <vector>

#include  <DMatrix.h>




class GraspPlanning_impl
{

public :
    static GraspPlanning_impl* instance();
	GraspPlanning_impl();
	~GraspPlanning_impl();

	bool SortGraspPosition(int ObjId, dmatrix::dVector3 objPos, dmatrix::dMatrix33 objOri);
	bool returnGraspPosition(dmatrix::dVector3& GraspPos, dmatrix::dMatrix33& GraspOri, dmatrix::dVector3& ApproachPos, dmatrix::dMatrix33& ApproachOri, double& angle, int& states);


protected :

	std::list < std::vector<double> > graspList;

	std::list < std::vector<double> > ::iterator it;

	//==getGraspPos==
	dmatrix::dVector3 palmPos;
	dmatrix::dMatrix33 palmRot;
};



#endif

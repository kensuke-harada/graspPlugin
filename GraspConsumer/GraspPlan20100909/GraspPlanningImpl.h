#ifndef _GraspController_H
#define _GraspController_H


#include <stdlib.h>
#include <time.h>

#include <vector>

#include <hrpUtil/Tvmet3d.h>
#include  <hrpUtil/uBlasCommonTypes.h>
#include <hrpModel/JointPath.h>


//
//using namespace std;
//using namespace boost;




class GraspPlanning_impl
{
	
public :
    static GraspPlanning_impl* instance();
	GraspPlanning_impl();
	~GraspPlanning_impl();
	
	bool SortGraspPosition(int ObjId, hrp::Vector3 objPos, hrp::Matrix33 objOri);
	bool SortReleasePosition(int ObjId, hrp::Vector3 objPos, hrp::Matrix33 objOri);
	bool returnGraspPosition(hrp::Vector3& GraspPos, hrp::Matrix33& GraspOri, hrp::Vector3& ApproachPos, hrp::Matrix33& ApproachOri, double& angle, int& states);
	
	
protected :
	

	virtual bool IK_arm(const hrp::Vector3 &p, const hrp::Matrix33 &R0);

	virtual hrp::dvector calcGradient(double a, double b);
	virtual double IndexFunc(double a, double b);
	virtual bool checkArmLimit();
	virtual double Manipulability();			
	virtual double avoidAngleLimit();
	virtual double differenceFromStandardPose(); 

	std::list < std::vector<double> > graspList;

	std::list < std::vector<double> > ::iterator it;

	
	hrp::JointPathPtr arm_path;
	hrp::Link *palm;

	//==getGraspPos==
	hrp::Vector3 palmPos;
	hrp::Matrix33 palmRot;

	hrp::Vector3 oGraspPos;
	hrp::Matrix33 oGraspRot;

	hrp::Vector3 gObjVisPos;
	hrp::Matrix33 gObjVisRot;
 
};



#endif

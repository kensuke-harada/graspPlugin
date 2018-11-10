/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "PlaceController.h"

#include <iostream>

using namespace grasp;
using namespace std;
using namespace cnoid;


PlaceController* PlaceController::instance( PlaceController* pc )
{
	static PlaceController* instance = (pc) ? pc : new PlaceController();
	if(pc) instance = pc;
	return instance;
}

PlaceController::PlaceController()
{
    targetSet = finalGripperSet = false;
}

PlaceController::~PlaceController()
{
}

    
void PlaceController::setTargetPose( cnoid::Vector3 pos, cnoid::Vector3 nvec )
{
    targetPoint = pos;
    targetNormVec = nvec;
    targetSet = true;
    cout << " --> Target set by the user: "<< pos <<", normal: "<< nvec <<"\n";
}


bool PlaceController::findFinalPose()
{
    if( !targetSet ) {
        cout <<"No target set! The final pose cannot be computed!\n";
        //return false;

        cout <<" (for tests..) Set to default target: 0.195604, -0.238957, 0.98183 | 0, 0, 1\n";
        targetPoint = Vector3(0.195604, -0.238957, 0.98183);
        targetNormVec = Vector3(0,0,1);
        targetSet = true;
    }
    PlanBase *tc = PlanBase::instance();

    // find the initial contact point and initial normal vector at the contact point
    double dmin = 1.e9;
    Vector3 contactPoint, contactNormal;
    for(int l=0; l<tc->objEnvPairs.size(); l++){
        Vector3 Po, Pf, objN2, fingerN;
        double ccp_res = PlanBase::calcContactPoint( tc->objEnvPairs[l], Po, Pf, objN2, fingerN );
        if( ccp_res < dmin ) {
            contactPoint = Pf;
            contactNormal = objN2;
            dmin = ccp_res;
        }
    }
    
    // get the initial object pose
    //Vector3 iniObjPos( tc->object()->p );
    //Matrix3 iniObjRot( tc->object()->attitude() );
    
    // get the initial grasping pose
    //~ Vector3 iniGraspPos( GraspController::instance()->targetArmFinger->palm->p );
    //~ Matrix3 iniGraspRot( GraspController::instance()->targetArmFinger->palm->attitude() );
    Vector3 iniGraspPos( tc->targetArmFinger->palm->p() );
    Matrix3 iniGraspRot( tc->targetArmFinger->palm->attitude() );
    
    // need to put the same contact point to the desired target point..
    Vector3 motionToTarget( targetPoint - contactPoint );
    //cout << "motionToTarget: "<< motionToTarget(0) <<" "<< motionToTarget(1) <<" "<< motionToTarget(2) <<"\n";

    finalGripperPos = iniGraspPos + motionToTarget;
    finalGripperOri = iniGraspRot;
    finalGripperSet = true;
    //cout << "finalGripperPos: "<< finalGripperPos(0) <<" "<< finalGripperPos(1) <<" "<< finalGripperPos(2) <<"\n";

    // check IK
    if( !tc->targetArmFinger->arm->IK_arm( finalGripperPos, finalGripperOri ) ) {
        cout << "Cannot find a joint configuration for the computed gripper pose!\n";
        return false;
    }
    tc->targetArmFinger->arm->arm_path->calcForwardKinematics();

    // check collisions between robot and environment
    for( int l=0; l<tc->robotEnvPairs.size(); l++ ) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        tc->robotEnvPairs[l]->model(0)->setPosition( tc->robotEnvPairs[l]->link(0)->R(), tc->robotEnvPairs[l]->link(0)->p() );
        tc->robotEnvPairs[l]->model(1)->setPosition( tc->robotEnvPairs[l]->link(1)->R(), tc->robotEnvPairs[l]->link(1)->p() );
#else
				tc->robotEnvPairs[l]->model(0)->setPosition( tc->robotEnvPairs[l]->link(0)->T() );
				tc->robotEnvPairs[l]->model(1)->setPosition( tc->robotEnvPairs[l]->link(1)->T() );
#endif
        int t1,t2;
        double p1[3],p2[3];
        double distance = tc->robotEnvPairs[l]->computeDistance( t1, p1, t2, p2 );
        if( distance < 1.0e-04 ) {
            cout << "Detected collision between robot final pose and the environment! "<< distance <<"\n";
            return false;
        }
    }

    cout << "Final gripper pose computed with success!\n";
    return true;
}


// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "mpkRobots.h"
#include "../Grasp/VectorMath.h"
#include "../Grasp/PlanBase.h"

using namespace cnoid;

mpkRobots::mpkRobots( cnoid::BodyItemPtr robots_)
{
       robots = robots_;
	   robotbodys = grasp::RobotBodyPtr(new grasp::RobotBody(robots_, false));

       //arm_path = robots->body()->getJointPath(base, tip);
       nJoints = robots->body()->numJoints();

       param_opts = new param_info[nJoints+6];
	
	map = false;
	
	return ;	
}

mpkRobots::mpkRobots(grasp::RobotBodyPtr robotbody_)
{
	robotbodys = robotbody_;
	robots = robotbodys->getRootBodyItem();
       //arm_path = robots->body()->getJointPath(base, tip);
	nJoints = robotbodys->numJoints();

       param_opts = new param_info[nJoints+6];
	
	map = false;
	
	return ;	
}

mpkRobots::~mpkRobots()
{
	delete[] param_opts;
}

void mpkRobots::set_config(mpkConfig &q)
{
	if(q.size() > nJoints+5) map =true;

	for(int i=0; i<nJoints; i++) robotbodys->joint(i)->q() = robotbodys->joint(i)->q_lower() + (robotbodys->joint(i)->q_upper() - robotbodys->joint(i)->q_lower())*q[i];

	if(map){
		int top = nJoints;
		Vector3 qpos = Vector3(q[top],q[top+1],q[top+2]);
		robotbodys->link(0)->p() = grasp::PlanBase::instance()->llimitMap + qpos.cwiseProduct(grasp::PlanBase::instance()->ulimitMap - grasp::PlanBase::instance()->llimitMap) ;

		top = top+3;
		Vector3 qrpy =  Vector3(q[top],q[top+1],q[top+2]);
		Vector3 rpyLimit= Vector3(7,7,7);
		robotbodys->link(0)->R() = grasp::rotFromRpy( -rpyLimit + qrpy.cwiseProduct( 2.0*rpyLimit ) );
	}
	return;
}

void mpkRobots::get_config(mpkConfig &q)
{
	if(q.size() > nJoints+5) map =true;

	for(int i=0; i<nJoints; i++) q[i] = (robotbodys->joint(i)->q() - robotbodys->joint(i)->q_lower())/(robotbodys->joint(i)->q_upper() - robotbodys->joint(i)->q_lower());

	if(map){
		int top = nJoints;
		Vector3 ppos ;
		ppos =  ( robotbodys->link(0)->p() - grasp::PlanBase::instance()->llimitMap).cwiseProduct( (grasp::PlanBase::instance()->ulimitMap - grasp::PlanBase::instance()->llimitMap).cwiseInverse() ) ;

		q[top] = ppos[0];
		q[top+1] = ppos[1];
		q[top+2] = ppos[2];

		top = top+3;
		Vector3 rpy = grasp::rpyFromRot( robotbodys->link(0)->R());
		Vector3 rpyLimit= Vector3(7,7,7);
		Vector3 qrpy = (rpy +  rpyLimit).cwiseProduct(rpyLimit.cwiseInverse()*0.5);
		q[top] = qrpy[0];
		q[top+1] = qrpy[1];
		q[top+2] = qrpy[2];
	}
	return;
}

void mpkRobots::compute_forward_kinematics()
{
	robotbodys->calcForwardKinematics();
	return;
}


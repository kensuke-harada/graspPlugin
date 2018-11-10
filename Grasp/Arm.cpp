/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include <boost/make_shared.hpp>

#include "Arm.h"
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/

#include "VectorMath.h"
#include "GraspController.h"
#include "UtilFunction.h"

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace boost;
#define SUGIHARA_METHOD

Arm::Arm(BodyPtr body, Link *base_, Link *palm_)
{
#ifdef  CNOID_10_11_12_13
	arm_path = body->getJointPath(base_, palm_);
#else
	arm_path = createJointPath(base_, palm_);
#endif
	palm = palm_;
	nJoints = arm_path->numJoints();

	searchBasePositionMode=false;

	isInterLink=true;	
	multithread_mode=false;

	graspingState = NOT_GRASPING;
	toolWristPos = Vector3::Zero();
	toolWristRot = Matrix3::Identity();
	target_grasp_objid = -1;
/*
	Matrix3 R1 ( Matrix3 (trans(base_->R)) * Matrix3(arm_path->joint(0)->R) );
	cout << R1 << Vector3( omegaFromRot( R1) );
	for(int i=0; i<nJoints-1;i++){
		Matrix3 R1 ( Matrix3 (trans(arm_path->joint(i+1)->R)) * Matrix3(arm_path->joint(i)->R) );
		cout << R1 << Vector3( omegaFromRot( R1) );
	}

	Vector3 axis (1,0,0);
	double q = 3.1415;
	Matrix3 R2 = rodrigues(axis,q);

	Vector3 axis2 (0,1,0);
	double q2 = -1.57;
	Matrix3 R3 = rodrigues(axis2,q2);

	cout << Matrix3(R3*R2) << Vector3( omegaFromRot( Matrix3(R3*R2)) );
*/

}

bool Arm::checkArmLimit() {
	bool withinLimit = true;
	for (int i = 0; i < arm_path->numJoints(); i++)
		if (arm_path->joint(i)->q_upper() < arm_path->joint(i)->q()  ||  arm_path->joint(i)->q_lower() > arm_path->joint(i)->q())
			withinLimit = false;

	return withinLimit;
}

// == Inverse kinematics of the arm considering the null kinematics ==
bool Arm::IK_arm(const Vector3 &p, const Matrix3 &R0) {

	static const int MAX_IK_ITERATION =100;
	static const double LAMBDA = 0.9;

	double maxIkErrorSqr = 1.0e-6 * 1.0e-6;

    for(unsigned int i=0;i<armStandardPose.size();i++) arm_path->joint(i)->q() = armStandardPose[i];

	const int n = arm_path->numJoints();

	if (multithread_mode) {
		setInterLink(); // for multithread
	} else{
		PlanBase::instance()->setInterLink();
	}
	arm_path->calcForwardKinematics();

	Matrix3 R(R0 );

	vector<double> qorg(n);
	for (int i = 0; i < n; ++i)
        qorg[i] = arm_path->joint(i)->q();

    MatrixXd J(6, n);
	VectorXd dq(n);
	VectorXd v(6);

	bool isConverged = false;

	for (int i = 0; i < MAX_IK_ITERATION; i++) {

		calcJacobian(J);

        Vector3 dp(p - palm->p());
        Vector3 omega(palm->R() * omegaFromRot( (palm->R()).transpose()* R));

		double errsqr = dot(dp, dp) + dot(omega, omega);
#if 1
		if(isnan(dot(omega,omega))){ //To remove
		    errsqr = dot(dp,dp);
		    omega << 0,0,0;
		}
#endif

		if (errsqr < maxIkErrorSqr) {
			isConverged = true;
			break;
		}
		setVector3(dp   , v, 0);
		setVector3(omega, v, 3);

		MatrixXd invJ;
		calcPseudoInverse(J, invJ);
#ifndef SUGIHARA_METHOD

		dq = invJ * v + ( MatrixXd::Identity(n, n) - invJ * J) * calcGradient(0.0, 1.0) ;
		dq = LAMBDA*dq;
#else
		//sugihara method
		MatrixXd H(n,n);
		H = J.transpose()*MatrixXd::Identity(6,6)*J + (errsqr*0.1 + 0.001)*MatrixXd::Identity(n,n);
		MatrixXd invH = H.inverse();
		MatrixXd invJ2 = invH*J.transpose();

//		dq = invJ2*v + (errsqr*0.1 + 0.001)*( MatrixXd::Identity(n, n) - invJ * J) * calcGradient(0.0, 1.0);
		dq = invJ2*v + ( MatrixXd::Identity(n, n) - invJ * J) * calcGradient(0.0, 1.0);
#endif
#ifdef DEBUG_MODE
		cout << "ik" << errsqr << " "<<dq.transpose() << endl;
#endif

        for (int j = 0; j < n; ++j) arm_path->joint(j)->q() +=  dq(j);
		if (multithread_mode) {
			setInterLink(); // for multithread
		} else{
			PlanBase::instance()->setInterLink();
		}
		arm_path->calcForwardKinematics();

	}

	if (!isConverged) {
		for (int i = 0; i < n; ++i) {
            arm_path->joint(i)->q() = qorg[i];
		}
		//arm_path->calcInverseKinematics(p,R0);
		arm_path->calcForwardKinematics();
	}

	return isConverged;
}



bool Arm::IK_arm(const Vector3& p, const Matrix3& R, double phi, const VectorXd& q_old){
	return IK_arm( p,  R);
}

bool Arm::IK_arm(const Vector3& p, const Matrix3& R, const VectorXd& q_old){
	return IK_arm( p,  R);
}

bool Arm::IK_arm_palm(const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R) {
	cnoid::Matrix3 wrist_R = palm_R * toolWristRot.transpose();
	cnoid::Vector3 wrist_p = palm_p - wrist_R * toolWristPos;
	return this->IK_arm(wrist_p, wrist_R);
}

bool Arm::IK_arm_palm(const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R, const cnoid::VectorXd& q_old) {
	cnoid::Matrix3 wrist_R = palm_R * toolWristRot.transpose();
	cnoid::Vector3 wrist_p = palm_p - wrist_R * toolWristPos;
	return this->IK_arm(wrist_p, wrist_R, q_old);
}

bool Arm::IK_arm_palm(const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R, double phi, const cnoid::VectorXd& q_old) {
	cnoid::Matrix3 wrist_R = palm_R * toolWristRot.transpose();
	cnoid::Vector3 wrist_p = palm_p - wrist_R * toolWristPos;
	return this->IK_arm(wrist_p, wrist_R, phi, q_old);
}

bool Arm::getPalmPos(const Vector3& Pco1, const Vector3& Pco2, const Matrix3& Rp, const Vector3& pPcr1, const Matrix3& pRcr1, Vector3& Pp, VectorXd& theta, double offset){
	return true;
}

bool Arm::getPalmPos(const Vector3& Pco1, const Vector3& Pco2, const Matrix3& Rp, const Vector3& pPcr1, const Matrix3& pRcr1, Vector3& Pp, VectorXd& theta, const vector<double>& offset){
	return true;
}

double Arm::IndexFunc(double a, double b) {

	return ( a * Manipulability() + b / avoidAngleLimit() );


	double dist = avoidAngleLimit2();
	if(dist < 0.01){
		return b*dist;
	}
	else{
		return 0;
//		return ( a * Manipulability() + b / avoidAngleLimit() );
		return ( a * Manipulability() + b / (1.0-dist) );
	}

}


VectorXd Arm::calcGradient(double a, double b) {

	VectorXd grad(arm_path->numJoints());
	double indx = IndexFunc(a, b), EPS = 0.0001;

	for (int i = 0; i < arm_path->numJoints(); i++) {
        arm_path->joint(i)->q() += EPS;
		grad(i) = (IndexFunc(a, b) - indx) / EPS;
        arm_path->joint(i)->q() -= EPS;
	}

	return grad;

}

double Arm::avoidAngleLimit() {

	//Close to the center
	double dist = 0.0;

	for (int i = 0; i < arm_path->numJoints(); i++){
		double edist =  (arm_path->joint(i)->q() - (arm_path->joint(i)->q_upper() + arm_path->joint(i)->q_lower()) / 2.0); // * 6.28 / (arm_path->joint(i)->ulimit - arm_path->joint(i)->llimit);
		dist +=  edist*edist;
	}

	if (isnan(sqrt(dist)))
		return 10000000.0;
	else
		return sqrt(dist);
//		return dist;
}


double Arm::avoidAngleLimit2() {

	//Close to the center
	double dist = 1000000000.0;



	for (int i = 0; i < arm_path->numJoints(); i++){
		double edist =  min(  - arm_path->joint(i)->q() + arm_path->joint(i)->q_upper() , arm_path->joint(i)->q() - arm_path->joint(i)->q_lower() )/(arm_path->joint(i)->q_upper()-arm_path->joint(i)->q_lower()) ; // * 6.28 / (arm_path->joint(i)->ulimit - arm_path->joint(i)->llimit);
		if(edist <  dist) dist = edist;
	}
	return dist;

}

void Arm::calcJacobian(cnoid::MatrixXd& J){
	arm_path->calcJacobian(J);

	if( !isInterLink ) return;
	
	if(pairs.empty()){
		vector <InterLink>& interLinkList = PlanBase::instance()->interLinkList;
		for(int i=0; i<interLinkList.size();i++){
			int master= -1,slave=-1;
			for(int j=0;j<arm_path->numJoints();j++){
				if( interLinkList[i].master == arm_path->joint(j)) master = j;
			}
			for(int j=0;j<arm_path->numJoints();j++){
				if( interLinkList[i].slave == arm_path->joint(j)) slave = j;
			}
			if( (master>-1) &&  (slave>-1) ){
				InterLink temp;
				temp.master = interLinkList[i].master;
				temp.slave = interLinkList[i].slave;
				temp.ratio = interLinkList[i].ratio;
				temp.masterId = master;
				temp.slaveId = slave;
				pairs.push_back(temp);
			}
		}
		if(pairs.empty()){
			isInterLink =false;
			return;
		}
	}
	for(int i=0;i<pairs.size();i++){
		J.col(pairs[i].masterId) += pairs[i].ratio*J.col(pairs[i].slaveId);
		J.col(pairs[i].slaveId).setZero();
	}

}


double Arm::Manipulability() {
	MatrixXd J;
	calcJacobian(J);

	double d = det(J * J.transpose() );

	if (isnan(sqrt(d)))
		return 10000000.0;
	else
		return sqrt(d);
}

bool Arm::closeArm(int lk, int iter, Vector3 &oPos, Vector3 &objN) {

	double epsiron = 0.001;

    Vector3 p = palm->p();
    Matrix3 R0 = palm->R();

	double dsn_old = 100.0, sgn = 1.0, delta = 0.0;
	bool finish = false;


	for (int loop = 0; loop < iter; loop++) {

//		cout << loop << endl;
		PlanBase::instance()->flush();

		if (! checkArmLimit() )  {
			iter = 0;
			break;
		}

		arm_path->calcForwardKinematics();

		Vector3 Po, Pf, fingerN;

		double dsn = PlanBase::calcContactPoint(palmObjPair, Po, Pf, objN, fingerN);

		if( dsn > 0 ){
			oPos = Po;
			if (fabs(dsn - dsn_old) != 0.0) sgn = -(dsn - dsn_old) / fabs(dsn - dsn_old);

			delta = sgn*epsiron;

			if (finish && dsn < 0.003) return true;
			else if (finish && dsn >= 0.003) return false;

		}else{
			delta = - sgn*epsiron*0.4;
			finish = true;
		}
		p = p + R0*closeDir*delta;
		IK_arm(p, R0);

/*
		fing_path->calcForwardKinematics();

		Vector3 Po, Pf;
		double dsn1=1;
		if( (lk-1) >= 0) dsn1=calcContactPoint(linkObjPair[lk-1], Po, Pf, objN);
		double dsn  = calcContactPoint(linkObjPair[lk], Po, Pf, objN);


		if ( (dsn > 0 && dsn1 > 0)) { // && fingtipGrasp()) || (dsn>0 && !fingtipGrasp()) ){

			oPos = Po;

			if (fabs(dsn - dsn_old) != 0.0) sgn = -(dsn - dsn_old) / fabs(dsn - dsn_old);

			delta = sgn*epsiron;

			if (finish && dsn < 0.003) return true;
			else if (finish && dsn >= 0.003) return false;

		} else {
			delta = - sgn*epsiron*0.4;

			finish = true;
		}
//		PlanBase::instance()->flush();

		dsn_old = dsn;
*/
	}
	return false;
}

void Arm::setPairs(const vector<InterLink>& interLinkList) {
	for(int i=0; i<interLinkList.size();i++){
		int master= -1,slave=-1;
		for(int j=0;j<arm_path->numJoints();j++){
			if( interLinkList[i].master == arm_path->joint(j)) master = j;
		}
		for(int j=0;j<arm_path->numJoints();j++){
			if( interLinkList[i].slave == arm_path->joint(j)) slave = j;
		}
		if( (master>-1) &&  (slave>-1) ){
			InterLink temp;
			temp.master = interLinkList[i].master;
			temp.slave = interLinkList[i].slave;
			temp.ratio = interLinkList[i].ratio;
			temp.masterId = master;
			temp.slaveId = slave;
			pairs.push_back(temp);
		}
	}
}

void Arm::setInterLink() {
	for(int i=0; i<interLinks.size();i++){
        interLinks[i].slave->q() = interLinks[i].master->q() *interLinks[i].ratio;
        if( interLinks[i].slave->q() < interLinks[i].slave->q_lower()) interLinks[i].slave->q() = interLinks[i].slave->q_lower();
        if( interLinks[i].slave->q() > interLinks[i].slave->q_upper()) interLinks[i].slave->q() = interLinks[i].slave->q_upper();
	}
}

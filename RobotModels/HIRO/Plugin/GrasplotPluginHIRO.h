/**
   c) Kensuke Harada (AIST)
*/

#ifndef GRASPLOTPLUGIN_HIRO_H
#define GRASPLOTPLUGIN_HIRO_H

#include <iostream>
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/  
#include <extplugin/graspPlugin/Grasp/Arm.h>
#include <extplugin/graspPlugin/Grasp/VectorMath.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#define m_pi 3.141592

namespace grasp{

class HIRO_Arm: public Arm
{
	public:
	
		HIRO_Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) : Arm(body, base, palm) {
			this->base = base;
			bothArm = true;
			};
		~HIRO_Arm() {	};
//		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix33 &R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, const cnoid::VectorXd& q_old);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old= cnoid::VectorXd::Zero(7));
		bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta);
		/*
		double IndexFunc(double a, double b){return Arm::IndexFunc(a,b);}
		cnoid::VectorXd calcGradient(double a, double b){return Arm::calcGradient(a,b);}
		bool checkArmLimit(){return Arm::checkArmLimit();}
		double Manipulability(){return Arm::Manipulability();}			
		double avoidAngleLimit(){return Arm::avoidAngleLimit();}
		double avoidAngleLimit2(){return Arm::avoidAngleLimit2();}
			
		bool closeArm(int lk, int iter, cnoid::Vector3 &oPos, cnoid::Vector3 &objN){return Arm::closeArm(lk,iter,oPos,objN);}
		*/

		
	private:
		cnoid::Link *base;
		bool bothArm;
};
}
#endif

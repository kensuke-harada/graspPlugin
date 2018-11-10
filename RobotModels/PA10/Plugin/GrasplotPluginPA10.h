/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef GRASPLOTPLUGIN_PA10_H
#define GRASPLOTPLUGIN_PA10_H

#include <iostream>
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/  

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <extplugin/graspPlugin/Grasp/Arm.h>
#include <extplugin/graspPlugin/Grasp/VectorMath.h>
#include <extplugin/graspPlugin/Grasp/exportdef.h>
#else
#include <ext/graspPlugin/Grasp/Arm.h>
#include <ext/graspPlugin/Grasp/VectorMath.h>
#include <ext/graspPlugin/Grasp/exportdef.h>
#endif

#include <iostream>
#include <string>
#include <stdlib.h>

#define m_pi 3.141592


namespace grasp{

class EXCADE_API PA10_Arm: public Arm
{
	public:
	
		PA10_Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) : Arm(body, base, palm) {
			this->base = base;
			};
		~PA10_Arm() {	};
//		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix33 &R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, const cnoid::VectorXd& q_old);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old= cnoid::VectorXd::Zero(7));
		bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta, double offset=0.0);
		bool getPalmPos(const cnoid::Vector3& Pco1, const cnoid::Vector3& Pco2, const cnoid::Matrix3& Rp, const cnoid::Vector3& pPcr1, const cnoid::Matrix3& pRcr1, cnoid::Vector3& Pp, cnoid::VectorXd& theta, const std::vector<double>& offset);
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
		void adjustPA10Wrist();
		cnoid::Link *base;
};	

}

#endif

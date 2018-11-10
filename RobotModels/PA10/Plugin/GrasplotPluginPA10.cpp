/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "GrasplotPluginPA10.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <extplugin/graspPlugin/Grasp/exportdef.h>
#else
#include <ext/graspPlugin/Grasp/exportdef.h>
#endif

using namespace std;
using namespace cnoid;
using namespace grasp;


bool PA10_Arm::IK_arm(const Vector3& p, const Matrix3& R){
	double phi=0;
	VectorXd q_old(nJoints);
	for(int i=0;i<nJoints;i++){
		q_old[i] = armStandardPose[i];
	}
	return IK_arm( p,  R, phi,  q_old);
}

bool PA10_Arm::IK_arm(const Vector3& p, const Matrix3& R, const VectorXd& q_old){
	double phi=0;
	return IK_arm( p,  R, phi,  q_old);
}

bool  PA10_Arm::IK_arm(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){

		double eps = 1.0e-20;
		VectorXd q(7);
		Matrix3 Rb_0, R_30, R3_4, R7_e, R;

		R = Rp; //arm_path->joint(6)->calcRfromAttitude(Rp);

		Rb_0 << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
		R7_e = trans(Rb_0);

		Matrix3 R_( (base->attitude()*Rb_0).transpose()*R*(R7_e).transpose() );
        Vector3  p_( (base->attitude()*Rb_0).transpose()*(p - base->p()) );

		double d_bs = 0.315;
		double d_se = 0.45;
		double a_ew = 0.0; //-0.0025;
		double d_ew = 0.5; //0.48->PA10_VVV
		double d_wt = 0.08;

		Vector3 l_bs(0.0, 0.0, d_bs);
		Vector3 l_se(0.0, -d_se, 0.0);
		Vector3 l_ew(0.0, a_ew, d_ew);
		Vector3 l_wt(0.0, 0.0, d_wt);

		Vector3 x_sw(p_ - l_bs - R_*l_wt);

		q(3) = acos((dot(x_sw, x_sw) - d_se*d_se - d_ew*d_ew - a_ew*a_ew )/(2.0*d_se*d_ew));
		if(q(3)<0.0)
				q(3)=0.0;

		double C4 = cos(q(3));
		double S4 = sin(q(3));

		Vector3 aa( S4*d_ew, -d_se-C4*d_ew, a_ew);

		double tmp=sqrt(aa(0)*aa(0)+aa(1)*aa(1));
		double q20 = acos(x_sw(2)/tmp) - atan2(aa(0)/tmp, -aa(1)/tmp);

		double acas = aa(0)*cos(q20) - aa(1)*sin(q20);
		tmp =  dbl(aa(2)) + dbl(acas);
		double q10 = atan2( ( - aa(2)*x_sw(0) + acas*x_sw(1) )/tmp , ( acas*x_sw(0) + aa(2)*x_sw(1) )/tmp );

		double S10=sin(q10);
		double C10=cos(q10);
		double S20=sin(q20);
		double C20=cos(q20);

		R_30 << C10*C20, -C10*S20, -S10,  S10*C20, -S10*S20, C10,  -S20, -C20, 0.0;

		Vector3 u_sw(x_sw/norm2(x_sw));

		Matrix3 As(  sqew(u_sw)*R_30);
		Matrix3 Bs( -sqew(u_sw)*sqew(u_sw)*R_30 );
		Matrix3 Cs(  m33(u_sw)*R_30 );

		double Cp = cos(phi);
		double Sp = sin(phi);

		if(acos(-Bs(2,1)-Cs(2,1))*q20 > 0.0)
				q(1) = acos(   -As(2,1)*Sp - Bs(2,1)*Cp - Cs(2,1) );
		else
				q(1) = -acos(   -As(2,1)*Sp - Bs(2,1)*Cp - Cs(2,1) );

		double S2 = sin(q(1));

		if(fabs(S2) > eps){
				q(0) = atan2( (-As(1,1)*Sp - Bs(1,1)*Cp - Cs(1,1))/S2 , (-As(0,1)*Sp - Bs(0,1)*Cp - Cs(0,1) )/S2 );
				q(2) = atan2( ( As(2,2)*Sp + Bs(2,2)*Cp + Cs(2,2))/S2 , (-As(2,0)*Sp - Bs(2,0)*Cp - Cs(2,0) )/S2 );
		}
		else return false;

		R3_4 << C4, 0.0, S4, S4, 0.0, -C4, 0.0, 1.0, 0.0;

		Matrix3 Aw(trans(R3_4)*trans(As)*R_);
		Matrix3 Bw(trans(R3_4)*trans(Bs)*R_);
		Matrix3 Cw(trans(R3_4)*trans(Cs)*R_);

		double idx = 0.0, S6;
		for(int i=0; i<3; i++){

				int j=1-2*(i%2);

				q(5) = j*acos(   Aw(2,2)*Sp + Bw(2,2)*Cp + Cw(2,2) );

				S6=sin(q(5));

				if(fabs(S6) > eps){
						q(4) = atan2( (Aw(1,2)*Sp + Bw(1,2)*Cp + Cw(1,2))/S6 , ( Aw(0,2)*Sp + Bw(0,2)*Cp + Cw(0,2) )/S6 );
						q(6) = atan2( (Aw(2,1)*Sp + Bw(2,1)*Cp + Cw(2,1))/S6 , (-Aw(2,0)*Sp - Bw(2,0)*Cp - Cw(2,0) )/S6 );

						double eps2=0.5;
						while( q(4)-q_old(4) >  2*m_pi-eps2)	q(4) -= 2*m_pi;
						while( q(5)-q_old(5) >  2*m_pi-eps2)	q(5) -= 2*m_pi;
						while( q(6)-q_old(6) >  2*m_pi-eps2)	q(6) -= 2*m_pi;
						while( q(4)-q_old(4) < -2*m_pi+eps2)	q(4) += 2*m_pi;
						while( q(5)-q_old(5) < -2*m_pi+eps2)	q(5) += 2*m_pi;
						while( q(6)-q_old(6) < -2*m_pi+eps2)	q(6) += 2*m_pi;

						if((q(4)-q_old(4))*(q(4)-q_old(4))+(q(5)-q_old(5))*(q(5)-q_old(5))+(q(6)-q_old(6))*(q(6)-q_old(6)) < idx)
						          break;

						idx = (q(4)-q_old(4))*(q(4)-q_old(4))+(q(5)-q_old(5))*(q(5)-q_old(5))+(q(6)-q_old(6))*(q(6)-q_old(6));
				}
		}

		if(fabs(S6)<eps)
				return false;


        for(int l=0; l<7; l++) arm_path->joint(l)->q() = q(l);

		//adjustPA10Wrist();

		arm_path->calcForwardKinematics();

		return true;
}

bool PA10_Arm::getPalmPos(const Vector3& Pco1, const Vector3& Pco2, const Matrix3& Rp, const Vector3& pPcr1, const Matrix3& pRcr1, Vector3& Pp, cnoid::VectorXd& theta, double offset)
{
		theta.resize(2);

		Vector3 P1 = Pco1 + offset*unit(Pco2-Pco1);
		Vector3 P2 = Pco2 + offset*unit(Pco1-Pco2);
		if(norm2(Pco1-Pco2) < offset*2.0){
			P1 = 0.5*(Pco1+Pco2);
			P2 = 0.5*(Pco1+Pco2);
		}

		Vector3 pNcr1 = col(pRcr1, 0);
		Vector3 pTcr1 = col(pRcr1, 1);

		Pp = 0.5*(P1 + P2 - Rp*pPcr1 - Rp*pPcr1 );//?
		theta(0) = -norm2(P1-P2)/2.0;
		theta(1) = -theta(0);

		return true;
}

bool PA10_Arm::getPalmPos(const Vector3& Pco1, const Vector3& Pco2, const Matrix3& Rp, const Vector3& pPcr1, const Matrix3& pRcr1, Vector3& Pp, cnoid::VectorXd& theta, const std::vector<double>& offset)
{
		theta.resize(2);

		Vector3 P1 = Pco1;
		Vector3 P2 = Pco2;

		Vector3 pNcr1 = col(pRcr1, 0);
		Vector3 pTcr1 = col(pRcr1, 1);

		Pp = 0.5*(P1 + P2 - Rp*pPcr1 - Rp*pPcr1 );//?
		theta(0) = -norm2(P1-P2)/2.0 + ((offset.size() > 0) ? offset[0] : 0);
		theta(1) = -theta(0) + ((offset.size() > 1) ? offset[1] : 0);

		return true;
}

extern "C" EXCADE_API void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm)
{
    return new PA10_Arm(body, base, palm);
}

/**
   c) Kensuke Harada (AIST)

   Manipulability Test code
   A.Bicchi, C.Melchiorri, and D. Balluchi, TRA 1995
   A.Bicchi, D.Prattichizzo, and C.Melciorri, IROS 1997
*/

#ifndef _ManipulabilityTest_H
#define _ManipulabilityTest_H

#include <cnoid/EigenTypes>
#include <Eigen/Cholesky>

#include "ConvexAnalysis.h"
#include "PlanBase.h"

#include "exportdef.h"


namespace grasp{

class ContactState
{
	public:
		ContactState();
		ContactState(cnoid::Vector3& lPlc_, cnoid::Vector3& oPc_, cnoid::Vector3& oNc_, cnoid::Matrix3& oRc_,
				int contactFinger_, std::string contactLinkName_, std::string contactStyle_);
		cnoid::Vector3 lPlc; //Contact point vector wrt robot coordinate system
		cnoid::Vector3 oPc; //Contact point vector wrt object coordinate system
		cnoid::Vector3 oNc; //Normal vector wrt object coordinate system
		cnoid::Matrix3 oRc;
		int contactFinger;
		std::string contactLinkName;
		std::string contactStyle;
		//std::string contactState[3]={"PointContactWithoutFriction", "PointContactWithFriction", "SoftFinger"};
};

class ManipulabilityTest
{
	public:
		ManipulabilityTest() {
			 GAMMA = 1000.0;
			 isOutputForceSpace=false;
			 isOutputTorqueSpace=false;
		}
		~ManipulabilityTest(){}

		void SetGAMMA(double GAMMA){
			this->GAMMA= GAMMA;
		}

		static ManipulabilityTest* instance(ManipulabilityTest *fct=NULL) {
			static ManipulabilityTest* instance = (fct) ? fct : new ManipulabilityTest();
			return instance;
		}

		void calcGraspMatrix();
		void calcJacobianMatrix();
		void decomposeSystemMatrix();
		void calcContactMatrix();

		double ManipulabilityEllipsoid();

		std::vector<ContactState *> state;

	protected:

		// GAMMA is scale value for moment
		double GAMMA;
		bool isOutputForceSpace;
		bool isOutputTorqueSpace;

		cnoid::MatrixXd G, J, H;
		cnoid::MatrixXd C11, C12, C22, C23;

};

}

#endif

/**
   c) Kensuke Harada (AIST)
*/

#include <fstream>
#include <iostream>

#include "ManipulabilityTest.h"
#include "VectorMath.h"

using namespace cnoid;
using namespace std;
using namespace grasp;

ContactState::ContactState(cnoid::Vector3& lPlc_, cnoid::Vector3& oPc_, cnoid::Vector3& oNc_, cnoid::Matrix3& oRc_,
		int contactFinger_, std::string contactLinkName_, std::string contactStyle_){

	lPlc = lPlc_;
	oPc = oPc_;
	oNc = oNc_;
	oRc = oRc_;
	contactFinger = contactFinger_;
	contactLinkName = contactLinkName_;
	contactStyle = contactStyle_;
}

void ManipulabilityTest::calcGraspMatrix(){

		PlanBase* tc = PlanBase::instance();

		G=MatrixXd::Zero(6, state.size()*6);

		Vector3 Po = tc->targetObject->bodyItemObject->body()->link(0)->p();	// 20140206 t.mizuno
		Matrix3 Ro = tc->targetObject->bodyItemObject->body()->link(0)->attitude();

		for(size_t i=0; i<state.size(); i++){
				Vector3 Pc = Po + Ro*state[i]->oPc;
				Matrix3 Rc = Ro*state[i]->oRc;

				G.block(0, 6*i,   3, 6*i+3 ) = Rc;
				G.block(3, 6*i+3, 6, 6*i+6 ) = Rc;
				G.block(3, 6*i,   6, 6*i+3 ) = sqew(Pc)*Rc;
		}
}

void ManipulabilityTest::calcJacobianMatrix(){

		PlanBase* tc = PlanBase::instance();

		int numJoints=0;
		for(int i=0; i<tc->nFing(); i++)
				numJoints += tc->fingers(i)->nJoints;

		Matrix3 Ro = tc->targetObject->bodyItemObject->body()->link(0)->attitude();

		FingerPtr *fingers_ = new FingerPtr[tc->nFing()];;
		for(size_t i=0; i<state.size(); i++)
				fingers_[i] = new Finger(tc->bodyItemRobot()->body(), tc->palm(), tc->bodyItemRobot()->body()->link(state[i]->contactLinkName) );

		J=MatrixXd::Zero(state.size()*6, numJoints);

		int nJ=0;
		for(size_t i=0; i<state.size(); i++){
				MatrixXd Jsub;
				tc->fingers(state[i]->contactFinger)->fing_path->calcJacobian(Jsub);

				Vector3 Plc = tc->bodyItemRobot()->body()->link(state[i]->contactLinkName)->p() + tc->bodyItemRobot()->body()->link(state[i]->contactLinkName)->attitude()*state[i]->lPlc;	// 20140206 t.mizuno
				Matrix3 Rc = Ro*state[i]->oRc;

				MatrixXd Gsub=MatrixXd::Zero(6,6);
				Gsub.block(0, 0, 3, 3) = Rc;
				Gsub.block(3, 3, 6, 6) = Rc;
				Gsub.block(3, 0, 6, 3) = sqew(Plc)*Rc;

				Jsub = trans(Gsub)*Jsub;
				int nJi = tc->fingers(state[i]->contactFinger)->fing_path->numJoints();
				J.block(6*i, nJ,  6*i+6, nJ+nJi) = Jsub;

				nJ += nJi;
		}
}

void ManipulabilityTest::decomposeSystemMatrix(){

		int s1 = G.rows();
		int s2 = J.cols();

		MatrixXd Q(H.rows(), s1+s2);

		Q << H*trans(G), -H*J;

		Eigen::JacobiSVD<cnoid::MatrixXd> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);

		cnoid::VectorXd s = svd.singularValues();

        double smin, smax=0.0;
        double sv_ratio = 1.0e-3;
        for (int j = 0; j < s.size() ; j++) if (s[j] > smax) smax = s[j];
        smin = smax*sv_ratio;

        vector<int> idx;
        for (int j = 0; j < s.size() ; j++)
        	if (s[j] <= smin) idx.push_back(j);

        MatrixXd V = svd.matrixV();

        MatrixXd Null(V.rows(), idx.size());
        for(size_t i=0; i<idx.size(); i++)
				Null.col(i) = V.col(idx[i]);

        int j1=0, j2=0, j3=0;
        for(size_t i=0; i<Null.cols(); i++){
				VectorXd e1 = Null.block(0,  i, s1,    i);
				VectorXd e2 = Null.block(s1, i, s1+s2, i);
				if(norm2(e1)<1.0e-5){
						idx[i] = 1;
						j3++;
				}
				else if(norm2(e2)<1.0e-5){
						idx[i] = -1;
						j1++;
				}
				else{
						idx[i] =0;
						j2++;
				}
        }

        MatrixXd C1(V.rows(), j1);
        MatrixXd C2(V.rows(), j2);
        MatrixXd C3(V.rows(), j3);

        j1=0; j2=0; j3=0;
        for(size_t i=0; i<idx.size(); i++){

				if(idx[i]==-1){
						C1.col(j1) = Null.col(i);
						j1++;
				}
				else if(idx[i]==0){
						C2.col(j2) = Null.col(i);
						j2++;
				}
				else{
						C2.col(j3) = Null.col(i);
						j3++;
				}
        }

        C11 = C1.block(0, 0, s1, j1 );
        C12 = C2.block(0, 0, s1, j2 );
        C22 = C2.block(s1, 0, s1+s2, j2);
        C23 = C3.block(s1, 0, s1+s2, j3);
}

void ManipulabilityTest::calcContactMatrix(){
		MatrixXd H_pcwf(6,3), H_soft(6,4), H_pcwof(6,1);

		H_pcwof << 1,0,0,0,0,0;
		H_pcwf  << 1,0,0, 0,1,0, 0,0,1, 0,0,0, 0,0,0, 0,0,0;
		H_soft  << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,0, 0,0,0,0, 0,0,0,1;

		int col=0, row=0;

		for(size_t i=0; i<state.size(); i++){
				row += 6;
				if(state[i]->contactStyle == "PointContactWithoutFriction")
						col += 1;
				else if(state[i]->contactStyle=="PointContactWithFriction")
						col += 3;
				else if(state[i]->contactStyle=="SoftFinger")
						col += 4;
		}

		H = MatrixXd::Zero(row, col);

		col = 0;
		row = 0;
		for(size_t i=0; i<state.size(); i++){

				if(state[i]->contactStyle == "PointContactWithoutFriction"){
						H.block(row, col,  row+6, col+1) = H_pcwof;
						col += 1;
				}
				else if(state[i]->contactStyle=="PointContactWithFriction"){
						H.block(row, col,  row+6, col+3) = H_pcwf;
						col += 3;
				}
				else if(state[i]->contactStyle=="SoftFinger"){
						H.block(row, col,  row+6, col+4) = H_soft;
						col += 4;
				}
				row +=6;
		}
}

double ManipulabilityTest::ManipulabilityEllipsoid()
{
		Vector3 x(1,0,0);
		for(size_t i=0; i<state.size(); i++){
				Vector3 z = state[i]->oNc;
				state[i]->oRc = v3(cross(z,x), cross(z, cross(z,x)), z);
		}

		calcGraspMatrix();
		calcJacobianMatrix();
		calcContactMatrix();
		decomposeSystemMatrix();

		MatrixXd A, CWC;

		if(C11.cols()==0 && C23.cols()==0){
				CWC = trans(C22)*C22;
		}
		else{
				MatrixXd iC23;
				calcPseudoInverse(C23, iC23);
				MatrixXd cC22 = (MatrixXd::Identity(C23.rows(), iC23.cols()) - C23*iC23)*C22;
				CWC = trans(cC22)*cC22;
		}

		Eigen::LLT<MatrixXd> LLT(CWC);
		MatrixXd L = LLT.matrixL();
		A = inverse(L)*trans(C12)*C12*inverse(trans(L));

		MatrixXd evec;
		VectorXd eval;
		int info;
		info = calcEigenVectors(A, evec, eval);

}

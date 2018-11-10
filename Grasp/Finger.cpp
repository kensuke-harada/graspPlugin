// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "Finger.h"

#include <cnoid/JointPath>
#include <iostream>

#include "VectorMath.h"
#include "GraspController.h"
#include <boost/make_shared.hpp>
#include "UtilFunction.h"

//#define DEBUG_MODE
#define SEARCH_INITIAL_ANGLE_IN_CLOSE_FINGER

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace boost;

Finger::Finger(BodyPtr body, Link *palm, Link *tip)
{
#ifdef  CNOID_10_11_12_13
	fing_path = body->getJointPath(palm, tip);
#else
	fing_path = createJointPath(palm, tip);
#endif
	nJoints = fing_path->numJoints();
	for(int j=0;j<nJoints;j++){
		compLink.push_back(0);
	}
	for(int j=0;j<nJoints;j++){
		contact.push_back(false);
		close.push_back(0.002);
	}
	contact.push_back(true);
	this->tip = tip;
	this->body = body;

	linkObjPair = new ColdetLinkPairPtr[nJoints];
	return ;
}

Finger::~Finger()
{
	delete[] linkObjPair;
}

void Finger::coldetLinkPair(cnoid::BodyItemPtr bo){
	for(int j=0;j<nJoints;j++){
#ifdef  CNOID_10_11_12_13
		if(j==nJoints-1) linkObjPair[j] = new ColdetLinkPair(tip, bo->body()->link(0));
		else linkObjPair[j] = new ColdetLinkPair(fing_path->joint(j), bo->body()->link(0));
#else
		if(j==nJoints-1) linkObjPair[j] = boost::make_shared<ColdetLinkPair>(body, tip, bo->body(), bo->body()->link(0));
		else linkObjPair[j] = boost::make_shared<ColdetLinkPair>(body, fing_path->joint(j), bo->body(), bo->body()->link(0));
#endif
	}
}


bool Finger::fingtipGrasp(void) {
	int s=0;
	for(int i=0;i<nJoints;i++) if(contact[i]) s++;
	if (s > 1 ) return false;
	return true;
}


bool Finger::closeFinger(int lk, int iter, Vector3 &oPos, Vector3 &objN, Vector3 &fingerN) {

	double dsn_old = 100.0, sgn = 1.0;
	//double distance = norm2(fing_path->joint(lk)->p-fing_path->joint(compLink[lk])->p);

	double epsiron = close[lk]; //0;
	//if(distance ==0) epsiron = close[lk];
	//else epsiron = 0.002/distance;

	double delta = 0.0;

	bool finish = false;
	for (int loop = 0; loop < iter; loop++) {


        fing_path->joint(compLink[lk])->q() += delta;

		// if(PlanBase::instance()->interLinkList.size()>0)
			PlanBase::instance()->setInterLink();

		if (! checkJointLimit(fing_path->joint(compLink[lk])) )  {
			iter = 0;
		}

		fing_path->calcForwardKinematics();
#ifdef DEBUG_MODE
		PlanBase::instance()->flush();
#endif
		Vector3 Po, Pf;

		bool col1=false;
		if( (lk-1) >= 0){
			linkObjPair[lk-1]->updatePositions();
			col1 = linkObjPair[lk-1]->checkCollision();
		}
		linkObjPair[lk]->updatePositions();
		bool col = linkObjPair[lk]->checkCollision();

#ifdef SEARCH_INITIAL_ANGLE_IN_CLOSE_FINGER
		if(loop == 0 && (col || col1)){
			bool col2 = false;
			do{
                fing_path->joint(compLink[lk])->q() -= sgn * epsiron * 300;
				fing_path->calcForwardKinematics();
				linkObjPair[lk]->updatePositions();
				if( (lk-1) >= 0){
					linkObjPair[lk-1]->updatePositions();
					col2 = linkObjPair[lk-1]->checkCollision();
				}
				if(! checkJointLimit(fing_path->joint(compLink[lk]))) break;
			}while(linkObjPair[lk]->checkCollision() || col2);
			continue;
		}
#endif

		if ( ((!col && !col1) && fingtipGrasp()) || (!col && !fingtipGrasp()) ){

			//Close direction is fixed now, since dead lock sometimes happens
			// if (fabs(dsn - dsn_old) != 0.0) sgn = -(dsn - dsn_old) / fabs(dsn - dsn_old);

			delta = sgn*epsiron;

			if(finish){
				double dsn  = PlanBase::calcContactPoint(linkObjPair[lk], Po, Pf, objN, fingerN);
				oPos = Po;
				if (dsn < 0.003) return true;
				else if (dsn >= 0.003) return false;
			}

		} else {
			delta = - sgn*epsiron*0.4;

			finish = true;
		}
		//cout <<"test" <<number <<" " <<delta << " "<< dsn << " "<< dsn_old << endl;

		//dsn_old = dsn;
	}

	return false;
}

bool Finger::contactSearch(int& cnt,  int iter, Vector3* oPos, Vector3* objN, Vector3* fingerN){
	
	bool nullFingerN=false;
	
	if(fingerN==NULL){
		fingerN = new Vector3 [50]; //tentative update for segmentation fault . this functuin should be designed again later.
		nullFingerN =true;
	}

	for(unsigned int i=0; i<contact.size(); i++){
		if(contact[i]){
			if (closeFinger(i, iter, oPos[cnt], objN[cnt], fingerN[cnt])){
				cnt++;
			}else{
				if(nullFingerN){
					delete [] fingerN;
				}
				return false;
			}
		}
	}
	if(nullFingerN){
		delete [] fingerN;
	}
	return true;
}


bool Finger::checkJointLimit(Link* joint){

	if (joint->q() < joint->q_lower() ) {
        joint->q() = joint->q_lower();
		return false;
	}
	if (joint->q() > joint->q_upper()) {
        joint->q() = joint->q_upper();
		return false;
	}
	return true;
}

bool Finger::checkFingLimit(){

	bool withinLimit = true;
	for (int i = 0; i < fing_path->numJoints(); i++)
		if (fing_path->joint(i)->q_upper() < fing_path->joint(i)->q()  ||  fing_path->joint(i)->q_lower() > fing_path->joint(i)->q())
			withinLimit = false;

	return withinLimit;
}

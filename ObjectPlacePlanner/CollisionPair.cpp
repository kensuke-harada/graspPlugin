// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */
#include <iostream>
#include "CollisionPair.h"

#define m_pi 3.141592
#define minDistance 0.001

using namespace std;
using namespace cnoid;
using namespace boost;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

grasp::PickAndPlacePlanner::CollisionPair::CollisionPair()  : 	os (MessageView::mainInstance()->cout() ) {
		colPairRob.clear();
		colPairSelf.clear();
		colPairObj.clear();
}

grasp::PickAndPlacePlanner::CollisionPair::~CollisionPair() {
}


void grasp::PickAndPlacePlanner::CollisionPair::setCollisionRob()
{
		PlanBase* tc = PlanBase::instance();
		colPairRob.clear();

		for(unsigned int j=0;j<tc->bodyItemRobot()->body()->numLinks();j++){
			for( list<BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it !=tc->bodyItemEnv.end(); it++){
				for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
					ColdetLinkPairPtr temp= new ColdetLinkPair(tc->bodyItemRobot()->body()->link(j), (*it)->body()->link(i));
#else
					ColdetLinkPairPtr temp = boost::make_shared<ColdetLinkPair>(tc->bodyItemRobot()->body(), tc->bodyItemRobot()->body()->link(j), (*it)->body(), (*it)->body()->link(i) );
#endif
					temp->updatePositions();
					int t1,t2;
					double p1[3],p2[3];
					double distance = temp->computeDistance(t1,p1,t2,p2);
					if(distance>1.0e-04)	colPairRob.push_back(temp);
	#ifdef DEBUG_MODE
					else os <<"collide on initial condition robot and env"  <<distance <<" "<< temp->model(0)->name() <<" " << (*it)->body()->name() << endl;
	#endif
				}
			}
		}

}

void grasp::PickAndPlacePlanner::CollisionPair::setCollisionSelf()
{
	PlanBase* tc = PlanBase::instance();

	colPairSelf.clear();

	for(unsigned int i=0;i<tc->bodyItemRobot()->body()->numLinks();i++){ // If initial position is not collided, it is stored as
		for(unsigned int j=i+1;j<tc->bodyItemRobot()->body()->numLinks();j++){
			bool pass = false;
			pair<multimap<string, string>::iterator, multimap<string, string>::iterator> ppp;
			ppp = tc->targetArmFinger->contactLinks.equal_range(tc->bodyItemRobot()->body()->link(i)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == tc->bodyItemRobot()->body()->link(j)->name()) pass = true;
			}
			ppp = tc->targetArmFinger->contactLinks.equal_range(tc->bodyItemRobot()->body()->link(j)->name() );
			for (multimap<string, string>::iterator it2 = ppp.first; it2 != ppp.second; ++it2){
				if(it2->second == tc->bodyItemRobot()->body()->link(i)->name()) pass = true;
			}
			if(pass) continue;
#ifdef  CNOID_10_11_12_13
			ColdetLinkPairPtr temp= new ColdetLinkPair(tc->bodyItemRobot()->body()->link(i), tc->bodyItemRobot()->body()->link(j));
#else
			ColdetLinkPairPtr temp = boost::make_shared<ColdetLinkPair>(tc->bodyItemRobot()->body(), tc->bodyItemRobot()->body()->link(i), tc->bodyItemRobot()->body(), tc->bodyItemRobot()->body()->link(j));
#endif
			temp->updatePositions();
			int t1,t2;
			double p1[3],p2[3];
			double distance = temp->computeDistance(t1,p1,t2,p2);
			if(distance>1.0e-04)	colPairSelf.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"collide on initial condition at robotSelfPair"  <<distance <<" "<< temp->model(0)->name() <<" " << temp->model(1)->name()  << endl;
#endif
		}
	}
}


void grasp::PickAndPlacePlanner::CollisionPair::setCollisionObj()
{
	PlanBase* tc = PlanBase::instance();
	colPairObj.clear();

	for(list<cnoid::BodyItemPtr>::iterator it = tc->bodyItemEnv.begin(); it !=tc->bodyItemEnv.end();it++){
		for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
			colPairObj.push_back(new ColdetLinkPair(tc->targetObject->object, (*it)->body()->link(i)));
#else
			colPairObj.push_back(boost::make_shared<ColdetLinkPair>(tc->targetObject->bodyItemObject->body(), tc->targetObject->bodyItemObject->body()->link(0), (*it)->body(), (*it)->body()->link(i)));
#endif
		}
	}

	for(list<PointCloudEnv*>::iterator it = tc->pointCloudEnv.begin(); it != tc->pointCloudEnv.end(); it++){
		pair<BodyItemPtr, PointCloudEnv*> obj_pointCloud(tc->targetObject->bodyItemObject, *it);
		colPairObjPointCloud.push_back(obj_pointCloud);
	}
}

bool grasp::PickAndPlacePlanner::CollisionPair::isColliding()
{
		for(int i=0;i<colPairRob.size();i++){
			ColdetLinkPairPtr testPair = colPairRob[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
#ifdef DEBUG_MODE
				cout <<"robot env collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}

		for(int i=0;i<colPairSelf.size();i++){
			ColdetLinkPairPtr testPair = colPairSelf[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
#ifdef DEBUG_MODE
				cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}

		for(int i=0;i<colPairObj.size();i++){
			ColdetLinkPairPtr testPair = colPairObj[i];
			testPair->updatePositions();
			bool coll = testPair->checkCollision();
			if(coll){
#ifdef DEBUG_MODE
				cout <<"robot obj collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
#endif
				return true;
			}
		}

		PlanBase* tc = PlanBase::instance();
		for(int i=0;i<colPairObjPointCloud.size();i++){
			bool coll = tc->isCollidingPointCloud(colPairObjPointCloud[i].second, colPairObjPointCloud[i].first);
			if(coll){
#ifdef DEBUG_MODE
				cout << "obj pointcloud collide" << endl;
#endif
				return true;
			}
		}

	return false;
}

void grasp::PickAndPlacePlanner::CollisionPair::getColPoints(vector<Vector3>& colpoints)
{
	PlanBase* tc = PlanBase::instance();

	for (vector<ColdetLinkPairPtr>::const_iterator it = colPairObj.begin(); it != colPairObj.end(); it++) {
		ColdetLinkPairPtr col = *it;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		col->model(0)->setPosition(col->link(0)->R(), col->link(0)->p());
		col->model(1)->setPosition(col->link(1)->R(), col->link(1)->p());
#else
		col->model(0)->setPosition(col->link(0)->T());
		col->model(1)->setPosition(col->link(1)->T());
#endif
		vector<collision_data> data = col->detectCollisions();
		for (size_t k = 0; k < data.size(); k++) {
			for (int i = 0; i < data[k].num_of_i_points; i++) {
				colpoints.push_back(data[k].i_points[i]);
			}
		}
	}

	for (vector<pair<BodyItemPtr, PointCloudEnv*> >::const_iterator it = colPairObjPointCloud.begin(); it != colPairObjPointCloud.end(); it++) {
		tc->getColPointCloud(colpoints, (*it).second, (*it).first);
	}
}

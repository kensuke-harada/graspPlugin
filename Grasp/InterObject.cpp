#include "InterObject.h"
#include "PlanBase.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

void InterObject::setInterObject(){
    slaveItem->body()->link(0)->R() = master->R()*relativeRot;
    slaveItem->body()->link(0)->p() = master->p() + master->R()*relativePos;
	slaveItem->notifyKinematicStateChange();
}

void InterObject::outputRelativePosition(){
	std::cout <<"inter object output " << master->name() << std::endl;
	std::cout << slaveItem->name() << std::endl;
    Vector3 tempv =  (master->R()).transpose()*(slaveItem->body()->link(0)->p() - master->p()) ;
	std::cout << tempv[0] << ", "<< tempv[1] << ", "<< tempv[2]  << std::endl;
    Matrix3 tempr =  (master->R()).transpose()*slaveItem->body()->link(0)->R()  ;
	for(int i=0;i<3;i++) for(int j=0;j<3;j++){
		if( fabs( tempr(i, j ) )< 1.0e-10) tempr(i,j) = 0;
		std::cout <<  tempr(i,j) << ", " ;
	}
	cout << std::endl;
}

void InterObject::initialCollision(){
	slaveEnvPairs.clear();
	
	for(unsigned int j=0;j<slaveItem->body()->numLinks();j++){
		for( list<BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it !=PlanBase::instance()->bodyItemEnv.end(); it++){
			for(unsigned int i=0;i<(*it)->body()->numLinks();i++){
#ifdef  CNOID_10_11_12_13
				ColdetLinkPairPtr temp= new ColdetLinkPair(slaveItem->body()->link(j), (*it)->body()->link(i));
#else
				ColdetLinkPairPtr temp = boost::make_shared<ColdetLinkPair>(slaveItem->body(),slaveItem->body()->link(j), (*it)->body(), (*it)->body()->link(i) );
#endif
				temp->updatePositions();
				int t1,t2;
				double p1[3],p2[3];
				double distance = temp->computeDistance(t1,p1,t2,p2);
				if(distance>1.0e-04)	slaveEnvPairs.push_back(temp);
#ifdef DEBUG_MODE
				else cout <<"collide on initial condition robot and env"  <<distance <<" "<< temp->model(0)->name() <<" " << (*it)->body()->name() << endl;
#endif
			}
		}
	}
	
	
}

bool InterObject::isColliding(){
	for(int i=0;i<slaveEnvPairs.size();i++){
		ColdetLinkPairPtr testPair = slaveEnvPairs[i];
		testPair->updatePositions();
		bool coll = testPair->checkCollision();
		if(coll){
			//colPairName[0] = testPair->model(0)->name();
			//colPairName[1] = testPair->model(1)->name();
	#ifdef DEBUG_MODE
			cout <<"self collide " <<testPair->model(0)->name() <<" "<<testPair->model(1)->name()<< endl;
	#endif
			return true;
		}
	}
	return false;
}

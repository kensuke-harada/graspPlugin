
#include <fstream>
#include <string>
#include <iostream>

#include <math.h>

#include <algorithm>
#include <list>
#include <vector>

#include <time.h>
#include <sys/resource.h>

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpUtil/MatrixSolvers.h>

#include "GraspPlanningImpl.h"
#include "DirectBodyLoader.h"

#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)


double getrusage_sec() {
	struct rusage t;
	struct timeval tv;
	getrusage(RUSAGE_SELF, &t);
	tv = t.ru_utime;
	return tv.tv_sec + (double)tv.tv_usec*1e-6;
}


using namespace std;
using namespace tvmet;
using namespace hrp;

GraspPlanning_impl::GraspPlanning_impl(){
	
	Excade::Robotics::DirectBodyLoader* directBodyLoader  = new Excade::Robotics::DirectBodyLoader;

	hrp::BodyPtr body = directBodyLoader->loadModelFile("SmartPal_standard/SmartPal5.wrl");
	
	palm = body->link("RARM_JOINT7");
	arm_path = body->getJointPath(body->link("WAIST_JOINT1"), palm);
	
	body->link(0)->p() = Vector3(0,0,-0.4365);
	body->calcForwardKinematics();
	
	cout << body->link("WAIST_JOINT1")->p() << body->link("WAIST_JOINT1")->R() << endl;
	

}


GraspPlanning_impl::~GraspPlanning_impl() {

}

GraspPlanning_impl* GraspPlanning_impl::instance() {
	static GraspPlanning_impl* instance = new GraspPlanning_impl();
	return instance;
}


// == Inverse kinematics of the arm considering the null kinematics ==
bool GraspPlanning_impl::IK_arm(const Vector3 &p, const Matrix33 &R0) {
	
	
	static const int MAX_IK_ITERATION =50;
	static const double LAMBDA = 0.9;

	double maxIkErrorSqr = 1.0e-6 * 1.0e-6;

	const int n = arm_path->numJoints();

	for(int i=0;i<n;i++){
		arm_path->joint(i)->q()	= 0;
	}
	arm_path->joint(4)->q() = 1.57;

	arm_path->calcForwardKinematics();
//	Matrix33 R(R0 * trans(palm->Rs));
//	cout << palm->Rs << endl;
//	exit(0);
	
	Matrix33 R(R0 );
	

	vector<double> qorg(n);
	for (int i = 0; i < n; ++i)
		qorg[i] = arm_path->joint(i)->q();

	dmatrix J(6, n);
	dvector dq(n);
	dvector v(6);

	bool isConverged = false;

	for (int i = 0; i < MAX_IK_ITERATION; i++) {
		
		arm_path->calcJacobian(J);

		Vector3 dp(p - palm->p());
		Vector3 omega(palm->R() * omegaFromRot(Matrix33(trans(palm->R()) * R)));

		double errsqr = tvmet::dot(dp, dp) + tvmet::dot(omega, omega);
		
//		cout << tvmet::dot(dp, dp) << " " << tvmet::dot(omega, omega) <<" " << errsqr  <<endl;

		if (errsqr < maxIkErrorSqr) {
			isConverged = true;
			break;
		}
		setVector3(dp   , v, 0);
		setVector3(omega, v, 3);

		dmatrix invJ;
		calcPseudoInverse(J, invJ);

		dq = prod(invJ, v) + prod( didentity(n, n) - prod(invJ, J), calcGradient(0.0, 1.0) );
//		dq = prod(invJ, v) ;


		for (int j = 0; j < n; ++j)
			arm_path->joint(j)->q() += LAMBDA * dq(j);


		arm_path->calcForwardKinematics();
		
//		GraspController::instance()->flush();
		
	}
	
	

	if (!isConverged) {
		for (int i = 0; i < n; ++i) {
			arm_path->joint(i)->q() = qorg[i];
		}
		arm_path->calcInverseKinematics(p,R0);
		arm_path->calcForwardKinematics();
	}else{
		for(int i=0; i<n;i++){
			qorg[i] = arm_path->joint(i)->q();
		}
	}

	
	return isConverged;
}

dvector GraspPlanning_impl::calcGradient(double a, double b) {

	dvector grad(arm_path->numJoints());
	double indx = IndexFunc(a, b), EPS = 0.0001;

	for (int i = 0; i < arm_path->numJoints(); i++) {
		arm_path->joint(i)->q() += EPS;
		grad(i) = (IndexFunc(a, b) - indx) / EPS;
		arm_path->joint(i)->q() -= EPS;
	}

	return grad;

}
double GraspPlanning_impl::IndexFunc(double a, double b) {

//	double ret = a * Manipulability() + b / avoidAngleLimit();
	double ret = a * Manipulability() + b / avoidAngleLimit();

	return ret;

}


double GraspPlanning_impl::avoidAngleLimit() {

	//Close to the center
	double dist = 0.0;
	double weight[8] = {1,1,5,1,1,1,1,1};

	for (int i = 0; i < arm_path->numJoints(); i++){
		double edist =  weight[i]*(arm_path->joint(i)->q() - (arm_path->joint(i)->q_upper() + arm_path->joint(i)->q_lower()) / 2.0); // * 6.28 / (arm_path->joint(i)->ulimit - arm_path->joint(i)->llimit);
		dist +=  edist*edist;
	}

	if (isnan(sqrt(dist)))
		return 10000000.0;
	else
		return sqrt(dist);
}



double GraspPlanning_impl::Manipulability() {
	dmatrix J = arm_path->Jacobian();

	double d = det(prod(J, trans(J)));

	if (isnan(sqrt(d)))
		return 10000000.0;
	else
		return sqrt(d);
}

bool GraspPlanning_impl::checkArmLimit() {
	bool withinLimit = true;
	for (int i = 0; i < arm_path->numJoints(); i++)
		if (arm_path->joint(i)->q_upper() < arm_path->joint(i)->q()  ||  arm_path->joint(i)->q_lower() > arm_path->joint(i)->q())
			withinLimit = false;

	return withinLimit;
}

double GraspPlanning_impl::differenceFromStandardPose() {

	//Close to the center
	double dist = 0.0;
	double standard[8] = {0,0,-0.5,0,1.57,0,0,0};
	double weight[8] = {1,1,1,1,1,1,1,1};

	for (int i = 0; i < arm_path->numJoints(); i++){
		double edist =  weight[i]*(arm_path->joint(i)->q() - standard[i]); // * 6.28 / (arm_path->joint(i)->ulimit - arm_path->joint(i)->llimit);
		dist +=  edist*edist;
	}

	if (isnan(sqrt(dist)))
		return 10000000.0;
	else
		return sqrt(dist);
}


bool dvector0comp(const vector<double> &d1, const vector<double> &d2)
{
	return d1[0] < d2[0];
}


bool GraspPlanning_impl::SortGraspPosition(int ObjId, Vector3 objVisPos, Matrix33 objVisRot) {
	
	graspList.clear();
	
	gObjVisPos = objVisPos;
	gObjVisRot = objVisRot;
	

	double qtmp;
	Vector3 rpp;
	Matrix33 rpr;

	string obj_data   = "data/obj_list.txt";
	ifstream fin_obj(obj_data.c_str());
	string line2;

	string pos_data;
	string tmp_obj;

	if(!fin_obj){
		return false;
	}
	while(fin_obj){
		getline(fin_obj,line2);
		stringstream li2;
		li2 << line2;

		li2 >> qtmp;
		li2 >> tmp_obj;

		if(ObjId==qtmp){
			pos_data = "data/" + tmp_obj;
		}
	}
	cout << pos_data << endl;

	ifstream fin_pos(pos_data.c_str());
	if(!fin_pos){
		cout << "id not found" << ObjId << endl;
		return false;
	}

	string line;

	vector<double> fpos, fpos_temp, fpos_cand;
	while(fin_pos){
		getline(fin_pos,line);
		stringstream li;
		li << line;
		
		li >> qtmp;

		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				li >> rpr(i,j);
			}
		}
		for(int i=0;i<3;i++){
			li >> rpp(i);
		}

		Matrix33 tmpRot( (objVisRot)*rpr );
		Vector3  tmpPos( (objVisRot)*rpp+objVisPos );

		// --- Evaluation of grasp configuration --- 
		double  leng= norm2(tmpPos);
		double  Quality= 1.0/leng;
		
		if( IK_arm(tmpPos, tmpRot) ){
			Quality =differenceFromStandardPose();
//			Quality =avoidAngleLimit();
			cout << Quality <<" " <<checkArmLimit() << endl;
			for (int i = 0; i < arm_path->numJoints(); i++){
				cout << arm_path->joint(i)->q()*180.0/3.14 << " ";
			}
			cout << endl;
			if( !checkArmLimit() ) Quality+=10.0;
		}else{
			Quality = leng+=20;			
		}

		
		
		
		vector<double> record;
		
		record.push_back(Quality);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				record.push_back(tmpRot(i,j));
			}
		}
		for(int i=0;i<3;i++){
			record.push_back(tmpPos(i));
		}

		while(! li.eof()){
			double temp;
			li >> temp;
			record.push_back(temp);
		}
		graspList.push_back(record);
	}

	graspList.sort(dvector0comp);
	
	for(it=graspList.begin();it != graspList.end(); it++){
		cout << (*it)[0] << " " << (*it)[1] << endl;
	}
	it=graspList.begin();

	
	return true;

}


bool GraspPlanning_impl::SortReleasePosition(int ObjId, Vector3 objVisPos, Matrix33 objVisRot) {
	

	if( graspList.size() == 0) return false;
	
	Matrix33 GraspOri;
	Vector3 GraspPos;
	
	if(it != graspList.begin())  --it;
	vector <double> &record (*it); 
	int k=1;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			GraspOri(i,j) = record[k++];
		}
	}
	for(int i=0;i<3;i++){
		GraspPos[i]= record[k++];
	}
	double angle = record[k];
	
//	Matrix33 oGraspOri( (gObjVisRot)*rpr );
	Matrix33 rpr ( trans(gObjVisRot) * GraspOri );
	Vector3  rpp  ( trans(gObjVisRot) * (GraspPos-gObjVisPos) );

	graspList.clear();

	vector<double> fpos, fpos_temp, fpos_cand;

	for(int i=0;i<18;i++){

		Vector3 trpy (0,0,2.0*3.141592*i/18.0);
		Matrix33 tr = rotFromRpy(trpy);
		
		Matrix33 tmpRot( tr*(objVisRot)*rpr );
		Vector3  tmpPos( tr*(objVisRot)*rpp+objVisPos );

		// --- Evaluation of grasp configuration --- 
		double  leng= norm2(tmpPos);
		double  Quality= 1.0/leng;
		
		if( IK_arm(tmpPos, tmpRot) ){
			Quality =differenceFromStandardPose();
//			Quality =avoidAngleLimit();
			cout << Quality <<" " <<checkArmLimit() << endl;
			for (int i = 0; i < arm_path->numJoints(); i++){
				cout << arm_path->joint(i)->q()*180.0/3.14 << " ";
			}
			cout << endl;
			if( !checkArmLimit() ) Quality+=10.0;
		}else{
			Quality = leng+=20;			
		}
		vector<double> record;
		
		record.push_back(Quality);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				record.push_back(tmpRot(i,j));
			}
		}
		for(int i=0;i<3;i++){
			record.push_back(tmpPos(i));
		}

		record.push_back(angle);

		graspList.push_back(record);
	}

	graspList.sort(dvector0comp);
	
	for(it=graspList.begin();it != graspList.end(); it++){
		cout << (*it)[0] << " " << (*it)[1] << endl;
	}
	it=graspList.begin();

	
	return true;

}


bool GraspPlanning_impl::returnGraspPosition(Vector3& GraspPos, Matrix33& GraspOri, Vector3& ApproachPos, Matrix33& ApproachOri, double& angle, int& states){
	
	if(it == graspList.end()){
		states = 1;
		return false;
	}
	vector <double> &record (*it); 

	int k=1;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			GraspOri(i,j) = record[k++];
		}
	}
	for(int i=0;i<3;i++){
		GraspPos[i]= record[k++];
	}
	angle = record[k];
	
	ApproachOri = GraspOri;

	Vector3 aproachShift(0.0, 0.03, -0.1);
	ApproachPos = GraspPos - Vector3(GraspOri*aproachShift);
	
	states=0;

	it++;
	return true;	
}


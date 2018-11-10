
#include <fstream>
#include <string>
#include <iostream>

#include <math.h>

#include <algorithm>
#include <list>
#include <vector>

#include <time.h>
#include <sys/resource.h>

#include "GraspPlanningImpl.h"


#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)

using namespace dmatrix;
using namespace std;


double getrusage_sec() {
	struct rusage t;
	struct timeval tv;
	getrusage(RUSAGE_SELF, &t);
	tv = t.ru_utime;
	return tv.tv_sec + (double)tv.tv_usec*1e-6;
}


GraspPlanning_impl::GraspPlanning_impl(){

}


GraspPlanning_impl::~GraspPlanning_impl() {

}

GraspPlanning_impl* GraspPlanning_impl::instance() {
	static GraspPlanning_impl* instance = new GraspPlanning_impl();
	return instance;
}


bool dvector0comp(const vector<double> &d1, const vector<double> &d2)
{
	return d1[0] > d2[0];
}


bool GraspPlanning_impl::SortGraspPosition(int ObjId, dVector3 objVisPos, dMatrix33 objVisRot) {

	graspList.clear();

	double qtmp;
	dVector3 rpp;
	dMatrix33 rpr;

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

		dMatrix33 tmpRot = prod(objVisRot, rpr);
		dVector3  tmpPos = prod(objVisRot, rpp) + objVisPos;

		// --- Evaluation of grasp configuration ---

		double  leng= norm_2(tmpPos);
		double  Quality= 1.0/leng;

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

//	for(it=graspList.begin();it != graspList.end(); it++){
//		cout << (*it)[0] << " " << (*it)[1] << endl;
//	}
	it=graspList.begin();


	return true;

}


bool GraspPlanning_impl::returnGraspPosition(dVector3& GraspPos, dMatrix33& GraspOri, dVector3& ApproachPos, dMatrix33& ApproachOri, double& angle, int& states){

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

	dVector3 aproachShift;
	aproachShift[2] = 0.1;
	dVector3 tmpPos = prod(GraspOri, aproachShift);
	ApproachPos = GraspPos - tmpPos;

	states=0;

	it++;
	return true;
}


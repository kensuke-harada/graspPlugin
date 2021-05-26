// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <fstream>
#include <string>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <time.h>
#include <sstream>
#ifndef WIN32
#include <sys/resource.h>
#endif

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>

#include <cnoid/JointPath>
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>

#include "GraspDataGenerator.h"
#include "../Grasp/readtext.h"
#include "../Grasp/ForceClosureTest.h"
#include "../GeometryHandler/GeometryHandle.h"
#include "ConvexHull.h"
#include "../PCL/PointCloudHandlerInterface.h"
#include "../PickAndPlacePlanner/GraspDatabaseManipulator.h"

#include "../Grasp/DrawUtility.h"

#include <cnoid/ExecutablePath>
#ifdef _DEBUG
#undef _DEBUG
#include <Python.h>
#define _DEBUG
#else
#include <Python.h>
#endif

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

#define THREAD

//#define NOCHECK_BOUNDINBOXSIZE
//#define FLIP_NORMAL

#define DEBUG_PRINT

double grasp::_drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif

	}
using namespace std;
using namespace cnoid;
using namespace grasp;

GraspDataGenerator::GraspDataGenerator()  : 	os (MessageView::mainInstance()->cout() )
{
	num_cluster = 1;
	vol_ratio_threshold = 0.2;
	grid_interval = 0.01;
	depth_grid_interval = 0.01;
	max_edge_len = 0.0;
	size_coeff = 1.0;
	is_show_all_cluster = true;
	grasp_type = T_SINGLE;
	rotation_type = R_OCP;
	clustering_type = C_BOUNDINGBOX;
}

GraspDataGenerator::~GraspDataGenerator() {
}

GraspDataGenerator* GraspDataGenerator::instance(GraspDataGenerator *gc) {
	static GraspDataGenerator* instance = (gc) ? gc : new GraspDataGenerator();
	if(gc) instance = gc;
	return instance;
}

void GraspDataGenerator::showClusterMesh() {
	tc = PlanBase::instance();

	if (tc == NULL) {
		os << "implmentation error: you have to call initial()" << endl;
		return;
	}

	if ( !tc->targetObject ) {
		os << "Please select traget object" << endl;
		return;
	}

	// make object shape
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr c =  tc->targetObject->bodyItemObject->body()->link(0)->coldetModel();
	vector<double> vertex;
	float tx, ty, tz;
	for (int k = 0; k < c->getNumVertices(); k++) {
			c->getVertex(k, tx, ty, tz);
			vertex.push_back(tx);
			vertex.push_back(ty);
			vertex.push_back(tz);
	}
	vector<int> crd;
	int t1, t2, t3;
	for (int k = 0; k < c->getNumTriangles(); k++) {
			c->getTriangle(k, t1, t2, t3);
			crd.push_back(t1);
			crd.push_back(t2);
			crd.push_back(t3);
			crd.push_back(-1);
	}
#else
	cnoid::MeshExtractor extractor;
	cnoid::SgMesh* mesh = extractor.integrate(tc->targetObject->bodyItemObject->body()->link(0)->collisionShape());
	cnoid::SgVertexArrayPtr vertices = mesh->vertices();
	cnoid::SgIndexArray indices = mesh->triangleVertices();

	vector<double> vertex;
	for(int k = 0; k < vertices->size(); k++){
		cnoid::Vector3f vec = vertices->at(k);
		vertex.push_back( vec[0] );
		vertex.push_back( vec[1] );
		vertex.push_back( vec[2] );
	}
	vector<int> crd;
	for(int k = 0; k < mesh->numTriangles(); k++){
		crd.push_back( indices[k*3] );
		crd.push_back( indices[k*3+1] );
		crd.push_back( indices[k*3+2] );
		crd.push_back( -1 );
	}
#endif
	// get triangle points of each clusters
	vector<vector<int> > ids;
	vector<Vector3> points;
	ObjectShape(vertex,crd).getClusterTrianglePoints(num_cluster,points,ids,vol_ratio_threshold,scale);

	// transform points
	for (size_t i = 0; i < points.size(); i++) {
		points[i] = tc->targetObject->object->R() * points[i] + tc->targetObject->object->p();
	}

	// draw
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	draw->points = points;
	draw->colors.clear();
	draw->triangles.clear();
	for (size_t i = 0; i < ids.size(); i++) {
		draw->triangles.push_back(ids[i]);
		draw->colors.push_back(Vector3(_drand(), _drand(), _drand()));
	}

	draw->displayTriangles();
}

void GraspDataGenerator::showCylinder() {
	tc = PlanBase::instance();

	if (tc == NULL) {
		os << "implmentation error: you have to call initial()" << endl;
		return;
	}

	if ( !tc->targetObject ) {
		os << "Please select traget object" << endl;
		return;
	}

	// make object shape
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr c =  tc->targetObject->bodyItemObject->body()->link(0)->coldetModel();
#else
	ColdetModelPtr c =  ColdetConverter::ConvertFrom(
		tc->targetObject->bodyItemObject->body()->link(0)->collisionShape());
#endif
	vector<double> vertex;
	float tx, ty, tz;
	for (int k = 0; k < c->getNumVertices(); k++) {
			c->getVertex(k, tx, ty, tz);
			vertex.push_back(tx);
			vertex.push_back(ty);
			vertex.push_back(tz);
	}
	vector<int> crd;
	int t1, t2, t3;
	for (int k = 0; k < c->getNumTriangles(); k++) {
			c->getTriangle(k, t1, t2, t3);
			crd.push_back(t1);
			crd.push_back(t2);
			crd.push_back(t3);
			crd.push_back(-1);
	}
	PointCloudHandlerInterface* ph = new PointCloudHandlerInterface();
	vector<PointCloudHandlerInterface::Cylinder> cylinders;
	ph->cylinderSegmentation(c, cylinders, dist_th, num_cluster);

	DrawUtility* draw = DrawUtility::instance();
	draw->clear();
	for (size_t i = 0; i < cylinders.size(); i++) {
		draw->cylinders.push_back(Cylinders(
			tc->targetObject->object->R() * cylinders[i].pos + tc->targetObject->object->p(),
			tc->targetObject->object->R() * cylinders[i].dir,
			cylinders[i].radius,
			cylinders[i].len,
			Vector3(_drand(), _drand(), _drand()), 0.7));
		//draw->boxes.push_back(Boxes(cylinders[i].pos,
		//	rotFromTwoVecs(Vector3(1,0,0),cylinders[i].dir),
		//	Vector3(cylinders[i].len,cylinders[i].radius*2,cylinders[i].radius*2),
		//	Vector3(_drand(), _drand(), _drand()), 0.7));
	}
	draw->displayShapes();
}

void GraspDataGenerator::showBoundingBox(){
	time_t start_time,end_time;
	double sec_time;
	time (&start_time);

	tc = PlanBase::instance();

	if(tc == NULL){
		os << "implmentation error: you have to call initial()" << endl;
	}

	if ( !tc->targetObject ) {
		os << "Please select traget object" << endl;
		return;
	}
	// make cluster
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    ColdetModelPtr c =  PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->coldetModel();
#else
	ColdetModelPtr c =  ColdetConverter::ConvertFrom(
		PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->collisionShape());
#endif
	vector<double> vertex;
	float tx, ty, tz;
	for (int k = 0; k < c->getNumVertices(); k++) {
			c->getVertex(k, tx, ty, tz);
			vertex.push_back(tx);
			vertex.push_back(ty);
			vertex.push_back(tz);
	}
	vector<int> crd;
	int t1, t2, t3;
	for (int k = 0; k < c->getNumTriangles(); k++) {
			c->getTriangle(k, t1, t2, t3);
			crd.push_back(t1);
			crd.push_back(t2);
			crd.push_back(t3);
			crd.push_back(-1);
	}
	vector<Vector3> edges;
	vector<Vector3> centers;
	vector<Matrix3> rots;

	ObjectShape(vertex,crd).getClusterParameters(num_cluster,&edges,&centers,&rots,vol_ratio_threshold,scale);

	vector<Vector3> target_edges;
	vector<Vector3> target_centers;
	vector<Matrix3> target_rots;

	double obj_edge_len=0;
	Vector3 edge,center,com;
	Matrix3 Rot;
	PlanBase::instance()->calcBoundingBox(c,edge,center,com,Rot);

	int num_target_cluster=0;

	obj_edge_len = edge(0) + edge(1) + edge(2);
	for(int i=0;i<edges.size();i++){
		double edge_len = (edges[i](0) + edges[i](1) + edges[i](2));
		if(0.15*obj_edge_len <  edge_len || is_show_all_cluster){
			target_edges.push_back(edges[i]);
			target_centers.push_back(centers[i]);
			target_rots.push_back(rots[i]);
			num_target_cluster++;
		}
	}

	// Clear shapes
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	addClusterBox(target_edges,target_centers,target_rots);

	draw->displayShapes();

	os << "The number of bouding boxes:" << edges.size() << endl;
	os << "The number of target cluters:" << num_target_cluster << endl;

		time(&end_time);

	sec_time = difftime(end_time,start_time);

#ifdef DEBUG_PRINT
	os << "time:" <<  sec_time << endl;
#endif

	return;
}

void GraspDataGenerator::showContactPoint(){
		tc = PlanBase::instance();

	if(tc == NULL){
		os << "implmentation error: you have to call initial()" << endl;
	}

	if ( !tc->targetObject ) {
		os << "Please select target object" << endl;
		return;
	}

	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	vector<DataPosturePos> postures;

	string cfile = tc->dataFilePath() + "contact_" + tc->targetObject->bodyItemObject->name() + ".txt";

	ifstream fin;

	fin.open(cfile.c_str());

	if(!fin){
		os << "cannot open " + cfile << endl;
	}

	while(!fin.eof()){
		string line;
		getline(fin,line);
		stringstream ss(line);
		vector<double> x;
		double tmp;
		DataPosturePos pos;
		while(!ss.eof()){
			ss >> tmp;
			if(ss.eof()) break;
			x.push_back(tmp);
		};
		if(x.size() < 7) break;
		pos.q = x[0];
		vector<Vector3> contact_pos;
		for(int i=1;i<x.size();i+=3){
			contact_pos.push_back(Vector3(x[i],x[i+1],x[i+2]));
		}
		pos.contact_pos = contact_pos;
		postures.push_back(pos);
	}

	fin.close();

	addContactPoint(postures);

	draw->displayShapes();
}

void GraspDataGenerator::showApproachPoint(){
	tc = PlanBase::instance();

	PrehensionPtr target_prehension = tc->targetArmFinger->getTargetPrehension()[0];
	settingPrehension(target_prehension);

	if (tc == NULL) {
		os << "implmentation error: you have to call initial()" << endl;
	}

	if (!tc->targetObject) {
		os << "Please select target object" << endl;
		return;
	}

	GraspDatabaseManipulator gdm;
	string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	gdm.readFile(filepath);
	GraspDatabaseManipulator::GraspPoses grasp_poses = gdm.getRecords();

	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	std::vector<DataPosturePos> postures;

	for (size_t i = 0; i < grasp_poses.size(); i++) {
		DataPosturePos pose;
		pose.p = grasp_poses[i].p;
		pose.R = grasp_poses[i].R;
		pose.q = grasp_poses[i].q;
		postures.push_back(pose);
	}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr c =  PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->coldetModel();
#else
	ColdetModelPtr c =  ColdetConverter::ConvertFrom(
		PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->collisionShape());
#endif

	vector<Vector3> edges;
	vector<Vector3> centers;
	vector<Matrix3> rots;

	ObjectShape(c).getClusterParameters(1, &edges, &centers, &rots, 0.2, 1);

	addClusterBox(edges, centers, rots);
	addApproachPoint(edges, centers, rots, postures);

	draw->displayShapes();
}

#if 0
// for test
void GraspDataGenerator::testGraspSide(){
		tc = PlanBase::instance();

	GraspController::instance()->initial(tc->targetObject,  tc->targetArmFinger);
	initial(tc->targetObject, tc->targetArmFinger);

	returnRefMotion(false,reffile[0]);

	int nContact = 0;
	for(int i=0;i<tc->nFing();i++) for(int j=0;j<tc->fingers(i)->nJoints;j++) if(tc->fingers(i)->contact[j]) nContact++;

	vector <Vector3> objPos(nContact), objN(nContact);

	int cnt=0;
	for(int i=0;i<tc->nFing();i++){
		fingers(i)->contactSearch(cnt,2000,&objPos[0],&objN[0]);
	}
	bodyItemRobot()->body()->calcForwardKinematics();
	tc->flush();

	objPos.resize(cnt);
	objN.resize(cnt);

	VectorXd wrench = VectorXd::Zero(6);

	double Qtmp;
	double mu = tc->targetArmFinger->mu;
	double fmax = tc->targetArmFinger->fmax;
	double mg = tc->targetObject->objMass * 9.8;

	//objPos.push_back(tc->targetObject->objCoM_);
	//objN.push_back(Vector3(0,0,1));
	//cnt++;

	for(int i=0;i<cnt;i++){
		os << "pos" << endl;
		os << objPos[i] - tc->targetObject->objCoM_ << endl;
		os << "N" << endl;
		os << objN[i] << endl;
	}

					Vector3 tmp_w;
                tmp_w = tc->targetObject->object->R().transpose() * Vector3(0,0,-mg);
				for(int i=0;i<3;i++) wrench(i) = tmp_w(i);
	Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench,&objPos[0],&objN[0],cnt,mu,fmax,tc->targetObject->objCoM_ );

	os << "Q(-mg)" << Qtmp << endl;

                tmp_w = tc->targetObject->object->R().transpose() * Vector3(0,0,-mg);
				for(int i=0;i<3;i++) wrench(i) = objN[0](i);
	Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench,&objPos[0],&objN[0],cnt,mu,fmax,tc->targetObject->objCoM_ );

	//Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoid(wrench,&objPos[0],&objN[0],cnt,mu,fmax,tc->targetObject->objCoM_ );

	os << "Q()" << Qtmp << endl;


                tmp_w = tc->targetObject->object->R().transpose() * Vector3(0,0,-mg);
				for(int i=0;i<3;i++) wrench(i) =  30 * objN[0](i);
	Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench,&objPos[0],&objN[0],cnt,mu,fmax,tc->targetObject->objCoM_ );

	os <<"Q2()" << Qtmp << endl;
	os << "mg" << endl;
	os << tmp_w << endl;

}
#endif

bool GraspDataGenerator::generateGraspPattern(){
	time_t start_time,end_time;
	double sec_time;
	time (&start_time);

	tc = PlanBase::instance();

	if(tc == NULL){
		os << "implmentation error: you have to call initial()" << endl;
		return false;
	}

	if ( !tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}
	GraspController::instance()->initial(tc->targetObject,  tc->targetArmFinger);

	SoftFingerStabilityHandler::instance()->initialize();

	initial(tc->targetObject, tc->targetArmFinger);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	obj_shape = createObjectShape(tc->targetObject->bodyItemObject->body()->link(0)->coldetModel());
#else
	obj_shape = createObjectShape(ColdetConverter::ConvertFrom(
		tc->targetObject->bodyItemObject->body()->link(0)->collisionShape()));
#endif
	for(int i=0;i<tc->targetArmFinger->nFing;i++){
        int idx = tc->fingers(i)->fing_path->joint(tc->fingers(i)->nJoints-1)->index();
		finger_shape.push_back(
			createObjectShape(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				tc->bodyItemRobot()->body()->link(idx)->coldetModel()
#else
				ColdetConverter::ConvertFrom(
					tc->bodyItemRobot()->body()->link(idx)->collisionShape())
#endif
			));
	}

	vector<Vector3> target_edges;
	vector<Vector3> target_centers;
	vector<Matrix3> target_rots;

	if(clustering_type == C_CYLINDER){

		// make object shape
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ColdetModelPtr c =  tc->targetObject->bodyItemObject->body()->link(0)->coldetModel();
#else
		ColdetModelPtr c =  ColdetConverter::ConvertFrom(
			tc->targetObject->bodyItemObject->body()->link(0)->collisionShape());
#endif
		vector<double> vertex;
		float tx, ty, tz;
		for (int k = 0; k < c->getNumVertices(); k++) {
				c->getVertex(k, tx, ty, tz);
				vertex.push_back(tx);
				vertex.push_back(ty);
				vertex.push_back(tz);
		}
		vector<int> crd;
		int t1, t2, t3;
		for (int k = 0; k < c->getNumTriangles(); k++) {
				c->getTriangle(k, t1, t2, t3);
				crd.push_back(t1);
				crd.push_back(t2);
				crd.push_back(t3);
				crd.push_back(-1);
		}

		PointCloudHandlerInterface* ph = new PointCloudHandlerInterface();
		vector<PointCloudHandlerInterface::Cylinder> cylinders;
		ph->cylinderSegmentation(c, cylinders, dist_th, num_cluster);

		for (size_t i = 0; i < cylinders.size(); i++) {
			Vector3 x(1,0,0);
			target_centers.push_back(cylinders[i].pos);
			target_edges.push_back(Vector3(cylinders[i].len,cylinders[i].radius*2,cylinders[i].radius*2));
			target_rots.push_back(rotFromTwoVecs(x,cylinders[i].dir));

			target_centers.push_back(cylinders[i].pos);
			target_edges.push_back(Vector3(cylinders[i].len,cylinders[i].radius*2,cylinders[i].radius*2));
			target_rots.push_back(rotFromTwoVecs(x,cylinders[i].dir) * rotFromRpy(M_PI/4.0, 0, 0));

			double edge_len = cylinders[i].len + cylinders[i].radius*2 + cylinders[i].radius*2;
			if(max_edge_len < edge_len) max_edge_len = edge_len;
		}
	}else{
		getBoundingboxParameters(num_cluster,vol_ratio_threshold,scale,target_edges,target_centers,target_rots);
	}
	vector<DataPosturePos> postures;

	// Clear shapes
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	searchGraspPattern(target_edges,target_centers,target_rots,postures);

	string pfile = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	writePreplanning(pfile,postures);

#ifdef OUTPUT_RESULT
	string enfile = tc->dataFilePath() + "en_" +  tc->targetObject->bodyItemObject->name() + ".txt";
	writeEnvalue(enfile,postures);

	string cfile = tc->dataFilePath() + "contact_" + tc->targetObject->bodyItemObject->name() + ".txt";
	writeContactpoint(cfile,postures);
#endif

	//display cluster bouding boxes
#if defined(DRAW_CONTACTPOINT_ON_BOX) || defined(DRAW_APPROACHPOINT)
	addClusterBox(edges,centers,rots);
#endif

	//display control points
#ifdef DRAW_APPROACHPOINT
	addApproachPoint(edges,centers,rots,postures);
#elif DRAW_CONTACTPOINT_ON_BOX
	addContactPointonBox(edges,centers,rots,postures);
#else
	addContactPoint(postures);
#endif

	draw->displayShapes();

	delete obj_shape;
	for(int i=0;i<finger_shape.size();i++){
		delete finger_shape[i];
	}
	finger_shape.clear();

	time(&end_time);

	sec_time = difftime(end_time,start_time);

#ifdef OUTPUT_RESULT
	string logfile = tc->dataFilePath() + "log_" + tc->targetObject->bodyItemObject->name() + ".txt";
	ofstream fout(logfile.c_str());
	fout << sec_time << " " << sum_success << " " << sum_approach_points << " " << sum_search_point;
	fout.close();
#endif

	os << "GraspPattern generating is finished" << endl;

#ifdef DEBUG_PRINT
	os << "time:" <<  sec_time << endl;
#endif
	return true;
}

void GraspDataGenerator::getBoundingboxParameters(int num_cluster,double vol_ratio_threshold,double scale,
	vector<Vector3>& target_edges,vector<Vector3>& target_centers,vector<Matrix3>& target_rots){
		vector<Vector3> edges;
		vector<Vector3> centers;
		vector<Matrix3> rots;

		ObjectShape(obj_shape).getClusterParameters(num_cluster,&edges,&centers,&rots,vol_ratio_threshold,scale);

		double obj_edge_len=0;
		max_edge_len = 0;
		Vector3 edge,center;
		Matrix3 Rot;

		ObjectShape(obj_shape).calcBoundingBox(edge,center,Rot);
		obj_edge_len = edge(0) + edge(1) + edge(2);
		for(int i=0;i<edges.size();i++){
			double edge_len = (edges[i](0) + edges[i](1) + edges[i](2));
			if(0.15*obj_edge_len <  edge_len){
				target_edges.push_back(edges[i]);
				target_centers.push_back(centers[i]);
				target_rots.push_back(rots[i]);
			}
		if(max_edge_len < edge_len) max_edge_len = edge_len;
		}

}

void GraspDataGenerator::searchGraspPattern(vector<Vector3>& target_edges,vector<Vector3>& target_centers,vector<Matrix3>& target_rots,vector<DataPosturePos>& postures){
	// set collision pair
	/*tc->robotObjPairs.clear();
	for (int i = 0;i < tc->nHandLink();i++){
        int l = tc->handJoint()->link(i)->jointId();
#ifdef  CNOID_10_11_12_13
		tc->robotObjPairs.push_back(new ColdetLinkPair(tc->handJoint()->link(i),tc->object()));
#else
		tc->robotObjPairs.push_back(boost::make_shared<ColdetLinkPair>(tc->body(),tc->handJoint()->link(i),tc->targetObject->bodyItemObject->body(),tc->object()));
#endif
	}*/
	tc->robotObjPairWithoutHand.clear();
	for (size_t i = 0; i < tc->handObjPair.size(); i++) {
		tc->handObjPair[i].clear();
	}
	for (int i = 0;i < tc->nHandLink();i++){
#ifdef  CNOID_10_11_12_13
		tc->handObjPair[0].push_back(new ColdetLinkPair(tc->handJoint()->link(i),tc->object()));
#else
		tc->handObjPair[0].push_back(boost::make_shared<ColdetLinkPair>(tc->body(),tc->handJoint()->link(i),tc->targetObject->bodyItemObject->body(),tc->object()));
#endif
	}


	sum_success = sum_approach_points = sum_search_point = 0;
	for(int i=0;i<target_edges.size();i++){
		num_success = 0;
		num_approach_points = 0;
		num_search_point = 0;
		if (rotation_type == R_OCP) {
			generateGraspPostures(i,target_edges[i],target_centers[i],target_rots[i],postures);
		} else {
			generateGraspPosturesStablePut(target_edges[i],target_centers[i],target_rots[i],postures);
		}
		sum_success += num_success;
		sum_approach_points += num_approach_points;
		sum_search_point += num_search_point;
#ifdef DEBUG_PRINT
		os << "edge id:" << i << endl;
		os << target_edges[i] << endl;
		os << "approach_points:" << num_approach_points << endl;
		os << "search_points:" << num_search_point << endl;
		os << "success:" << num_success << endl;
#endif
	}
#ifdef DEBUG_PRINT
	os << "end" << endl;
#endif

	sort(postures.begin(),postures.end(),DataPosturePos::sortPosturePosData);
}

void GraspDataGenerator::writePreplanning(string filename,vector<DataPosturePos>& postures){
	ofstream fout( filename.c_str());

	for(int i=0;i<postures.size();i++){
		fout << postures[i].q << " ";
		 for(int m=0;m<3;m++){
			 for(int n=0;n<3;n++){
				 fout << postures[i].R(m,n) << " ";
			 }
		 }
		 for(int m=0;m<3;m++){
			 fout << postures[i].p(m) << " ";
		 }
		 for (int m = 0;m < postures[i].hand_angle.size();m++){
			 fout <<  postures[i].hand_angle[m] << " ";
		 }
		 fout << endl;
	}

	fout.close();
}

void GraspDataGenerator::writeContactpoint(string filename,vector<DataPosturePos>& postures){
	ofstream fout(filename.c_str());

	for(int i=0;i<postures.size();i++){
		fout << postures[i].q << " ";
		for(int m=0;m<postures[i].contact_pos.size();m++){
			for(int n=0;n<3;n++){
				fout << postures[i].contact_pos[m](n) << " ";
			}
		}
		fout << endl;
	}
	fout.close();
}

void GraspDataGenerator::writeEnvalue(string filename,vector<DataPosturePos>& postures){
	ofstream fout(filename.c_str());

	for(int i=0;i<postures.size();i++){
		fout << postures[i].q << " ";

		for(int m=0;m<postures[i].en.size();m++){
			fout << postures[i].en[m] << " ";
			fout << postures[i].area[m] << " ";
			for(int n=0;n<3;n++){
				fout << postures[i].contact_pos[m][n] << " ";
			}
			for(int n=0;n<3;n++){
				fout << postures[i].contact_N[m][n] << " ";
			}
			for(int n=0;n<3;n++){
				fout << postures[i].fing_pos[m][n] << " ";
			}
			for(int n=0;n<3;n++){
				fout << postures[i].fing_N[m][n] << " ";
			}
		}
		fout << endl;
	}
	fout.close();

}

void GraspDataGenerator::addClusterBox(vector<Vector3>& edges,vector<Vector3>& centers,vector<Matrix3>& rots){
	DrawUtility* draw = DrawUtility::instance();
	for(int i=0;i<edges.size();i++){
		Vector3 color = Vector3(_drand(),_drand(),_drand());
        Matrix3 objR = tc->targetObject->object->R();
        Vector3 p = tc->targetObject->object->p() + objR * centers[i];
		draw->boxes.push_back(Boxes(p,objR* rots[i],edges[i],color,0.8));
	}
}

void GraspDataGenerator::addApproachPoint(vector<Vector3>& edges,vector<Vector3>& centers,vector<Matrix3>& rots,vector<DataPosturePos>& postures){
	DrawUtility* draw = DrawUtility::instance();
	double radius = 0.001;

	std::vector<Vector3> ns;
	ns.push_back(Vector3(1,0,0));
	ns.push_back(Vector3(-1,0,0));
	ns.push_back(Vector3(0,1,0));
	ns.push_back(Vector3(0,-1,0));
	ns.push_back(Vector3(0,0,1));
	ns.push_back(Vector3(0,0,-1));

	for(int i=0;i<postures.size();i++){
		Vector3 color = getColor(postures[i].q);

        Matrix3 objR = tc->targetObject->object->R();
		Vector3 control_n = objR * postures[i].R * GRCmax.R * Vector3(0,1,0);
        Vector3 control_p = tc->targetObject->object->p() + objR * (postures[i].p +  postures[i].R * GRCdes.p)-control_n;

		Vector3 closest_point;
		double min_dist = 10e8;

		// project control point onto bounding box
		for(int k=0;k<edges.size();k++){
			for(int n=0;n<ns.size();n++){
				Vector3 bb_n = objR * rots[k] * ns[n];
                Vector3 bb_p = tc->targetObject->object->p() + objR * centers[k] + objR * rots[k] * Vector3(ns[n](0) *  edges[k](0)/2,ns[n](1) *  edges[k](1)/2,ns[n](2) *  edges[k](2)/2);
				double tmp = control_n.dot(bb_n);
				if(fabs(tmp) < 0.0001)
					continue;
				double t = (bb_n.dot(bb_p) - bb_n.dot(control_p)) / tmp;
				Vector3 cross_p = t * control_n + control_p;
				Vector3 diff_control_cross = control_p - cross_p;
				double dist = fabs(diff_control_cross(0)) + fabs(diff_control_cross(1)) + fabs(diff_control_cross(2));

				if(k != postures[i].cluster_num){
                    Vector3 point_cluster = (objR * rots[k]).transpose() * (cross_p-(tc->targetObject->object->p()+ objR * centers[k]));
					if((fabs(ns[n](0)) < 0.5) && ((point_cluster(0) > edges[k](0)/2) || (point_cluster(0) < -edges[k](0)/2))) continue;
					if((fabs(ns[n](1)) < 0.5) && ((point_cluster(1) > edges[k](1)/2) || (point_cluster(1) < -edges[k](1)/2))) continue;
					if((fabs(ns[n](2)) < 0.5) && ((point_cluster(2) > edges[k](2)/2) || (point_cluster(2) < -edges[k](2)/2))) continue;
				}
				if(min_dist > dist){
					closest_point = cross_p;
					min_dist = dist;
				}
			}
		}
		draw->spheres.push_back(Spheres(closest_point,radius,color));
	}
}

void GraspDataGenerator::addContactPoint(vector<DataPosturePos>& postures){
	DrawUtility* draw = DrawUtility::instance();
	double radius = 0.0005;

	for(int i=0;i<postures.size();i++){
		Vector3 color = getColor(postures[i].q);
        Matrix3 objR = tc->targetObject->object->R();

		for(int j=0;j<postures[i].contact_pos.size();j++){
            Vector3 contact_point = tc->targetObject->object->p() + objR * postures[i].contact_pos[j];
			draw->spheres.push_back(Spheres(contact_point,radius,color));
		}
	}
}

void GraspDataGenerator::addContactPointonBox(vector<Vector3>& edges,vector<Vector3>& centers,vector<Matrix3>& rots,vector<DataPosturePos>& postures){
	DrawUtility* draw = DrawUtility::instance();
	double radius = 0.0005;

	std::vector<Vector3> ns;
	ns.push_back(Vector3(1,0,0));
	ns.push_back(Vector3(-1,0,0));
	ns.push_back(Vector3(0,1,0));
	ns.push_back(Vector3(0,-1,0));
	ns.push_back(Vector3(0,0,1));
	ns.push_back(Vector3(0,0,-1));

	for(int i=0;i<postures.size();i++){
		Vector3 color = getColor(postures[i].q);
        Matrix3 objR = tc->targetObject->object->R();

		for(int j=0;j<postures[i].contact_pos.size();j++){
            Vector3 contact_p = tc->targetObject->object->p() + objR * postures[i].contact_pos[j];
			Vector3 contact_n = objR * postures[i].R * GRCmax.R * Vector3(0,0,1);
			//Vector3 contact_n = postures[i].contact_N[j];

			Vector3 closest_point;
			double min_dist = 10e8;

			// project control point onto bounding box
			for(int k=0;k<edges.size();k++){
				for(int n=0;n<ns.size();n++){
					Vector3 bb_n = objR * rots[k] * ns[n];
                    Vector3 bb_p = tc->targetObject->object->p() + objR * centers[k] + objR * rots[k] * Vector3(ns[n](0) *  edges[k](0)/2,ns[n](1) *  edges[k](1)/2,ns[n](2) *  edges[k](2)/2);
					double tmp = contact_n.dot(bb_n);
					if(fabs(tmp) < 0.0001)
						continue;
					double t = (bb_n.dot(bb_p) - bb_n.dot(contact_p)) / tmp;
					Vector3 cross_p = t * contact_n + contact_p;
					Vector3 diff_control_cross = contact_p - cross_p;
					double dist = fabs(diff_control_cross(0)) + fabs(diff_control_cross(1)) + fabs(diff_control_cross(2));

					if(k != postures[i].cluster_num){
                        Vector3 point_cluster = (objR * rots[k]).transpose() * (cross_p-(tc->targetObject->object->p()+ objR * centers[k]));
						if((fabs(ns[n](0)) < 0.5) && ((point_cluster(0) > edges[k](0)/2) || (point_cluster(0) < -edges[k](0)/2))) continue;
						if((fabs(ns[n](1)) < 0.5) && ((point_cluster(1) > edges[k](1)/2) || (point_cluster(1) < -edges[k](1)/2))) continue;
						if((fabs(ns[n](2)) < 0.5) && ((point_cluster(2) > edges[k](2)/2) || (point_cluster(2) < -edges[k](2)/2))) continue;
					}

					if(min_dist > dist){
						closest_point = cross_p;
						min_dist = dist;
					}
				}
			}
			draw->spheres.push_back(Spheres(closest_point,radius,color));
		}
	}
}

cnoid::Vector3 GraspDataGenerator::getColor(double value){
	Vector3 min_color = Vector3(1,1,1);
	Vector3 max_color = Vector3(1,0,0);
	double min_q = 0.0;
	double max_q = 0.17;

	double r=min_color(0) + std::min(((std::max((value-min_q),0.0))/(max_q-min_q)),1.0)*(max_color(0)-min_color(0));
	double g=min_color(1) + std::min(((std::max((value-min_q),0.0))/(max_q-min_q)),1.0)*(max_color(1)-min_color(1));
	double b=min_color(2) + std::min(((std::max((value-min_q),0.0))/(max_q-min_q)),1.0)*(max_color(2)-min_color(2));
	return Vector3(r,g,b);
}

void GraspDataGenerator::generateGraspPostures(int cluster_num,cnoid::Vector3& edge,cnoid::Vector3& center,cnoid::Matrix3& rot,vector<DataPosturePos>& postures){
	tc = PlanBase::instance();

	returnRefMotion(false, tc->targetArmFinger->prehensionList[0]);

	Box OCP;
	OCP.edge = edge;

	//grid_interval = 0.3 * (edge(0) + edge(1) + edge(2)) / 3;
	double edge_len = edge(0) + edge(1) + edge(2);
	grid_interval = 0.25*(pow(max_edge_len/edge_len,0.5))*edge_len / 3;
	size_coeff = (edge(0) + edge(1) + edge(2)) / max_edge_len;

	Homogenous relOCPface;
	Homogenous Palm;
	Vector3 ey(0, 1, 0);
	relOCPface.p << 0, 0, 0;
	relOCPface.R << 0, 0, 1, 1, 0, 0, 0, 1, 0;

	// save initial finger angles
	vector <double* > link_org_q(nFing());
	for (int l = 0; l < nFing(); l++) {
		link_org_q[l] = new double[ fingers(l)->nJoints ];
		for (int m = 0; m < fingers(l)->nJoints; m++) {
            link_org_q[l][m] = fingers(l)->fing_path->joint(m)->q();
		}
	}

    Palm.R = palm()->R();
    Palm.p = palm()->p();

	OCP.R =  Palm.R * GRCmax.R * (relOCPface.R);

	//==For six faces of OCP_==
	for (int i = 0; i < 6; i++) {
		Homogenous OCPface_;
		OCPface_.R = calcObjRot(OCP.R, i);
		//==For four rotation angles about OCP_ normal==
		for (int j = 0; j < 4; j++) {
			Homogenous OCPface;
			OCPface.R = calcHandRot(OCPface_.R,j);
            tc->targetObject->object->R() = OCPface.R * rot.transpose();
            tc->targetObject->object->p() = Palm.p - Palm.R * dGRC_Pos_ +  Palm.R * GRCdes.p -OCPface.R  * rot.transpose() *  center;
			tc->targetObject->bodyItemObject->body()->calcForwardKinematics();
			tc->flush();

			Vector3 d0 ((Palm.R *  GRCmax.R).transpose() * OCPface.R * OCP.edge );

			switch (grasp_type) {
			case T_SINGLE:
#ifdef NOCHECK_BOUNDINBOXSIZE
			dif = Vector3 ( fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[0] ),
			 				fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[1] ),
			 				fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[2] ) );
#else
			if( fabs(d0[2])>GRCmax.edge[2] ) continue;
			if( fabs(d0[1])>GRCmax.edge[1] ) continue;

			if (fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[2] ) > -fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[2] ) + fabs( GRCmax.edge[2]) ){
			dif = Vector3 ( fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[0] ),
			 				fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[1] ),
							fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[2] ) );
			}else{
				dif = Vector3 ( fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[0] ),
			 				fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[1] ),
			 				-fabs( (trans(Matrix3(Palm.R*GRCmax.R))*OCPface.R*OCP.edge)[2] ) + fabs( GRCmax.edge[2]) );
			}
#endif
			searchGraspPosture(cluster_num,Palm,link_org_q,postures);
			break;
			case T_LIFTUP:
				if(fabs(d0[1]) < GRCmax.edge[1]) continue;
				searchGraspPostureLiftUp(cluster_num,Palm,d0,link_org_q,postures);
				break;
			case T_SIDEDUAL:
				searchGraspPostureFromSide(cluster_num,Palm,d0,link_org_q,postures);
				break;
			}
		}
	}

	// restore finger angles
	for(int m=0;m<nFing();m++){
		for (int n = 0; n < fingers(m)->nJoints; n++) {
            fingers(m)->fing_path->joint(n)->q() = link_org_q[m][n] ;
		}
	}
	bodyItemRobot()->body()->calcForwardKinematics();

	tc->flush();

	for(int i=0;i<link_org_q.size();i++){
		delete[] link_org_q[i];
	}

}

void GraspDataGenerator::generateGraspPosturesStablePut(cnoid::Vector3& edge,cnoid::Vector3& center,cnoid::Matrix3& rot,vector<DataPosturePos>& postures){
	tc = PlanBase::instance();

	returnRefMotion(false, tc->targetArmFinger->prehensionList[0]);

	Box OCP;
	OCP.edge = edge;

	//grid_interval = 0.3 * (edge(0) + edge(1) + edge(2)) / 3;
	double edge_len = edge(0) + edge(1) + edge(2);
	grid_interval = 0.25*(pow(max_edge_len/edge_len,0.5))*edge_len / 3;
	size_coeff = (edge(0) + edge(1) + edge(2)) / max_edge_len;

	Homogenous relOCPface;
	Homogenous Palm;
	Vector3 ey(0, 1, 0);
	relOCPface.p << 0, 0, 0;
	//relOCPface.R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
	relOCPface.R << 1,0,0,0,1,0,0,0,1;

	// save initial finger angles
	vector <double* > link_org_q(nFing());
	for (int l = 0; l < nFing(); l++) {
		link_org_q[l] = new double[ fingers(l)->nJoints ];
		for (int m = 0; m < fingers(l)->nJoints; m++) {
            link_org_q[l][m] = fingers(l)->fing_path->joint(m)->q();
		}
	}

    Palm.R = palm()->R();
    Palm.p = palm()->p();
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr c =  PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->coldetModel();
	vector<double> vertex;
	float tx, ty, tz;
	for (int k = 0; k < c->getNumVertices(); k++) {
		c->getVertex(k, tx, ty, tz);
		vertex.push_back( tx );
		vertex.push_back( ty );
		vertex.push_back( tz );
	}
#else
	cnoid::MeshExtractor extractor;
	cnoid::SgMesh* mesh = extractor.integrate(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->collisionShape());
  cnoid::SgVertexArrayPtr vertices = mesh->vertices();
  cnoid::SgIndexArray indices = mesh->triangleVertices();

  vector<double> vertex;
  for(int k = 0; k < vertices->size(); k++){
    cnoid::Vector3f vec = vertices->at(k);
    vertex.push_back( vec[0] );
    vertex.push_back( vec[1] );
    vertex.push_back( vec[2] );
  }
#endif

	ConvexHull* ch = new ConvexHull();
	ch->createConvexHull(vertex);

	const int x_bit = 0x0001;
	const int y_bit = 0x0002;
	const int z_bit = 0x0004;
	vector<Vector3> ver;
	for (unsigned int i = 0; i < 8; i++) {
		Vector3 sign;
		sign(0) = (i & x_bit) ? 1 : -1;
		sign(1) = (i & y_bit) ? 1 : -1;
		sign(2) = (i & z_bit) ? 1 : -1;
		ver.push_back((edge/2.0).cwiseProduct(sign));
	}

	Face::FaceList faces = ch->getFaces();
	for (Face::FaceListIterator fli = faces.begin(); fli != faces.end(); ++fli) {
		Face* target_face = (*fli);
		if(!ch->isStable((*fli)->getID(), tc->targetObject->objCoM_)) continue;
		Vector3 n = -target_face->getNormal();
		Vector3 intersect_p = ch->getIntersectPoint(n, tc->targetObject->objCoM_);
		double dist = norm2(intersect_p - tc->targetObject->objCoM_);
		Vector3 gravity_dir(0, 0, -1);
		Matrix3 base_r = rotFromTwoVecs(n, gravity_dir);

		for (double l = 0; l < 2 * M_PI; l += ((2 * M_PI) / 12.0)) {
			Matrix3 r = rotFromRpy(0,0,l) * base_r;
			//Matrix3 r = rotFromRpy(l,0,0) * base_r;
            tc->targetObject->object->R() = Palm.R * GRCmax.R * relOCPface.R  * r;
			tc->targetObject->object->p() = Palm.p - Palm.R * dGRC_Pos_ +  Palm.R * GRCdes.p  - r * tc->targetObject->objCoM_;// + Vector3(0, 0, dist);

			Vector3 min, max;
			min = r * rot * ver[0];
			max = min;
			for (int j = 0; j < ver.size(); j++) {
				Vector3 rotated_ver = r * rot * ver[j];
				for (int k = 0; k < 3; k++) {
					if (rotated_ver[k] < min[k]) min[k] = rotated_ver[k];
					if (rotated_ver[k] > max[k]) max[k] = rotated_ver[k];
				}
			}
			Vector3 diff = max - min;
			tc->flush();

			searchGraspPostureFromSide(0,Palm,diff,link_org_q,postures);

		}
	}

	// restore finger angles
	for(int m=0;m<nFing();m++){
		for (int n = 0; n < fingers(m)->nJoints; n++) {
            fingers(m)->fing_path->joint(n)->q() = link_org_q[m][n] ;
		}
	}
	bodyItemRobot()->body()->calcForwardKinematics();

	tc->flush();

	for(int i=0;i<link_org_q.size();i++){
		delete[] link_org_q[i];
	}

}


void GraspDataGenerator::searchGraspPostureLiftUp(int cluster_num,Homogenous& Palm,const Vector3& edge,vector<double*>& link_org_q,vector<DataPosturePos>& postures){

	int yn = floor(((fabs(edge[1])-GRCdes.edge[1])/2)/0.05);

	int nContact=0;

    Vector3 org_p = tc->targetObject->object->p();

	for(int i=0;i<tc->nFing();i++) for(int j=0;j<tc->fingers(i)->nJoints;j++) if(tc->fingers(i)->contact[j]) nContact++;

	vector <Vector3> objPos(nContact), objN(nContact);
	for(int y=-yn;y<yn+1;y++){
		double y_offset = y * 0.05;

		for(double z=-(GRCdes.edge[2]-fabs(edge[2])/2);z<=fabs(edge[2])/2;z+=0.05){
			double Qmax = 0;
			Vector3 bestP;
			Matrix3 bestR;
			vector<double> bestHandAngle;
			vector<Vector3> bestContactPos;
			vector<Vector3> bestContactN;

			num_approach_points++;
			//for grid search
			for(double x=fabs(edge[0])/2;x<=fabs(edge[0])/2+GRCdes.edge[0];x+=0.05){
				num_search_point++;

				Vector3 dPos(x,y_offset,z);

                tc->targetObject->object->p() = (org_p + Palm.R * GRCmax.R * dPos);
				tc->targetObject->bodyItemObject->body()->calcForwardKinematics();

				//if(tc->isColliding()) cout  << "Collision " << tc->colPairName[0] << " " << tc->colPairName[1] << endl;

				for(int m=0;m<nFing();m++){
					for (int n = 0; n < fingers(m)->nJoints; n++) {
                        fingers(m)->fing_path->joint(n)->q() = link_org_q[m][n] ;
					}
				}
				bodyItemRobot()->body()->calcForwardKinematics();

				int cnt=0;

				objPos.resize(nContact);
				objN.resize(nContact);
				for(int i=0;i<nFing();i++){
					fingers(i)->contactSearch(cnt,2000,&objPos[0],&objN[0]);
					bodyItemRobot()->body()->calcForwardKinematics();
					tc->flush() ;
				}

				//if(cnt < nContact) continue;
				if(cnt < 1) continue;

				VectorXd wrench = VectorXd::Zero(6);

				double Qtmp, q_threshold = 0.00;
				double max_area = 0.01*0.01;
				double mu = tc->targetArmFinger->mu;
				double fmax = tc->targetArmFinger->fmax;
				double mg = tc->targetObject->objMass * 9.8;

				objPos.resize(cnt);
				objN.resize(cnt);

				Vector3 tmp_w;
                tmp_w = tc->targetObject->object->R().transpose() * Vector3(0,0,-mg);
				for(int i=0;i<3;i++) wrench(i) = tmp_w(i);
				Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench,&objPos[0],&objN[0],cnt,mu,fmax,tc->targetObject->objCoM_ );
#ifdef DEBUG_PRINT
				if(cnt == 4){
					os << "cnt:" << cnt << endl;
					os << "Q:" << Qtmp << endl;
					os << "N:" << endl;
					for(int i=0;i<cnt;i++){
						os << objN[i] << endl;
					}
					os << "mg:" << endl;
					os << tmp_w << endl;
				}
#endif
				if(Qtmp <= q_threshold) continue;

				if (Qtmp > q_threshold && Qtmp > Qmax) {
					bodyItemRobot()->body()->calcForwardKinematics();

					if(tc->isColliding()) continue;

					Qmax = Qtmp;
					bestR = Palm.R;
                    bestP = tc->targetObject->object->p();
					bestHandAngle.clear();
					//for(int k=1;k<tc->nHandLink();k++){
					//	bestHandAngle.push_back(tc->handJoint()->link(k)->q);
					//}
					for(int k=0;k<tc->nFing();k++){
						for(int m=0;m<tc->fingers(k)->fing_path->numJoints();m++){
                            bestHandAngle.push_back(tc->fingers(k)->fing_path->joint(m)->q());
						}
					}

					bestContactPos.clear();
					bestContactN.clear();
					for(int k=0;k<objPos.size();k++){
						bestContactPos.push_back(objPos[k]);
						bestContactN.push_back(objN[k]);
					}
				}

			}

			if(Qmax != 0){
				num_success++;
                Vector3 target_p = (tc->targetObject->object->R()).transpose() * (Palm.p - bestP);
                Matrix3 target_r = (tc->targetObject->object->R()).transpose() * bestR;

				DataPosturePos posture;
				posture.p = target_p;
				posture.R = target_r;
				for (int m = 0;m < bestHandAngle.size();m++){
					posture.hand_angle.push_back( bestHandAngle[m]);
				}
				posture.q = Qmax * size_coeff;
				for (int m = 0; m < bestContactPos.size();m++){
					posture.contact_pos.push_back( bestContactPos[m]);
					posture.contact_N.push_back(bestContactN[m]);
				}
				posture.cluster_num = cluster_num;

				postures.push_back(posture);
			}
		}
	}
}


void GraspDataGenerator::searchGraspPostureFromSide(int cluster_num,Homogenous& Palm,const Vector3& edge,vector<double*>& link_org_q,vector<DataPosturePos>& postures){

	int yn = floor(((fabs(edge[1]))/2)/0.20);
	//int yn = floor(((fabs(edge[1])-GRCdes.edge[1])/2)/0.05);

	int nContact=0;

    Vector3 org_p = tc->targetObject->object->p();

	for(int i=0;i<tc->nFing();i++) for(int j=0;j<tc->fingers(i)->nJoints;j++) if(tc->fingers(i)->contact[j]) nContact++;

	vector <Vector3> objPos(nContact), objN(nContact);
	for(int y=-yn;y<yn+1;y++){
		//double y_offset = y * 0.05;
		double y_offset = y * 0.2;
		//for(double z=-(GRCdes.edge[2]-fabs(edge[2])/2);z<=fabs(edge[2])/2;z+=0.05){
		for(double z=-GRCdes.edge[2];z<=0;z+=0.05){
			double Qmax = 0;
			Vector3 bestP;
			Matrix3 bestR;
			vector<double> bestHandAngle;
			vector<Vector3> bestContactPos;
			vector<Vector3> bestContactN;

			num_approach_points++;
			//for(double x=fabs(edge[0])/2;x<=fabs(edge[0])/2+GRCdes.edge[0];x+=0.05){
			for(double x=fabs(edge[0])/2;x<=fabs(edge[0])/2+GRCdes.edge[0];x+=0.2){
				num_search_point++;

				Vector3 dPos(x,y_offset,z);

                tc->targetObject->object->p() = (org_p + Palm.R * GRCmax.R * dPos);
				tc->targetObject->bodyItemObject->body()->calcForwardKinematics();

				// TODO: modify to read the number of joint to be rotated from YAML/prm file.
				//const int f0tip = fingers(0)->nJoints-1;
				//const int f1tip = fingers(1)->nJoints-1;
				const int f0tip = fingers(0)->nJoints-4;
				const int f1tip = fingers(1)->nJoints-4;
				for (double q1 = 0; q1 >= -1.57; q1-=0.25) {
					for (double q2 = 0; q2 >= -1.57; q2-=0.25) {
						for(int m=0;m<nFing();m++){
							for (int n = 0; n < fingers(m)->nJoints; n++) {
								fingers(m)->fing_path->joint(n)->q() = link_org_q[m][n] ;
							}
						}

                        fingers(1)->fing_path->joint(f1tip-1)->q() = q2;
                        fingers(0)->fing_path->joint(f0tip-1)->q() = q1;
                        fingers(1)->fing_path->joint(f1tip)->q() = -1.57-q2;
                        fingers(0)->fing_path->joint(f0tip)->q() = -1.57-q1;

						bodyItemRobot()->body()->calcForwardKinematics();
						int cnt=0;

						objPos.resize(nContact);
						objN.resize(nContact);
						for(int i=0;i<nFing();i++){
							fingers(i)->contactSearch(cnt,2000,&objPos[0],&objN[0]);
							bodyItemRobot()->body()->calcForwardKinematics();
							tc->flush() ;
						}
						if(cnt < 1) break;
						if(cnt < 2) continue;


						VectorXd wrench = VectorXd::Zero(6);

						double Qtmp, q_threshold = 0.00;
						double max_area = 0.01*0.01;
						double mu = tc->targetArmFinger->mu;
						double fmax = tc->targetArmFinger->fmax;
						double mg = tc->targetObject->objMass * 9.8;

						objPos.resize(cnt);
						objN.resize(cnt);

						Vector3 tmp_w;
                        tmp_w = tc->targetObject->object->R().transpose() * Vector3(0,0,-mg);
						for(int i=0;i<3;i++) wrench(i) = tmp_w(i);
						Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench,&objPos[0],&objN[0],cnt,mu,fmax,tc->targetObject->objCoM_ );

						if(Qtmp <= q_threshold) continue;
						if (Qtmp > q_threshold && Qtmp > Qmax) {
							bodyItemRobot()->body()->calcForwardKinematics();

							if(tc->isColliding()) continue;
#ifdef DEBUG_PRINT
							os << "cnt:" << cnt << endl;
							os << "Q:" << Qtmp << endl;
							os << "N:" << endl;
							for(int i=0;i<cnt;i++){
								os << objN[i] << endl;
							}
							os << "P:"  << endl;
							for(int i=0;i<cnt;i++){
								os << objPos[i] << endl;
							}
							os << "com:" << endl;
							os << tc->targetObject->objCoM_ << endl;
							os << "mg:" << endl;
							os << tmp_w << endl;
#endif
							Qmax = Qtmp;
							bestR = Palm.R;
                            bestP = tc->targetObject->object->p();
							bestHandAngle.clear();
							//for(int k=1;k<tc->nHandLink();k++){
							//	bestHandAngle.push_back(tc->handJoint()->link(k)->q);
							//}
							for(int k=0;k<tc->nFing();k++){
								for(int m=0;m<tc->fingers(k)->fing_path->numJoints();m++){
                                    bestHandAngle.push_back(tc->fingers(k)->fing_path->joint(m)->q());
								}
							}

							bestContactPos.clear();
							bestContactN.clear();
							for(int k=0;k<objPos.size();k++){
								bestContactPos.push_back(objPos[k]);
								bestContactN.push_back(objN[k]);
							}
						}
					}
				}
			}
			if(Qmax != 0){
				num_success++;
                Vector3 target_p = (tc->targetObject->object->R()).transpose() * (Palm.p - bestP);
                Matrix3 target_r = (tc->targetObject->object->R()).transpose() * bestR;

				DataPosturePos posture;
				posture.p = target_p;
				posture.R = target_r;
				for (int m = 0;m < bestHandAngle.size();m++){
					posture.hand_angle.push_back( bestHandAngle[m]);
				}
				posture.q = Qmax * size_coeff;
				for (int m = 0; m < bestContactPos.size();m++){
					posture.contact_pos.push_back( bestContactPos[m]);
					posture.contact_N.push_back(bestContactN[m]);
				}
				posture.cluster_num = cluster_num;

				postures.push_back(posture);
			}
		}
	}
}




void GraspDataGenerator::searchGraspPosture(int cluster_num,Homogenous& Palm,vector<double*>& link_org_q,vector<DataPosturePos>& postures){
	//double grid_interval = 0.02; // grid interval for height and width
	//depth_grid_interval = grid_interval; // grid interval for depth
	int ite = 1; // number of probe point for depth

	int xn = floor((dif(0)/2)/grid_interval);
	int yn = floor((dif(1)/2)/depth_grid_interval);
	int zn = floor((dif(2)/2)/grid_interval);

	int nContact=0;

	if(tc->arm()->palmContact) nContact++;

    Vector3 org_p = tc->targetObject->object->p();

	for(int i=0;i<tc->nFing();i++) for(int j=0;j<tc->fingers(i)->nJoints;j++) if(tc->fingers(i)->contact[j]) nContact++;

	vector <Vector3> objPos(nContact), objN(nContact);
	for(int x=-xn;x<xn+1;x++){
		double x_offset = x * grid_interval;
		for(int z=-zn;z<zn+1;z++){
			double z_offset = z * grid_interval;

			double Qmax = 0;
			Vector3 bestP;
			Matrix3 bestR;
			vector<double> bestHandAngle;
			vector<Vector3> bestContactPos;
			vector<Vector3> bestContactN;
			vector<Vector3> bestFingPos;
			vector<Vector3> bestFingN;

			vector<Vector3> FingPos;
			vector<Vector3> FingN;

			vector<double> bestArea;
			vector<double> bestEn;

			num_approach_points++;
			/* //for randam search
			for(int times = 0; times < ite; times++){
				double y_offset = 0.8*(_drand() -0.5)*dif(1);
				if(!times) y_offset = 0;
			*/
			//for grid search
			for(int y=-yn;y<yn+1;y++){
				double y_offset = y * depth_grid_interval;
				num_search_point++;

				Vector3 dPos(x_offset,y_offset,z_offset);

                tc->targetObject->object->p() = (org_p + Palm.R * GRCmax.R * dPos);
				tc->targetObject->bodyItemObject->body()->calcForwardKinematics();

				//if(tc->isColliding()) cout  << "Collision " << tc->colPairName[0] << " " << tc->colPairName[1] << endl;

				for(int m=0;m<nFing();m++){
					for (int n = 0; n < fingers(m)->nJoints; n++) {
                        fingers(m)->fing_path->joint(n)->q() = link_org_q[m][n] ;
					}
				}
				bodyItemRobot()->body()->calcForwardKinematics();

				int cnt=0;
				if(tc->arm()->palmContact){
					if( tc->arm()->closeArm(0,1000,objPos[cnt], objN[cnt]) ){
						cnt++;
					}
					else continue;
				}

				tc->flush();
				for(int i=0;i<nFing();i++){
					if(!fingers(i)->contactSearch(cnt,2000,&objPos[0],&objN[0])){
						tc->flush();
						break;
					}
					tc->flush() ;
				}

				if(cnt < nContact) continue;

#ifdef FLIP_NORMAL
				for(int i=1;i<nContact;i++){
					if(dot(objN[0],objN[i]) > 0) objN[i] = -objN[i];
				}
#endif

				VectorXd wrench = VectorXd::Zero(6);

				double Qtmp, q_threshold = 0.00;
				double max_area = 0.01*0.01;
				double mu = tc->targetArmFinger->mu;
				double fmax = tc->targetArmFinger->fmax;
				double mg = tc->targetObject->objMass * 9.8;

				std::vector<double> en_tmp;
				std::vector<double> area_temp;

				if(nFing()==0)
					Qtmp = q_threshold + 1.0;  //for suction hands

				else if(cnt < 3 || nContact<3){
					double tmp_en = 8.0/15.0*sqrt(max_area);
					vector<double> en;
					for(int i=0;i<nContact;i++){
						en.push_back(tmp_en);
					}

					Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSoftFingerSubspace(wrench,&objPos[0],&objN[0],nContact,mu,fmax,en,tc->targetObject->objCoM_ ) - mg;

					if(Qtmp > q_threshold){
						Qtmp = SoftFingerStabilityHandler::instance()->calcStability(nContact,&objPos[0],&objN[0],en_tmp,area_temp,FingPos,FingN);
					}
				}else{
					Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench,&objPos[0],&objN[0],nContact,mu,fmax,tc->targetObject->objCoM_ ) - mg;
				}

				if(Qtmp <= q_threshold) continue;

				Vector3 dummy1,dummy2,dummy3,dummy4;
				bool is_tipgrasp = true;
				for(int i=0;i<nFing();i++){
					if(!fingers(i)->fingtipGrasp()) continue;
					int tips_id = 0;
					for(int j=0;j<fingers(i)->nJoints;j++){
						if(fingers(i)->contact[j]) tips_id = j;
					}
					if(tips_id - 1 < 0) continue;
					double dsn_tip = PlanBase::calcContactPoint(fingers(i)->linkObjPair[tips_id],dummy1,dummy2,dummy3,dummy4);
					double dsn_base = PlanBase::calcContactPoint(fingers(i)->linkObjPair[tips_id-1],dummy1,dummy2,dummy3,dummy4);
					if(dsn_base <= dsn_tip) { is_tipgrasp = false; break;}
				}
				if(!is_tipgrasp) continue;

				if (Qtmp > q_threshold && Qtmp > Qmax) {
					bodyItemRobot()->body()->calcForwardKinematics();

					if(tc->isColliding()) continue;

					Qmax = Qtmp;
					bestR = Palm.R;
                    bestP = tc->targetObject->object->p();
					bestHandAngle.clear();
					for(int k=1;k<tc->nHandLink();k++){
                        bestHandAngle.push_back(tc->handJoint()->link(k)->q());
					}
					bestContactPos.clear();
					bestContactN.clear();
					for(int k=0;k<objPos.size();k++){
						bestContactPos.push_back(objPos[k]);
						bestContactN.push_back(objN[k]);
					}

					bestEn.clear();
					bestArea.clear();
					bestFingPos.clear();
					bestFingN.clear();
					for(int k=0;k<en_tmp.size();k++){
						bestArea.push_back(area_temp[k]);
						bestEn.push_back(en_tmp[k]);
						bestFingPos.push_back(FingPos[k]);
						bestFingN.push_back(FingN[k]);
					}
				}

			}

			if(Qmax != 0){
				num_success++;
                Vector3 target_p = (tc->targetObject->object->R()).transpose() * (Palm.p - bestP);
                Matrix3 target_r = (tc->targetObject->object->R()).transpose() * bestR;

				DataPosturePos posture;
				posture.p = target_p;
				posture.R = target_r;
				for (int m = 0;m < bestHandAngle.size();m++){
					posture.hand_angle.push_back( bestHandAngle[m]);
				}
				//posture.q = Qmax;
				posture.q = Qmax * size_coeff;
				for (int m = 0; m < bestContactPos.size();m++){
					posture.contact_pos.push_back( bestContactPos[m]);
					posture.contact_N.push_back(bestContactN[m]);
				}
				posture.cluster_num = cluster_num;

				for(int m=0;m<bestEn.size();m++){
					posture.en.push_back(bestEn[m]);
					posture.area.push_back(bestArea[m]);
					posture.fing_N.push_back(bestFingN[m]);
					posture.fing_pos.push_back(bestFingPos[m]);
				}

				postures.push_back(posture);
			}
		}
	}
}

bool GraspDataGenerator::searchHandScale(double start_x,double start_y,double start_z,double end,double step,SearchMode mode){
	time_t start_time,end_time;
	double sec_time;
	time (&start_time);

	tc = PlanBase::instance();

	if(tc == NULL){
		os << "implmentation error: you have to call initial()" << endl;
		return false;
	}

	if ( !tc->targetObject){
		os << "Please select Grasped Object" << endl;
		return false;
	}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	obj_shape = createObjectShape(tc->targetObject->bodyItemObject->body()->link(0)->coldetModel());
#else
	obj_shape = createObjectShape(ColdetConverter::ConvertFrom(
		tc->targetObject->bodyItemObject->body()->link(0)->collisionShape()));
#endif

	vector<Vector3> target_edges;
	vector<Vector3> target_centers;
	vector<Matrix3> target_rots;

	getBoundingboxParameters(num_cluster,vol_ratio_threshold,scale,target_edges,target_centers,target_rots);

	bool finished = false;
	double x_scale=start_x,y_scale=start_y,z_scale=start_z;

	BodyItemPtr bodyitem;
	loadScaledTrobot(x_scale,y_scale,z_scale,bodyitem);

	GraspController::instance()->initial(tc->targetObject,  tc->targetArmFinger);

	initial(tc->targetObject, tc->targetArmFinger);

	string output_filename;
	output_filename =  tc->dataFilePath() + tc->targetObject->bodyItemObject->name() + "Scaling";
	switch (mode){
	case SCALE_X:
		output_filename += "X";
		break;
	case SCALE_Y:
		output_filename += "Y";
		break;
	case SCALE_Z:
		output_filename += "Z";
		break;
	case SCALE_XYZ:
		output_filename += "XYZ";
		break;
	}
	output_filename += ".txt";

	ofstream fout( output_filename.c_str());

	bodyitem->detachFromParentItem();

	while(!finished){
		BodyItemPtr bodyitem;
		loadScaledTrobot(x_scale,y_scale,z_scale,bodyitem);

		GraspController::instance()->initial(tc->targetObject,  tc->targetArmFinger);
		initial(tc->targetObject, tc->targetArmFinger);

		for(int i=0;i<tc->targetArmFinger->nFing;i++){
			finger_shape.push_back(createObjectShape(
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				tc->fingers(i)->body->link(tc->fingers(i)->nJoints)->coldetModel()
#else
				ColdetConverter::ConvertFrom(
					tc->fingers(i)->body->link(tc->fingers(i)->nJoints)->collisionShape())
#endif
			));
		}
		vector<DataPosturePos> postures;

		searchGraspPattern(target_edges,target_centers,target_rots,postures);

		for(int i=0;i<finger_shape.size();i++){
			delete finger_shape[i];
		}
		finger_shape.clear();

		stringstream suffix;

		suffix << "x" << x_scale << "y" << y_scale << "z" << z_scale;

		string pfile = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + suffix.str() + ".txt";

		writePreplanning(pfile,postures);

#ifdef OUTPUT_RESULT
		string cfile = tc->dataFilePath() + "contact_" + tc->targetObject->bodyItemObject->name() + suffix.str() + ".txt";
		writeContactpoint(cfile,postures);
#endif

		fout << postures.size() << " " << x_scale << " " << y_scale << " " << z_scale << endl;

		switch (mode){
		case SCALE_X:
			x_scale+=step;
			finished = (x_scale > end);
			break;
		case SCALE_Y:
			y_scale+=step;
			finished = (y_scale > end);
			break;
		case SCALE_Z:
			z_scale+=step;
			finished = (z_scale > end);
			break;
		case SCALE_XYZ:
			x_scale+=step;
			y_scale+=step;
			z_scale+=step;
			finished = (x_scale > end);
			break;
		default:
			finished = true;
		}
		bodyitem->detachFromParentItem();
	}

	fout.close();

	delete obj_shape;
	time(&end_time);
	sec_time = difftime(end_time,start_time);

	os << "Searching scaling hand is finished" << endl;

#ifdef DEBUG_PRINT
	os << "time:" <<  sec_time << endl;
#endif
	return true;
}

bool GraspDataGenerator::loadScaledTrobot(double scale_x,double scale_y,double scale_z,BodyItemPtr& bodyitem){

	string script_dir = cnoid::executableTopDirectory() + "/extplugin/graspPlugin/GraspDataGen/";
	string Trobot_dir = cnoid::executableTopDirectory() + "/extplugin/hrgPlugin/RobotModels/TRobot/";

	PyObject *pName,*pModule,*pFunc,*pArgs,*pVal1,*pVal2,*pVal3,*pVal4,*pVal5;
	Py_Initialize();
	PySys_SetPath(const_cast<char*>(script_dir.c_str()));
	pName = PyString_FromString("scaling");

	pModule = PyImport_Import(pName);
	Py_DECREF(pName);

	stringstream suffix;
	suffix << "x" << scale_x << "y" << scale_y << "z" << scale_z;

	if (pModule != NULL) {
    pFunc = PyObject_GetAttrString(pModule, "generateScaledTrobot");
		if (pFunc && PyCallable_Check(pFunc)) {
			pArgs = PyTuple_New(5);
			pVal1 = PyString_FromString(Trobot_dir.c_str());
			PyTuple_SetItem(pArgs, 0, pVal1);
			pVal2 = PyFloat_FromDouble(scale_x);
			PyTuple_SetItem(pArgs, 1, pVal2);
			pVal3 = PyFloat_FromDouble(scale_y);
			PyTuple_SetItem(pArgs, 2, pVal3);
			pVal4 = PyFloat_FromDouble(scale_z);
			PyTuple_SetItem(pArgs, 3, pVal4);
			pVal5 = PyString_FromString(suffix.str().c_str());
			PyTuple_SetItem(pArgs, 4, pVal5);
		}
		PyObject_CallObject(pFunc, pArgs);
    Py_DECREF(pArgs);
		Py_DECREF(pModule);
	}else{
		Py_Finalize();
		os << "Error: generating scaled Trobot is failed" << endl;
		return false;
	}
	Py_Finalize();

	bodyitem = new BodyItem();
	if(bodyitem->load(Trobot_dir + "scaled_TRobot" + suffix.str() + ".yaml","OpenHRP-VRML-MODEL")){
		RootItem::mainInstance()->addChildItem(bodyitem);
		ItemTreeView::mainInstance()->checkItem(bodyitem,true);
	}

	if(PlanBase::instance()->SetGraspingRobot(bodyitem)){
		os << "Scaled Trobot (x:" << scale_x << " y:" << scale_y << " z:" << scale_z << ") is grasping robot" << endl;
	}else{
		os << "Error: graspping robot setting is failed" << endl;
		return false;
	}
	return true;
}

void GraspDataGenerator::seqGenerateGraspPattern(string filename){
	ifstream fin;
	fin.open(filename.c_str());

	if(!fin){
		os << "cannot read " << filename << endl;
		return;
	}

	depth_grid_interval = 0.01;
	num_cluster = 1;
	vol_ratio_threshold = 0.2;

	while(!fin.eof()){
		string line;
		getline(fin,line);


		if(line == "") continue;

		string file_path;
		stringstream ss(line);
		ss >> scale;
		ss >> file_path;

		BodyItemPtr bodyitem = new BodyItem();
		if(bodyitem->load(file_path.c_str(),"OpenHRP-VRML-MODEL")){
			RootItem::mainInstance()->addChildItem(bodyitem);
			ItemTreeView::mainInstance()->checkItem(bodyitem,true);
		}
		PlanBase::instance()->SetGraspedObject(bodyitem);

		generateGraspPattern();

		bodyitem->detachFromParentItem();
	}
}

bool GraspDataGenerator::grasp(int index){

	tc = PlanBase::instance();

	if(tc == NULL){
		os << "implmentation error: you have to call initial()" << endl;
		return false;
	}

	if ( !tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;

	}

	initial(tc->targetObject, tc->targetArmFinger);

	string db_path =  tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";

	ifstream ifs;
	ifs.open(db_path.c_str());

	int idx = 0;
	while(!ifs.eof()){
		string line;
		getline(ifs,line);
		idx++;

		if(idx!=index) continue;

		int nf = 0;
		for (int i = 0; i < tc->nFing(); i++) {
			nf += tc->fingers(i)->nJoints;
		}
		// for(int i=0;i<tc->nFing();i++){
		// 	nf += tc->fingers(0,i)->fing_path->numJoints();
		// }
		// nf = tc->nHandLink();

		Matrix3 R;
		Vector3 p;
		VectorXd finger_angle(nf);

		stringstream is(line);
		vector<double> x;
		double tmp;
		while(!is.eof()){
			is >> tmp;
			x.push_back(tmp);
		};
		if(x.size() < 13) break;
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				R(i,j) = x[i*3+j+1];
			}
		}
		for(int i=0;i<3;i++){
			p(i) = x[10+i];
		}
		int k=0;
		for(int i=13;i<x.size();i++){
			finger_angle(k++) = x[i];
		}
        tc->targetObject->object->p() = palm()->p() - palm()->R() * R.transpose() * p;
        tc->targetObject->object->R() = palm()->R() * R.transpose();

		k=0;
		for(int i=0; i<tc->nFing(); i++){
			for(int j=0;j<fingers(i)->fing_path->numJoints(); j++){
                fingers(i)->fing_path->joint(j)->q() = finger_angle(k++);
			}
		}

		tc->targetObject->bodyItemObject->body()->calcForwardKinematics();
		bodyItemRobot()->body()->calcForwardKinematics();
		tc->flush();
		break;
	}
	ifs.close();

	return true;
}

bool GraspDataGenerator::grasp(int index, bool is_move_arm) {
	tc = PlanBase::instance();

	if (tc == NULL) {
		os << "implmentation error: you have to call initial()" << endl;
		return false;
	}

	if (!tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;

	}

	initial(tc->targetObject, tc->targetArmFinger);

	GraspDatabaseManipulator gdm;
	string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	gdm.readFile(filepath);
	GraspDatabaseManipulator::GraspPoses grasp_poses = gdm.getRecords();

	if (grasp_poses.size() < index) return false;

	if (is_move_arm) {
		Vector3 obj_p = tc->targetObject->object->p();
		Matrix3 obj_R = tc->targetObject->object->R();
		Vector3 target_p = obj_p + obj_R * grasp_poses[index-1].p;
		Matrix3 target_R = obj_R * grasp_poses[index-1].R;

		if (!tc->targetArmFinger->arm->IK_arm(target_p, target_R)) return false;
	} else {
		tc->targetObject->object->p() = palm()->p() - palm()->R() * grasp_poses[index-1].R.transpose() * grasp_poses[index-1].p;
		tc->targetObject->object->R() = palm()->R() * grasp_poses[index-1].R.transpose();

		tc->targetObject->bodyItemObject->body()->calcForwardKinematics();
	}

	int k=0;
	for(int i=0; i<tc->nFing(); i++){
		for(int j=0;j<fingers(i)->fing_path->numJoints(); j++){
			fingers(i)->fing_path->joint(j)->q() = grasp_poses[index-1].finger_q(k++);
		}
	}

	// bodyItemRobot()->body()->calcForwardKinematics();
	tc->robotBody()->calcForwardKinematics();

	tc->flush();

	return true;
}

ObjectShape* GraspDataGenerator::createObjectShape(ColdetModelPtr c){
	vector<double> vertex;
	float tx, ty, tz;
	for(int k=0;k<c->getNumVertices();k++){
		c->getVertex(k, tx, ty, tz);
		vertex.push_back( tx );
		vertex.push_back( ty );
		vertex.push_back( tz );
	}
	vector<int> crd;
	int t1, t2, t3;
	for(int k=0;k<c->getNumTriangles();k++){
		c->getTriangle(k, t1, t2, t3);
		crd.push_back( t1 );
		crd.push_back( t2 );
		crd.push_back( t3 );
		crd.push_back( -1 );
	}

	return new ObjectShape(vertex,crd);
}

GraspDataAppender::GraspDataAppender() {
	cur_pos = Vector3::Zero();
	cur_rpy = Vector3::Zero();
	cur_bbid = 0;
	cur_fid = 0;
	cur_q = 0;
	gdm = new GraspDatabaseManipulator();
}

GraspDataAppender::~GraspDataAppender() {
	delete gdm;
}

GraspDataAppender* GraspDataAppender::instance(GraspDataAppender *gda) {
	static GraspDataAppender* instance = (gda) ? gda : new GraspDataAppender();
	if (gda) instance = gda;
	return instance;
}

bool GraspDataAppender::init() {
	tc = PlanBase::instance();

	if (tc == NULL) {
		os << "implmentation error: you have to call initial()" << endl;
		return false;
	}

	if (!tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}

	initial(tc->targetObject, tc->targetArmFinger);

	returnRefMotion(false, tc->targetArmFinger->prehensionList[0]);

	return true;
}

void GraspDataAppender::calcBoundingBoxCluster(int num_cluster, double vol_ratio_th, double scale) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr c = tc->targetObject->object->coldetModel();
#else
	ColdetModelPtr c = ColdetConverter::ConvertFrom(tc->targetObject->object->collisionShape());
#endif
	vector<Vector3> edges;
	vector<Vector3> centers;
	vector<Matrix3> rots;
	boundingboxes.clear();

	ObjectShape(c).getClusterParameters(num_cluster,&edges, &centers, &rots, vol_ratio_th, scale);

	for (size_t i = 0; i < edges.size(); i++) {
		Box bb;
		bb.p = centers[i];
		bb.R = rots[i];
		bb.edge = edges[i];
		boundingboxes.push_back(bb);
	}
	sortBoundingbox();
}

void GraspDataAppender::calcBoundingBoxCylinder(int num_cluster, double dist_th) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr c = tc->targetObject->object->coldetModel();
#else
	ColdetModelPtr c = ColdetConverter::ConvertFrom(tc->targetObject->object->collisionShape());
#endif
	PointCloudHandlerInterface* ph = new PointCloudHandlerInterface();
	vector<PointCloudHandlerInterface::Cylinder> cylinders;
	ph->cylinderSegmentation(c, cylinders, dist_th, num_cluster);
	boundingboxes.clear();

	Vector3 x(1,0,0);
	for (size_t i = 0; i < cylinders.size(); i++) {
		Box bb;
		bb.p = cylinders[i].pos;
		bb.R = rotFromTwoVecs(x, cylinders[i].dir);
		bb.edge = Vector3(cylinders[i].len, cylinders[i].radius*2, cylinders[i].radius*2);

		boundingboxes.push_back(bb);

		Box bb2;
		bb2.p = cylinders[i].pos;
		bb2.R = rotFromTwoVecs(x, cylinders[i].dir)  * rotFromRpy(M_PI/4.0, 0, 0);
		bb2.edge = Vector3(cylinders[i].len, cylinders[i].radius*2, cylinders[i].radius*2);

		boundingboxes.push_back(bb2);
	}
	sortBoundingbox();
}

void GraspDataAppender::moveInit() {
	cur_pos = Vector3::Zero();
	cur_rpy = Vector3::Zero();
	cur_bbid = 0;
	cur_fid = 0;

	string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	gdm->readFile(filepath);

	move(Vector3::Zero(), Vector3::Zero());
}

void GraspDataAppender::move(const Vector3& diff_pos, const Vector3& diff_rpy) {
	cur_pos += diff_pos;
	cur_rpy += diff_rpy;

	Matrix3 R = rotFromRpy(cur_rpy);
	Matrix3 bbR = calcObjRot(Matrix3::Identity(), cur_fid / 4);
	Matrix3 faceR = rotFromRpy(Vector3(0, (M_PI / 2.0) * (cur_fid % 4), 0));

	Vector3 offset = palm()->R() * GRCdes.p;
	Vector3 dp =  palm()->R() * GRCmax.R * cur_pos;
	Vector3 offset_bb = -palm()->R() * GRCmax.R * R * faceR * bbR * boundingboxes[cur_bbid].R.transpose() * boundingboxes[cur_bbid].p;
	tc->targetObject->object->p() = palm()->p() + offset + dp + offset_bb;
	tc->targetObject->object->R() = palm()->R() * GRCmax.R * R * faceR * bbR * boundingboxes[cur_bbid].R.transpose();
	tc->flush();
	showTargetBB(cur_bbid);
	cur_q = calcStability();

	if (cur_state == S_SUCTION) {
		Vector3 com = tc->targetObject->objCoM_;
		Vector3 bb_center = boundingboxes[cur_bbid].p;
		Vector3 bb_offset = Vector3::Zero();
		int n = cur_fid / 4;
		if (n == 0) {bb_offset(1) = -boundingboxes[cur_bbid].edge[1] / 2.0;}
		else if (n == 1) {bb_offset(2) = -boundingboxes[cur_bbid].edge[2] / 2.0;}
		else if (n == 2) {bb_offset(0) = -boundingboxes[cur_bbid].edge[0] / 2.0;}
		else if (n == 3) {bb_offset(0) = boundingboxes[cur_bbid].edge[0] / 2.0;}
		else if (n == 4) {bb_offset(1) = boundingboxes[cur_bbid].edge[1] / 2.0;}
		else if (n == 5) {bb_offset(2) = boundingboxes[cur_bbid].edge[2] / 2.0;}

		cur_q =  1.0 / norm2(com - (boundingboxes[cur_bbid].R * bb_offset + bb_center));
	}
}

void GraspDataAppender::nextFace() {
	cur_fid++;
	if (cur_fid >= 24) {
		cur_fid = 0;
	}

	Vector3 pos = Vector3::Zero();
	Vector3 rpy = Vector3::Zero();
	cur_pos = pos;
	cur_rpy = rpy;
	move(pos, rpy);

	showTargetBB(cur_bbid);
}

void GraspDataAppender::nextBB() {
	cur_bbid++;
	cur_fid = 0;
	if (cur_bbid >= boundingboxes.size()) {
		cur_bbid = 0;
	}

	Vector3 pos = Vector3::Zero();
	Vector3 rpy = Vector3::Zero();
	cur_pos = pos;
	cur_rpy = rpy;
	move(pos, rpy);

	showTargetBB(cur_bbid);
}

void GraspDataAppender::append() {
	if (cur_q < 0) {
		os << "Unstable grasp." << endl;
		return;
	}

	GraspDatabaseManipulator::GraspPosture pose;
	pose.q = cur_q;
	pose.p = (tc->targetObject->object->R()).transpose() * (palm()->p() - tc->targetObject->object->p());
	pose.R = (tc->targetObject->object->R()).transpose() * palm()->R();

	int fing_num_joint = 0;
	for (int i = 0; i < tc->nFing(); i++) {
		fing_num_joint += tc->fingers(i)->nJoints;
	}
	VectorXd fing_angle(fing_num_joint);
	int k = 0;
	for (int i = 0; i < tc->nFing(); i++) {
		for (int j = 0; j < tc->fingers(i)->nJoints; j++) {
			fing_angle[k++] = tc->fingers(i)->joint(j)->q();
		}
	}

	pose.finger_q = fing_angle;

	gdm->addRecord(pose);
	gdm->sort();
	string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	gdm->writeFile(filepath);

	os << "append the pose to the graspDB" << endl;
}

void GraspDataAppender::sortBoundingbox() {
	Vector3 com = tc->targetObject->objCoM_;

	for (size_t i = 0; i < boundingboxes.size()-1; i++) {
		for (size_t j = i + 1; j < boundingboxes.size(); j++) {
			double diff1 = norm2(com - boundingboxes[i].p);
			double diff2 = norm2(com - boundingboxes[j].p);
			if (diff1 > diff2) {
				Box tmp = boundingboxes[i];
				boundingboxes[i] = boundingboxes[j];
				boundingboxes[j] = tmp;
			}
		}
	}
}

void GraspDataAppender::showTargetBB(int bbid) {
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	Vector3 color = Vector3(1.0, 0, 0);
	Matrix3 objR = tc->targetObject->object->R();
  Vector3 p = tc->targetObject->object->p() + objR * boundingboxes[bbid].p;
	draw->boxes.push_back(Boxes(p, objR * boundingboxes[bbid].R, boundingboxes[bbid].edge, color, 0.2));

	draw->displayShapes();
}

double GraspDataAppender::calcStability() {
	cur_state = S_GRASP;
	is_collide = false;

	// for suction hand
	if (nFing() == 0) {
		cur_state = S_SUCTION;
		return 1.0;
	}

	readMotionFile(refMotion.c_str());

	int nContact=0;

	if (tc->arm()->palmContact) nContact++;
	for (int i = 0; i < tc->nFing(); i++) for (int j = 0; j < tc->fingers(i)->nJoints; j++) if (tc->fingers(i)->contact[j]) nContact++;

	vector<Vector3> objPos(nContact), objN(nContact);
	vector<Vector3> FingPos, FingN;

	int cnt=0;
	if (tc->arm()->palmContact) {
		if(tc->arm()->closeArm(0, 1000, objPos[cnt], objN[cnt])) {
			cnt++;
		}
		else {
			cur_state = S_NOTCONTACT;
			return -1;
		}
	}
	for (int i = 0; i < nFing(); i++) {
		if (!fingers(i)->contactSearch(cnt, 4000, &objPos[0], &objN[0])) {
			tc->flush();
			break;
		}
		tc->flush();
	}
	if (cnt < nContact) {
		cur_state = S_NOTCONTACT;
		return -1;
	}

	VectorXd wrench = VectorXd::Zero(6);

	double Qtmp, q_threshold = 0.00;
	double mu = tc->targetArmFinger->mu;
	double fmax = tc->targetArmFinger->fmax;
	double mg = tc->targetObject->objMass * 9.8;

	vector<double> en_tmp;
	vector<double> area_temp;

	if (cnt < 3 || nContact<3) {
		Qtmp = SoftFingerStabilityHandler::instance()->calcStability(nContact, &objPos[0], &objN[0], en_tmp, area_temp, FingPos, FingN);
	}else{
		Qtmp = ForceClosureTest::instance()->forceClosureTestEllipsoidSubspace(wrench, &objPos[0], &objN[0], nContact, mu, fmax, tc->targetObject->objCoM_) - mg;
	}

	if (Qtmp <= q_threshold) return Qtmp;

	Vector3 dummy1, dummy2, dummy3, dummy4;
	bool is_tipgrasp = true;
	for (int i = 0; i < nFing(); i++) {
		if (!fingers(i)->fingtipGrasp()) continue;
		int tips_id = 0;
		for (int j = 0; j < fingers(i)->nJoints; j++) {
			if (fingers(i)->contact[j]) tips_id = j;
		}
		if (tips_id - 1 < 0) continue;
		double dsn_tip = PlanBase::calcContactPoint(fingers(i)->linkObjPair[tips_id], dummy1, dummy2, dummy3, dummy4);
		double dsn_base = PlanBase::calcContactPoint(fingers(i)->linkObjPair[tips_id-1], dummy1, dummy2, dummy3, dummy4);
		if (dsn_base <= dsn_tip) {is_tipgrasp = false; break;}
	}
	if (!is_tipgrasp) cur_state = S_NOTTIPCONTACT;

	is_collide = tc->isColliding();
	return Qtmp;
}

GraspDataUpdater::GraspDataUpdater() {
}

GraspDataUpdater::~GraspDataUpdater() {
}

void GraspDataUpdater::moveInit(int gid) {
	ori_pos_ = tc->targetObject->object->p();
	ori_R_ = tc->targetObject->object->R();
	gid_ = gid;

	GraspDataAppender::moveInit();
}

void GraspDataUpdater::move(const Vector3& diff_pos, const Vector3& diff_rpy) {
	cur_pos += diff_pos;
	cur_rpy += diff_rpy;

	Matrix3 wRgrc = palm()->R() * GRCmax.R;
	Matrix3 R = wRgrc * rotFromRpy(cur_rpy) * wRgrc.transpose();

	Vector3 dp =  palm()->R() * GRCmax.R * cur_pos;
	tc->targetObject->object->p() = ori_pos_ + dp;
	tc->targetObject->object->R() = R * ori_R_;
	tc->flush();
	cur_q = calcStability();

	if (cur_state == S_SUCTION) {
		cur_q = gdm->getRecords()[gid_].q;
	}
}

void GraspDataUpdater::update() {
	GraspDatabaseManipulator::GraspPoses records = gdm->getRecords();
	GraspDatabaseManipulator::GraspPosture& pose = records[gid_];
	pose.q = cur_q;
	pose.p = (tc->targetObject->object->R()).transpose() * (palm()->p() - tc->targetObject->object->p());
	pose.R = (tc->targetObject->object->R()).transpose() * palm()->R();

	int fing_num_joint = 0;
	for (int i = 0; i < tc->nFing(); i++) {
		fing_num_joint += tc->fingers(i)->nJoints;
	}
	VectorXd fing_angle(fing_num_joint);
	int k = 0;
	for (int i = 0; i < tc->nFing(); i++) {
		for (int j = 0; j < tc->fingers(i)->nJoints; j++) {
			fing_angle[k++] = tc->fingers(i)->joint(j)->q();
		}
	}

	pose.finger_q = fing_angle;

	gdm->setRecords(records);
	string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
	gdm->writeFile(filepath);
}

void GraspDataUpdater::updateDataBase() {
	if (nFing() != 0) {
		gdm->sort();
		string filepath = tc->dataFilePath() + "preplanning_" + tc->targetObject->bodyItemObject->name() + ".txt";
		gdm->writeFile(filepath);
	}

}

GraspDataUpdater* GraspDataUpdater::instance() {
	static GraspDataUpdater* instance = new GraspDataUpdater();
	return instance;
}

GraspControllerWithInterface* GraspControllerWithInterface::instance(GraspControllerWithInterface *gc) {
	static GraspControllerWithInterface* instance = (gc) ? gc : new GraspControllerWithInterface();
	if(gc) instance = gc;
	return instance;
}

bool GraspControllerWithInterface::doGrasping(const vector<vector<double> >& fing_angles) {
	tc = PlanBase::instance();
	tc->setGraspingStateMainArm(PlanBase::UNDER_GRASPING);

	if ( !tc->targetObject || !tc->targetArmFinger) {
		os << "Please select Grasped Object and Grasping Robot" << endl;
		return false;
	}

	initial(tc->targetObject, tc->targetArmFinger);

	os << GRCmax.edge << endl;
	if (getPalmPosture() > 1.e10) {
		os << "cannot find palm position" << endl;
		return false;
	};

	tc->arm()->IK_arm(palmPos, palmRot);

	for (size_t i = 0; i < fing_angles.size(); i++) {
		if (i >= (size_t)tc->nFing()) break;
		for (size_t j = 0; j < fing_angles[i].size(); j++) {
			if (j >= tc->fingers(i)->nJoints) break;
			tc->fingers(i)->joint(j)->q() = fing_angles[i][j];
		}
	}

	bodyItemRobot()->body()->calcForwardKinematics();
  palmPos = palm()->p();
  palmRot = palm()->R();

	if (sampleFinalPos()){
		os << "Success: Grasp Plannig" <<endl;
		return true;
	}
	else{
		os << "Fail: Grasp Plannig" << endl;
		return false;
	}

	return true;
}

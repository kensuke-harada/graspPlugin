// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <cnoid/MessageView>

#include "../Grasp/ForceClosureTest.h"
#include "SoftFingerStabilityHandler.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

using namespace std;
using namespace cnoid;
using namespace grasp;

void ContactRegionForDraw::calcForDraw(const Vector3& p_obj, const Vector3& p_fing, const Vector3& n_fing, const vector<vector<Vector3> >& boundary_fing, double fmax) {
	double h = calc(p_obj, p_fing, n_fing, boundary_fing, fmax);
	//cout << "h" << h << endl;
	//cout << "f" << force << endl;
	//cout << "area" << sum_area << endl;
	//cout << "bound_size" << boundary_fing.size() << endl;
	contact_tlist.clear();
	generateTargetTriangleList(h, contact_tlist);
}

void ContactRegionForDraw::getTrianglesSplittedNoOverlap(vector<const Triangle*>& triangles) const {
	triangles.clear();

	for (ContactTriangleVecConstIte vi = splitted_nocandidate_triangles.begin(); vi != splitted_nocandidate_triangles.end(); ++vi) {
		triangles.push_back((*vi)->tri);
	}
}

SoftFingerStabilityHandler::SoftFingerStabilityHandler() 
{
	obj_shape = NULL;
	initialize();
}

SoftFingerStabilityHandler::~SoftFingerStabilityHandler()
{
	if(obj_shape != NULL) delete obj_shape;
	for(int i=0;i<finger_shape.size();i++){
		delete finger_shape[i];
	}
	finger_shape.clear();
}

SoftFingerStabilityHandler* SoftFingerStabilityHandler::instance(SoftFingerStabilityHandler *sh) {
	static SoftFingerStabilityHandler* instance = (sh) ? sh : new SoftFingerStabilityHandler();
	if(sh) instance = sh;
	return instance;
}

void SoftFingerStabilityHandler::initialize()
{
	tc = PlanBase::instance();

	if(obj_shape != NULL) delete obj_shape;
	for(int i=0;i<finger_shape.size();i++){
		delete finger_shape[i];
	}
	finger_shape.clear();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	obj_shape = createObjectShape(tc->targetObject->bodyItemObject->body()->link(0)->coldetModel());
	for(int i=0;i<tc->targetArmFinger->nFing;i++){
		finger_shape.push_back(createObjectShape(tc->fingers(i)->fing_path->joint(tc->fingers(i)->nJoints-1)->coldetModel()));
	}
#else
	obj_shape = createObjectShape(tc->targetObject->bodyItemObject->body()->link(0)->collisionShape());
	for(int i=0;i<tc->targetArmFinger->nFing;i++){
		finger_shape.push_back(createObjectShape(tc->fingers(i)->fing_path->joint(tc->fingers(i)->nJoints-1)->collisionShape()));
	}
#endif
}

double SoftFingerStabilityHandler::calcStability(int nContact,Vector3 objPos[], Vector3 objN[])
{
	vector<double> dummy1,dummy2;
	vector<Vector3> dummy3,dummy4;
	return calcStability(nContact,&objPos[0],&objN[0],dummy1,dummy2,dummy3,dummy4);
}

double SoftFingerStabilityHandler::calcStability(int nContact,Vector3 objPos[], Vector3 objN[],
	vector<double>& en,vector<double>& area,vector<Vector3>& fingPos,vector<Vector3>& fingN)
{
	VectorXd wrench = VectorXd::Zero(6);
	double mu = tc->targetArmFinger->mu;
	double fmax = tc->targetArmFinger->fmax;
	double hmax = tc->targetArmFinger->hmax;
	double mg = tc->targetObject->objMass * 9.8;
	en.clear();
	area.clear();
	fingPos.clear();
	fingN.clear();
	Vector3* N = new Vector3[nContact];

	EnCalculator crc;
	crc.setObject(obj_shape);
	crc.setHmax(hmax);

	for(unsigned int i = 0; i < nContact; i++) {
		ObjectShape finger_obj = ObjectShape(finger_shape[i]);
		Vector3 Po, Pf, No, Nf;
		int oTid, fTid;
		Link *objLink, *fingLink;

		objLink = tc->targetObject->object;
		fingLink = tc->fingers(i)->fing_path->joint(tc->fingers(i)->nJoints-1);
		calcContactPoint(tc->fingers(i)->linkObjPair[tc->fingers(i)->nJoints-1], Po, Pf, No, Nf, oTid, fTid);

		getFingerNormal(finger_obj, Pf, Po, fingLink, objLink, Nf, fTid);

        Vector3 Pf_o = trans(Matrix3(objLink->R())) * ((fingLink->R() * Pf) + fingLink->p() - objLink->p());
        Vector3 nf_o = trans(Matrix3(objLink->R())) * fingLink->R() * Nf;

		vector<vector<Vector3> > boundaries = finger_obj.getContactRegionBoundaryPoints(Pf,Nf,fTid);
		for(int j=0;j<boundaries.size();j++){
			for(int k=0;k<boundaries[j].size();k++){
                boundaries[j][k] = trans(Matrix3(objLink->R())) * ((fingLink->R() * boundaries[j][k]) + fingLink->p() - objLink->p());
			}
		}

		crc.calc(Po, Pf_o, nf_o, boundaries, fmax);
		double en_val = crc.getEn();
		double area_val = crc.getArea();

		if (area_val < DBL_MIN) {
			return -1000;
		}

		en.push_back(en_val);
		area.push_back(area_val);
		N[i] = crc.getNormal();
		fingPos.push_back(Pf);
		fingN.push_back(Nf);
	}

	double q =  ForceClosureTest::instance()->forceClosureTestEllipsoidSoftFingerSubspace(wrench,&objPos[0],&N[0],nContact,mu,fmax,en,tc->targetObject->objCoM_ ) - mg;

	delete [] N;

	return q;
}

void SoftFingerStabilityHandler::showContactCluster(int index,bool is_show_obj,bool is_show_fing)
{
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();
	int tidx = 0;
	if (is_show_obj) showContactClusterObject(tidx);
	if (is_show_fing) showContactClusterFinger(tidx);
	draw->displayTriangles();
}

void SoftFingerStabilityHandler::showContactClusterObject(int& tidx) const {
	ContactRegionForDraw crc;
	crc.setObject(obj_shape);
	crc.setHmax(tc->targetArmFinger->hmax);
	crc.setHDivisionNum(100);
	OverlapRegionCalculator orc;
	map<int, bool> isOverlapRegion;

	DrawUtility* draw = DrawUtility::instance();
	vector<int> indeces_contact,indeces_nocontact;

	for (unsigned int i = 0; i < tc->nFing(); i++) {
		ObjectShape finger_obj = ObjectShape(finger_shape[i]);
		Vector3 Po, Pf, No, Nf;
		int oTid, fTid;
		Link *objLink, *fingLink;

		objLink = tc->targetObject->object;
		fingLink = tc->fingers(i)->fing_path->joint(tc->fingers(i)->nJoints-1);
		calcContactPoint(tc->fingers(i)->linkObjPair[tc->fingers(i)->nJoints-1], Po, Pf, No, Nf, oTid, fTid);

		getFingerNormal(finger_obj, Pf, Po, fingLink, objLink, Nf, fTid);

		Vector3 Pf_o = trans(Matrix3(objLink->R())) * ((fingLink->R() * Pf) + fingLink->p() - objLink->p());
		Vector3 nf_o = trans(Matrix3(objLink->R())) * fingLink->R() * Nf;
		
		vector<vector<Vector3> > boundaries = finger_obj.getContactRegionBoundaryPoints(Pf, Nf, fTid);
		for (unsigned int j = 0; j < boundaries.size(); j++) {
			for (unsigned int k = 0; k < boundaries[j].size(); k++) {
				boundaries[j][k] = trans(Matrix3(objLink->R())) * ((fingLink->R() * boundaries[j][k]) + fingLink->p() - objLink->p());
			}
		}

		crc.calcForDraw(Po, Pf_o, nf_o, boundaries,tc->targetArmFinger->fmax);


		vector<const ContactTriangle*> cluster_triangles;
		vector<const Triangle*> not_overlap_triangles;

		for (unsigned int j = 0; j < crc.contact_tlist.size(); j++) {
			const Triangle* tri = crc.contact_tlist[j]->tri;
			bool is_overlap = false;
			for (unsigned int k = 0; k < boundaries.size(); k++) {
				orc.calc(tri, boundaries[k], tri->ver[0]->pos,tri->normal, nf_o);
				is_overlap |= orc.isOverlap();
			}
			if (is_overlap) {
				isOverlapRegion[tri->id] = true;
			} else {
				if (crc.contact_tlist[j]->splitted) {
					for(int k=0;k<3;k++){
                        draw->points.push_back(tc->targetObject->object->R()*(crc.contact_tlist[j]->tri->ver[k]->pos)+tc->targetObject->object->p());
						indeces_nocontact.push_back(tidx++);
					}
				}
			}
		}

		crc.getTrianglesSplittedNoOverlap(not_overlap_triangles);

		for(int j=0;j<not_overlap_triangles.size();j++){
			if (isOverlapRegion.count(not_overlap_triangles[j]->id) > 0){
				for(int k=0;k<3;k++){
                    draw->points.push_back(tc->targetObject->object->R()*(not_overlap_triangles[j]->ver[k]->pos)+tc->targetObject->object->p());
					indeces_nocontact.push_back(tidx++);
				}
			}
		}
	}


	OverlapRegionCalculator::BoundaryVec in_boundary, out_boundary;
	in_boundary	= orc.getOvelapBoundary();
	out_boundary = orc.getNotOverlapBoundary();
	
	
	Triangle* tlist = obj_shape->triangles;
	for(int i=0;i<obj_shape->nTriangles;i++){
		if(isOverlapRegion.count(tlist[i].id) > 0) continue;
		for(int j=0;j<3;j++){
            draw->points.push_back(tc->targetObject->object->R()*Vector3(tlist[i].ver[j]->pos[0],tlist[i].ver[j]->pos[1],tlist[i].ver[j]->pos[2])+tc->targetObject->object->p());
			indeces_nocontact.push_back(tidx++);
		}
	}

	for(int i=0;i<in_boundary.size();i++){
		vector<vector<Vector3> > triangles;
		OverlapRegionCalculator::dividePolygon(in_boundary[i],triangles);
		for(int j=0;j<triangles.size();j++){
			for(int k=0;k<3;k++){
                draw->points.push_back(tc->targetObject->object->R()*triangles[j][k]+tc->targetObject->object->p());
				indeces_contact.push_back(tidx++);
			}
		}
	}

	for(int i=0;i<out_boundary.size();i++){
		vector<vector<Vector3> > triangles;
		OverlapRegionCalculator::dividePolygon(out_boundary[i],triangles);
		for(int j=0;j<triangles.size();j++){
			for(int k=0;k<3;k++){
				draw->points.push_back(tc->targetObject->object->R()*triangles[j][k]+tc->targetObject->object->p());
				indeces_nocontact.push_back(tidx++);
			}
		}
	}

	draw->triangles.push_back(indeces_contact);
	draw->colors.push_back(Vector3(0.8,0,0));
	draw->triangles.push_back(indeces_nocontact);
	draw->colors.push_back(Vector3(0.8,0.8,0.8));
}

void SoftFingerStabilityHandler::showContactClusterFinger(int& tidx) const {
	ContactRegionForDraw crc;
	crc.setObject(obj_shape);
	crc.setHmax(tc->targetArmFinger->hmax);
	crc.setHDivisionNum(100);
	map<int, bool> isOverlapRegion;

	for (unsigned int i = 0; i < tc->nFing(); i++) {
		ObjectShape finger_obj = ObjectShape(finger_shape[i]);
		Vector3 Po, Pf, No, Nf;
		int oTid, fTid;
		Link *objLink, *fingLink;

		objLink = tc->targetObject->object;
		fingLink = tc->fingers(i)->fing_path->joint(tc->fingers(i)->nJoints-1);
		calcContactPoint(tc->fingers(i)->linkObjPair[tc->fingers(i)->nJoints-1], Po, Pf, No, Nf, oTid, fTid);

		getFingerNormal(finger_obj, Pf, Po, fingLink, objLink, Nf, fTid);

		Vector3 Pf_o = trans(Matrix3(objLink->R())) * ((fingLink->R() * Pf) + fingLink->p() - objLink->p());
		Vector3 nf_o = trans(Matrix3(objLink->R())) * fingLink->R() * Nf;

		vector<vector<Vector3> > boundaries = finger_obj.getContactRegionBoundaryPoints(Pf, Nf, fTid);
		for (unsigned int j = 0; j < boundaries.size(); j++) {
			for (unsigned int k = 0; k < boundaries[j].size(); k++) {
				boundaries[j][k] = trans(Matrix3(objLink->R())) * ((fingLink->R() * boundaries[j][k]) + fingLink->p() - objLink->p());
			}
		}

		crc.calcForDraw(Po, Pf_o, nf_o, boundaries, tc->targetArmFinger->fmax);
		OverlapRegionCalculator orc;
		
		for (int k = 0; k < crc.contact_tlist.size(); k++) {
			vector<Vector3> boundary_tri;
			for (int l = 0; l < 3; l++) {
				boundary_tri.push_back(trans(Matrix3(fingLink->R())) * ((objLink->R() * crc.contact_tlist[k]->tri->ver[l]->pos) + objLink->p() - fingLink->p()));
			}
			for (int j = 0; j < finger_obj.clusters[fTid].triangleList.size(); j++) {
				Triangle* tri = finger_obj.clusters[fTid].triangleList[j];
				orc.calc(tri, boundary_tri, tri->ver[0]->pos + (unit(tri->normal) * 0.00001), tri->normal, Nf);
			}
		}

		OverlapRegionCalculator::BoundaryVec contact_boundary;
		contact_boundary	= orc.getOvelapBoundary();
	

		DrawUtility* draw = DrawUtility::instance();

		vector<int> indeces_contact,indeces_nocontact;
		for(int l=0;l<contact_boundary.size();l++){
			vector<vector<Vector3> > triangles;
			OverlapRegionCalculator::dividePolygon(contact_boundary[l],triangles);
			for(int j=0;j<triangles.size();j++){
				for(int k=0;k<3;k++){
					draw->points.push_back(fingLink->R()*triangles[j][k]+fingLink->p());
					indeces_contact.push_back(tidx++);
				}
			}
		}

		Triangle* tlist = finger_shape[i]->triangles;
	for(int l=0;l<finger_shape[i]->nTriangles;l++){
		for(int j=0;j<3;j++){
			draw->points.push_back(fingLink->R()*Vector3(tlist[l].ver[j]->pos[0],tlist[l].ver[j]->pos[1],tlist[l].ver[j]->pos[2])+fingLink->p());
			indeces_nocontact.push_back(tidx++);
		}
	}


	draw->triangles.push_back(indeces_contact);
	draw->colors.push_back(Vector3(0.8,0,0));
	draw->triangles.push_back(indeces_nocontact);
	draw->colors.push_back(Vector3(0.8,0.8,0.8));
	}
}

void SoftFingerStabilityHandler::displayClusterData() {
	int nContact = tc->nFing();
	vector<Vector3> objPos, objN, FingPos, FingN;
	vector<double> en, area;

	for (int i = 0; i < nContact; i++) {
		Vector3 Po, Pf, No, Nf;
		int oTid, fTid;
		calcContactPoint(tc->fingers(i)->linkObjPair[tc->fingers(i)->nJoints-1], Po, Pf, No, Nf, oTid, fTid);
		objPos.push_back(Po);
		objN.push_back(No);
		FingPos.push_back(Pf);
		FingN.push_back(Nf);
	}
	double Qtmp = calcStability(nContact, &objPos[0], &objN[0], en, area, FingPos, FingN);

	MessageView::mainInstance()->cout() << "ForceClosure: " << Qtmp << endl;
	for (int i = 0; i < nContact; i++) {
		MessageView::mainInstance()->cout() << " Cluster"<< i << ": Area:" << area[i] << " En:" << en[i] << endl;
	}
}

/**
* obtain a normal vector of the finger.
* @param[in] finger ObjectShape of the finger
* @param[in] p_fing contact point on the finger
* @param[in] p_obj  contact point on the object
* @param[in] link_fing  link of the finger
* @param[in] link_obj link of the object
* @param[in,out] n_fing normal vector of the finger
* @param[in,out] tid id of triangle at contact point of the finger
*/
void SoftFingerStabilityHandler::getFingerNormal(const ObjectShape& finger, const Vector3& p_fing, const Vector3& p_obj, const Link* link_fing, const Link* link_obj, Vector3& n_fing, int& tid) const{
	vector<int> tids;
	bool is_corner_contact = finger.isCornerPoint(p_fing, tids);

	Vector3 p_fing_o = trans(Matrix3(link_obj->R())) * ((link_fing->R() * p_fing) + link_fing->p() - link_obj->p());
	Vector3 n_fing_o = trans(Matrix3(link_obj->R())) * link_fing->R() * n_fing;
	Vector3 diff = unit(p_obj - p_fing_o);

	if (is_corner_contact) {
		double max_d = -DBL_MAX;
		for (int i = 0; i < tids.size(); i++) {
			Vector3 tmp_n = trans(Matrix3(link_obj->R())) * link_fing->R() * finger.triangles[tids[i]].normal;
			double d = fabs(dot(diff, tmp_n));
			if (d > max_d) {
				max_d = d;
				n_fing = finger.triangles[tids[i]].normal;
				tid = tids[i];
			}
		}
	}
}

void SoftFingerStabilityHandler::calcContactPoint(ColdetLinkPairPtr cPair, Vector3 &Po, Vector3 &Pf, Vector3 &objN, Vector3 &fingN, int &objTid, int &fingTid) const{
	double p1[3] = {0}, p2[3] = {0};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    cPair->model(0)->setPosition(cPair->link(0)->R(), cPair->link(0)->p());
    cPair->model(1)->setPosition(cPair->link(1)->R(), cPair->link(1)->p());
#else
	cPair->model(0)->setPosition(cPair->link(0)->T());
	cPair->model(1)->setPosition(cPair->link(1)->T());
#endif

	cPair->computeDistance(fingTid, &p1[0], objTid, &p2[0]);

	Link* links[2];
	links[0] = cPair->link(0);
	links[1] = cPair->link(1);

	int v[2][3];
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	links[0]->coldetModel()->getTriangle(fingTid, v[0][0], v[0][1], v[0][2]);
	links[1]->coldetModel()->getTriangle(objTid, v[1][0], v[1][1], v[1][2]);
#else
	cPair->model(0)->getTriangle(fingTid, v[0][0], v[0][1], v[0][2]);
	cPair->model(1)->getTriangle(objTid, v[1][0], v[1][1], v[1][2]);
#endif

	Pf = Vector3(p1[0], p1[1], p1[2]);
	Po = Vector3(p2[0], p2[1], p2[2]);

    Pf = trans(Matrix3(cPair->link(0)->R())) * (Pf - cPair->link(0)->p());
    Po = trans(Matrix3(cPair->link(1)->R())) * (Po - cPair->link(1)->p());

	float p[3];
	Vector3 n[3];

	for (int i = 0; i < 3; i++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		links[0]->coldetModel()->getVertex(v[0][i], p[0], p[1], p[2]);
#else
		cPair->model(0)->getVertex(v[0][i], p[0], p[1], p[2]);
#endif
		n[i] = Vector3 (p[0], p[1], p[2]);
	}

	Vector3 objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	fingN = objN2 / norm2(objN2);

	for (int i = 0; i < 3; i++) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		links[1]->coldetModel()->getVertex(v[1][i], p[0], p[1], p[2]);
#else
		cPair->model(1)->getVertex(v[1][i], p[0], p[1], p[2]);
#endif
		n[i] = Vector3 (p[0], p[1], p[2]);
	}

	objN2 = cross(Vector3(n[1] - n[0]), Vector3(n[2] - n[0]));
	objN = objN2 / norm2(objN2);
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
ObjectShape* SoftFingerStabilityHandler::createObjectShape(ColdetModelPtr c){
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
#else
ObjectShape* SoftFingerStabilityHandler::createObjectShape(SgNode* c){

	cnoid::SgMeshPtr mesh = ColdetConverter::ExtractMesh(c);
    cnoid::SgVertexArrayPtr vertices = mesh->vertices();

	vector<double> vertex;
	for(int k = 0; k < vertices->size(); k++){
		cnoid::Vector3f vec = vertices->at(k);
		vertex.push_back( vec[0] );
		vertex.push_back( vec[1] );
		vertex.push_back( vec[2] );
	}

    cnoid::SgIndexArray indices = mesh->triangleVertices();
	vector<int> crd;
	for(int k = 0; k < mesh->numTriangles(); k++){
		crd.push_back( indices[k*3] );
		crd.push_back( indices[k*3+1] );
		crd.push_back( indices[k*3+2] );
		crd.push_back( -1 );
	}
	return new ObjectShape(vertex,crd);
}
#endif


/**
 * @file   SweptVolume.cpp
 * @author Akira Ohchi
*/

#include "SweptVolume.h"

#include <limits>
#include <set>
#include <map>
#include <stack>
#include <vector>
#include <algorithm>
#include <fstream>
#include <cmath>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <cnoid/ColdetModel>

#include "PenaltyFunction.h"
#include "AbstractClassifier.h"

#include "../Grasp/Finger.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/ConvexAnalysis.h"
#include "../Grasp/DrawUtility.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"
#include "../Grasp/UtilFunction.h"
#include "../Grasp/VectorMath.h"

#define THREAD
//#define CLASSIFIER_UPDATEMODE

Polyhedron::Polyhedron() :
	is_closed_(true),
	is_convex_(false),
	aabb_min_(cnoid::Vector3::Zero()),
	aabb_max_(cnoid::Vector3::Zero()) {
}

Polyhedron::~Polyhedron() {
	clear();
}

void Polyhedron::addVertex(const cnoid::Vector3& v) {
	vertices_.push_back(v);
}

void Polyhedron::addTrianglePtr(Triangle* tri) {
	triangles_.push_back(tri);
}

void Polyhedron::setClosed(bool closed) {
	is_closed_ = closed;
}

Polyhedron::VertexVec& Polyhedron::vertices() {
	return vertices_;
}

const Polyhedron::VertexVec& Polyhedron::vertices() const {
	return vertices_;
}

cnoid::Vector3& Polyhedron::vertex(int i) {
	return vertices_[i];
}

const cnoid::Vector3& Polyhedron::vertex(int i) const {
	return vertices_[i];
}

Polyhedron::TrianglePtrVec& Polyhedron::triangles() {
	return triangles_;
}

const Polyhedron::TrianglePtrVec& Polyhedron::triangles() const {
	return triangles_;
}

Triangle* Polyhedron::triangle(int i) {
	return triangles_[i];
}

const Triangle* Polyhedron::triangle(int i) const {
	return triangles_[i];
}

cnoid::ColdetModelPtr Polyhedron::coldet_model() {
	return coldet_model_;
}

bool Polyhedron::isClosed() const {
	return is_closed_;
}

bool Polyhedron::isConvex() const {
	return is_convex_;
}

void Polyhedron::getAABB(cnoid::Vector3& min, cnoid::Vector3& max) const {
	min = aabb_min_;
	max = aabb_max_;
}

void Polyhedron::construct() {
	bindNeighbors();
	computeNormals();
	computeBBs();
	convexityTest();
	computeAABB();
	buildColdet();
}

void Polyhedron::computeNormals() {
	for (int i = 0; i < triangles_.size(); i++) {
		Triangle* tri = triangles_[i];
		tri->e1 = vertices_[tri->vid[1]] - vertices_[tri->vid[0]];
		tri->e2 = vertices_[tri->vid[2]] - vertices_[tri->vid[0]];
		tri->normal = tri->e1.cross(tri->e2).normalized();
	}
}

void Polyhedron::computeBBs() {
	for (int i = 0; i < triangles_.size(); i++) {
		double x_max = -std::numeric_limits<double>::max();
		double x_min = std::numeric_limits<double>::max();
		double z_max = -std::numeric_limits<double>::max();
		double z_min = std::numeric_limits<double>::max();
		Triangle* tri = triangles_[i];
		for (int i = 0; i < 3; i++) {
			if (x_max < vertices_[tri->vid[i]].x()) {
				x_max = vertices_[tri->vid[i]].x();
			}
			if (x_min > vertices_[tri->vid[i]].x()) {
				x_min = vertices_[tri->vid[i]].x();
			}
			if (z_max < vertices_[tri->vid[i]].z()) {
				z_max = vertices_[tri->vid[i]].z();
			}
			if (z_min > vertices_[tri->vid[i]].z()) {
				z_min = vertices_[tri->vid[i]].z();
			}
		}
		tri->x_max = x_max;
		tri->x_min = x_min;
		tri->z_max = z_max;
		tri->z_min = z_min;
	}
}

void Polyhedron::convexityTest() {
	for (int i = 0; i < triangles_.size(); i++) {
		Triangle* tri = triangles_[i];
		for (int k = 0; k < 3; k++) {
			if (tri->nbr[k] == NULL) continue;
			Vector3 v = (vertices_[tri->vid[(k+1)%3]] - vertices_[tri->vid[k]]).normalized();
			if (tri->normal.dot(tri->nbr[k]->normal.cross(v)) < 0 && tri->normal.dot(tri->nbr[k]->normal) < 1.0 - std::numeric_limits<double>::epsilon()) {
				is_convex_ = false;
				return;
			}
		}
	}
	is_convex_ = true;
}

void Polyhedron::computeAABB() {
	aabb_min_ = std::numeric_limits<double>::max() *  Vector3::Ones();
	aabb_max_ = -std::numeric_limits<double>::max() * Vector3::Ones();
	for (int i = 0; i < vertices_.size(); i++) {
		if (aabb_min_.x() > vertices_[i].x()) aabb_min_.x() = vertices_[i].x();
		if (aabb_min_.y() > vertices_[i].y()) aabb_min_.y() = vertices_[i].y();
		if (aabb_min_.z() > vertices_[i].z()) aabb_min_.z() = vertices_[i].z();
		if (aabb_max_.x() < vertices_[i].x()) aabb_max_.x() = vertices_[i].x();
		if (aabb_max_.y() < vertices_[i].y()) aabb_max_.y() = vertices_[i].y();
		if (aabb_max_.z() < vertices_[i].z()) aabb_max_.z() = vertices_[i].z();
	}
}

void Polyhedron::bindNeighbors() {
	for (int i = 0; i < triangles_.size(); i++) {
		for (int k = 0; k < 3; k++) {
			triangles_[i]->nbr[k] = NULL;
		}
		for (int j = 0; j < triangles_.size(); j++) {
			if (i == j) continue;
			int count_same_point = 0;
			for (int k = 0; k < 3; k++) {
				bool has_same_vid1 = false;
				bool has_same_vid2 = false;
				for (int l = 0; l < 3; l++) {
					if (triangles_[i]->vid[k] == triangles_[j]->vid[l]) has_same_vid1 = true;
					if (triangles_[i]->vid[(k+1)%3] == triangles_[j]->vid[l]) has_same_vid2 = true;
				}
				if (has_same_vid1 && has_same_vid2) {
					triangles_[i]->nbr[k] = triangles_[j];
				}
			}
		}
	}
}

void Polyhedron::buildColdet() {
	coldet_model_ = grasp::createColdetModel();
	for (size_t i = 0; i < vertices_.size(); i++) {
		coldet_model_->addVertex(vertices_[i].x(), vertices_[i].y(), vertices_[i].z());
	}
	for (size_t i = 0; i < triangles_.size(); i++) {
		coldet_model_->addTriangle(triangles_[i]->vid[0], triangles_[i]->vid[1], triangles_[i]->vid[2]);
	}
	coldet_model_->build();
}

void Polyhedron::clear() {
	vertices_.clear();
	for (size_t i = 0; i < triangles_.size(); i++) {
		delete triangles_[i];
		triangles_[i] = NULL;
	}
	triangles_.clear();
}

SweptVolumePolyhedron::SweptVolumePolyhedron()
	: is_finger_(false) {
}

SweptVolumePolyhedron::~SweptVolumePolyhedron() {
}

void SweptVolumePolyhedron::setType(TYPE type) {
	type_ = type;
}

bool SweptVolumePolyhedron::isModelPart() const {
	return (type_ == MODEL);
}

bool SweptVolumePolyhedron::isApproachPart() const {
	return (type_ == APPROACH);
}

bool SweptVolumePolyhedron::isGraspPart() const {
	return (type_ == GRASP);
}

void SweptVolumePolyhedron::setFingerPart(bool is_finger) {
	is_finger_ = is_finger;
}

bool SweptVolumePolyhedron::isFingerPart() const {
	return is_finger_;
}

ModelDivider::ModelDivider() {
}

ModelDivider::~ModelDivider() {
}

void ModelDivider::divide(const cnoid::Link* target_link, PolyhedronPtrVec& polyhedra) {
	vertices_.clear();
	triangles_.clear();
	makeTriangles(target_link);
	bindNeighbors();
	makePolyhedra(polyhedra);
}

void ModelDivider::makeTriangles(const cnoid::Link* target_link) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr coldet = target_link->coldetModel();

	int n_ver = coldet->getNumVertices();

	float tx, ty, tz;
	for (int i = 0; i < n_ver; i++) {
		coldet->getVertex(i, tx, ty, tz);
		vertices_.push_back(Vector3(tx, ty, tz));
	}

	int n_tri = coldet->getNumTriangles();
	for (int i = 0; i < n_tri; i++) {
		Triangle* tri = new Triangle;
		coldet->getTriangle(i, tri->vid[0], tri->vid[1], tri->vid[2]);
		triangles_.push_back(tri);
	}
#else
	cnoid::MeshExtractor extractor;
	cnoid::SgMesh* mesh = extractor.integrate(target_link->collisionShape());
        SgVertexArrayPtr vertices = mesh->vertices();
	for (int i = 0; i < vertices->size(); i++) {
		cnoid::Vector3f vec = vertices->at(i);
		vertices_.push_back(Vector3(vec[0], vec[1], vec[2]));
	}
        SgIndexArray indices = mesh->triangleVertices();
	for (int i = 0; i < mesh->numTriangles(); i++) {
		Triangle* tri = new Triangle;
		tri->vid[0] = indices[i*3];
		tri->vid[1] = indices[i*3+1];
		tri->vid[2] = indices[i*3+2];
		triangles_.push_back(tri);
	}
#endif
}

void ModelDivider::bindNeighbors() {
	for (int i = 0; i < triangles_.size(); i++) {
		for (int k = 0; k < 3; k++) {
			triangles_[i]->nbr[k] = NULL;
		}
		for (int j = 0; j < triangles_.size(); j++) {
			if (i == j) continue;
			int count_same_point = 0;
			for (int k = 0; k < 3; k++) {
				bool has_same_vid1 = false;
				bool has_same_vid2 = false;
				for (int l = 0; l < 3; l++) {
					if (triangles_[i]->vid[k] == triangles_[j]->vid[l]) has_same_vid1 = true;
					if (triangles_[i]->vid[(k+1)%3] == triangles_[j]->vid[l]) has_same_vid2 = true;
				}
				if (has_same_vid1 && has_same_vid2) {
					triangles_[i]->nbr[k] = triangles_[j];
				}
			}
		}
	}
}

void ModelDivider::makePolyhedra(PolyhedronPtrVec& polyhedra) {
	typedef std::set<Triangle*> TriangleSet;
	typedef TriangleSet::iterator TriangleSetIte;

	TriangleSet triangle_set;

	for (int i = 0; i < triangles_.size(); i++) {
		triangle_set.insert(triangles_[i]);
	}

	while (!triangle_set.empty()) {
		TriangleSetIte target_ite = triangle_set.begin();
		Triangle* target_tri = *target_ite;
		triangle_set.erase(target_ite);

		bool closed = true;
		TrianglePtrVec tris;

		std::stack<Triangle*> nbr_cand;
		nbr_cand.push(target_tri);
		tris.push_back(target_tri);
		while (!nbr_cand.empty()) {
			Triangle* tri = nbr_cand.top();
			nbr_cand.pop();
			for (int i = 0; i < 3; i++) {
				if (tri->nbr[i] == NULL) {
					closed = false;
					continue;
				}
				TriangleSetIte nbr_ite = triangle_set.find(tri->nbr[i]);
				if (nbr_ite != triangle_set.end()) {
					nbr_cand.push(tri->nbr[i]);
					tris.push_back(tri->nbr[i]);
					triangle_set.erase(nbr_ite);
				}
			}
		}

		Polyhedron* poly = new Polyhedron();
		poly->setClosed(closed);
		makePolyhedron(tris, poly);
		polyhedra.push_back(poly);
	}
}

void ModelDivider::makePolyhedron(const TrianglePtrVec& triangles, Polyhedron* poly) {
	typedef std::map<int, int> LUT;
	typedef LUT::iterator LUTIte;

	LUT vid_lut;
	for (int i = 0; i < triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			vid_lut[triangles[i]->vid[k]] = 1;
		}
	}

	int id = 0;
	for (LUTIte ite = vid_lut.begin(); ite != vid_lut.end(); ++ite) {
		poly->addVertex(vertices_[ite->first]);
		ite->second = id++;
	}

	for (int i = 0; i < triangles.size(); i++) {
		for (int k = 0; k < 3; k++) {
			triangles[i]->vid[k] = vid_lut[triangles[i]->vid[k]];
		}
		poly->addTrianglePtr(triangles[i]);
	}
}

SweptVolume::SweptVolume() :
	app_len_(0.1),
	num_grasp_step_(1) {
	gc = grasp::PlanBase::instance();
}

SweptVolume::~SweptVolume() {
	clear();
}

void SweptVolume::setGRCR(const cnoid::Matrix3& palm_R_GRC) {
	palm_R_GRC_ = palm_R_GRC;
}

cnoid::Matrix3 SweptVolume::getGRCR() const {
	return palm_R_GRC_;
}

double SweptVolume::getAppLength() const {
	return app_len_;
}

void SweptVolume::setAppLength(double len) {
	app_len_ = len;
}

void SweptVolume::setNumGraspStep(unsigned int n) {
	if (n < 1) return;
	num_grasp_step_ = n;
}

void SweptVolume::makeSweptVolume() {
	clear();

	storeFinger();

	setCloseFinger();

	makeLinkPolyhedra(gc->palm());

	for (int i = 0; i < gc->nFing(); i++) {
		grasp::FingerPtr finger = gc->fingers(i);
		for (int j = 0; j < finger->nJoints; j++) {
			cnoid::Link* target_link = finger->joint(j);
			makeLinkPolyhedra(target_link);
		}
	}

	restoreFinger();

	hand_name_ = gc->targetArmFinger->handName;
}

SweptVolume::SVPolyhedronPtrVec& SweptVolume::polyhedra() {
	return polyhedra_;
}

const SweptVolume::SVPolyhedronPtrVec& SweptVolume::polyhedra() const {
	return polyhedra_;
}

void SweptVolume::clear() {
	for (size_t i = 0; i < polyhedra_.size(); i++) {
		delete polyhedra_[i];
		polyhedra_[i] = NULL;
	}
	polyhedra_.clear();
}

std::string SweptVolume::hand_name() const {
	return hand_name_;
}

void SweptVolume::storeFinger() {
	finger_q_.clear();
	for (int i = 0; i < gc->nFing(); i++) {
		grasp::FingerPtr finger = gc->fingers(i);
		for (int j = 0; j < finger->nJoints; j++) {
			finger_q_.push_back(finger->joint(j)->q());
		}
	}
}

void SweptVolume::restoreFinger() {
	int n = 0;
	for (int i = 0; i < gc->nFing(); i++) {
		grasp::FingerPtr finger = gc->fingers(i);
		for (int j = 0; j < finger->nJoints; j++) {
			finger->joint(j)->q() = finger_q_[n++];
		}
	}
	gc->bodyItemRobot()->body()->calcForwardKinematics();
	gc->flush();
}

void SweptVolume::setCloseFinger() {
	for (int i = 0; i < gc->nFing(); i++) {
		grasp::FingerPtr finger = gc->fingers(i);
		// open pose
		for (int j = 0; j < finger->nJoints; j++) {
			finger->joint(j)->q() = finger->fingerOpenPose[j];
		}
		for (size_t c = 0; c < finger->contact.size(); c++) {
			if (!finger->contact[c]) continue;
			cnoid::Link* target_joint = finger->fing_path->joint(finger->compLink[c]);
			target_joint->q() = finger->fingerOpenPose[c] + (1000 * finger->close[c]);
			if (target_joint->q() < target_joint->q_lower()) {
				target_joint->q() = target_joint->q_lower();
			}
			if (target_joint->q() > target_joint->q_upper()) {
				target_joint->q() = target_joint->q_upper();
			}
			grasp::PlanBase::instance()->setInterLink();
		}
		// set grasp pose
		finger->fingerGraspPose.resize(finger->nJoints);
		for (int j = 0; j < finger->nJoints; j++) {
			finger->fingerGraspPose[j] = finger->joint(j)->q();
		}
	}
}

void SweptVolume::openFinger() {
	graspFinger(0, 1);
}

void SweptVolume::closeFinger() {
	graspFinger(1, 1);
}

void SweptVolume::graspFinger(int n, int n_close) {
	double t = static_cast<double>(n)/static_cast<double>(n_close);
	for (int i = 0; i < gc->nFing(); i++) {
		grasp::FingerPtr finger = gc->fingers(i);
		for (int j = 0; j < finger->nJoints; j++) {
			double offset = t * (finger->fingerGraspPose[j] - finger->fingerOpenPose[j]);
			finger->joint(j)->q() = finger->fingerOpenPose[j] + offset;
		}
	}
	gc->bodyItemRobot()->body()->calcForwardKinematics();
}

void SweptVolume::makeLinkPolyhedra(const cnoid::Link* target_link) {
	PolyhedronPtrVec model_polyhedra;

	divider_.divide(target_link, model_polyhedra);

	for (int i = 0; i < model_polyhedra.size(); i++) {
		openFinger();
		SweptVolumePolyhedron* model_poly = new SweptVolumePolyhedron();
		model_poly->setType(SweptVolumePolyhedron::MODEL);
		model_poly->setFingerPart(target_link != gc->palm());
		makeLinkModelPolyhedron(target_link, model_polyhedra[i], model_poly);

		polyhedra_.push_back(model_poly);

		SVPolyhedronPtrVec approach_polys;

		makeApproachPolyhedra(model_poly, approach_polys);

		for (int j = 0; j < approach_polys.size(); j++) {
			approach_polys[j]->setFingerPart(target_link != gc->palm());
		}

		polyhedra_.insert(polyhedra_.end(), approach_polys.begin(), approach_polys.end());

		if (target_link == gc->palm()) continue;

		SVPolyhedronPtrVec grasp_polys;

		int n_divide = ((target_link->parent() == gc->palm()) && target_link->isSlideJoint()) ?  0 : num_grasp_step_-1;
		makeGraspPolyhedra(target_link, model_polyhedra[i], grasp_polys, n_divide);

		polyhedra_.insert(polyhedra_.end(), grasp_polys.begin(), grasp_polys.end());
	}

	for (int i = 0; i < model_polyhedra.size(); i++) {
		delete model_polyhedra[i];
		model_polyhedra[i] = NULL;
	}
	model_polyhedra.clear();
}

void SweptVolume::makeLinkModelPolyhedron(const cnoid::Link* target_link, const Polyhedron* base_poly,
																					SweptVolumePolyhedron* model_poly) const {
	Link* palm_link = gc->palm();
	Vector3 palm_p = palm_link->p();
	Matrix3 palm_R = palm_link->R();
	Vector3 link_p = palm_R.transpose() * (target_link->p() - palm_p);
	Matrix3 link_R = palm_R.transpose() * target_link->R();

	Matrix3 GRC_R_palm = palm_R_GRC_.transpose();
	for (int n = 0; n < (base_poly->vertices()).size(); n++) {
		Vector3 p;
		p = GRC_R_palm * (link_R * base_poly->vertex(n) + link_p);
		model_poly->addVertex(p);
	}

	for (int n = 0; n < (base_poly->triangles()).size(); n++) {
		Triangle* tri = new Triangle();
		for (int k = 0; k < 3; k++) {
			tri->vid[k] = base_poly->triangle(n)->vid[k];
		}
		model_poly->addTrianglePtr(tri);
	}

	model_poly->construct();
}

void SweptVolume::makeApproachPolyhedra(const SweptVolumePolyhedron* model_polyhedron, SVPolyhedronPtrVec& app_polyhedra) const {
	typedef std::set<const Triangle*> TriangleSet;
	typedef TriangleSet::iterator TriangleSetIte;

	cnoid::Vector3 app_vec(0, 1, 0);

	TriangleSet triangle_set;

	for (int i = 0; i < model_polyhedron->triangles().size(); i++) {
		triangle_set.insert(model_polyhedron->triangle(i));
	}

	while (!triangle_set.empty()) {
		TriangleSetIte target_ite = triangle_set.begin();
		const Triangle* target_tri = *target_ite;
		triangle_set.erase(target_ite);

		if (target_tri->normal.dot(app_vec) < 0) continue;

		TriangleConstPtrVec incident_facet;

		std::stack<const Triangle*> nbr_cand;
		nbr_cand.push(target_tri);
		incident_facet.push_back(target_tri);

		while (!nbr_cand.empty()) {
			const Triangle* tri = nbr_cand.top();
			nbr_cand.pop();
			for (int i = 0; i < 3; i++) {
				if (tri->nbr[i] == NULL) {
					continue;
				}
				TriangleSetIte nbr_ite = triangle_set.find(tri->nbr[i]);
				if (nbr_ite != triangle_set.end()) {
					triangle_set.erase(nbr_ite);
					if (tri->nbr[i]->normal.dot(app_vec) >= 0) {
						nbr_cand.push(tri->nbr[i]);
						incident_facet.push_back(tri->nbr[i]);
					}
				}
			}
		}
		SweptVolumePolyhedron* trans_poly = new SweptVolumePolyhedron();
		trans_poly->setType(SweptVolumePolyhedron::APPROACH);
		makeApproachPolyhedronByFacet(model_polyhedron, incident_facet, trans_poly);
		app_polyhedra.push_back(trans_poly);
	}
}

void SweptVolume::makeApproachPolyhedronByFacet(const SweptVolumePolyhedron* model_polyhedron, const TriangleConstPtrVec& facet, SweptVolumePolyhedron* app_polyhedron) const {
	typedef std::map<int, int> LUT;
	typedef LUT::iterator LUTIte;

	cnoid::Vector3 app_vec(0, app_len_, 0);

	LUT vid_lut;
	for (int i = 0; i < facet.size(); i++) {
		for (int k = 0; k < 3; k++) {
			vid_lut[facet[i]->vid[k]] = 1;
		}
	}

	// make vertices
	int id = 0;
	for (LUTIte ite = vid_lut.begin(); ite != vid_lut.end(); ++ite) {
		app_polyhedron->addVertex(model_polyhedron->vertex(ite->first));
		ite->second = id++;
	}

	int n_ver_facet = app_polyhedron->vertices().size();
	for (int i = 0; i < n_ver_facet; i++) {
		app_polyhedron->addVertex(app_polyhedron->vertex(i) + app_vec);
	}

	// make triangles
	for (int i = 0; i < facet.size(); i++) {
		Triangle* tri = new Triangle;
		tri->vid[0] = vid_lut[facet[i]->vid[0]];
		tri->vid[1] = vid_lut[facet[i]->vid[2]];
		tri->vid[2] = vid_lut[facet[i]->vid[1]];
		app_polyhedron->addTrianglePtr(tri);

		Triangle* tri_translate = new Triangle;
		tri_translate->vid[0] = vid_lut[facet[i]->vid[0]] + n_ver_facet;
		tri_translate->vid[1] = vid_lut[facet[i]->vid[1]] + n_ver_facet;
		tri_translate->vid[2] = vid_lut[facet[i]->vid[2]] + n_ver_facet;
		app_polyhedron->addTrianglePtr(tri_translate);

		for (int k = 0; k < 3; k++) {
			if (facet[i]->nbr[k] == NULL || facet[i]->nbr[k]->normal.dot(app_vec) < 0) {
				Triangle* side_tri1 = new Triangle;
				side_tri1->vid[0] = vid_lut[facet[i]->vid[k]];
				side_tri1->vid[1] = vid_lut[facet[i]->vid[(k+1)%3]];
				side_tri1->vid[2] = vid_lut[facet[i]->vid[k]] + n_ver_facet;
				app_polyhedron->addTrianglePtr(side_tri1);
				Triangle* side_tri2 = new Triangle;
				side_tri2->vid[0] = vid_lut[facet[i]->vid[(k+1)%3]] + n_ver_facet;
				side_tri2->vid[1] = vid_lut[facet[i]->vid[k]] + n_ver_facet;
				side_tri2->vid[2] = vid_lut[facet[i]->vid[(k+1)%3]];
				app_polyhedron->addTrianglePtr(side_tri2);
			}
		}
	}

	app_polyhedron->construct();
}

void SweptVolume::makeGraspPolyhedra(const cnoid::Link* target_link, const Polyhedron* base_poly, SVPolyhedronPtrVec& grasp_polyhedra, int n_divide) {
	openFinger();
	Link* palm_link = gc->palm();
	Vector3 palm_p = palm_link->p();
	Matrix3 palm_R = palm_link->R();

	Matrix3 GRC_R_palm = palm_R_GRC_.transpose();

	cnoid::Vector3 app_vec(0, app_len_, 0);

	for (int n = 0; n < n_divide + 1; n++) {
		graspFinger(n, n_divide + 1);

		Vector3 link_p = palm_R.transpose() * (target_link->p() - palm_p);
		Matrix3 link_R = palm_R.transpose() * target_link->R();

		SweptVolumePolyhedron* grasp_poly = new SweptVolumePolyhedron();
		grasp_poly->setType(SweptVolumePolyhedron::GRASP);
		std::vector<double> ver;
		for (int i = 0; i < (base_poly->vertices()).size(); i++) {
			Vector3 p;
			p = GRC_R_palm * (link_R * base_poly->vertex(i) + link_p) + app_vec;
			for (int k = 0; k < 3; k++) {
				ver.push_back(p(k));
			}
			grasp_poly->addVertex(p);
		}

		graspFinger(n + 1, n_divide + 1);
		link_p = palm_R.transpose() * (target_link->p() - palm_p);
		link_R = palm_R.transpose() * target_link->R();

		for (int i = 0; i < (base_poly->vertices()).size(); i++) {
			Vector3 p;
			p = GRC_R_palm * (link_R * base_poly->vertex(i) + link_p) + app_vec;
			for (int k = 0; k < 3; k++) {
				ver.push_back(p(k));
			}
			grasp_poly->addVertex(p);
		}

		std::vector<double> pt_out;
		std::vector<int> idx_out;
		grasp::ConvexAnalysis ca;
		ca.calcConvexHull(3, ver, pt_out, idx_out, false);

		for (int i = 0; i < idx_out.size(); i++) {
			int size = idx_out[i];
			int start = i + 1;
			std::vector<int> indices;
			for (int j = start; j < start+size; j++) {
				i = j;
				indices.push_back(idx_out[j]);
			}
			for (int j = 2; j < indices.size(); j++) {
				Triangle* tri = new Triangle();
				tri->vid[2] = indices[0];
				tri->vid[1] = indices[j-1];
				tri->vid[0] = indices[j];
				grasp_poly->addTrianglePtr(tri);
			}
		}

		grasp_poly->construct();

		grasp_polyhedra.push_back(grasp_poly);
	}
}

SweptVolumeChecker::SweptVolumeChecker(const SweptVolume* sv) :
	sv_(NULL), est_sol_(NULL),
	margin_(0.0),
	voxel_(NULL),
	unknown_region_check_mode_(false) {
	penalty_calc_ = new PenaltyCalculator();
	sv_ = sv;
}

SweptVolumeChecker::~SweptVolumeChecker() {
	delete penalty_calc_;
}

void SweptVolumeChecker::setObjPoseEstimateSol(const grasp::ObjPoseEstimateSol* est_sol) {
	est_sol_ = est_sol;
}

double SweptVolumeChecker::check(const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R) {
	if (sv_ == NULL) {
		std::cout << "error: SweptVolume is not set in SweptVolumeChecker!" << std::endl;
		return 0;
	}
	if (est_sol_ == NULL) {
		std::cout << "error: ObjPoseEstimateSol is not set int SweptVolumeChecker!" << std::endl;
		return 0;
	}

	// compute AABB of the swept volume
	computeAABB();

	std::vector<int> indices;
	getInsideRotatedAABBIndices(palm_R, palm_p, indices);

	transToGRCcoordinate(palm_R, palm_p, indices);

	// Listing indices which are inside the AABB
	std::vector<int> indices_sv;
	getInsideAABBIndices(indices, indices_sv);

	labelingInsidePoints(indices_sv);

	double score;
	bool update_mode = false;
#ifdef	CLASSIFIER_UPDATEMODE
	update_mode = true;
#endif
	if (update_mode && (!penalty_calc_->hasEnoughData())) {
		computeFeatureSum();
		score = penalty_calc_->getDefaultScore(feature_);
		if (penalty_calc_->useHistogramFeature()) {
			computeFeatureHistogram(penalty_calc_->getHistogramParameter());
		}
	} else {
		if (penalty_calc_->useHistogramFeature()) {
			computeFeatureHistogram(penalty_calc_->getHistogramParameter());
		} else {
			computeFeatureSum();
		}
		score = penalty_calc_->getScore(feature_);
		if (in_indices_.empty()) score = 0.5;
	}

	if (unknown_region_check_mode_) {
		score -= computeUnknownRegionHeuristicScore(palm_R, palm_p);
	}

	// return score;
	return score + 0.5;
}

double SweptVolumeChecker::computeScore() {
	double score;
	computeFeatureSum();
	score = penalty_calc_->getDefaultScore(feature_);
	return score;
}

void SweptVolumeChecker::getFeatureVector(std::vector<double>& feature) const {
	feature = feature_;
}

void SweptVolumeChecker::computeAABB() {
	// Compute the AABB of swept volumes
	bb_min_ = std::numeric_limits<double>::max() *  Vector3::Ones();
	bb_max_ = -std::numeric_limits<double>::max() * Vector3::Ones();
	for (int i = 0; i < sv_->polyhedra().size(); i++) {
		Vector3 aabb_min, aabb_max;
		sv_->polyhedra()[i]->getAABB(aabb_min, aabb_max);
		bb_min_ = bb_min_.array().min(aabb_min.array());
		bb_max_ = bb_max_.array().max(aabb_max.array());
	}
	bb_min_ -= margin_ * Vector3::Ones();
	bb_max_ += margin_ * Vector3::Ones();
}

void SweptVolumeChecker::getInsideRotatedAABBIndices(const cnoid::Matrix3& palm_R, const cnoid::Vector3& palm_p, std::vector<int>& indices) const {
	int indices_size = est_sol_->outlier_indices.size();
	indices.clear();
	indices.reserve(indices_size);

	cnoid::Vector3 r_bb_min = std::numeric_limits<double>::max() *  Vector3::Ones();
	cnoid::Vector3 r_bb_max = -std::numeric_limits<double>::max() * Vector3::Ones();

	std::vector<cnoid::Vector3> bb_point(8);
	bb_point[0] = bb_min_;
	bb_point[1] = cnoid::Vector3(bb_max_.x(), bb_min_.y(), bb_min_.z());
	bb_point[2] = cnoid::Vector3(bb_min_.x(), bb_max_.y(), bb_min_.z());
	bb_point[3] = cnoid::Vector3(bb_min_.x(), bb_min_.y(), bb_max_.z());
	bb_point[4] = cnoid::Vector3(bb_max_.x(), bb_max_.y(), bb_min_.z());
	bb_point[5] = cnoid::Vector3(bb_max_.x(), bb_min_.y(), bb_max_.z());
	bb_point[6] = cnoid::Vector3(bb_min_.x(), bb_max_.y(), bb_max_.z());
	bb_point[7] = bb_max_;

	Matrix3 wRtip = palm_R * sv_->getGRCR();
	for (int i = 0; i < 8; i++) {
		cnoid::Vector3 tmp = wRtip * bb_point[i] + palm_p;
		r_bb_min = r_bb_min.array().min(tmp.array());
		r_bb_max = r_bb_max.array().max(tmp.array());
	}

	for (int i = 0; i < indices_size; i++) {
		int idx = est_sol_->outlier_indices[i];
		if ((r_bb_min.array() > (est_sol_->env_point_cloud->p())[idx].array()).any()) continue;
		if ((r_bb_max.array() < (est_sol_->env_point_cloud->p())[idx].array()).any()) continue;
		indices.push_back(idx);
	}
}

void SweptVolumeChecker::transToGRCcoordinate(const cnoid::Matrix3& palm_R, const cnoid::Vector3& palm_p, const std::vector<int>& indices) {
	Matrix3 tipRw = (palm_R * sv_->getGRCR()).transpose();

	int n_points = est_sol_->env_point_cloud->p().size();
	points_GRC_.clear();
	normals_GRC_.clear();
	points_GRC_.resize(n_points);
	normals_GRC_.resize(n_points);

#ifdef THREAD
	int num_threads = boost::thread::hardware_concurrency();
	boost::thread_group* thr_grp = new boost::thread_group();
	for (int i = 0; i < num_threads; i++) {
		int n = indices.size() / num_threads;
		int r = indices.size() % num_threads;
		int offset = (i > r) ? r : i;
		int offset_n = (i+1 > r) ? r : i+1;
		int start = i * n + offset;
		int end = (i+1) * n + offset_n;
		thr_grp->create_thread(boost::bind(&SweptVolumeChecker::transToGRCcoordinateProc, this, tipRw, palm_p, indices, start, end));
	}
	thr_grp->join_all();
	delete thr_grp;
#else
	transToGRCcoordinateProc(tipRw, palm_p, indices, 0, indices.size());
#endif
}

void SweptVolumeChecker::transToGRCcoordinateProc(const cnoid::Matrix3& tipRw, const cnoid::Vector3& palm_p, const std::vector<int>& indices,
																									int start, int end) {
	Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
	trans.topLeftCorner(3, 3) = tipRw;
	int idx;
	Eigen::Vector4d p(0, 0, 0, 1);
	Eigen::Vector4d n(0, 0, 0, 1);

	for (int i = start; i < end; i++) {
		idx = indices[i];
		p.head(3) = ((est_sol_->env_point_cloud->p())[idx]) - palm_p;
		//n.head(3) = ((est_sol_->env_point_cloud->n())[idx]);
		points_GRC_[idx] = (trans * p).head<3>();
		// normals_GRC_[idx] = (trans * n).head<3>();
	}
}

void SweptVolumeChecker::getInsideAABBIndices(const std::vector<int>& target_indices, std::vector<int>& indices) const {
	indices.clear();

	for (int i = 0; i < target_indices.size(); i++) {
		int idx = target_indices[i];
		if ((bb_min_.array() > points_GRC_[idx].array()).any()) continue;
		if ((bb_max_.array() < points_GRC_[idx].array()).any()) continue;
		indices.push_back(idx);
	}
}

void SweptVolumeChecker::labelingInsidePoints(const std::vector<int>& indices) {
	SVPolyhedronPtrVec palm_app_poly;
	SVPolyhedronPtrVec finger_app_poly;
	SVPolyhedronPtrVec grasp_poly;

	for (int i = 0; i < sv_->polyhedra().size(); i++) {
		SweptVolumePolyhedron* poly = sv_->polyhedra()[i];
		if (poly->isModelPart() || poly->isApproachPart()) {
			if (poly->isFingerPart()) {
				finger_app_poly.push_back(poly);
			} else {
				palm_app_poly.push_back(poly);
			}
		} else if (poly->isGraspPart()) {
			grasp_poly.push_back(poly);
		}
	}
	app_poly_.clear();
	app_poly_.insert(app_poly_.end(), palm_app_poly.begin(), palm_app_poly.end());
	app_poly_.insert(app_poly_.end(), finger_app_poly.begin(), finger_app_poly.end());

	indices_app_palm_.clear();
	indices_app_finger_.clear();
	indices_grasp_.clear();

	for (int i = 0; i < indices.size(); i++) {
		int idx = indices[i];
		if (isInsidePolyhedra(palm_app_poly, points_GRC_[idx])) {
			indices_app_palm_.push_back(idx);
			continue;
		}
		if (isInsidePolyhedra(finger_app_poly, points_GRC_[idx])) {
			indices_app_finger_.push_back(idx);
			continue;
		}
		if (isInsidePolyhedra(grasp_poly, points_GRC_[idx])) {
			indices_grasp_.push_back(idx);
			continue;
		}
	}

	in_indices_.clear();
	in_indices_.insert(in_indices_.end(), indices_app_palm_.begin(), indices_app_palm_.end());
	in_indices_.insert(in_indices_.end(), indices_app_finger_.begin(), indices_app_finger_.end());
	in_indices_.insert(in_indices_.end(), indices_grasp_.begin(), indices_grasp_.end());

	// Compute the AABB of volumes swept generated by fingers
	bb_finger_min_ = std::numeric_limits<double>::max() *  Vector3::Ones();
	bb_finger_max_ = -std::numeric_limits<double>::max() * Vector3::Ones();
	for (int i = 0; i < finger_app_poly.size(); i++) {
		Vector3 aabb_min, aabb_max;
		finger_app_poly[i]->getAABB(aabb_min, aabb_max);
		bb_finger_min_ = bb_finger_min_.array().min(aabb_min.array());
		bb_finger_max_ = bb_finger_max_.array().max(aabb_max.array());
	}
	bb_finger_min_ -= margin_ * Vector3::Ones();
	bb_finger_max_ += margin_ * Vector3::Ones();
}

void SweptVolumeChecker::computeFeatureSum() {
	double hp = computeHeightPenalty(points_GRC_, in_indices_, bb_max_);
	double dp = computeDistancePenalty(points_GRC_, indices_app_palm_, bb_min_, bb_max_);
	dp += computeDistancePenalty(points_GRC_, indices_app_finger_, bb_finger_min_, bb_finger_max_);
	dp += computeDistancePenalty(points_GRC_, indices_grasp_, bb_finger_min_, bb_finger_max_);

	feature_.clear();
	feature_.resize(2);
	feature_[0] = hp;
	feature_[1] = dp;
}

void SweptVolumeChecker::computeFeatureHistogram(const HistogramParameter* param) {
	feature_.clear();
	feature_.resize(param->h_dim * param->d_dim, 0);

	computeFeatureHistogramSub(indices_app_palm_, bb_max_, bb_min_, param);

	computeFeatureHistogramSub(indices_app_finger_, bb_finger_max_, bb_finger_min_, param);

	computeFeatureHistogramSub(indices_grasp_, bb_finger_max_, bb_finger_min_, param);
}

void SweptVolumeChecker::computeFeatureHistogramSub(const std::vector<int>& indices, const cnoid::Vector3& bb_max, const cnoid::Vector3& bb_min,
																						 const HistogramParameter* param) {
	int max_h_idx = param->h_dim - 1;
	int max_d_idx = param->d_dim - 1;
	for (int i = 0; i < indices.size(); i++) {
		int idx = indices[i];
		double height = bb_max_.y() - points_GRC_[idx].y();
		// double dist = std::min(std::min(bb_max.x() - points_GRC_[idx].x(), points_GRC_[idx].x() - bb_min.x()), std::min(bb_max.z() - points_GRC_[idx].z(), points_GRC_[idx].z() - bb_min.z()));
		double dist = std::min(bb_max.z() - points_GRC_[idx].z(), points_GRC_[idx].z() - bb_min.z());
		int h_idx = std::min(static_cast<int>(std::floor(height / param->h_step)), max_h_idx);
		int d_idx = std::max(std::min(static_cast<int>(std::floor(dist / param->d_step)), max_d_idx), 0);
		double center_h = (h_idx + 0.5) * param->h_step;
		double center_d = (d_idx + 0.5) * param->d_step;
		int neigh_h_idx = ((height - center_h) > 0) ? std::min(h_idx + 1, max_h_idx) : std::max(h_idx - 1, 0);
		int neigh_d_idx = ((dist - center_d) > 0) ? std::min(d_idx + 1, max_d_idx) : std::max(d_idx - 1, 0);
		double rate_h = 1.0 - std::fabs(height - center_h) / param->h_step;
		double rate_d = 1.0 - std::fabs(dist - center_d) / param->d_step;

		feature_[h_idx * param->d_dim + d_idx] += rate_h * rate_d;
		feature_[h_idx * param->d_dim + neigh_d_idx] += rate_h * (1.0 - rate_d);
		feature_[neigh_h_idx * param->d_dim + d_idx] += (1.0 - rate_h) * rate_d;
		feature_[neigh_h_idx * param->d_dim + neigh_d_idx] += (1.0 - rate_h) * (1.0 - rate_d);
	}
}

void SweptVolumeChecker::outputPointsGRC(const std::string& filepath) const {
	std::ofstream ofs(filepath.c_str());

	ofs << bb_min_.x() << " " << bb_min_.y() << " "  << bb_min_.z() << " "  << bb_max_.x() << " "  << bb_max_.y() << " "  << bb_max_.z() << std::endl;
	ofs << bb_finger_min_.x() << " "  << bb_finger_min_.y() << " "  << bb_finger_min_.z() << " "  << bb_finger_max_.x() << " "  << bb_finger_max_.y() << " "  << bb_finger_max_.z() << std::endl;
	for (int i = 0; i < in_indices_.size(); i++) {
		ofs << points_GRC_[in_indices_[i]].x() << " "  << points_GRC_[in_indices_[i]].y() << " "  << points_GRC_[in_indices_[i]].z() << std::endl;
	}
	ofs.close();
}

bool SweptVolumeChecker::isInsidePolyhedra(const SVPolyhedronPtrVec& polyhedra, const cnoid::Vector3& p) const {
	for (int j = 0; j < polyhedra.size(); j++) {
		Vector3 aabb_min, aabb_max;
		polyhedra[j]->getAABB(aabb_min, aabb_max);
		aabb_min -= margin_ * Vector3::Ones();
		aabb_max += margin_ * Vector3::Ones();
		if ((aabb_min.array() > p.array()).any()) continue;
		if ((aabb_max.array() < p.array()).any()) continue;
		bool is_inside;
		if (polyhedra[j]->isConvex()) {
			is_inside = isInsideConvexPolyhedron(p, polyhedra[j]);
		} else {
			is_inside = isInsideConcavePolyhedron(p, polyhedra[j]);
		}
		if (is_inside) {
			return true;
		}
		if (margin_ > 0) {
			std::vector<cnoid::Vector3> points(1);
			points[0] = p;
			if (polyhedra[j]->coldet_model()->checkCollisionWithPointCloud(points, margin_)) {
				return true;
			}
		}
	}
	return false;
}

bool SweptVolumeChecker::isInsideConvexPolyhedron(const cnoid::Vector3& p, Polyhedron* poly) const {
	for (int i = 0; i < poly->triangles().size(); i++) {
		Triangle* tri = poly->triangle(i);
		Vector3 v = poly->vertex(tri->vid[0]);
		if ((v - p).dot(tri->normal) < 0) {
			return false;
		}
	}
	return true;
}

bool SweptVolumeChecker::isInsideConcavePolyhedron(const cnoid::Vector3& p, Polyhedron* poly) const {
	int count = 0;
	Vector3 d = Vector3(0, 1, 0);
	double shortest_dist = std::numeric_limits<double>::max();
	Vector3 n0;
	for (int i = 0; i < poly->triangles().size(); i++) {
		Triangle* tri = poly->triangle(i);
		if (p.x() > tri->x_max || p.x() < tri->x_min || p.z() > tri->z_max || p.z() < tri->z_min) continue;
		Vector3 v0 = poly->vertex(tri->vid[0]);
		Vector3 r = p - v0;
		Vector3 u = d.cross(tri->e2);
		Vector3 v = r.cross(tri->e1);
		double t = u.dot(tri->e1);
		if (fabs(t) < std::numeric_limits<double>::epsilon()) continue;
		double beta = u.dot(r);
		double gamma = v.dot(d);
		double dist = v.dot(tri->e2);
		bool intersectTriangle = false;
		if (t > 0) {
			if (0 < beta && 0 < gamma && beta + gamma < t) {
				intersectTriangle = true;
			}
		} else {
			if (0 > beta && 0 > gamma && beta + gamma > t) {
				intersectTriangle = true;
			}
		}
		if (!intersectTriangle) continue;
		count++;
		dist /= t;
		if (dist >= 0) {
			if (shortest_dist > dist) {
				shortest_dist = dist;
				n0 = tri->normal;
			}
		} else {
			if (shortest_dist > -dist) {
				shortest_dist = -dist;
				n0 = -tri->normal;
			}
		}
	}

	if (count == 0) {
		return false;
	}
	if (d.dot(n0) < 0) {
		return false;
	}
	return true;
}

double SweptVolumeChecker::computeHeightPenalty(const std::vector<Vector3>& points, const std::vector<int>& indices,
																								const cnoid::Vector3& bb_max) const {
	double penalty = 0;
	for (size_t i = 0; i < indices.size(); i++) {
		penalty += bb_max.y() - points[indices[i]].y();
	}
	return penalty;
}

double SweptVolumeChecker::computeDistancePenalty(const std::vector<Vector3>& points, const std::vector<int>& indices,
																									const cnoid::Vector3& bb_min, const cnoid::Vector3& bb_max) const {
	double penalty = 0;
	for (size_t i = 0; i < indices.size(); i++) {
		int idx = indices[i];
		penalty += std::min(std::min(bb_max.x() - points[idx].x(), points[idx].x() - bb_min.x()), std::min(bb_max.z() - points[idx].z(), points[idx].z() - bb_min.z()));
	}
	return penalty;
}

double SweptVolumeChecker::computeApproachRegionPenalty(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& normals,
																												const std::vector<int>& indices, const cnoid::Vector3& bb_min, const cnoid::Vector3& bb_max,
																												const SVPolyhedronPtrVec& polyhedra) const {
	double penalty = 0;

	std::vector<double> height_vec;
	computeHeights(points, indices, polyhedra, height_vec);

	cnoid::Vector3 center = 0.5 * (bb_min + bb_max);

	for (size_t i = 0; i < indices.size(); i++) {
		int idx = indices[i];
		double h = height_vec[i];
		double d = std::min(std::min(bb_max.x() - points[idx].x(), points[idx].x() - bb_min.x()), std::min(bb_max.z() - points[idx].z(), points[idx].z() - bb_min.z()));

		cnoid::Vector3 n = normals[idx];
		if (grasp::isnan(n.x()) || grasp::isnan(n.y()) || grasp::isnan(n.z())) {
			n = cnoid::Vector3(0, -1, 0);
		}
		cnoid::Vector3 t = (points[idx] - center);
		t.y() = 0;
		t.normalize();

		penalty += sqrt(h * d) * (n.dot(t) + 1.5);
	}
	return penalty;
}

void SweptVolumeChecker::computeHeights(const std::vector<cnoid::Vector3>& points, const std::vector<int>& indices, const SVPolyhedronPtrVec& polyhedra,
																				std::vector<double>& height_vec) const {
	Vector3 d(0, 1, 0);
	height_vec.clear();
	height_vec.resize(indices.size());
	for (size_t i = 0; i < indices.size(); i++) {
		int idx = indices[i];
		double h_max = 0;
		for (size_t j = 0; j < polyhedra.size(); j++) {
			Polyhedron* poly = polyhedra[j];
			for (int k = 0; k < poly->triangles().size(); k++) {
				Triangle* tri = poly->triangle(k);
				if (points[idx].x() > tri->x_max || points[idx].x() < tri->x_min || points[idx].z() > tri->z_max || points[idx].z() < tri->z_min) {
					continue;
				}
				if (d.dot(tri->normal) < 0) {
					continue;
				}
				Vector3 v0 = poly->vertex(tri->vid[0]);
				Vector3 r = points[idx] - v0;
				Vector3 u = d.cross(tri->e2);
				Vector3 v = r.cross(tri->e1);
				double t = u.dot(tri->e1);
				if (fabs(t) < std::numeric_limits<double>::epsilon()) continue;
				double beta = u.dot(r);
				double gamma = v.dot(d);
				double dist = v.dot(tri->e2);
				bool intersectTriangle = false;
				if (t > 0) {
					if (0 < beta && 0 < gamma && beta + gamma < t) {
						intersectTriangle = true;
					}
				} else {
					if (0 > beta && 0 > gamma && beta + gamma > t) {
						intersectTriangle = true;
					}
				}
				if (!intersectTriangle) continue;
				dist /= t;
				if (dist < 0) continue;
				if (h_max < dist) h_max = dist;
				if (poly->isConvex()) break;
			}
		}
		height_vec[i] = h_max;
	}
}

double SweptVolumeChecker::computeUnknownRegionHeuristicScore(const cnoid::Matrix3& palm_R, const cnoid::Vector3& palm_p) const {
	double score = 0.0;
	cnoid::Vector3 radius;
	double penalty;
	for (size_t i = 0; i < voxel_->size(); i++) {
		grasp::OccupiedVoxel& target_voxel = voxel_->at(i);
		if (i == 0) {
			radius = target_voxel.len.maxCoeff() * cnoid::Vector3::Ones();
			penalty = 5 * (target_voxel.len.prod() / (bb_max_ - bb_min_).prod());
		}
		if (target_voxel.data != grasp::OccupiedVoxel::UNVISITED) continue;
		cnoid::Vector3 center_GRC_ = (palm_R * sv_->getGRCR()).transpose() * (target_voxel.center - palm_p);
		if ((bb_min_.array() > (center_GRC_ + radius).array()).any()) continue;
		if ((bb_max_.array() < (center_GRC_ - radius).array()).any()) continue;

		std::vector<cnoid::Vector3> corner_offset(8);
		corner_offset[0] = target_voxel.len;
		corner_offset[1] = cnoid::Vector3( target_voxel.len.x(),	target_voxel.len.y(), -target_voxel.len.z());
		corner_offset[2] = cnoid::Vector3( target_voxel.len.x(), -target_voxel.len.y(),	 target_voxel.len.z());
		corner_offset[3] = cnoid::Vector3( target_voxel.len.x(), -target_voxel.len.y(), -target_voxel.len.z());
		corner_offset[4] = cnoid::Vector3(-target_voxel.len.x(),	target_voxel.len.y(),	 target_voxel.len.z());
		corner_offset[5] = cnoid::Vector3(-target_voxel.len.x(),	target_voxel.len.y(), -target_voxel.len.z());
		corner_offset[6] = cnoid::Vector3(-target_voxel.len.x(), -target_voxel.len.y(),	 target_voxel.len.z());
		corner_offset[7] = cnoid::Vector3(-target_voxel.len.x(), -target_voxel.len.y(), -target_voxel.len.z());

		for (int j = 0; j < 8; j++) {
			cnoid::Vector3 corner_GRC_ = (palm_R * sv_->getGRCR()).transpose() * target_voxel.R * corner_offset[j];
			if ((bb_min_.array() > (corner_GRC_).array()).any()) continue;
			if ((bb_max_.array() < (corner_GRC_).array()).any()) continue;
			score += penalty;
			break;
		}
	}
	return score;
}

void SweptVolumeChecker::getInlierIndices(std::vector<int>& indices) {
	indices.clear();
	indices.insert(indices.end(), in_indices_.begin(), in_indices_.end());
}

void SweptVolumeChecker::setMargin(double margin) {
	margin_ = margin;
}

void SweptVolumeChecker::setObjName(const std::string& obj_name) {
	// penalty_calc_->update(sv_->hand_name(), obj_name);
	penalty_calc_->loadModel(sv_->hand_name(), obj_name);
}

void SweptVolumeChecker::setUnknownRegionCheckMode(bool mode) {
	unknown_region_check_mode_ = mode;
}

void SweptVolumeChecker::setOccupiedVoxelVec(grasp::OccupiedVoxelVec* voxel) {
	voxel_ = voxel;
}

SweptVolumeDrawer::SweptVolumeDrawer(SweptVolume* sv) :
	sv_(sv),
	points_ptr_(NULL),
	indices_ptr_(NULL) {
}

SweptVolumeDrawer::~SweptVolumeDrawer() {
}

void SweptVolumeDrawer::draw(const cnoid::Vector3& p_offset) {
	DrawUtility* draw = DrawUtility::instance();
	draw->points.clear();
	draw->triangles.clear();
	draw->colors.clear();

	grasp::PlanBase* gc = grasp::PlanBase::instance();
	Matrix3 w_R_GRC = gc->palm()->R() * sv_->getGRCR();

	int offset = 0;

	for (int i = 0; i < sv_->polyhedra().size(); i++) {
		SweptVolumePolyhedron* poly = sv_->polyhedra()[i];
		Vector3 color;
		if (poly->isModelPart()) {
			color = Vector3(0.5, 0.5, 0.5);
		} else if (poly->isApproachPart()) {
			color = Vector3(0, 1, 0);
		} else if (poly->isGraspPart()) {
			color = Vector3(0.5, 0.5, 0);
		}

		for (int j = 0; j < poly->vertices().size(); j++) {
			Vector3 p = w_R_GRC * poly->vertex(j) + gc->palm()->p() + p_offset;
			draw->points.push_back(p);
		}

		for (int j = 0; j < poly->triangles().size(); j++) {
			std::vector<int> indices(3);
			for (int k = 0; k < 3; k++) {
				indices[k] = poly->triangle(j)->vid[k] + offset;
			}
			draw->triangles.push_back(indices);
			draw->colors.push_back(color);
		}

		offset += poly->vertices().size();
	}

	if (points_ptr_ != NULL) {
		for (size_t i = 0; i < indices_ptr_->size(); i++) {
			int idx = (*indices_ptr_)[i];
			draw->spheres.push_back(Spheres((*points_ptr_)[idx], 0.0005, Vector3(1, 0, 0)));
		}
	}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cnoid::OSGSceneObjectPtr osg_shape;
	cnoid::OSGSceneObjectPtr osg_triangle;
	draw->displayShapes(osg_shape);
	draw->displayTriangles(0.3, osg_triangle);
	draw->objList.push_back(osg_shape);
	draw->objList.push_back(osg_triangle);
#else
	draw->displayShapes();
	draw->displayTriangles(0.3);
#endif
}

void SweptVolumeDrawer::clear() {
	DrawUtility* draw = DrawUtility::instance();
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	if (draw->objList.empty()) return;
	draw->clear();
	for (int i = 0; i < draw->objList.size(); i++) {
		draw->deleteanobj(draw->objList[i]);
	}
	draw->objList.clear();
	draw->redraw();
#else
	draw->clear();
#endif
}

void SweptVolumeDrawer::addPoints(const std::vector<cnoid::Vector3>& points) {
	points_.clear();
	points_.insert(points_.end(), points.begin(), points.end());
}

void SweptVolumeDrawer::addPointsPtr(const std::vector<cnoid::Vector3>* points, const std::vector<int>* indices) {
	points_ptr_ = points;
	indices_ptr_ = indices;
}


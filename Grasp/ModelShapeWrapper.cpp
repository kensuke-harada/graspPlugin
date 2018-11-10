#include "ModelShapeWrapper.h"

#ifdef CNOID_GE_15
#include "ColdetConverter.h"
#endif

using namespace grasp;

ModelShapeWrapper::ModelShapeWrapper() {
}

ModelShapeWrapper::ModelShapeWrapper(cnoid::Link* link) {
#ifdef CNOID_GE_15
	initProc(link->collisionShape());
#else
	initProc(link->coldetModel());
#endif
}

ModelShapeWrapper::ModelShapeWrapper(const ShapePtr& shape) {
	initProc(shape);
}

ModelShapeWrapper::~ModelShapeWrapper() {
}

int ModelShapeWrapper::numTriangles() const {
	return n_triangles_;
}

int ModelShapeWrapper::numVertices() const {
	return n_vertices_;
}

void ModelShapeWrapper::getTriangle(int tri_id, int& ver_id1, int& ver_id2, int& ver_id3) const {
#ifdef CNOID_GE_15
	cnoid::SgIndexArray& indices = geometry_->triangleVertices();
	ver_id1 = indices[tri_id * 3];
	ver_id2 = indices[tri_id * 3 + 1];
	ver_id3 = indices[tri_id * 3 + 2];
#else
	geometry_->getTriangle(tri_id, ver_id1, ver_id2, ver_id3);
#endif
}

void ModelShapeWrapper::getVertex(int ver_id, float& x, float& y, float& z) const {
#ifdef CNOID_GE_15
	cnoid::Vector3f& v = geometry_->vertices()->at(ver_id);
	x = v[0];
	y = v[1];
	z = v[2];
#else
	geometry_->getVertex(ver_id, x, y, z);
#endif
}

void ModelShapeWrapper::getVertex(int ver_id, cnoid::Vector3f& v) const {
#ifdef CNOID_GE_15
	v = geometry_->vertices()->at(ver_id);
#else
	geometry_->getVertex(ver_id, v[0], v[1], v[2]);
#endif
}

void ModelShapeWrapper::getVertex(int ver_id, cnoid::Vector3& v) const {
	cnoid::Vector3f v_temp;
	getVertex(ver_id, v_temp);
	v = v_temp.cast<double>();
}

void ModelShapeWrapper::initProc(const ShapePtr& shape) {
#ifdef CNOID_GE_15
	geometry_ = ColdetConverter::ExtractMesh(shape);
	n_triangles_ = geometry_->numTriangles();
	n_vertices_ = geometry_->vertices()->size();
#else
	geometry_ = shape;
	n_triangles_ = geometry_->getNumTriangles();
	n_vertices_ = geometry_->getNumVertices();
#endif	
}

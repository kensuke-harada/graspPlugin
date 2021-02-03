#ifndef _COLDET_CONVERTER_H
#define _COLDET_CONVERTER_H

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)

#include <cnoid/ColdetModel>
#include <cnoid/SceneShape>
#include <cnoid/MeshExtractor>
#include <cnoid/MeshNormalGenerator>

#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

#include "exportdef.h"
#include "UtilFunction.h"

namespace grasp{
	
class EXCADE_API ColdetConverter {
private:
	cnoid::MeshExtractor* meshExtractor;

	ColdetConverter() {
		meshExtractor = new cnoid::MeshExtractor();	
	}

	virtual ~ColdetConverter() {
		delete meshExtractor;
	}

	cnoid::ColdetModelPtr Convert(cnoid::SgNodePtr node) {
		cnoid::ColdetModelPtr model = createColdetModel();
#ifdef CNOID_GE_17
		if(meshExtractor->extract(node.get(), [&]() { addMesh(model); })){
#else
		if(meshExtractor->extract(node, boost::bind(&ColdetConverter::addMesh, this, model))){
#endif
			model->setName(node->name());
			model->build();
		}
		return model;
	}

	void addMesh(cnoid::ColdetModelPtr model)
	{
		cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
		const cnoid::Affine3& T = meshExtractor->currentTransform();
    
		const int vertexIndexTop = model->getNumVertices();
    
		const cnoid::SgVertexArray& vertices = *mesh->vertices();
		const int numVertices = vertices.size();
		for(int i=0; i < numVertices; ++i){
			const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
			model->addVertex(v.x(), v.y(), v.z());
		}

		const int numTriangles = mesh->numTriangles();
		for(int i=0; i < numTriangles; ++i){
			cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
			const int v0 = vertexIndexTop + tri[0];
			const int v1 = vertexIndexTop + tri[1];
			const int v2 = vertexIndexTop + tri[2];
			model->addTriangle(v0, v1, v2);
		}
	}

	cnoid::SgMeshPtr Extract(cnoid::SgNodePtr node) {
		cnoid::SgMeshPtr meshPtr(meshExtractor->integrate(node));
		/* meshExtractor->extract(node, boost::bind(&ColdetConverter::getMesh, this, &meshPtr)); */
		return meshPtr;
	}

	void getMesh(cnoid::SgMeshPtr* meshPtr)
	{
		*meshPtr = meshExtractor->currentMesh();
	}

public:
	static cnoid::SgMeshPtr ExtractMesh(cnoid::SgNode* node) {
		ColdetConverter converter;
		return converter.Extract(node);
	}

	static cnoid::ColdetModelPtr ConvertFrom(cnoid::SgNode* node) {
		ColdetConverter converter;
		return converter.Convert(node);
	}

	static cnoid::SgShapePtr ConvertTo(cnoid::ColdetModelPtr model) {
		cnoid::SgShapePtr shape = new cnoid::SgShape();
		cnoid::SgMeshPtr mesh = shape->setMesh(new cnoid::SgMesh());
		cnoid::SgVertexArrayPtr vertices = mesh->setVertices(new cnoid::SgVertexArray());
		{
			shape->setName(model->name());

			float x, y, z;
			vertices->resize(model->getNumVertices());
			for (int i = 0; i < model->getNumVertices(); i++) {
				model->getVertex(i, x, y, z);
				(*vertices)[i] << x, y, z;
			}

			int a, b, c;
			for (int i = 0; i < model->getNumTriangles(); i++) {
				model->getTriangle(i, a, b, c);
				mesh->addTriangle(a, b, c);
			}

			mesh->texCoordIndices() = mesh->triangleVertices();

			mesh->updateBoundingBox();

			cnoid::MeshNormalGenerator normalGenerator;
			normalGenerator.generateNormals(mesh, 0);
		}
		return shape;
	}
};

}

#endif

#endif


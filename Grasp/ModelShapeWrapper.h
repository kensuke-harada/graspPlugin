#ifndef _GRASP_MODELSHAPEWRAPPER_H_
#define _GRASP_MODELSHAPEWRAPPER_H_

#include <cnoid/EigenTypes>
#include <cnoid/Link>
#include <cnoid/ColdetModel>
#ifdef CNOID_GE_15
#include <cnoid/SceneShape>
#endif

#include "exportdef.h"

namespace grasp {
	class EXCADE_API ModelShapeWrapper {
	public:
#ifdef CNOID_GE_15
		typedef cnoid::SgNode* ShapePtr;
		typedef cnoid::SgMeshPtr GeometryPtr;
#else
		typedef cnoid::ColdetModelPtr ShapePtr;
		typedef cnoid::ColdetModelPtr GeometryPtr;
#endif

		explicit ModelShapeWrapper(cnoid::Link* link);
		explicit ModelShapeWrapper(const ShapePtr& shape);
		virtual ~ModelShapeWrapper();

		int numTriangles() const;
		int numVertices() const;

		void getTriangle(int tri_id, int& ver_id1, int& ver_id2, int& ver_id3) const;
		void getVertex(int ver_id, float& x, float& y, float& z) const;
		void getVertex(int ver_id, cnoid::Vector3f& v) const;
		void getVertex(int ver_id, cnoid::Vector3& v) const;

	private:
		ModelShapeWrapper();

		void initProc(const ShapePtr& shape);

		GeometryPtr geometry_;
		int n_triangles_;
		int n_vertices_;
	};
}

#endif /* _GRASP_MODELSHAPEWRAPPER_H_ */

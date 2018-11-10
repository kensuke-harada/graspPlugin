#ifndef _GEOMETRYHANDLER_SGVISITSHAPEGET_H_
#define _GEOMETRYHANDLER_SGVISITSHAPEGET_H_

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SceneGraph>
#else
#ifdef CNOID_15
#include <cnoid/SceneVisitor>
#endif
#include <cnoid/GLSceneRenderer>
#include <GL/glew.h>
#endif


#include "GeometryHandle.h"

#include "ShapeExtractor.h"

namespace grasp {

class SgClusterRenderer;
#ifndef CNOID_GE_16
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
class SgVisitShapeGet : public cnoid::SgVisitor
#else
class SgVisitShapeGet : public cnoid::SceneVisitor
#endif
{
	public:
//		SgVisitShapeGet(){	}
		virtual void visitShape(cnoid::SgShape* shape){
			this->shape.push_back(shape);
		}
		//virtual bool visitClusterShape(SgClusterRenderer *clusterShape){
		//	this->clusterShape.push_back(clusterShape);
		//}
		std::vector<cnoid::SgShape*> shape;
		//std::vector<SgClusterRenderer*> clusterShape;
};
#endif

class SgClusterRenderer
#if CNOID_GE_16
	: public cnoid::SgNode
#else
	: public cnoid::SgCustomGLNode
#endif
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef CNOID_GE_16
		SgClusterRenderer(ObjectShape* object) :
			cnoid::SgNode(findPolymorphicId<SgClusterRenderer>()) {
#else
		SgClusterRenderer(ObjectShape* object){
			setRenderingFunction(boost::bind(&SgClusterRenderer::renderCluster, this));
#endif
			this->object = object;
		}

#ifdef CNOID_GE_16
		void renderCluster(cnoid::SceneRenderer* render) {
#else
		void renderCluster(){
#endif
			glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
			glDisable(GL_LIGHTING);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			glBegin(GL_TRIANGLES);

			for(int i=0;i<object->nTriangles;i++){
				int cid = object->triangles[i].idCluster;
				float *c = object->clusterNodes[cid].color;
				glColor3f(c[0],c[1],c[2]);
				grasp::VertexLink** ver = object->triangles[i].ver;
				glVertex3d(ver[0]->pos[0],ver[0]->pos[1],ver[0]->pos[2]);
				glVertex3d(ver[1]->pos[0],ver[1]->pos[1],ver[1]->pos[2]);
				glVertex3d(ver[2]->pos[0],ver[2]->pos[1],ver[2]->pos[2]);
			}

			glEnd();
			glPopAttrib();
		}

#ifndef CNOID_GE_16
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		virtual void accept(cnoid::SgVisitor& visitor){
#else
		virtual void accept(cnoid::SceneVisitor& visitor){
#endif
			cnoid::SgCustomGLNode::accept(visitor);
			SgLastRenderer(this, true);
		}
#endif
		static SgClusterRenderer* SgLastRenderer(SgClusterRenderer* sg, bool isWrite){
			static SgClusterRenderer* last;
			if(isWrite) last=sg;
			return last;
		}

		ObjectShape* object;
};

//tajima SgClusterRendererFunc
class SgClusterRendererFunc
#if CNOID_GE_16
	: public cnoid::SgNode
#else
	: public cnoid::SgCustomGLNode
#endif
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#ifdef CNOID_GE_16
		SgClusterRendererFunc(ObjectShape* object) :
			cnoid::SgNode(findPolymorphicId<SgClusterRendererFunc>()) {
#else
		SgClusterRendererFunc(ObjectShape* object){
			setRenderingFunction(boost::bind(&SgClusterRendererFunc::renderCluster, this));
#endif
			this->object = object;
		}

#ifdef CNOID_GE_16
		void renderCluster(cnoid::SceneRenderer* render) {
#else
		void renderCluster(){
#endif
			glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
			glDisable(GL_LIGHTING);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			glBegin(GL_TRIANGLES);

			for(int i=0;i<object->nTriangles;i++){
				int cid = object->triangles[i].idCluster;
				float *c = object->clusterNodes[cid].color;
				if (object->petid[0]==object->triangles[i].idCluster) {
					c[0] = 1.0;
					c[1] = 0.0;
					c[2] = 0.0;
				}
				if (object->petid[1]==object->triangles[i].idCluster) {
					c[0] = 0.0;
					c[1] = 1.0;
					c[2] = 0.0;
				}
				if (object->cupid==object->triangles[i].idCluster) {
					c[0] = 0.0;
					c[1] = 0.0;
					c[2] = 1.0;
				}
				glColor3f(c[0],c[1],c[2]);
				grasp::VertexLink** ver = object->triangles[i].ver;
				glVertex3d(ver[0]->pos[0],ver[0]->pos[1],ver[0]->pos[2]);
				glVertex3d(ver[1]->pos[0],ver[1]->pos[1],ver[1]->pos[2]);
				glVertex3d(ver[2]->pos[0],ver[2]->pos[1],ver[2]->pos[2]);
			}

			glEnd();
			glPopAttrib();
		}

#ifndef CNOID_GE_16
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		virtual void accept(cnoid::SgVisitor& visitor){
#else
		virtual void accept(cnoid::SceneVisitor& visitor){
#endif
			cnoid::SgCustomGLNode::accept(visitor);
			SgLastRenderer(this, true);
		}
#endif
		static SgClusterRendererFunc* SgLastRenderer(SgClusterRendererFunc* sg, bool isWrite){
			static SgClusterRendererFunc* last;
			if(isWrite) last=sg;
			return last;
		}

		ObjectShape* object;
};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
typedef boost::intrusive_ptr<SgClusterRenderer> SgClusterRendererPtr;
#else
typedef cnoid::ref_ptr<SgClusterRenderer> SgClusterRendererPtr;
#endif

#ifdef CNOID_GE_16
 class SgVisitShapeGet {
 public:
	 void getShapes(cnoid::SgNode* node) {
		 ShapeExtractor extractor;
		 auto& functions = extractor.visitFunctions();
		 functions.setFunction<SgClusterRenderer>
			 ([=](cnoid::SgNode* node) {
				 SgClusterRenderer::SgLastRenderer(static_cast<SgClusterRenderer*>(node), true);
			 });
		 functions.updateDispatchTable();
		 extractor.collect(node, shape);
	 }
	 std::vector<cnoid::SgShape*> shape;
 };
#endif

}

#endif

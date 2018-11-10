// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
 c) Kensuke Harada (AIST)
 */

#ifndef DRAW_UTILITY_H
#define DRAW_UTILITY_H

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Light>
#include <osg/Depth>
#include <osg/LightSource>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/CullFace>
#include <osg/LineWidth>
//#include <cnoid/SceneView>
//#include <cnoid/SceneBodyManager>
//#include <cnoid/SceneObject>
  #ifdef CNOID_ENABLE_OSG
  #include <cnoid/OSGSceneView>
  #include <cnoid/OSGSceneBodyManager>
  #include <cnoid/OSGSceneObject>
  #endif
#include <cnoid/SignalProxy>
#else
#include <iostream>
#include <boost/bind.hpp>
#include <cnoid/SceneShape>
#include <cnoid/SceneView>
#include <cnoid/MeshGenerator>
#include <cnoid/MeshNormalGenerator>
#include <cnoid/GLSceneRenderer>
#include <GL/glew.h>
#include "ColdetConverter.h"
#endif

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <cnoid/RootItem>

#include "VectorMath.h"
#include "exportdef.h"

//#include <BodyPlugin/BodyItem.h>

using namespace cnoid;
/*
namespace cnoid {
    class MessageView;
}
*/
class Cylinders
{
public:
    Cylinders();

    Cylinders(const cnoid::Vector3& pos_, const cnoid::Vector3& dir_, double radius_, double length_, cnoid::Vector3 color_=Vector3(0.5,0.5,0.5),double alpha_ = 1.0){
        this->pos = pos_;
        this->dir = dir_;
        this->radius = radius_;
        this->length = length_;
        this->rgb = color_;
        this->alpha = alpha_;
    }
    cnoid::Vector3 pos, dir, rgb;
    double alpha;
    double radius, length;
};

class Boxes
{
public:
    Boxes();

    Boxes(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_, const cnoid::Vector3& length_, cnoid::Vector3 color_=Vector3(0.5,0.5,0.5),double alpha_ = 1.0){
        this->pos = pos_;
        this->R = R_;
        this->length = length_;
        this->rgb = color_;
        this->alpha = alpha_;
    }

    cnoid::Vector3 pos,length,rgb;
    double alpha;
    cnoid::Matrix3 R;
};

class Spheres
{
public:
    Spheres();

    Spheres(const cnoid::Vector3& pos_, double radius_, cnoid::Vector3 color_=Vector3(0.5,0.5,0.5),double alpha_ = 1.0){
        this->pos = pos_;
        this->radius = radius_;
        this->rgb = color_;
        this->alpha = alpha_;
    }

    cnoid::Vector3 pos, rgb;
    double alpha;
    double radius;
};

class Ellipsoid
{
public:
    Ellipsoid();

    Ellipsoid(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_, const cnoid::Vector3& length_,cnoid::Vector3 color_=Vector3(0.5,0.5,0.5),double alpha_ = 1.0){
        this->pos = pos_;
        this->R = R_;
        this->length = length_;
        this->rgb = color_;
        this->alpha = alpha_;
    }

    cnoid::Vector3 pos, length,rgb;
    double alpha;
    cnoid::Matrix3 R;

};

class Cones
{
public:
    Cones();

    Cones(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_, double radius_, double length_, cnoid::Vector3 color_=Vector3(0.5,0.5,0.5), double alpha_ = 1.0){
        this->pos = pos_;
        this->R = R_;
        this->length = length_;
        this->radius = radius_;
        this->rgb = color_;
        this->alpha = alpha_;
    }

    cnoid::Vector3 pos, rgb;
    double length, radius;
    double alpha;
    cnoid::Matrix3 R;
};

class CoordinateAxis
{
public:
    CoordinateAxis();
    CoordinateAxis(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_, double size_){
        this->pos = pos_;
        this->R = R_;
        this->size = size_;
        this->rgb = Vector3(1,0,0);
    }
    CoordinateAxis(const cnoid::Vector3& pos_, const cnoid::Matrix3& R_, double size_, const cnoid::Vector3& color_){
        this->pos = pos_;
        this->R = R_;
        this->size = size_;
        this->rgb = color_;
    }


    cnoid::Matrix3 R;
    cnoid::Vector3 pos, rgb;
    double size;
};

// #if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#ifdef CNOID_15
class ShapeRenderer : public cnoid::SgCustomGLNode
{
public:
        ShapeRenderer(SgShapePtr shape, Vector3 color) {
            setRenderingFunction(boost::bind(&ShapeRenderer::render, this));
            shape_ = shape;
            color_ = color;
        }

        SgShape* shape() { return shape_.get(); }

        void render() {

            glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            float transparency = shape_->getOrCreateMaterial()->transparency();
            glColor4f(color_[0], color_[1], color_[2], transparency);

            SgMesh* mesh = shape_->mesh();
            SgVertexArray* vertices = mesh->vertices();
            SgColorArray* colorArray = mesh->getOrCreateColors();
            SgIndexArray& indices = mesh->triangleVertices();

            glBegin(GL_TRIANGLES);
            {
                int colorsSize = colorArray->size();
                for(int i = 0; i < mesh->numTriangles(); i++) {
                    Vector3f vec;
                    if (i < colorsSize) {
                        vec = colorArray->at(i);
                        glColor4f(vec[0], vec[1], vec[2], transparency);
                    }
                    for (int j = 0; j < 3; j++) {
                        vec = vertices->at(indices[i * 3 + j]);
                        glVertex3d(vec[0], vec[1], vec[2]);
                    }
                }
            }
            glEnd();
            glPopAttrib();
        }

        virtual void accept(SceneVisitor& visitor){
            cnoid::SgCustomGLNode::accept(visitor);
            LastRenderer(this, true);
        }

        static ShapeRenderer* LastRenderer(ShapeRenderer* sr, bool isWrite){
            static ShapeRenderer* last;
            if (isWrite) last = sr;
            return last;
        }

private:
        SgShapePtr shape_;
        Vector3 color_;
};
typedef ref_ptr<ShapeRenderer> ShapeRendererPtr;

class PointRenderer : public cnoid::SgCustomGLNode
{
public:
        PointRenderer(std::vector<cnoid::Vector3>* points, std::vector<cnoid::Vector3>* colors) {
            setRenderingFunction(boost::bind(&PointRenderer::render, this));
            points_ = points;
            colors_ = colors;
        }

        std::vector<cnoid::Vector3>* points() { return points_; }
        std::vector<cnoid::Vector3>* colors() { return colors_; }

        void render() {

            glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glPointSize(5.0);
            glBegin(GL_POINTS);
            {
                int size = points_->size();
                for(int i = 0; i < size; i++) {
					Vector3 color = colors_->at(i);
					Vector3 point = points_->at(i);
                    glColor3f(color[0], color[1], color[2]);
                    glVertex3d(point[0], point[1], point[2]);
                }
            }
            glEnd();
            glPopAttrib();
        }

        virtual void accept(SceneVisitor& visitor){
            cnoid::SgCustomGLNode::accept(visitor);
            LastRenderer(this, true);
        }

        static PointRenderer* LastRenderer(PointRenderer* sr, bool isWrite){
            static PointRenderer* last;
            if (isWrite) last = sr;
            return last;
        }

private:
        std::vector<cnoid::Vector3>* points_;
        std::vector<cnoid::Vector3>* colors_;
};
typedef ref_ptr<PointRenderer> PointRendererPtr;

class LineRenderer : public cnoid::SgCustomGLNode
{
public:
        LineRenderer(std::vector<CoordinateAxis>* axis, const double alpha, const int width) {
            setRenderingFunction(boost::bind(&LineRenderer::render, this));
            axis_ = axis;
			alpha_ = alpha;
			width_ = width;
        }

        std::vector<CoordinateAxis>* axis() { return axis_; }

        void render() {

            glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glLineWidth(width_);
            glBegin(GL_LINES);
            {
                for(int i = 0; i < axis_->size(); i++) {
					CoordinateAxis axis = axis_->at(i);
                    glColor4f(axis.pos(0), axis.pos(1), axis.pos(2), alpha_);
					for(int j = 0; j < 3; j++){
                    	glVertex3d(axis.rgb(0), axis.rgb(1), axis.rgb(2));
            			Vector3 a = axis.pos + grasp::col(axis.R, j) * axis.size;
                    	glVertex3d(a(0), a(1), a(2));
					}
				}
            }
            glEnd();
            glPopAttrib();
        }

        virtual void accept(SceneVisitor& visitor){
            cnoid::SgCustomGLNode::accept(visitor);
            LastRenderer(this, true);
        }

        static LineRenderer* LastRenderer(LineRenderer* sr, bool isWrite){
            static LineRenderer* last;
            if (isWrite) last = sr;
            return last;
        }

private:
        std::vector<CoordinateAxis>* axis_;
		double alpha_;
		int width_;
};
typedef ref_ptr<LineRenderer> LineRendererPtr;

typedef std::vector<SgCustomGLNode*> RendererList;
#endif

class EXCADE_API DrawUtility
{
public:

    static DrawUtility* instance();
    DrawUtility();
    virtual ~DrawUtility();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    // for back compatibility
    bool displayPoints();
    bool displayLines();
    bool displayCoordinateAxes();
    bool displayCylinders();
    bool displayBoxes();
    bool displaySpheres();
    bool displayEllipsoids();
    bool displayShapes();
    bool displayTriangles(double alpha=1.0);
    // for the plotplugin extension
    bool displayPoints(cnoid::OSGSceneObjectPtr& plothandle);
    bool displayLines(double alpha, cnoid::OSGSceneObjectPtr& plothandle,
                      const int linewidth = 5);
    bool displayCoordinateAxes(cnoid::OSGSceneObjectPtr& plothandle);
    bool displayCylinders(cnoid::OSGSceneObjectPtr& plothandle);
    bool displayBoxes(cnoid::OSGSceneObjectPtr& plothandle);
    bool displaySpheres(cnoid::OSGSceneObjectPtr& plothandle);
    bool displayEllipsoids(cnoid::OSGSceneObjectPtr& plothandle);
    bool displayShapes(cnoid::OSGSceneObjectPtr& plothandle);
    bool displayTriangles(double alpha,
                          cnoid::OSGSceneObjectPtr& plothandle);
    void deleteanobj(const cnoid::OSGSceneObjectPtr& plothandle);
#else
	bool displayPoints();
	bool displayPoints(cnoid::SgGroupPtr& node);
	bool addPointNodes(cnoid::SgGroupPtr node);
	bool displayLines(const double alpha = 1.0, const double lineWdith = 1.0);
	bool displayLines(cnoid::SgGroupPtr& node, const double alpha = 1.0, const double lineWdith = 1.0);
	bool addLineNodes(cnoid::SgGroupPtr node, const double alpha, const double lineWdith);
    bool displayCoordinateAxes(const double alpha = 1.0, const double lineWidth = 1.0);
	bool displayCoordinateAxes(cnoid::SgGroupPtr& node, const double alpha = 1.0, const double lineWidth = 1.0);
	bool addCoordinateAxeNodes(cnoid::SgGroupPtr node, const double alpha, double lineWidth);
	bool displayCylinders();
	bool displayCylinders(cnoid::SgGroupPtr& node);
	bool addCylinderNodes(cnoid::SgGroupPtr node);
	bool displayBoxes();
	bool displayBoxes(cnoid::SgGroupPtr& node);
	bool addBoxNodes(cnoid::SgGroupPtr node);
	bool displaySpheres();
    bool displaySpheres(cnoid::SgGroupPtr& node);
	bool addSphereNodes(cnoid::SgGroupPtr node);
	bool displayEllipsoids();
    bool displayEllipsoids(cnoid::SgGroupPtr& node);
	bool addEllipsoidNodes(cnoid::SgGroupPtr node);
	bool displayShapes();
    bool displayShapes(cnoid::SgGroupPtr& node);
    bool displayTriangles(const double alpha = 1.0);
	bool displayTriangles(const double alpha, cnoid::SgGroupPtr& node);
	bool addTriangleNode(cnoid::SgGroupPtr node, const double alpha);
	bool displayCones();
	bool displayCones(cnoid::SgGroupPtr& node);
	bool addConeNodes(cnoid::SgGroupPtr node);
	void deleteanobj(cnoid::SgGroupPtr plothandle);
#endif
    void clear();
    void redraw();

    cnoid::Vector3 rgb;
    std::vector<Cylinders> cylinders;
    std::vector<Boxes> boxes;
    std::vector<Spheres> spheres;
    std::vector<Ellipsoid> ellipsoids;
    std::vector<Cones> cones;
    std::vector<cnoid::Vector3> points, colors;
    std::vector<std::vector<int> > triangles;
    std::vector<CoordinateAxis> axes;

    //obj list for plotting
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    std::vector<cnoid::OSGSceneObjectPtr> objList;
#endif

private:
    std::ostream& os;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    BodyItem kinect;
    BodyItem cube;

    osg::ref_ptr<osg::Geode> pclNode;
    osg::Vec3Array * curPoint;
    osg::Vec4Array * curColor;
    osg::Geometry* geom;

    cnoid::OSGSceneView * viewer;
    std::vector<osg::ShapeDrawable*> shape;
    std::vector<osg::Cylinder*> cylinder;
    std::vector<osg::Shape*> shapes;
    cnoid::OSGSceneObjectPtr ellipsoid_obj;
#else
	std::vector<cnoid::SgGroupPtr> sgnodes_;
#endif
};

#endif

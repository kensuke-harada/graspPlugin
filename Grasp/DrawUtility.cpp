// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Kensuke Harada (AIST)
*/

#include <math.h>

#include <limits>

#include <QtGui>

#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Archive>
//#include "../GeometryHandler/GeometryHandle.h"
#include "DrawUtility.h"

//void write_vrml_data(char *file,grasp::ObjectShape *wo);

using namespace std;
using namespace boost;
using namespace cnoid;

DrawUtility* DrawUtility::instance()
{
    static DrawUtility* instance = new DrawUtility();
    return instance;
}

DrawUtility::DrawUtility() : os (MessageView::mainInstance()->cout() )
{
    rgb = Vector3(1,0,0);
    geom = NULL;
    viewer = NULL;
    ellipsoid_obj = NULL;
}


DrawUtility::~DrawUtility()
{
}

/**
 * @brief DrawUtility::displayCylinders
 * @param plothandle if plothandle != -1,
 *          sthe hanle of the plotted object will be set to it
 * @return
 */
bool DrawUtility::displayCylinders(cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = cylinders.size();
    shape.resize(size);
    cylinder.resize(size);

    for(int i=0; i<size; i++){

            Vector3 c = cylinders[i].pos;
            Vector3 d = cylinders[i].dir;
            double r = cylinders[i].radius;
            double l = cylinders[i].length;

            Vector3 ax = grasp::unit(Vector3(0,0,1).cross(d));
            float th = acos(Vector3(0,0,1).dot(d));

            cylinder[i] = new osg::Cylinder(osg::Vec3(c(0),c(1),c(2)), r, l );
            cylinder[i]->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shape[i] = new osg::ShapeDrawable(cylinder[i]);

            shape[i]->setColor(osg::Vec4(0.7, 0.7, 0.7, 1));
            pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displayCylinders()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = cylinders.size();
    shape.resize(size);
    cylinder.resize(size);

    for(int i=0; i<size; i++){

            Vector3 c = cylinders[i].pos;
            Vector3 d = cylinders[i].dir;
            double r = cylinders[i].radius;
            double l = cylinders[i].length;

            Vector3 ax = grasp::unit(Vector3(0,0,1).cross(d));
            float th = acos(Vector3(0,0,1).dot(d));

            cylinder[i] = new osg::Cylinder(osg::Vec3(c(0),c(1),c(2)), r, l );
            cylinder[i]->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shape[i] = new osg::ShapeDrawable(cylinder[i]);

            shape[i]->setColor(osg::Vec4(0.7, 0.7, 0.7, 1));
            pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

bool DrawUtility::displayBoxes(cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = boxes.size();
    shape.resize(size);
    shapes.resize(size);

    for(int i=0; i<boxes.size(); i++){

            Vector3 c = boxes[i].pos;
            Vector3 l = boxes[i].length;
            Vector3 color = boxes[i].rgb;
            Matrix3 R = boxes[i].R;

            Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
            float th = acos((R.trace()-1)/2);

            osg::Box* box = new osg::Box(osg::Vec3(c(0),c(1),c(2)), l(0),l(1),l(2) );
            box->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[i] = box;
            shape[i] = new osg::ShapeDrawable(shapes[i]);

            shape[i]->setColor(osg::Vec4(color(0), color(1), color(2), 0.1));
            pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displayBoxes()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = boxes.size();
    shape.resize(size);
    shapes.resize(size);

    for(int i=0; i<boxes.size(); i++){

            Vector3 c = boxes[i].pos;
            Vector3 l = boxes[i].length;
            Vector3 color = boxes[i].rgb;
            Matrix3 R = boxes[i].R;

            Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
            float th = acos((R.trace()-1)/2);

            osg::Box* box = new osg::Box(osg::Vec3(c(0),c(1),c(2)), l(0),l(1),l(2) );
            box->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[i] = box;
            shape[i] = new osg::ShapeDrawable(shapes[i]);

            shape[i]->setColor(osg::Vec4(color(0), color(1), color(2), 0.1));
            pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

bool DrawUtility::displaySpheres(cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = spheres.size();
    shape.resize(size);
    shapes.resize(size);


    for(int i=0; i<spheres.size(); i++){

            Vector3 c = spheres[i].pos;
            double r = spheres[i].radius;
            Vector3 color = spheres[i].rgb;
            double a = spheres[i].alpha;

            shapes[i] = new osg::Sphere(osg::Vec3(c(0),c(1),c(2)), r);
            shape[i] = new osg::ShapeDrawable(shapes[i]);

            shape[i]->setColor(osg::Vec4(color(0), color(1), color(2), a));
            pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displaySpheres()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = spheres.size();
    shape.resize(size);
    shapes.resize(size);


    for(int i=0; i<spheres.size(); i++){

            Vector3 c = spheres[i].pos;
            double r = spheres[i].radius;
            Vector3 color = spheres[i].rgb;
            double a = spheres[i].alpha;

            shapes[i] = new osg::Sphere(osg::Vec3(c(0),c(1),c(2)), r );
            shape[i] = new osg::ShapeDrawable(shapes[i]);

            shape[i]->setColor(osg::Vec4(color(0), color(1), color(2), a));
            pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

bool DrawUtility::displayEllipsoids(cnoid::OSGSceneObjectPtr& plothandle)
{
    viewer = cnoid::OSGSceneView::mainInstance();

    if(ellipsoid_obj != NULL){
        ellipsoid_obj->removeChildren(0,ellipsoid_obj->getNumChildren());
    }else{
        ellipsoid_obj = new cnoid::OSGSceneObject();
        viewer->addSceneObject(ellipsoid_obj);
    }

    for(int i=0; i<ellipsoids.size(); i++){
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
        osg::StateSet* state = geode->getOrCreateStateSet();
        osg::CullFace* cull = new osg::CullFace;
        cull->setMode(osg::CullFace::BACK);
        state->setAttributeAndModes(cull, osg::StateAttribute::ON);
        state->setRenderBinDetails(11, "RenderBin");
        state->setMode(GL_BLEND, osg::StateAttribute::ON);

        Vector3 c = ellipsoids[i].pos;
        Vector3 l = ellipsoids[i].length;
        Vector3 color = ellipsoids[i].rgb;
        Matrix3 R = ellipsoids[i].R;
        double a = ellipsoids[i].alpha;

        Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
        float th = acos((R.trace()-1)/2);

        osg::ref_ptr<osg::PositionAttitudeTransform> pat
            = new osg::PositionAttitudeTransform;
        pat->setPosition(osg::Vec3(c(0), c(1), c(2)));
        pat->setScale(osg::Vec3(l(0), l(1), l(2)));
        pat->setAttitude(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
        osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0,0,0), 1 );
        osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sphere);

        sd->setColor(osg::Vec4(color(0), color(1), color(2), a));
        geode->addDrawable(sd);
        pat->addChild(geode);
        ellipsoid_obj->addChild(pat);
    }
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = ellipsoid_obj;

    return true;
}

bool DrawUtility::displayEllipsoids()
{
    viewer = cnoid::OSGSceneView::mainInstance();

    if(ellipsoid_obj != NULL){
        ellipsoid_obj->removeChildren(0,ellipsoid_obj->getNumChildren());
    }else{
        ellipsoid_obj = new cnoid::OSGSceneObject();
        viewer->addSceneObject(ellipsoid_obj);
    }

    for(int i=0; i<ellipsoids.size(); i++){
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
        osg::StateSet* state = geode->getOrCreateStateSet();
        osg::CullFace* cull = new osg::CullFace;
        cull->setMode(osg::CullFace::BACK);
        state->setAttributeAndModes(cull, osg::StateAttribute::ON);
        state->setRenderBinDetails(11, "RenderBin");
        state->setMode(GL_BLEND, osg::StateAttribute::ON);

        Vector3 c = ellipsoids[i].pos;
        Vector3 l = ellipsoids[i].length;
        Vector3 color = ellipsoids[i].rgb;
        Matrix3 R = ellipsoids[i].R;
        double a = ellipsoids[i].alpha;

        Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
        float th = acos((R.trace()-1)/2);

        osg::ref_ptr<osg::PositionAttitudeTransform> pat
            = new osg::PositionAttitudeTransform;
        pat->setPosition(osg::Vec3(c(0), c(1), c(2)));
        pat->setScale(osg::Vec3(l(0), l(1), l(2)));
        pat->setAttitude(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
        osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0,0,0), 1 );
        osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sphere);

        sd->setColor(osg::Vec4(color(0), color(1), color(2), a));
        geode->addDrawable(sd);
        pat->addChild(geode);
        ellipsoid_obj->addChild(pat);
    }
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

bool DrawUtility::displayShapes(cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = cylinders.size() + boxes.size() + spheres.size() + cones.size();
    shape.resize(size);
    shapes.resize(size);

    int k=0;
    for(int i=0; i< cylinders.size(); i++,k++){
            Vector3 c = cylinders[i].pos;
            Vector3 d = cylinders[i].dir;
            double r = cylinders[i].radius;
            double l = cylinders[i].length;
            Vector3 color = cylinders[i].rgb;
            double alpha = cylinders[i].alpha;

            Vector3 ax = grasp::unit(Vector3(0,0,1).cross(d));
            float th = acos(Vector3(0,0,1).dot(d));

            osg::Cylinder* cyl = new osg::Cylinder(osg::Vec3(c(0),c(1),c(2)), r, l );
            cyl->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[k] = cyl;
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }

    for(int i=0; i<boxes.size(); i++,k++){

            Vector3 c = boxes[i].pos;
            Vector3 l = boxes[i].length;
            Vector3 color = boxes[i].rgb;
            double alpha = boxes[i].alpha;
            Matrix3 R = boxes[i].R;

            Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
            float t = (R.trace()-1)/2;
            float eps = std::numeric_limits<float>::epsilon();
            if (t >= 1) t = 1 - eps ; if (t <= -1) t = -1 + eps;
            float th = acos(t);

            osg::Box* box = new osg::Box(osg::Vec3(c(0),c(1),c(2)), l(0),l(1),l(2) );
            box->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[k] = box;
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }

    for(int i=0; i<spheres.size(); i++,k++){

            Vector3 c = spheres[i].pos;
            double r = spheres[i].radius;
            Vector3 color = spheres[i].rgb;
            double alpha = spheres[i].alpha;

            shapes[k] = new osg::Sphere(osg::Vec3(c(0),c(1),c(2)), r );
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }

    for(int i=0; i<cones.size(); i++,k++){
            Vector3 c = cones[i].pos;
            double r = cones[i].radius;
            double l = cones[i].length;
            Vector3 color = cones[i].rgb;
            double alpha = cones[i].alpha;
            Matrix3 R = cones[i].R;

            Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
            float t = (R.trace()-1)/2;
            float eps = std::numeric_limits<float>::epsilon();
            if (t >= 1) t = 1 - eps ; if (t <= -1) t = -1 + eps;
            float th = acos(t);

            osg::Cone* cone = new osg::Cone(osg::Vec3(c(0),c(1),c(2)),r,l);
            cone->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[k] = cone;
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }
    obj->addChild(pclNode);

    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displayShapes()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    int size = cylinders.size() + boxes.size() + spheres.size() + cones.size();
    shape.resize(size);
    shapes.resize(size);

    int k=0;
    for(int i=0; i< cylinders.size(); i++,k++){
            Vector3 c = cylinders[i].pos;
            Vector3 d = cylinders[i].dir;
            double r = cylinders[i].radius;
            double l = cylinders[i].length;
            Vector3 color = cylinders[i].rgb;
            double alpha = cylinders[i].alpha;

            Vector3 ax = grasp::unit(Vector3(0,0,1).cross(d));
            float th = acos(Vector3(0,0,1).dot(d));

            osg::Cylinder* cyl = new osg::Cylinder(osg::Vec3(c(0),c(1),c(2)), r, l );
            cyl->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[k] = cyl;
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }

    for(int i=0; i<boxes.size(); i++,k++){

            Vector3 c = boxes[i].pos;
            Vector3 l = boxes[i].length;
            Vector3 color = boxes[i].rgb;
            double alpha = boxes[i].alpha;
            Matrix3 R = boxes[i].R;

            Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
            float t = (R.trace()-1)/2;
            float eps = std::numeric_limits<float>::epsilon();
            if (t >= 1) t = 1 - eps ; if (t <= -1) t = -1 + eps;
            float th = acos(t);

            osg::Box* box = new osg::Box(osg::Vec3(c(0),c(1),c(2)), l(0),l(1),l(2) );
            box->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[k] = box;
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }

    for(int i=0; i<spheres.size(); i++,k++){

            Vector3 c = spheres[i].pos;
            double r = spheres[i].radius;
            Vector3 color = spheres[i].rgb;
            double alpha = spheres[i].alpha;

            shapes[k] = new osg::Sphere(osg::Vec3(c(0),c(1),c(2)), r );
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }

    for(int i=0; i<cones.size(); i++,k++){
            Vector3 c = cones[i].pos;
            double r = cones[i].radius;
            double l = cones[i].length;
            Vector3 color = cones[i].rgb;
            double alpha = cones[i].alpha;
            Matrix3 R = cones[i].R;

            Vector3 ax = grasp::unit(Vector3(R(2,1)-R(1,2),R(0,2)-R(2,0),R(1,0)-R(0,1)));
            float t = (R.trace()-1)/2;
            float eps = std::numeric_limits<float>::epsilon();
            if (t >= 1) t = 1 - eps ; if (t <= -1) t = -1 + eps;
            float th = acos(t);

            osg::Cone* cone = new osg::Cone(osg::Vec3(c(0),c(1),c(2)),r,l);
            cone->setRotation(osg::Quat(th, osg::Vec3(ax(0),ax(1),ax(2))));
            shapes[k] = cone;
            shape[k] = new osg::ShapeDrawable(shapes[k]);

            shape[k]->setColor(osg::Vec4(color(0), color(1), color(2), alpha));
            pclNode->addDrawable(shape[k]);
    }
    obj->addChild(pclNode);

    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

bool DrawUtility::displayPoints(cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

    osg::StateSet* state = pclNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0){
        pclNode->removeDrawables(0, prevNumDrawables);
    }

    int size = points.size();
    os << "Points: " << size << endl;

    curPoint = new osg::Vec3Array;
    curColor = new osg::Vec4Array;

    int i = 0;
    geom = new osg::Geometry;
    //geom->removePrimitiveSet(0);

    for (size_t i=0; i<points.size(); i++) {
        osg::Vec3 v3(points[i](0), points[i](1), points[i](2));
        curPoint->push_back(v3);
    }

    for (size_t i=0; i<colors.size(); i++) {
        osg::Vec4 v4(colors[i](0), colors[i](1), colors[i](2), 1.0);
        curColor->push_back(v4);
    }

    int cloud_size = curPoint->size();

    geom->setVertexArray(curPoint);
    if(colors.size()>0){
        geom->setColorArray(curColor);
        geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    pclNode->addDrawable(geom);

    viewer = cnoid::OSGSceneView::mainInstance();
    obj->addChild(pclNode);
    viewer->addSceneObject(obj);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cloud_size));
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displayPoints()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

    osg::StateSet* state = pclNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0){
        pclNode->removeDrawables(0, prevNumDrawables);
    }

    int size = points.size();
    os << "Points: " << size << endl;

    curPoint = new osg::Vec3Array;
    curColor = new osg::Vec4Array;

    int i = 0;
    geom = new osg::Geometry;
    //geom->removePrimitiveSet(0);

    for (size_t i=0; i<points.size(); i++) {
        osg::Vec3 v3(points[i](0), points[i](1), points[i](2));
        curPoint->push_back(v3);
    }

    for (size_t i=0; i<colors.size(); i++) {
        osg::Vec4 v4(colors[i](0), colors[i](1), colors[i](2), 1.0);
        curColor->push_back(v4);
    }

    int cloud_size = curPoint->size();

    geom->setVertexArray(curPoint);
    if(colors.size()>0){
        geom->setColorArray(curColor);
        geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    pclNode->addDrawable(geom);

    viewer = cnoid::OSGSceneView::mainInstance();
    obj->addChild(pclNode);
    viewer->addSceneObject(obj);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, cloud_size));
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

//bool DrawUtility::displayLines(double alpha, cnoid::OSGSceneObjectPtr& plothandle)
//{
//    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
//    pclNode = new osg::Geode;
//    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

//    osg::StateSet* state = pclNode->getOrCreateStateSet();
//    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
//    state->setAttributeAndModes(new osg::LineWidth(5), osg::StateAttribute::ON);

//    bool prevNumDrawables = pclNode->getNumDrawables();
//    if(prevNumDrawables > 0)
//        pclNode->removeDrawables(0, prevNumDrawables);

//    curPoint = new osg::Vec3Array;
//    curColor = new osg::Vec4Array;

//    geom = new osg::Geometry;
//    //geom->removePrimitiveSet(0);

//    for (size_t i=0; i<points.size(); i++) {
//        osg::Vec3 v3(points[i](0), points[i](1), points[i](2));
//        curPoint->push_back(v3);
//    }

//    for (size_t i=0; i<colors.size(); i++) {
//        osg::Vec4 v4(colors[i](0), colors[i](1), colors[i](2), alpha);
//        curColor->push_back(v4);
//    }

//    geom->setVertexArray(curPoint);
//    geom->setColorArray(curColor);
//    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
//    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0,
//                                              curPoint->size()));
//    pclNode->addDrawable(geom);
//    viewer = cnoid::OSGSceneView::mainInstance();
//    obj->addChild(pclNode);
//    viewer->addSceneObject(obj);
//    viewer->requestRedraw();
////    MessageView::mainInstance()->flush();

//    plothandle = obj;
//    return true;
//}

bool DrawUtility::displayLines()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

    osg::StateSet* state = pclNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    curPoint = new osg::Vec3Array;
    curColor = new osg::Vec4Array;

    int i = 0;
    geom = new osg::Geometry;
    //geom->removePrimitiveSet(0);

    for (size_t i=0; i<axes.size(); i++) {

        osg::Vec3 v30(axes[i].pos(0), axes[i].pos(1), axes[i].pos(2));
        osg::Vec4 v4(axes[i].rgb(0), axes[i].rgb(1), axes[i].rgb(2), 1.0);

        for(int j=0; j<3; j++){
            Vector3 a = axes[i].pos + grasp::col(axes[i].R, j)*axes[i].size;
            osg::Vec3 v31(a(0), a(1), a(2));
            curPoint->push_back(v30);
            curColor->push_back(v4);
            curPoint->push_back(v31);
            curColor->push_back(v4);
        }
    }

    for (size_t i=0; i<points.size(); i++) {
        osg::Vec3 v3(points[i](0), points[i](1), points[i](2));
        curPoint->push_back(v3);
    }

    for (size_t i=0; i<colors.size(); i++) {
        osg::Vec4 v4(colors[i](0), colors[i](1), colors[i](2), 1.0);
        curColor->push_back(v4);
    }

    geom->setVertexArray(curPoint);
    geom->setColorArray(curColor);
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    pclNode->addDrawable(geom);

    viewer = cnoid::OSGSceneView::mainInstance();
    obj->addChild(pclNode);
    viewer->addSceneObject(obj);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, curPoint->size()));
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

bool DrawUtility::displayCoordinateAxes(cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

    osg::StateSet* state = pclNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    curPoint = new osg::Vec3Array;
    curColor = new osg::Vec4Array;

    int i = 0;
    geom = new osg::Geometry;
    //geom->removePrimitiveSet(0);

    for (size_t i=0; i<axes.size(); i++) {

        osg::Vec3 v30(axes[i].pos(0), axes[i].pos(1), axes[i].pos(2));
        osg::Vec4 v4(axes[i].rgb(0), axes[i].rgb(1), axes[i].rgb(2), 1.0);

        for(int j=0; j<3; j++){
            Vector3 a = axes[i].pos + grasp::col(axes[i].R, j)*axes[i].size;
            osg::Vec3 v31(a(0), a(1), a(2));
            curPoint->push_back(v30);
            curColor->push_back(v4);
            curPoint->push_back(v31);
            curColor->push_back(v4);
        }
    }

    geom->setVertexArray(curPoint);
    geom->setColorArray(curColor);
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    pclNode->addDrawable(geom);

    viewer = cnoid::OSGSceneView::mainInstance();
    obj->addChild(pclNode);
    viewer->addSceneObject(obj);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, curPoint->size()));
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displayCoordinateAxes()
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

    osg::StateSet* state = pclNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    curPoint = new osg::Vec3Array;
    curColor = new osg::Vec4Array;

    int i = 0;
    geom = new osg::Geometry;
    //geom->removePrimitiveSet(0);

    for (size_t i=0; i<axes.size(); i++) {

        osg::Vec3 v30(axes[i].pos(0), axes[i].pos(1), axes[i].pos(2));
        osg::Vec4 v4(axes[i].rgb(0), axes[i].rgb(1), axes[i].rgb(2), 1.0);

        for(int j=0; j<3; j++){
            Vector3 a = axes[i].pos + grasp::col(axes[i].R, j)*axes[i].size;
            osg::Vec3 v31(a(0), a(1), a(2));
            curPoint->push_back(v30);
            curColor->push_back(v4);
            curPoint->push_back(v31);
            curColor->push_back(v4);
        }
    }

    geom->setVertexArray(curPoint);
    geom->setColorArray(curColor);
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    pclNode->addDrawable(geom);

    viewer = cnoid::OSGSceneView::mainInstance();
    obj->addChild(pclNode);
    viewer->addSceneObject(obj);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, curPoint->size()));
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}

//bool DrawUtility::displayPolygon(){
//
//	os << "cloud_xyz: " << points.size() << endl;
//	os << "cloud_xyz: " << triangles.size() << endl;
//
//	ColdetModelPtr coldetModel(new ColdetModel());
//
//	coldetModel->setNumVertices(points.size());
//	coldetModel->setNumTriangles(triangles.size());
//
//	int vertexIndex = 0;
//	int triangleIndex = 0;
//
//	vector<double> vertex;
//	vector<int> crd;
//
//
//	for(size_t i=0; i<points.size(); i++){
//
//		Vector3 v2 = points[i];
//		coldetModel->setVertex(vertexIndex++, (float)v2[0], (float)v2[1],(float)v2[2]);
//		vertex.push_back(v2[0]);
//		vertex.push_back(v2[1]);
//		vertex.push_back(v2[2]);
//	}
//
//	for (size_t i=0; i<triangles.size(); ++i) {
//		vector<int> idx;
//		for (size_t j=0; j<3; ++j) {
//			idx.push_back( triangles[i][j] );
//		}
//		coldetModel->setTriangle(triangleIndex++, idx[0], idx[1], idx[2] );
//		crd.push_back(idx[0]);
//		crd.push_back(idx[1]);
//		crd.push_back(idx[2]);
//		crd.push_back(-1);
//	}
//
//	coldetModel->setName("Env");
//	coldetModel->build();
//
//	grasp::ObjectShape shape(vertex, crd);
//	string wrlfile = "extplugin/graspPlugin/Draw/models/coldetmodel.wrl";
//	write_vrml_data((char*)wrlfile.c_str(), &shape);
//	string wrlhrpfile = "extplugin/graspPlugin/Draw/models/coldetmodelhrp.wrl";
//	BodyItemPtr item(new BodyItem());
//    if(item->load(wrlhrpfile, "OpenHRP-VRML-MODEL")) {
//        RootItem::mainInstance()->addChildItem(item);
//    	ItemTreeView::mainInstance()->checkItem(item, true);
//    }
//		return true;
//}

bool DrawUtility::displayTriangles(double alpha,
                                   cnoid::OSGSceneObjectPtr& plothandle)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    shape.resize(triangles.size());
    shapes.resize(triangles.size());

    osg::Vec3Array* vertices = new osg::Vec3Array;
    for(int i=0;i<points.size();i++){
        vertices->push_back(osg::Vec3(points[i][0],points[i][1],points[i][2]));
    }

    for(int i=0;i<triangles.size();i++){
        osg::TriangleMesh* tri = new osg::TriangleMesh();
        tri->setVertices(vertices);
        osg::TemplateIndexArray<unsigned int,osg::Array::UIntArrayType,1,1>* indeces = new osg::TemplateIndexArray<unsigned int,osg::Array::UIntArrayType,1,1>();
        for(int j=0;j<triangles[i].size();j++){
            indeces->push_back(triangles[i][j]);
        }
        tri->setIndices(indeces);

        shapes[i] = tri;
        shape[i] = new osg::ShapeDrawable(shapes[i]);
        shape[i]->setColor(osg::Vec4(colors[i][0],colors[i][1],colors[i][2],alpha));
        pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;

    return true;
}

bool DrawUtility::displayTriangles(double alpha)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC
    osg::StateSet* state = pclNode->getOrCreateStateSet();
    osg::CullFace* cull = new osg::CullFace;
    cull->setMode(osg::CullFace::BACK);
    state->setAttributeAndModes(cull, osg::StateAttribute::ON);
    state->setRenderBinDetails(11, "RenderBin");
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    shape.resize(triangles.size());
    shapes.resize(triangles.size());

    osg::Vec3Array* vertices = new osg::Vec3Array;
    for(int i=0;i<points.size();i++){
        vertices->push_back(osg::Vec3(points[i][0],points[i][1],points[i][2]));
    }

    for(int i=0;i<triangles.size();i++){
        osg::TriangleMesh* tri = new osg::TriangleMesh();
        tri->setVertices(vertices);
        osg::TemplateIndexArray<unsigned int,osg::Array::UIntArrayType,1,1>*
                indeces = new osg::TemplateIndexArray<unsigned int,
                osg::Array::UIntArrayType,1,1>();
        for(int j=0;j<triangles[i].size();j++){
            indeces->push_back(triangles[i][j]);
        }
        tri->setIndices(indeces);

        shapes[i] = tri;
        shape[i] = new osg::ShapeDrawable(shapes[i]);
        shape[i]->setColor(osg::Vec4(colors[i][0],colors[i][1],colors[i][2],alpha));
        pclNode->addDrawable(shape[i]);
    }
    obj->addChild(pclNode);
    viewer = cnoid::OSGSceneView::mainInstance();
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    return true;
}


void DrawUtility::clear()
{
    os <<  "Clear" << endl;

    if(geom != NULL)
        geom->removePrimitiveSet(0);
    for(size_t i=0; i<shape.size(); i++)
        pclNode->removeDrawable(shape[i]);
//    for(vector<cnoid::OSGSceneObjectPtr>::iterator it = objList.begin();
//        it != objList.end(); it++)
//    {
//        viewer->removeSceneObject(*it);
//    }
    if(viewer != NULL) {
        viewer->requestRedraw();
//        MessageView::mainInstance()->flush();
    }

    points.clear();
    colors.clear();
    cylinders.clear();
    boxes.clear();
    ellipsoids.clear();
    cones.clear();
    triangles.clear();
    axes.clear();
}

/**
 * @brief DrawUtility::deleteanobj remove the obj from the objList, if redraw
 *          is called, the object will not display in the view (deleted).
 *          However, if redraw is not called, the object remains there
 * @param obj a pointer to the OSGSceneObject that you would like to remove
 */
void DrawUtility::deleteanobj(const cnoid::OSGSceneObjectPtr& plothandle)
{
    viewer->removeSceneObject(plothandle);
    redraw();
}

/**
 * @brief DrawUtility::redraw replots the OSGSceneObjectPtrs in the objList
 */
void DrawUtility::redraw()
{
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();
}

bool DrawUtility::displayLines(double alpha,
                               cnoid::OSGSceneObjectPtr &plothandle,
                               const int linewidth)
{
    cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();
    pclNode = new osg::Geode;
    pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

    osg::StateSet* state = pclNode->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    state->setAttributeAndModes(new osg::LineWidth(linewidth),
                                osg::StateAttribute::ON);

    bool prevNumDrawables = pclNode->getNumDrawables();
    if(prevNumDrawables > 0)
        pclNode->removeDrawables(0, prevNumDrawables);

    curPoint = new osg::Vec3Array;
    curColor = new osg::Vec4Array;

    geom = new osg::Geometry;
    //geom->removePrimitiveSet(0);

    for (size_t i=0; i<points.size(); i++) {
        osg::Vec3 v3(points[i](0), points[i](1), points[i](2));
        curPoint->push_back(v3);
    }

    for (size_t i=0; i<colors.size(); i++) {
        osg::Vec4 v4(colors[i](0), colors[i](1), colors[i](2), alpha);
        curColor->push_back(v4);
    }

    geom->setVertexArray(curPoint);
    geom->setColorArray(curColor);
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0,
                                              curPoint->size()));
    pclNode->addDrawable(geom);
    viewer = cnoid::OSGSceneView::mainInstance();
    obj->addChild(pclNode);
    viewer->addSceneObject(obj);
    viewer->requestRedraw();
//    MessageView::mainInstance()->flush();

    plothandle = obj;
    return true;
}

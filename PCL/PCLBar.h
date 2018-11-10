// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
 c) Kensuke Harada (AIST)
 */

#ifndef PCLBAR_H
#define PCLBAR_H

#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
//#include <cnoid/SceneView>
//#include <cnoid/SceneBodyManager>
//#include <cnoid/SceneObject>

#include <cnoid/RootItem>
#include <BodyPlugin/BodyItem.h>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneView>
#include <cnoid/OSGSceneBodyManager>
#include <cnoid/OSGSceneObject>
#endif
#include <cnoid/SignalProxy>
#endif

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(<, 1, 7, 0) && EIGEN_VERSION_AT_LEAST(3,2,0)
#include "EigenInternalMath.h"
#endif
#include <pcl/point_types.h>
#include "PointCloudHandler.h"
#include "../Grasp/PlanBase.h"

using namespace cnoid;

namespace cnoid {
    class MessageView;
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
class PCLBar : public cnoid::ToolBar, public boost::signals::trackable
#else
class PCLBar : public cnoid::ToolBar
#endif
{
public:

    static PCLBar* instance();
    virtual ~PCLBar();

private:

    PCLBar();
    void onRGB();
    void onCapture();
    void onSegment();
    void onTriangle();
    void onCylinder();
    void onCalibrate();
    void onClear();
    void onCube();
    void onEst();
    void onEstNoDiag();
    void onGrabSwitch();
    void onCapture2();
    void onReadEnv();
    void onTriangle2();
    void onGendes();
    void onFitting();
    void onReloadCM();
	void onCalibCap();
	void onCalibration();
/*
    BodyItem kinect;
    BodyItem cube;
    osg::ref_ptr<osg::Geode> pclNode;
    osg::Vec3Array * curPoint;
    osg::Vec4Array * curColor;
    osg::Geometry* geom;
    */
    MessageView& mes;
    std::ostream& os;
    bool rgbmode;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp;
    cnoid::BodyItemPtr envItem;

#ifdef ENABLE_OSG
    cnoid::OSGSceneView * viewer;
#endif
    //std::vector<osg::ShapeDrawable*> shape;
    //std::vector<osg::Cylinder*> cylinder;

    bool capture();
    bool segmentate(int mode);
    bool displayPointCloud();
    bool displayApproachVec();
    bool displayCylinder();
    bool displayPolygon();
    void addCube(float x, float y, float z, BodyItemPtr item);
	void setTransMatrix();


    class Comparator
    {
    public:
        bool operator()(const cnoid::Vector3& v1, const cnoid::Vector3& v2) {
            return v1(2) < v2(2);
        }
    };

    class Calibrator
    {
    public:
        Calibrator(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
        {
            this->xmin = xmin;
            this->xmax = xmax;
            this->ymin = ymin;
            this->ymax = ymax;
            this->zmin = zmin;
            this->zmax = zmax;
        }

        bool operator()( const cnoid::Vector3 & v3 ) const
        {
            return	xmin > v3(0) || v3(0) > xmax ||
                ymin > v3(1) || v3(1) > ymax ||
                zmin > v3(2) || v3(2) > zmax;
        }

    private:
        float xmin, xmax, ymin, ymax, zmin, zmax;
    };

};

#endif

// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef _PCHANDLER_H
#define _PCHANDLER_H

// #define USE_OPENNI2

#include <iostream>
#include <boost/filesystem.hpp>
#include <cnoid/MessageView>
//#include <cnoid/SceneBody>
//#include <cnoid/SceneView>
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>
#include <cnoid/OSGSceneView>
#endif // 20140222
#else
#include <cnoid/ColdetModel>
#endif

#include <Eigen/Core>
#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(< , 1, 7, 0) && EIGEN_VERSION_AT_LEAST(3, 2, 0)
#include "EigenInternalMath.h"
#endif
#if !(PCL_VERSION_COMPARE(<, 1, 8, 0))
#define USE_OPENNI2
#endif
#ifndef USE_OPENNI2
#include <pcl/io/openni_grabber.h>
#else
#include <pcl/io/openni2_grabber.h>
#endif
//#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/range_image/range_image.h>
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include <pcl/io/openni_camera/openni_depth_image.h>
#endif

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/passthrough.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/octree/octree.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#ifndef PCL_REVISION_VER
#define PCL_REVISION_VER 0
#endif

//#define EXTERNAL_WINDOW
#ifdef EXTERNAL_WINDOW
#include <pcl/visualization/cloud_viewer.h>
#endif

#include "../Grasp/VectorMath.h"
#include <cnoid/Link>
class PointCloudHandler
{

public :
#ifdef EXTERNAL_WINDOW
    PointCloudHandler(): viewer ("PCL OpenNI Viewer"){
        leafsize = 0.001;
        minSensorDistance = 0.0;
        maxSensorDistance = 1.5;
        radiusTolerance = 0.05;
        minRadius = 0.0;
        maxRadius = 0.1;
        minLength = 0.01;
        maxLength = 0.2;
        minCloudDistance = 0.02;
    }
    pcl::visualization::CloudViewer viewer;
#else
    PointCloudHandler():
        cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>()),
        cloud_xyzrgba(new pcl::PointCloud<pcl::PointXYZRGBA>())
    {
        leafsize = 0.001;        //used in VoxelFilter
        minSensorDistance = 0.0; //used in PassThroughFilter
        maxSensorDistance = 1.5; //used in PassThroughFilter
        radiusTolerance = 0.05;  //used in CylinderSegmentation
        minRadius = 0.0;         //used in CylinderSegmentation
        maxRadius = 0.1;         //used in CylinderSegmentation
        desRadius = 0.1;         //used in CylinderSegmentation
        minLength = 0.01;        //used in CylinderSegmentation
        maxLength = 0.2;         //used in CylinderSegmentation
        cameraPos << 0,0,0;      //used in CylinderSegmentation
        vertical << 0,0,1;       //used in CylinderSegmentation
        minCloudDistance = 0.02; //used in splitPointClouds
    }
#endif

    static PointCloudHandler* instance(){
        static PointCloudHandler* instance = new PointCloudHandler();
        return instance;
    }

    boost::shared_ptr<pcl::Grabber> grabber_;

    void make_range_image(pcl::PointCloud<pcl::PointWithRange>::Ptr& ri,
                          pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud);
    void capture(bool rgb = false);
    void segmentate(int t=0);
    void PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output);
    void EuclidCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output);
    void extractEuclidCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, std::vector<pcl::PointIndices>& cluster_indices);
    void NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
    void CylinderSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster);
    void VoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                           double min, double max, std::string coord);
    void OutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void Difference(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB, std::vector<int>& newPointIdx);
    void Smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void Smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output);
    void Triangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const cnoid::Vector3& viewpoint = cnoid::Vector3(0, 0, 0));
    void captureWithoutInit(bool rgb = false);
    void startGrabber(bool rgb = false);
    void stopGrabber();
    bool isGrabberActive() const;

    static void convertColdetModel2PointCloud(const cnoid::ColdetModelPtr object, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double sampling_density=10e-6);
    void cylidnerSegmentationPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<cnoid::Vector3>& pos, std::vector<cnoid::Vector3>& dir, std::vector<double>& radius, std::vector<double>& length, int max_num);

    void setTransMatrix();
    cnoid::Vector3 Trans(double x, double y, double z);
    cnoid::Vector3 Dir(double x, double y, double z);
    cnoid::Vector3 t1, t2, t3;
    cnoid::Matrix3 R1, R2, R3;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba;
    double leafsize;
    double minSensorDistance, maxSensorDistance;
    double radiusTolerance, minRadius, maxRadius, desRadius, minLength, maxLength; //Cylinder segmentation
    pcl::PolygonMesh triangles;
    double minCloudDistance;
    cnoid::Vector3 cameraPos, vertical;

    void passThroughFilterRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input,
                               pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output,
                               double min, double max, std::string coord);
protected:
    void xyz_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud);
    void xyzrgb_cb(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr const& cloud);
    void xyzseg_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud);
    bool cap;
    std::string device_;
    int type;
};

#endif

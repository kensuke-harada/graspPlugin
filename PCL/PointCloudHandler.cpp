// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include "PointCloudHandler.h"
#include "CylinderParameter.h"

#include <pcl/impl/point_types.hpp>
#include <pcl/surface/impl/mls.hpp>


//#define READ_PCD
//#define READ_PLY
//#define WRITE_PCD

#define SHOW_OUTLIER //out of object
#define SHOW_INLIER //in the object

#ifdef WIN32
#include <windows.h>
void usleep(int t){
  Sleep(t/1000);
}
#endif

#include <cnoid/ExecutablePath>
#define PLUGIN_PATH cnoid::executableTopDirectory() + string("/")

using namespace std;
using namespace cnoid;

void PointCloudHandler::xyz_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr
                               const& cloud)
{
#ifdef EXTERNAL_WINDOW
    if (!viewer.wasStopped())
            viewer.showCloud (cloud);
#endif

    if(cap){
#ifdef WRITE_PCD
        pcl::io::savePCDFileASCII (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_pcd.pcd", *cloud);
#endif
        pcl::PointCloud<pcl::PointXYZ> cloud_input(*cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud_output (new pcl::PointCloud<pcl::PointXYZ>);

#if defined(READ_PCD)
        pcl::io::loadPCDFile<pcl::PointXYZ>(PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_pcd.pcd", cloud_input);
#elif defined(READ_PLY)
        pcl::io::loadPLYFile<pcl::PointXYZ>(PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_ply.ply", cloud_input);
		//pcl::PLYReader reader;
		//reader.read<pcl::PointXYZ> (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_ply.ply", cloud_input);
#endif

        VoxelFilter(cloud_input.makeShared(), cloud_output);
        pcl::copyPointCloud(*cloud_output, *cloud_xyz);
        cap = false;
    }
}

void PointCloudHandler::xyzrgb_cb(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr const& cloud)
{
#ifdef EXTERNAL_WINDOW
        if (!viewer.wasStopped())
                viewer.showCloud (cloud);
#endif

        if(cap){
#ifdef WRITE_PCD
                pcl::io::savePCDFileASCII (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_pcd.pcd", *cloud);
#endif
                pcl::copyPointCloud(*cloud, *cloud_xyzrgba);
                cap = false;

#if defined(READ_PCD)
		        pcl::io::loadPCDFile<pcl::PointXYZRGBA> (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_pcd.pcd", *cloud_xyzrgba);
#elif defined(READ_PLY)
				pcl::io::loadPLYFile<pcl::PointXYZRGBA> (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_ply.ply", *cloud_xyzrgba);
				//pcl::PLYReader reader;
				//reader.read<pcl::PointXYZ> (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_ply.ply", *cloud_xyzrgba);
#endif
        }
}

void PointCloudHandler::xyzseg_cb(pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
{
#ifdef EXTERNAL_WINDOW
        if (!viewer.wasStopped())
                viewer.showCloud (cloud);
#endif


        if(cap){
#ifdef WRITE_PCD
                pcl::io::savePCDFileASCII (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_pcd.pcd", *cloud);
#endif
                pcl::PointCloud<pcl::PointXYZ> cloud_input(*cloud);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGBA>);

#if defined(READ_PCD)
		        pcl::io::loadPCDFile<pcl::PointXYZ>(PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_pcd.pcd", cloud_input);
#elif defined(READ_PLY)
				pcl::io::loadPLYFile<pcl::PointXYZ>(PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_ply.ply", cloud_input);
				//pcl::PLYReader reader;
				//reader.read<pcl::PointXYZ> (PLUGIN_PATH + "extplugin/graspPlugin/PCL/test_ply.ply", cloud_input);
#endif

                VoxelFilter(cloud_input.makeShared(), cloud_filtered);

                if(type==0){
                        PlaneSegmentation(cloud_filtered, cloud_output);
                        pcl::copyPointCloud(*cloud_output, *cloud_xyzrgba);
                }
                else if(type==1){
                        EuclidCluster(cloud_filtered, cloud_output);
                        pcl::copyPointCloud(*cloud_output, *cloud_xyzrgba);
                }
                else if(type==2){
                        Smoothing(cloud_filtered, cloud_filtered2);
                        Triangulation(cloud_filtered2);
                }
                else if(type==3){
                        Triangulation(cloud_filtered);
                }
                else{
                        //cout << "Distance from sensor, min:" << minSensorDistance << ", max:" << maxSensorDistance << endl;
                        PassThroughFilter(cloud_filtered, cloud_filtered2, minSensorDistance, maxSensorDistance, "z");
                        CylinderSegmentation(cloud_filtered2, cloud_output);
                        pcl::copyPointCloud(*cloud_output, *cloud_xyzrgba);
                }

                cap = false;
        }
}

void PointCloudHandler::make_range_image(
        pcl::PointCloud<pcl::PointWithRange>::Ptr& ri,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr const& cloud)
{
        ri->height = cloud->height;
        ri->width = cloud->width;
        ri->points.resize(cloud->size());
        for (unsigned int ii(0); ii < cloud->size(); ++ii){
                pcl::PointXYZ const& s = cloud->points[ii];
                pcl::PointWithRange& d = ri->points[ii];
                d.x = s.x;
                d.y = s.y;
                d.z = s.z;
                d.range = d.getVector3fMap().norm();
        }
}

void PointCloudHandler::capture(bool rgb)
{
    cap = true;

#if defined(READ_PCD) | defined(READ_PLY)
    if(!rgb){
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                cloud(new pcl::PointCloud<pcl::PointXYZ>);
        xyz_cb(cloud);
    }
#else
	if (isGrabberActive()) {
			grabber_->stop();
	}
#ifndef USE_OPENNI2
    grabber_.reset(new pcl::OpenNIGrabber(device_));
#else
	grabber_.reset(new pcl::io::OpenNI2Grabber(device_));
#endif
    if (!rgb) {
        boost::function<void (pcl::PointCloud<pcl::PointXYZ>::ConstPtr const&)>
                f = boost::bind(&PointCloudHandler::xyz_cb, this, _1);
        grabber_->registerCallback(f);
    }
    else {
        boost::function<void (pcl::PointCloud<pcl::PointXYZRGBA>::
                              ConstPtr const&)> f =
                boost::bind(&PointCloudHandler::xyzrgb_cb, this, _1);
        grabber_->registerCallback(f);
    }
    grabber_->start();

#ifdef EXTERNAL_WINDOW
    while(!viewer.wasStopped()) {
        sleep (1);
    }
#else
    while(cap) {
        usleep(10);
    }
#endif
    grabber_->stop();
    //grabber_.reset();
#endif
}

void PointCloudHandler::segmentate(int t)
{
        cap = true;
        type = t;
#if defined(READ_PCD) | defined(READ_PLY)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        xyzseg_cb(cloud);
#else
		if (isGrabberActive()) {
				grabber_->stop();
		}
#ifndef USE_OPENNI2
		grabber_.reset(new pcl::OpenNIGrabber(device_));
#else
		grabber_.reset(new pcl::io::OpenNI2Grabber(device_));
#endif

        boost::function<void (pcl::PointCloud<pcl::PointXYZ>::ConstPtr const&)> f =
                boost::bind(&PointCloudHandler::xyzseg_cb, this, _1);
        grabber_->registerCallback(f);

        grabber_->start();

#ifdef EXTERNAL_WINDOW
        while(!viewer.wasStopped())
                sleep (1);
#else
        while(cap)
                usleep(10);
#endif

        grabber_->stop();
        //grabber_.reset();
#endif
}

void PointCloudHandler::PlaneSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_segmented) {

#define SHOW_CONVEXHULL
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);  //Set the maximum number of iterations before giving up.
        seg.setDistanceThreshold (0.02);  //Distance to the model threshold

        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};

        int nr_points = cloud_input->points.size ();
        while (cloud_input->points.size () > 0.3 * nr_points){
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_input);
                seg.segment (*inliers, *coefficients); //*
                if (inliers->indices.size () == 0){
                        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                        break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud (cloud_input);
                extract.setIndices (inliers);
                extract.setNegative (false);
                extract.filter (*cloud_plane); //*

#ifdef SHOW_CONVEXHULL

                // Creating the KdTree object for the search method of the extraction
                std::vector<pcl::PointIndices> cluster_indices;
                extractEuclidCluster(cloud_plane, cluster_indices);

                int j, i=0;
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

                        j=0;
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

                        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                                cloud_cluster->resize(j+1);
                                cloud_cluster->points[j].x = cloud_plane->points[*pit].x;
                                cloud_cluster->points[j].y = cloud_plane->points[*pit].y;
                                cloud_cluster->points[j].z = cloud_plane->points[*pit].z;
                                j++;
                        }

                        // Project the model inliers
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());
                        pcl::ProjectInliers<pcl::PointXYZ> proj;
                        proj.setModelType (pcl::SACMODEL_PLANE);
                        proj.setInputCloud (cloud_cluster);
                        proj.setModelCoefficients (coefficients);
                        proj.filter (*cloud_projected);

                        // Create a Convex Hull representation of the projected inliers
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::ConcaveHull<pcl::PointXYZ> chull;
                        chull.setInputCloud (cloud_projected);
                        chull.setAlpha (0.1);//for concave hull
                        chull.setDimension(2);
                        chull.reconstruct (*cloud_hull);

                        j=0;
                        int s1 = cloud_segmented->size();
                        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                                cloud_segmented->resize(s1+j+1);

                                cloud_segmented->points[s1+j].x = cloud_plane->points[*pit].x;
                                cloud_segmented->points[s1+j].y = cloud_plane->points[*pit].y;
                                cloud_segmented->points[s1+j].z = cloud_plane->points[*pit].z;

                                cloud_segmented->points[s1+j].r = colors[i%6][0];
                                cloud_segmented->points[s1+j].g = colors[i%6][1];
                                cloud_segmented->points[s1+j].b = colors[i%6][2];
                                j++;
                        }
                        i++;

                        s1 = cloud_segmented->size();
                        int s3 = cloud_hull->size();
                        cloud_segmented->resize(s1 + s3);

                        for (int j = 0; j < s3; ++j){
                                cloud_segmented->points[s1+j].x = cloud_hull->points[j].x;
                                cloud_segmented->points[s1+j].y = cloud_hull->points[j].y;
                                cloud_segmented->points[s1+j].z = cloud_hull->points[j].z;

                                cloud_segmented->points[s1+j].r = colors[i%6][0];
                                cloud_segmented->points[s1+j].g = colors[i%6][1];
                                cloud_segmented->points[s1+j].b = colors[i%6][2];
                        }
                        i++;
                }

                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_input); //*

#else
                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_input); //*

                int s1 = cloud_segmented->size();
                int s2 = cloud_plane->size();
                cloud_segmented->resize(s1+s2);

                for (size_t j = 0; j < s2; ++j){
                        cloud_segmented->points[s1+j].x = cloud_plane->points[j].x;
                        cloud_segmented->points[s1+j].y = cloud_plane->points[j].y;
                        cloud_segmented->points[s1+j].z = cloud_plane->points[j].z;

                        cloud_segmented->points[s1+j].r = colors[i%6][0];
                        cloud_segmented->points[s1+j].g = colors[i%6][1];
                        cloud_segmented->points[s1+j].b = colors[i%6][2];
                }
                i++;
#endif
        }

}

void PointCloudHandler::EuclidCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster) {

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);  //Maximum number of iterations before giving up.
        seg.setDistanceThreshold (0.02);  //Determine how close a point must be to the model in order to be considered an inlier.


        int nr_points = cloud_input->points.size ();
        while (cloud_input->points.size () > 0.3 * nr_points){
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_input);
                seg.segment (*inliers, *coefficients); //*
                if (inliers->indices.size () == 0){
                        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                        break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud (cloud_input);
                extract.setIndices (inliers);
                //extract.setNegative (false);
                //extract.filter (*cloud_plane); //*

                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter(*cloud_input);
        }

        // Creating the KdTree object for the search method of the extraction
        std::vector<pcl::PointIndices> cluster_indices;
        extractEuclidCluster(cloud_input, cluster_indices);

        int j = 0;
        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
        pcl::copyPointCloud(*cloud_input, *cloud_cluster);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++) {
                        cloud_cluster->points[*pit].r = colors[j%6][0];
                        cloud_cluster->points[*pit].g = colors[j%6][1];
                        cloud_cluster->points[*pit].b = colors[j%6][2];
                }
                j++;
        }
}

void PointCloudHandler::extractEuclidCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, std::vector<pcl::PointIndices>& cluster_indices){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_input);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (minCloudDistance); // Spatial cluster tolerance as a measure in the L2 Euclidean space.
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud( cloud_input);
        ec.extract (cluster_indices);
}

void PointCloudHandler::NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_input);
        ne.setKSearch (50);
        ne.compute (*cloud_normals);
}

void PointCloudHandler::CylinderSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster) {

        // Estimate point normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        NormalEstimation(cloud_filtered, cloud_normals);

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);  //Maximum number of iterations before giving up.
        seg.setDistanceThreshold (0.02);  //Determine how close a point must be to the model in order to be considered an inlier.

        int nr_points = cloud_filtered->points.size ();
        pcl::ExtractIndices<pcl::Normal> extract_normals;

        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);

        while (cloud_filtered->points.size () > 0.2 * nr_points){
                // Segment the largest planar component from the remaining cloud
                seg.setInputCloud(cloud_filtered);
                seg.setInputNormals(cloud_normals);
                seg.segment (*inliers_plane, *coefficients_plane); //*
                if (inliers_plane->indices.size () == 0){
                        std::ostream& os = MessageView::mainInstance()->cout();
                        os << "Could not estimate a planar model for the given dataset." << std::endl;
                        break;
                }

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud (cloud_filtered);
                extract.setIndices (inliers_plane);

                // Remove the planar inliers, extract the rest
                extract.setNegative (true);
                extract.filter (*cloud_filtered);

                extract_normals.setNegative (true);
                extract_normals.setInputCloud (cloud_normals);
                extract_normals.setIndices (inliers_plane);
                extract_normals.filter (*cloud_normals);
        }

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold(radiusTolerance);
        seg.setRadiusLimits(minRadius, maxRadius);

        pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

        pcl::PointCloud<pcl::PointXYZRGBA> cloud_output;
        pcl::PointCloud<pcl::PointXYZ> cloud_neighbor;

        int cloud_idx=0;

        /*
        pcl::copyPointCloud(*cloud_filtered, cloud_output);

        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
        for (size_t j=0; j<cloud_output.points.size(); j++){
            cloud_output.points[j].r = colors[cloud_idx%6][0];
            cloud_output.points[j].g = colors[cloud_idx%6][1];
            cloud_output.points[j].b = colors[cloud_idx%6][2];
        }
         */

        std::vector<pcl::PointIndices> cluster_indices;
        extractEuclidCluster(cloud_filtered, cluster_indices);

        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

                nr_points = it->indices.size();
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                        cloud_filtered2->points.push_back( cloud_filtered->points[*pit]);

                NormalEstimation(cloud_filtered2, cloud_normals);

#if defined(SHOW_INLIER) && defined(SHOW_OUTLIER)

                pcl::PointCloud<pcl::PointXYZRGBA> cloud_cylinder2;
                pcl::copyPointCloud(*cloud_filtered2, cloud_cylinder2);

                float colors[9][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}, {255,255,128}, {128,255,255}, {255,128,255}};
                for (size_t j=0; j<cloud_cylinder2.points.size(); j++){
                        cloud_cylinder2.points[j].r = colors[cloud_idx%9][0];
                        cloud_cylinder2.points[j].g = colors[cloud_idx%9][1];
                        cloud_cylinder2.points[j].b = colors[cloud_idx%9][2];
                }
                cloud_output += cloud_cylinder2;
                cloud_idx++;
#endif

                Eigen::Vector3f axis(0,0,0);
                bool exec = false;

//				while (cloud_filtered2->points.size () > 0.05 * nr_points){
                        seg.setAxis(axis);
                        seg.setEpsAngle(45.0/180.0*3.1416);

                        seg.setInputCloud (cloud_filtered2);
                        seg.setInputNormals (cloud_normals);
                        seg.segment (*inliers_cylinder, *coefficients_cylinder);

                        pcl::ExtractIndices<pcl::PointXYZ> extract;
                        extract.setInputCloud(cloud_filtered2);
                        extract.setIndices(inliers_cylinder);
                        extract.setNegative (false);
                        pcl::PointCloud<pcl::PointXYZ> cloud_cylinder;

                        extract.filter (cloud_cylinder);

                        CylinderFeatures * cf = CylinderFeatures::instance();
                        cf->minLength = minLength;
                        cf->maxLength = maxLength;
                        cf->cameraPos = cameraPos;

                        if(cloud_cylinder.size()==0) continue;

                        if(cf->extractFeatures(cloud_cylinder.makeShared(), coefficients_cylinder, desRadius) || exec){
#ifdef OBJECT_DISTRIBUTION_MODE
                                if(cf->cylinderGenerated){
                                        cf->t1_ = t1;
                                        cf->t2_ = t2;
                                        cf->R1_ = R1;
                                        cf->R2_ = R2;

                                        cf->calcPointCloudDistribution(cloud_filtered2, cloud_cylinder.makeShared(), coefficients_cylinder, desRadius);
                                }
#endif

#if !defined(SHOW_INLIER) && defined(SHOW_OUTLIER) //#ifndef PC_ON_OBJECT
                                pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
                                pcl:copyPointCloud(*cloud_filtered2, cloud_tmp);
#endif

                                extract.setNegative(true);
                                extract.filter(*cloud_filtered2);
#if !defined(SHOW_INLIER) && defined(SHOW_OUTLIER) //#ifndef PC_ON_OBJECT
                                if(cloud_filtered2->points.size()==0 )
                                            cloud_neighbor += cloud_tmp;
                                else if(! cf->cylinderGenerated )
                                    cloud_neighbor += cloud_cylinder;
#endif
                                extract_normals.setNegative (true);
                                extract_normals.setInputCloud (cloud_normals);
                                extract_normals.setIndices (inliers_cylinder);
                                extract_normals.filter (*cloud_normals);
                                axis = Eigen::Vector3f(0,0,0);
                                exec = false;
                        }
                        else{
                                Vector3 axis_o (coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5]);
                                Matrix3 R = grasp::rodrigues(vertical, 3.1416/2.0);
                                axis_o = R*axis_o;
                                axis = Eigen::Vector3f(axis_o(0), axis_o(1), axis_o(2));
                                exec = true;
                        }

#if defined(SHOW_INLIER) && !defined(SHOW_OUTLIER) //#ifdef PC_ON_OBJECT
                        pcl::PointCloud<pcl::PointXYZRGBA> cloud_cylinder3;
                        pcl::copyPointCloud(cloud_cylinder, cloud_cylinder3);

                        float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};
                        for (size_t j=0; j<cloud_cylinder3.points.size(); j++){
                            cloud_cylinder3.points[j].r = colors[cloud_idx%6][0];
                            cloud_cylinder3.points[j].g = colors[cloud_idx%6][1];
                            cloud_cylinder3.points[j].b = colors[cloud_idx%6][2];
                        }
                        cloud_output += cloud_cylinder3;
#endif
                        cloud_idx++;
//				}
        }
#if !defined(SHOW_INLIER) && defined(SHOW_OUTLIER) //#ifndef PC_ON_OBJECT
        pcl::copyPointCloud(cloud_neighbor, cloud_output);

        for (size_t j=0; j<cloud_output.points.size(); j++){
            cloud_output.points[j].r = 255;
            cloud_output.points[j].g = 0;
            cloud_output.points[j].b = 0;
        }
#endif
        pcl::copyPointCloud(cloud_output, *cloud_cluster);

        //Connection of two cylinders
        CylinderFeatures::instance()->connectCylinders();

        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);
        //cf->addCylinders(cloud_cluster, cloud_tmp, desRadius, radiusTolerance, acos(0.3) );
}

void PointCloudHandler::VoxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (input);
        vg.setLeafSize (leafsize, leafsize, leafsize);
        vg.filter (*output);
}

void PointCloudHandler::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                                          double min, double max, string coord) {

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (input);
        pass.setFilterFieldName (coord.c_str());
        pass.setFilterLimits (min, max);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*output);
}

void PointCloudHandler::passThroughFilterRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input,
                                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output,
                                          double min, double max, string coord) {

        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud (input);
        pass.setFilterFieldName (coord.c_str());
        pass.setFilterLimits (min, max);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*output);
}

void PointCloudHandler::OutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (input);
        sor.setMeanK (50); //Number of neighbors to analyze for each point
        sor.setStddevMulThresh (1.0); //Standard deviation multiplier
        sor.filter (*output);
}

void PointCloudHandler::Difference(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB, vector<int>& newPointIdx){
        float resolution = leafsize*3.0; // Octree resolution - side length of minumum octree voxels

        pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

        octree.setInputCloud (cloudA);
        octree.addPointsFromInputCloud ();

        octree.switchBuffers ();

        octree.setInputCloud (cloudB);
        octree.addPointsFromInputCloud ();

        octree.getPointIndicesFromNewVoxels (newPointIdx);
}

void PointCloudHandler::Smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

        mls.setInputCloud (input);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.04);//0.03 original

        mls.process (*output);
}

void PointCloudHandler::Smoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output){

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

        mls.setComputeNormals (true);
        mls.setInputCloud (input);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.04);//0.03 original

        mls.process (*output);
}

void PointCloudHandler::Triangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const cnoid::Vector3& viewpoint){
        // remove Nan from PointCloud
        vector<int> index;
        bool tmp_dense = cloud->is_dense;
        cloud->is_dense = false;
        pcl::removeNaNFromPointCloud(*cloud,*cloud,index);
        cloud->is_dense = tmp_dense;

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);
        n.setInputCloud (cloud);
        n.setSearchMethod (tree);
        n.setKSearch (20);
		n.setViewPoint (viewpoint(0), viewpoint(1), viewpoint(2));
        n.compute (*normals);

        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

//Two methods (Greedy Projection / Poisson) are supported
#define GREEDY_PROJECTION
#ifdef GREEDY_PROJECTION
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

        gp3.setSearchRadius (0.025); //maximum distance between connected points (maximum edge length)

        gp3.setMu (2.5);
        gp3.setMaximumNearestNeighbors (100);
        gp3.setMaximumSurfaceAngle(M_PI/4);
        gp3.setMinimumAngle(M_PI/18);
        gp3.setMaximumAngle(2*M_PI/3);
        gp3.setNormalConsistency(true);
        gp3.setConsistentVertexOrdering(true); //This function is included only in the developer's version

        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);
#else
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(12);//9
//		poisson.setDegree(5);
//		poisson.setSamplesPerNode(2);
//		poisson.setScale(1.25);
//		poisson.setIsoDivide(8);
//		poisson.setConfidence(1);
//		poisson.setManifold(0);
//		poisson.setOutputPolygons(0);
        poisson.setSolverDivide(8);
        poisson.setInputCloud(cloud_with_normals);
        poisson.reconstruct(triangles);
#endif

#if PCL_VERSION_COMPARE(<, 1, 7, 0)
        pcl::fromROSMsg(triangles.cloud, *cloud_xyz);
#else
        fromPCLPointCloud2(triangles.cloud, *cloud_xyz);
#endif
        std::ostream& os = MessageView::mainInstance()->cout();
        os << "Triangulation:" << endl;
        os << "  cloud size: " << cloud->size() << endl;
        os << "  triangle size: " << triangles.polygons.size() << endl;
        // Additional vertex information
        //std::vector<int> parts = gp3.getPartIDs();
        //std::vector<int> states = gp3.getPointStates();

}

void PointCloudHandler::setTransMatrix()
{
    char line[1024];
    string calibFile = PLUGIN_PATH +
            "extplugin/graspPlugin/PCL/calibtools/calibmat.txt";

    FILE *ifp0=NULL;
    ifp0 = fopen(calibFile.c_str(),"rb");

    if(ifp0 != NULL){
        if(! fgets(line,256,ifp0) )
                printf("result mat: Broken format1 \n");

        int j=0;
        while(1){

            while(fgets(line,256,ifp0) != NULL) {
                if(line[0] != '#' && line[0]!=' ' &&
                        line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') {
                    break;
                }
            }

            if(line[0] != '4') {
                printf("result mat: Broken format1 \n");
            }

            Vector3 P0;
            Matrix3 R0;

            int i=0;
            while(fgets(line,256,ifp0) != NULL) {
                if(i<3 && sscanf(line,"%lf%lf%lf%lf",&R0(i,0),
                                 &R0(i,1),&R0(i,2),&P0(i)) != 4) {
                    printf("result mat: Broken format2 %d \n",i);
                    fclose(ifp0);
                    return;
                }
                i++;
                if(i >= 4) {
                    break;
                }
            }
            if(j==0) {R1 = R0; t1 = P0;}
            if(j==1) {R2 = R0; t2 = P0;}
            if(j==2) {R3 = R0; t3 = P0;}
            if(++j == 3) break;;
        }
        fclose(ifp0);
    }
}

Vector3 PointCloudHandler::Trans(double x, double y, double z)
{
    Vector3 v1(x,y,z);

    return Vector3( t3 + R3*(t2 + R2*(t1 + R1*v1)) );
}

Vector3 PointCloudHandler::Dir(double x, double y, double z)
{
    Vector3 v1(x,y,z);

    return Vector3( R3*R2*R1*v1 );
}

void PointCloudHandler::captureWithoutInit(bool rgb)
{
    if(!isGrabberActive()) {
        startGrabber(rgb);
    }

    cap = true;
    while(cap){
        usleep(10);
    }
}

void PointCloudHandler::startGrabber(bool rgb)
{
    cap = false;
#ifndef USE_OPENNI2
	grabber_.reset(new pcl::OpenNIGrabber(device_));
#else
	grabber_.reset(new pcl::io::OpenNI2Grabber(device_));
#endif
    if (!rgb) {
        boost::function<void (pcl::PointCloud<pcl::PointXYZ>::ConstPtr const&)>
                f = boost::bind(&PointCloudHandler::xyz_cb, this, _1);
        grabber_->registerCallback(f);
    }
    else {
        boost::function<void (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr
                              const&)> f = boost::
                bind(&PointCloudHandler::xyzrgb_cb, this, _1);
        grabber_->registerCallback(f);
    }
    grabber_->start();
}

void PointCloudHandler::stopGrabber()
{
    if(grabber_.get() == NULL) return;
    grabber_->stop();
}

bool PointCloudHandler::isGrabberActive() const
{
    return ((grabber_.get() == NULL) ? false : (grabber_->isRunning()));
}

void PointCloudHandler::convertColdetModel2PointCloud(const ColdetModelPtr object, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double sampling_density) {
    int t[3];
    for(int k = 0; k < object->getNumTriangles(); k++) {
        object->getTriangle(k, t[0], t[1], t[2]);
        float tx, ty, tz;
        vector<Vector3> ver;
        for (int i = 0; i < 3; i++) {
            object->getVertex(t[i], tx, ty, tz);
            ver.push_back(Vector3(tx, ty, tz));
        }
        double area = grasp::norm2(grasp::cross(ver[1]-ver[0], ver[2]-ver[0]))/2.0;
        double num = area / sampling_density;
        int i_num = (int)floor(num);
        if ((double)rand()/(double)RAND_MAX < (num-(double)i_num)) {
            i_num++;
        }
        for (int i = 0; i < i_num; i++) {
            double alpha = (double)rand()/(double)RAND_MAX;
            double beta = ((double)rand()/(double)RAND_MAX) * (1.0 - alpha);
            double gamma = 1.0 - alpha - beta;
            pcl::PointXYZ p;
            Vector3 v_tmp = alpha * ver[0] + beta * ver[1] + gamma * ver[2];
            p.x = v_tmp(0);
            p.y = v_tmp(1);
            p.z = v_tmp(2);
            cloud->push_back(p);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
}

void PointCloudHandler::cylidnerSegmentationPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<Vector3>& pos, vector<Vector3>& dir, vector<double>& radius, vector<double>& length, int max_num) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);
    NormalEstimation(cloud, cloud_normals);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(radiusTolerance);
  seg.setRadiusLimits(minRadius, maxRadius);

    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);

    Eigen::Vector3f axis(0,0,0);
    seg.setAxis(axis);
    seg.setEpsAngle(90.0/180.0*3.1416);

    int n_points = cloud->points.size();
    pcl::copyPointCloud(*cloud, *target_cloud);
    pcl::copyPointCloud(*cloud_normals, *target_normals);

    int num_cylinder = 0;
    while ((target_cloud->points.size() > 0.05 * n_points) && (num_cylinder++ < max_num)){
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr tmp_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(target_cloud);
        seg.setInputNormals(target_normals);
        seg.segment (*inliers_cylinder, *coefficients_cylinder);

        extract.setInputCloud(target_cloud);
        extract.setIndices(inliers_cylinder);
        extract.setNegative (false);
        extract.filter(*cloud_cylinder);

        extract.setNegative (true);
        extract.filter(*tmp_cloud);
        pcl::copyPointCloud(*tmp_cloud, *target_cloud);
        extract_normals.setInputCloud(target_normals);
        extract_normals.setIndices(inliers_cylinder);
        extract_normals.setNegative(true);
        extract_normals.filter(*tmp_normals);
        pcl::copyPointCloud(*tmp_normals, *target_normals);

        if(cloud_cylinder->size()==0) break;


        Vector3 c_pos, c_dir;
        double c_radius, c_length;
        c_pos << coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2];
        c_dir << coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5];
        c_radius = coefficients_cylinder->values[6];
        c_dir = grasp::unit(c_dir);

        vector<double> lengSet;
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud_cylinder->begin(); it!=cloud_cylinder->end(); ++it){
            Vector3 r = Vector3(it->x, it->y, it->z);
            lengSet.push_back(Vector3(r - c_pos).dot(c_dir));
        }
        sort(lengSet.begin(), lengSet.end());

        double max_len = lengSet.back() - lengSet.front();
        double max_gap = max_len * 0.1;
        double lmax=0.0, lmin=0.0, tmp_lmax, tmp_lmin;

        bool is_start = true;
        for(size_t i=0; i<lengSet.size()-1; i++){
            if(is_start){
                tmp_lmin = lengSet[i];
                tmp_lmax = lengSet[i];
                is_start = false;
            }
            if(lengSet[i+1]-lengSet[i] > max_gap){
                if((lmax-lmin) < (tmp_lmax-tmp_lmin)){
                    lmax = tmp_lmax;
                    lmin = tmp_lmin;
                }
                is_start = true;
            }else{
                tmp_lmax = lengSet[i+1];
            }
        }
        if((lmax-lmin) < (tmp_lmax-tmp_lmin)){
            lmax = tmp_lmax;
            lmin = tmp_lmin;
        }

        c_pos = c_pos + c_dir*(lmax+lmin)*0.5;

        pos.push_back(c_pos);
        dir.push_back(c_dir);
        radius.push_back(c_radius);
        length.push_back(lmax-lmin);
    }
}

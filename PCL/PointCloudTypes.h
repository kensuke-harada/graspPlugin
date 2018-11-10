#ifndef _PCL_POINTCLOUDTYPES_H_
#define _PCL_POINTCLOUDTYPES_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZ               PointT;
typedef pcl::PointCloud<PointT>     PointCloudT;
typedef PointCloudT::Ptr            PointCloudTPtr;
typedef PointCloudT::ConstPtr       PointCloudTConstPtr;
typedef PointCloudT::const_iterator PointCloudTConstIterator;

typedef pcl::PointXYZRGBA               ColorPointT;
typedef pcl::PointCloud<ColorPointT>    ColorPointCloud;
typedef ColorPointCloud::Ptr            ColorPointCloudPtr;
typedef ColorPointCloud::ConstPtr       ColorPointCloudConstPtr;
typedef ColorPointCloud::const_iterator ColorPointCloudConstIterator;

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef NormalCloud::Ptr             NormalCloudPtr;
typedef NormalCloud::ConstPtr        NormalCloudConstPtr;
typedef NormalCloud::const_iterator  NormalCloudConstIterator;

typedef pcl::search::KdTree<PointT> KdTreePointT;
typedef KdTreePointT::Ptr           KdTreePointTPtr;
typedef KdTreePointT::ConstPtr      KdTreePointTConstPtr;

typedef pcl::search::KdTree<ColorPointT> KdTreeColorPointT;
typedef KdTreeColorPointT::Ptr           KdTreeColorPointTPtr;
typedef KdTreeColorPointT::ConstPtr      KdTreeColorPointTConstPtr;

#endif /* _PCL_POINTCLOUDTYPES_H_ */

#ifndef _PCL_CVFHESTIMATOR_H_
#define _PCL_CVFHESTIMATOR_H_

#include <limits>
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/features/cvfh.h>
#include <pcl/filters/filter_indices.h>
//#include <pcl/features/crh.h>
#include "crh.h"
#include "cvfhext.h"
#include "crhext.h"
#include "PointCloudTypes.h"


#define USE_EXTCVFH

POINT_CLOUD_REGISTER_POINT_STRUCT (Histogram<90>,
	(float[90], histogram, histogram)
	)

POINT_CLOUD_REGISTER_POINT_STRUCT (Histogram<340>,
	(float[340], histogram, histogram)
	)

class DescriptorPair {
public:
	typedef pcl::Histogram<90> CRHDescriptor;
#ifdef USE_EXTCVFH
	typedef pcl::Histogram<340> CVFHDescriptor;
#else
	typedef pcl::VFHSignature308 CVFHDescriptor;
#endif
	typedef pcl::PointCloud<CRHDescriptor> CRHCloud;
	typedef pcl::PointCloud<CVFHDescriptor> CVFHCloud;

	CRHDescriptor crh_descriptor;
	CVFHDescriptor cvfh_descriptor;
	CRHDescriptor crhext_descriptor;
	Eigen::Vector3f centroid;
};

template <typename T = pcl::PointXYZ>
class CVFHEstimator {
public:
	typedef DescriptorPair::CVFHDescriptor CVFHDescriptor;
	typedef DescriptorPair::CRHDescriptor CRHDescriptor;
	typedef pcl::PointCloud<T> Cloud;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;
	typedef typename Cloud::const_iterator CloudConstIterator;
	typedef DescriptorPair::CRHCloud CRHCloud;
	typedef DescriptorPair::CVFHCloud CVFHCloud;

	CVFHEstimator() :
		eps_angle_threshold_(0.785),
		curvature_threshold_(1.0),
		normalize_bins_(true),
		cluster_tolerance_factor_(3.0),
		radius(0.01),
		grid_resolution(0.010),
		max_length_(1.0f) {
	}

	void estimate(CloudConstPtr input, NormalCloudConstPtr normals, const Eigen::Vector3d& view, std::vector<DescriptorPair>& des_pairs) const {
#ifdef USE_EXTCVFH
		pcl::CVFHExtEstimation<T, pcl::Normal, CVFHDescriptor> cvfh;
#else
		pcl::CVFHEstimation<T, pcl::Normal, CVFHDescriptor> cvfh;
#endif

		// extract indices which don't have NaN normal
		pcl::IndicesPtr indices = pcl::IndicesPtr(new std::vector<int>);
		NormalCloudPtr normal_ = NormalCloudPtr(new NormalCloud());
		pcl::removeNaNNormalsFromPointCloud(*normals, *normal_, *indices);

		// The cvfh implementation in PCL returns wrong results when input PointCloud's viewpoint is not (0, 0, 0)
		// So we translate input point cloud.
		Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
		trans.block<3,1>(0, 3) << -view.x(), -view.y(), -view.z();
		CloudPtr translated_input = CloudPtr(new Cloud());
		pcl::transformPointCloud(*input, *translated_input, trans);

		// CVFH estiamtion setting
		cvfh.setInputCloud(translated_input);
		cvfh.setInputNormals(normals);
		cvfh.setIndices(indices);
		cvfh.setEPSAngleThreshold(eps_angle_threshold_);
		cvfh.setCurvatureThreshold(curvature_threshold_);
		cvfh.setNormalizeBins(normalize_bins_);
		cvfh.setClusterTolerance(grid_resolution);
		cvfh.setRadiusNormals(radius);
		cvfh.setMinPoints(50);
		cvfh.setViewPoint(0, 0, 0);
#ifdef USE_EXTCVFH
		cvfh.setMaxLength(max_length_);
#endif

		// compute cvfh descritors
		CVFHCloud cvfh_signatures;
		cvfh.compute(cvfh_signatures);
		std::vector<Eigen::Vector3f> centroids;
		std::vector<Eigen::Vector3f> ori_centroids;
		cvfh.getCentroidClusters(centroids);

		// translate centroids to original coordinate system
		ori_centroids.resize(centroids.size());
		for (size_t j = 0; j < centroids.size(); j++) {
			for (int i = 0; i < 3; i++) {
				ori_centroids[j][i] = centroids[j][i] + view[i];
			}
		}

		// CRH estiamtion setting
		pcl::CRHEstimation<T, pcl::Normal, CRHDescriptor> crh;
		crh.setInputCloud(translated_input);
		crh.setInputNormals(normals);
		crh.setIndices(indices);

		// CRHExt estimation setting
		pcl::CRHExtEstimation<T, pcl::Normal, CRHDescriptor> crhext;
		crhext.setInputCloud(translated_input);
		crhext.setInputNormals(normals);
		crhext.setIndices(indices);

		des_pairs.clear();
		des_pairs.resize(cvfh_signatures.size());
		for (size_t i = 0; i < cvfh_signatures.size(); i++) {
			CRHCloud crh_histogram;
			CRHCloud crhext_histogram;
			Eigen::Vector4f centroid4 = Eigen::Vector4f::Zero();
			centroid4.head(3) = centroids[i];

			// compute CRH descriptors
			crh.setCentroid(centroid4);
			crh.compute(crh_histogram);

			// compute CRHExt descriptors
			crhext.setCentroid(centroid4);
			crhext.compute(crhext_histogram);

			// store descriptors
			des_pairs[i].centroid = ori_centroids[i];
			des_pairs[i].cvfh_descriptor = cvfh_signatures[i];
			des_pairs[i].crh_descriptor = crh_histogram[0];
			des_pairs[i].crhext_descriptor = crhext_histogram[0];
		}
	}

	void setMaxLength(float max_length) {
		max_length_ = max_length;
	}

private:
	float eps_angle_threshold_;
	float curvature_threshold_;
	float cluster_tolerance_factor_;
	bool normalize_bins_;
	double radius;
	double grid_resolution;
	float max_length_;
};

#endif

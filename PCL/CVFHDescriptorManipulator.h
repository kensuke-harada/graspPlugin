#ifndef _PCL_CVFHDESCRIPTORMANIPULATOR_H_
#define _PCL_CVFHDESCRIPTORMANIPULATOR_H_

#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include "CVFHEstimator.h"

#include "PointCloudTypes.h"
#include "PointCloudUtility.h"

class CVFHDesPose {
 public:
	DescriptorPair des_pair;
	Eigen::Matrix4f pose;
  Eigen::Vector3f bounding_box;
	std::string pcd_filepath;
	int id;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MyPointRepresentation: public pcl::PointRepresentation<pcl::Histogram<340> > {
 public:
	MyPointRepresentation() {
		nr_dimensions_ = 340;
	}

	virtual void copyToFloatArray(const pcl::Histogram<340> &p, float* out) const {
		for (int i = 0; i < nr_dimensions_; i++) {
			out[i] = p.histogram[i];
		}
	}
};

class CVFHDescriptorManipulator {
 public:
	typedef std::vector<CVFHDesPose, Eigen::aligned_allocator<CVFHDesPose> > CVFHDescriptors;
	typedef DescriptorPair::CRHCloud CRHCloud;
	typedef DescriptorPair::CVFHCloud CVFHCloud;

	CVFHDescriptorManipulator();
	virtual ~CVFHDescriptorManipulator();

	void computeDescriptor(PointCloudTConstPtr target_cloud, const Eigen::Vector3d& view, double max_length, CVFHDescriptors& des);
	bool generateDescriptors(const std::string& pcd_dir, double voxel_leaf = 0.005);
	void setNormalEstimator(NormalEstimator* est) {normal_estimator_ = est;}
	void readDescriptors(const std::string& pcd_dir);
	CVFHDescriptors getDescriptors() const {return descriptors_;}

	static bool writeMatrix(const std::string& file_path, const Eigen::Matrix4f& mat);
	static bool readMatrix(const std::string& file_path, Eigen::Matrix4f& mat);
	static bool writeCentroid(const std::string& file_path, const Eigen::Vector3f& centroid);
	static bool readCentroid(const std::string& file_path, Eigen::Vector3f& centroid);

 private:
	CVFHDescriptors descriptors_;
	NormalEstimator* normal_estimator_;

	bool writeGeneratingParam(const std::string& file_path, double voxel_leaf, double normal_radius) const;
  void computeProjectedBB(PointCloudTConstPtr target_cloud, Eigen::Vector3f& edge) const;
};

class PartialPointCloudCache {
 public:
	static PartialPointCloudCache* instance();
	virtual ~PartialPointCloudCache();

	void init(int size, const std::string& des_dir);
	PointCloudTPtr getPartialCloud(const CVFHDesPose& target_des_pose);

 private:
	PartialPointCloudCache();

	std::vector<PointCloudTPtr> pc_cache_;
	std::string des_dir_;
};

class CVFHDescriptorCache {
 public:
	typedef CVFHDescriptorManipulator::CVFHDescriptors CVFHDescriptors;
	static CVFHDescriptorCache* instance();
	virtual ~CVFHDescriptorCache();

	void getCVFHDescriptors(const std::string& des_dir, CVFHDescriptors& descriptors);

 private:
	CVFHDescriptorCache();

	CVFHDescriptors des_cache_;
	std::string des_dir_;
};

#endif

#endif

#ifndef _PCL_REGISTRATOR_H_
#define _PCL_REGISTRATOR_H_

#include <Eigen/Core>
#include <pcl/pcl_config.h>

#include <boost/shared_ptr.hpp>

#include "PointCloudTypes.h"

class FeatureCloud;

typedef boost::shared_ptr<FeatureCloud> FeatureCloudPtr;

class AbstractPoseEstimator {
 public:
	AbstractPoseEstimator();
	virtual ~AbstractPoseEstimator();

	void setObjFeatureCloud(FeatureCloudPtr _obj);
	void setSceneFeatureCloud(FeatureCloudPtr _scene);
	PointCloudTPtr getAlignedObjCloud() const;
	double getScore() const;
	Eigen::Matrix3d getRot() const;
	Eigen::Vector3d getTrans() const;
	void setMaxCorrespondenceDist(double dist);

	virtual bool estimate() = 0;

	PointCloudTConstPtr obj_cloud;
	PointCloudTConstPtr scene_cloud;

	Eigen::Matrix4f init_matrix;

 protected:
	FeatureCloudPtr obj_feature;
	FeatureCloudPtr scene_feature;

	double max_correspondence_dist;

	PointCloudTPtr aligned_obj_cloud;
	Eigen::Matrix4d trans_matrix;
	double score;

 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

class ICPEstimator : public AbstractPoseEstimator {
 public:
	ICPEstimator();
	~ICPEstimator();

	void setIteration(unsigned int _ite);
	void setTransEps(double trans_eps);
	void setFitEps(double fit_eps);
	void setTree(pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

	bool estimate();

 private:
	unsigned int iteration;
	double trans_eps_;  ///< icp convergence criteria (transformation matrix difference)
	double fit_eps_;  ///< icp convergence criteria (relative MSE)
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
};

class SAC_IAEstimator : public AbstractPoseEstimator {
 public:
	SAC_IAEstimator();
	~SAC_IAEstimator();

	bool estimate();

 private:
	double min_sample_distance;
	unsigned int max_iteration;
};


#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
class RobustPoseEstimator : public AbstractPoseEstimator {
 public:
	RobustPoseEstimator() {;}
	~RobustPoseEstimator() {;}

	bool estimate();
};


class RobustRejectivePoseEstimator : public AbstractPoseEstimator {
 public:
	RobustRejectivePoseEstimator() {;}
	virtual ~RobustRejectivePoseEstimator() {;}

	bool estimate();
};
#endif

typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef LocalFeatures::Ptr                    LocalFeaturesPtr;
typedef pcl::search::KdTree<pcl::PointXYZ>    SearchMethod;
typedef SearchMethod::Ptr                     SearchMethodPtr;

class FeatureCloud {
public:
    FeatureCloud();
    virtual ~FeatureCloud();

    void setInputCloud(PointCloudTConstPtr _cloud);
    PointCloudTConstPtr getCloud() const;
    LocalFeaturesPtr getLocalFeatures() const;
    NormalCloudPtr getNormals() const;
    void setViewPoint(const Eigen::Vector3d& _viewpoint);
    void setFeatureRadius(double _radius);
    void setNormalRadius(double _radius);
    void setFeatureK(unsigned int _k);
    void setNormalK(unsigned int _k);
    void useFeatureKSearch(bool _use);
    void useNormalKSearch(bool _use);

    void computeLocalFeatures();
    void computeNormals();

private:
    PointCloudTConstPtr cloud;
    LocalFeaturesPtr features;
    NormalCloudPtr normals;
    SearchMethodPtr method;
    Eigen::Vector3d view_point;
    double feature_radius;
    double normal_radius;
    unsigned int normal_k;
    unsigned int feature_k;
    bool use_normal_ksearch;
    bool use_feature_ksearch;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* _PCL_REGISTRATOR_H_ */

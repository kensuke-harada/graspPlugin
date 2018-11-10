#ifndef _PCL_SEGMENTER_H_
#define _PCL_SEGMENTER_H_

#include <vector>

#include <cnoid/EigenTypes>

#include "PointCloudTypes.h"

#include "exportdef.h"

class EXCADE_API AbstractSegmenter {
public:
	AbstractSegmenter();
	virtual ~AbstractSegmenter();

	virtual bool segment() = 0;

	void setInputCloud(PointCloudTConstPtr _cloud);
	PointCloudTPtr getSegmentedCloud() const;
	std::vector<pcl::PointIndices> getClusterPointIndices() const;
	ColorPointCloudPtr getClusteredCloud() const;

	void setDistThresholdPlaneRemovement(double dist);
	void setRemovePlaneNum(unsigned int num);
	void setRemovePlaneRate(double rate);
	void setTolerance(double tol);
	void setVerticalTolerance(double v_tol);
	void setViewPoint(const cnoid::Vector3& viewpoint);

	void generateClusteredCloud();
	void setClusterPointIndices(const std::vector<pcl::PointIndices>& indices);
	bool reSegment(const pcl::PointIndices& apply_indices, std::vector<pcl::PointIndices>& segmented_indices,
								 double reseg_radius, double boundary_radius);

 protected:
	void removePlane();


	std::vector<pcl::PointIndices> cluster_indices;
	PointCloudTConstPtr cloud;
	PointCloudTPtr segmented_cloud;

	double dist_th;
	int num_plane_remove;
	double tolerance;
	double vertical_tolerance_;
	double rate_plane_remove;
	bool use_rate_in_remove;
	cnoid::Vector3 viewpoint_;

	ColorPointCloudPtr clustered_cloud;
};

class EXCADE_API Segmenter  : public AbstractSegmenter {
 public:
	Segmenter();
	~Segmenter();

	bool segment();

 private:
	bool clustering();
};

class EXCADE_API ConditionalSegmenter :
public AbstractSegmenter {
 public:
	ConditionalSegmenter();
	~ConditionalSegmenter();

	bool segment();

 private:
	bool clustering();

	static bool conditionFunction(const PointT& seed_p, const PointT& cand_p, float squared_dist);
	static double squared_dist;
	static double squared_vdist;
	static cnoid::Vector3 viewpoint;
};

class EXCADE_API LccpSegmenter :
public AbstractSegmenter {
 public:
	LccpSegmenter();
	~LccpSegmenter();

	bool segment();

	void setSuperVoxelParameter(double voxel_resolution, double seed_resolution, double spatial, double normal, double color);
	void setConcavityTolerance(double angle_degree);
	void setMinClusterSize(int num_points);

 private:
	double voxel_resolution_;
	double seed_resolution_;
	double spatial_importance_;
	double normal_importance_;
	double color_importance_;
	double concavility_tolerance_;
	int min_cluster_size_;

	bool clustering();
};

#endif /* _PCL_SEGMENTER_H_ */

#ifndef _PCL_POINTCLOUDMERGER_H_
#define _PCL_POINTCLOUDMERGER_H_

#include <vector>

#include <boost/utility.hpp>

#include <cnoid/ColdetModel>

#include "PointCloudTypes.h"

namespace grasp {
	class ObjPoseEstimateSol;
}

class ClusterInfo {
 public:
	enum PrevResultMatch { NONE, FULL, PARTIAL};
	PointCloudTPtr cloud;
	cnoid::Vector3 p;
	cnoid::Matrix3 R;
	bool has_prev_sol;
	grasp::ObjPoseEstimateSol* sol;
	double score;
	PrevResultMatch prev_match;
	bool is_feasible;
};

class PointCloudClustersHolder :
boost::noncopyable {
 public:
	static PointCloudClustersHolder* instance();

	PointCloudTPtr getCloudPtr(int i) const;
	ClusterInfo& getCluster(int i);
	const ClusterInfo& getCluster(int i) const;
	bool isPrevGraspTarget(int i) const;
	int size() const;

	ClusterInfo& addCluster();
	void clear();
 private:
	PointCloudClustersHolder();

	std::vector<ClusterInfo> clusters_;
};

class PointCloudMerger {
 public:
	PointCloudMerger();
	~PointCloudMerger();

	void merge(PointCloudTPtr cloud, const cnoid::Vector3& viewpoint, double leafsize = 0.0025);

	std::vector<cnoid::ColdetModelPtr> getModels() const;
	std::vector<cnoid::Vector3> getColors() const;
	std::vector<PointCloudTPtr> getModelPoints() const;

	void getUnchangedClusterIDs(std::vector<int>& ids) const;
 private:
	void generateMeshes(std::vector<cnoid::ColdetModelPtr>& models, const cnoid::Vector3& viewpoint) const;
	bool isIntersectAABBray(const cnoid::Vector3& aabb_min, const cnoid::Vector3& aabb_max,
													const cnoid::Vector3& viewpoint, const cnoid::Vector3& dir,
													const cnoid::Vector3& dir_inv, const std::vector<bool>& zero_flag,
													cnoid::Vector3& region_p_min, cnoid::Vector3& region_p_max) const;
	std::vector<cnoid::ColdetModelPtr> models_;
	std::vector<cnoid::Vector3> model_colors_;
	std::vector<PointCloudTPtr> model_points_;

	std::vector<int> unchanged_cluster_ids_;

	double dist_th_;
	double reject_ratio_th_;
};

#endif /* _PCL_POINTCLOUDMERGER_H_ */

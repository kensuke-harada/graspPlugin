#ifndef _PCL_POSEESTIMATIONRESULTMATCHER_H_
#define _PCL_POSEESTIMATIONRESULTMATCHER_H_

#include <iostream>
#include <vector>

#include <cnoid/EigenTypes>

#include <pcl/common/common.h>

#include "PointCloudTypes.h"

class ClusterInfo;

class PoseEstimationResultMatcher {
 public:
	PoseEstimationResultMatcher();
	~PoseEstimationResultMatcher();

	void setMaxCorrespondenceDistance(double dist);
	void setAcceptanceThreshold(double th);

	void setPrevObjectCloud(const std::vector<ClusterInfo>* clusters, const PointCloudTPtr& obj_cloud);

	void matchPrevResults(const PointCloudTPtr& target_cloud);

	void matchPrevResultsToPointcloud(const PointCloudTPtr& target_cloud, bool& is_match, bool& is_reuse_result) const;

	int getReuseSolSize() const;
	void getReuseSol(int i, cnoid::Vector3& p, cnoid::Matrix3& R, PointCloudTPtr& cloud) const;

 protected:
	double correspondence_dist_;
	double accept_rmse_th_;

	bool done_init_;

	class PrevObjCloud {
	public:
		PointCloudTPtr cloud;
		PointCloudTPtr cluster_cloud;
		KdTreePointTPtr tree;
		cnoid::Vector3 aabb_min;
		cnoid::Vector3 aabb_max;
		bool match_result;
		cnoid::Vector3 p;
		cnoid::Matrix3 R;
		bool is_feasible;
	};

	std::vector<PrevObjCloud> prev_objclouds_;
	std::vector<PrevObjCloud*> reuse_objclouds_;
};

#endif /* _PCL_POSEESTIMATIONRESULTMATCHER_H_ */

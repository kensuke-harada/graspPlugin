#include "PoseEstimationResultMatcher.h"

#include "PointCloudMerger.h"
#include "PointCloudUtility.h"

PoseEstimationResultMatcher::PoseEstimationResultMatcher() :
	done_init_(false),
	correspondence_dist_(0.02),
	accept_rmse_th_(0.003) {
}

PoseEstimationResultMatcher::~PoseEstimationResultMatcher() {
}

void PoseEstimationResultMatcher::setMaxCorrespondenceDistance(double dist) {
	correspondence_dist_ = dist;
}

void PoseEstimationResultMatcher::setAcceptanceThreshold(double th) {
	accept_rmse_th_ = th;
}

/**
 * Register cluterinfos to this instance
 * @param[in] clusters previous cluster infos
 * @param[in] obj_cloud object point cloud
 */
void PoseEstimationResultMatcher::setPrevObjectCloud(const std::vector<ClusterInfo>* clusters, const PointCloudTPtr& obj_cloud) {
	done_init_ = true;

	int cluster_size = clusters->size();
	prev_objclouds_.clear();
	prev_objclouds_.resize(cluster_size);
	reuse_objclouds_.clear();

	for (int i = 0; i < cluster_size; i++) {
		PrevObjCloud& target_cloud = prev_objclouds_[i];
		target_cloud.cluster_cloud = PointCloudTPtr(new PointCloudT());
		pcl::copyPointCloud(*(clusters->at(i).cloud), *(target_cloud.cluster_cloud));
		target_cloud.cloud = PointCloudTPtr(new PointCloudT());
		target_cloud.p = clusters->at(i).p;
		target_cloud.R = clusters->at(i).R;
		PointCloudUtil::transCloud(obj_cloud, target_cloud.cloud, clusters->at(i).R, clusters->at(i).p);
		target_cloud.tree = KdTreePointTPtr(new KdTreePointT(false));
		target_cloud.tree->setInputCloud(target_cloud.cluster_cloud);
		PointCloudUtil::getMinMax(target_cloud.cluster_cloud, target_cloud.aabb_min, target_cloud.aabb_max);
		target_cloud.match_result = false;
		target_cloud.is_feasible = clusters->at(i).is_feasible;
	}
}

/**
 * Matching between a point cloud (inside box) and previous point clouds.
 * @param[in] target_cloud target point cloud
 */
void PoseEstimationResultMatcher::matchPrevResults(const PointCloudTPtr& target_cloud) {
	if (!done_init_) {
		std::cerr << "implementation error in PoseEstimationResultMatcher::matchPrevResults" << std::endl;
		return;
	}

	// iintialize
	int prev_result_size = prev_objclouds_.size();

	// const double max_dist = 0.25 * correspondence_dist_ * correspondence_dist_;
	const double max_dist = 0.005 * 0.005;
	const double th = 0.3;

	KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
	tree->setInputCloud(target_cloud);

	std::vector<int> nn_indices(1);
	std::vector<float> nn_distances(1);
	for (int i = 0; i < prev_result_size; i++) {
		// create object point cloud
		if (!prev_objclouds_[i].is_feasible) {
			prev_objclouds_[i].match_result = false;
			continue;
		}

		PointCloudTPtr& target_cloud = prev_objclouds_[i].cloud;

		int nr = 0;
		for (size_t j = 0; j < target_cloud->points.size(); j++) {
			if (!pcl::isFinite(target_cloud->points[j]))
				continue;
			tree->nearestKSearch(target_cloud->points[j], 1, nn_indices, nn_distances);
			if (nn_distances[0] < max_dist) {
				nr++;
			}
		}
		prev_objclouds_[i].match_result = (th * target_cloud->points.size()  < nr);
	}

	for (int i = 0; i < prev_result_size; i++) {
		if (prev_objclouds_[i].match_result) {
			reuse_objclouds_.push_back(&prev_objclouds_[i]);
		}
	}
}

/**
 * Matching between a clustered point cloud and previous point clouds.
 * @param[in] target_cloud target clusterd point cloud
 * @param[out] is_match matching result
 * @param[out] is_reuse_result true if the estimation solution of target cluster is set to previous solution.
 */
void PoseEstimationResultMatcher::matchPrevResultsToPointcloud(const PointCloudTPtr& target_cloud, bool& is_match, bool& is_reuse_result) const {
	int prev_result_size = prev_objclouds_.size();

	is_match = false;
	is_reuse_result = false;

	cnoid::Vector3 min_p;
	cnoid::Vector3 max_p;
	PointCloudUtil::getMinMax(target_cloud, min_p, max_p);

	std::vector<int> nn_indices(1);
	std::vector<float> nn_distances(1);

	const double max_dist = correspondence_dist_ * correspondence_dist_;
	const double rate_th = 0.6;

	min_p -= correspondence_dist_ * cnoid::Vector3::Ones();
	max_p += correspondence_dist_ * cnoid::Vector3::Ones();

	for (int i = 0; i < prev_result_size; i++) {
		if (!prev_objclouds_[i].is_feasible) continue;
		if ((min_p.array() > prev_objclouds_[i].aabb_max.array()).all()) continue;
		if ((max_p.array() < prev_objclouds_[i].aabb_min.array()).all()) continue;

		int n = 0;

		for (size_t j = 0; j < target_cloud->points.size(); j++) {
			if (!pcl::isFinite(target_cloud->points[j]))
				continue;
			prev_objclouds_[i].tree->nearestKSearch(target_cloud->points[j], 1, nn_indices, nn_distances);
			if (nn_distances[0] < max_dist) {
				n++;
			}
		}
		if ((n > rate_th * target_cloud->points.size()) && (n > rate_th * prev_objclouds_[i].cluster_cloud->points.size())) {
			is_match = true;
			is_reuse_result = prev_objclouds_[i].match_result;
			return;
		}
	}
}

int PoseEstimationResultMatcher::getReuseSolSize() const {
	return reuse_objclouds_.size();
}

void PoseEstimationResultMatcher::getReuseSol(int i, cnoid::Vector3& p, cnoid::Matrix3& R, PointCloudTPtr& cloud) const {
	p = reuse_objclouds_[i]->p;
	R = reuse_objclouds_[i]->R;
	pcl::copyPointCloud(*reuse_objclouds_[i]->cluster_cloud, *cloud);
}

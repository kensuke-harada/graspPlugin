#include "PointCloudMerger.h"

#include "PointCloudHandler.h"
#include "PointCloudUtility.h"

#include "../Grasp/ObjectPoseEstimationSolution.h"

#define NOOUTPUT_TIMELOG
#include "../GraspDataGen/Util/StopWatch.h"

#include <iostream>

//#define SKIP_POINTCLOUDMERGE

PointCloudClustersHolder::PointCloudClustersHolder() {
}

PointCloudClustersHolder* PointCloudClustersHolder::instance() {
	static PointCloudClustersHolder* instance = new PointCloudClustersHolder();
	return instance;
}

PointCloudTPtr PointCloudClustersHolder::getCloudPtr(int i) const {
	return clusters_[i].cloud;
}

ClusterInfo& PointCloudClustersHolder::getCluster(int i) {
	return clusters_[i];
}

const ClusterInfo& PointCloudClustersHolder::getCluster(int i) const {
	return clusters_[i];
}

bool PointCloudClustersHolder::isPrevGraspTarget(int i) const {
	if (clusters_[i].sol == NULL) return false;
	return clusters_[i].sol->is_target;
}

int PointCloudClustersHolder::size() const {
	return clusters_.size();
}

ClusterInfo& PointCloudClustersHolder::addCluster() {
	clusters_.push_back(ClusterInfo());
	return clusters_.back();
}

void PointCloudClustersHolder::clear() {
	clusters_.clear();
}

PointCloudMerger::PointCloudMerger() :
	dist_th_(0.01),
	reject_ratio_th_(0.5) {
}

PointCloudMerger::~PointCloudMerger() {
	models_.clear();
}

void PointCloudMerger::merge(PointCloudTPtr cloud, const cnoid::Vector3& viewpoint, double leafsize) {
#ifdef SKIP_POINTCLOUDMERGE
	return;
#endif
	models_.clear();
	model_colors_.clear();
	model_points_.clear();
	unchanged_cluster_ids_.clear();

	PointCloudClustersHolder* cluster_holder = PointCloudClustersHolder::instance();
	int cluster_size = cluster_holder->size();

	// n_p is the number of points where a ray from viewpoint to a point in the cloud and cluster's surface intersect.
	// n_out is the number of points which exceeds threshold distance from cluster's surface.
	std::vector<int> n_p(cluster_size, 0);
	std::vector<int> n_out(cluster_size, 0);

	PointCloudT::const_iterator ite;
	PointCloudT::const_iterator ite_c;

	double dist_intersect_th = leafsize * leafsize;
	double squared_dist_th = dist_th_ * dist_th_;

	grasp::StopWatch timer;
	timer.start();

	// compute AABB of each cluster
	std::vector<cnoid::Vector3> min_p(cluster_size);
	std::vector<cnoid::Vector3> max_p(cluster_size);
	for (int i = 0; i < cluster_size; i++) {
		Eigen::Vector4f min_point, max_point;
		pcl::getMinMax3D(*(cluster_holder->getCloudPtr(i)), min_point, max_point);
		min_point -= leafsize * Eigen::Vector4f::Ones();
		max_point += leafsize * Eigen::Vector4f::Ones();
		min_p[i] = min_point.head(3).cast<double>();
		max_p[i] = max_point.head(3).cast<double>();
	}

	for (ite = cloud->begin(); ite != cloud->end(); ++ite) {
		cnoid::Vector3 p0(ite->x, ite->y, ite->z);
		cnoid::Vector3 dir = p0 - viewpoint;
		double intersect_th = dir.squaredNorm() * dist_intersect_th;

		// for intersect detection between AABB and ray(dir)
		//bool has_zero = dir.isZero(epsilon);
		std::vector<bool> zero_flag(3, false);
		cnoid::Vector3 dir_inv(0, 0, 0);
		for (int j = 0; j < 3; j++) {
			if (fabs(dir[j]) < 1e-15) {
				zero_flag[j] = true;
			} else {
				dir_inv[j] = 1.0 / dir[j];
			}
		}

		cnoid::Vector3 box_max(0, 0, 0);
		cnoid::Vector3 box_min(0, 0, 0);
		for (int i = 0; i < cluster_size; i++) {
			if (!isIntersectAABBray(min_p[i], max_p[i], viewpoint, dir, dir_inv, zero_flag, box_min, box_max)) continue;

			box_max += leafsize * cnoid::Vector3::Ones();
			box_min -= leafsize * cnoid::Vector3::Ones();

			PointCloudTPtr cluster_cloud = cluster_holder->getCloudPtr(i);
			double min_dist = std::numeric_limits<double>::max();
			cnoid::Vector3 near_p(0, 0, 0);
			for (ite_c = cluster_cloud->begin(); ite_c != cluster_cloud->end(); ++ite_c) {
				cnoid::Vector3 p1(ite_c->x, ite_c->y, ite_c->z);
				if ((box_min.array() > p1.array()).any() || (box_max.array() < p1.array()).any()) continue;
				double squared_dist_numerator = (dir.cross(p1 - viewpoint)).squaredNorm();
				if (min_dist > squared_dist_numerator) {
					min_dist = squared_dist_numerator;
					near_p = p1;
				}
			}
			if (min_dist < intersect_th) {
				n_p[i]++;
				if ((p0 - near_p).squaredNorm() > squared_dist_th) {
					n_out[i]++;
				}
			}
		}
	}

	timer.stopAndPrintTime("merge");

	// pick clusters which rate of n_out is low or n_p is zero.
	for (int i = 0; i < cluster_size; i++) {
		if (cluster_holder->isPrevGraspTarget(i)) continue;
		std::cout << "n_out:" << n_out[i] << " n_cross:" << n_p[i] << " points:" << cluster_holder->getCloudPtr(i)->size() <<  std::endl;
		if (n_p[i] != 0 && (static_cast<double>(n_out[i])/static_cast<double>(n_p[i]) > reject_ratio_th_)) {
			continue;
		}

		model_points_.push_back(PointCloudTPtr(new PointCloudT()));
		pcl::copyPointCloud(*(cluster_holder->getCloudPtr(i)), *(model_points_.back()));
		(*cloud) += *(cluster_holder->getCloudPtr(i));

		PointCloudTPtr cluster_cloud = PointCloudClustersHolder::instance()->getCloudPtr(i);
		// std::cout << "cloud_size" << cluster_cloud->size() << std::endl;
		if ((n_p[i] > 0.5 * cluster_cloud->size()) && (static_cast<double>(n_out[i])/static_cast<double>(n_p[i]) < 0.2)) {
			unchanged_cluster_ids_.push_back(i);
		}
	}

	PointCloudTPtr tmp(new PointCloudT);
	PointCloudUtil::voxelFilter(cloud, tmp, leafsize);
	pcl::copyPointCloud(*tmp, *cloud);
}

bool PointCloudMerger::isIntersectAABBray(const cnoid::Vector3& aabb_min, const cnoid::Vector3& aabb_max,
																					const cnoid::Vector3& viewpoint, const cnoid::Vector3& dir,
																					const cnoid::Vector3& dir_inv, const std::vector<bool>& zero_flag,
																					cnoid::Vector3& region_p_min, cnoid::Vector3& region_p_max) const {

	bool has_zero = (zero_flag[0] || zero_flag[1] || zero_flag[2]);

	if (has_zero) {
		bool is_intersect = true;
		for (int i = 0; i < 3; i++) {
			if (zero_flag[i]) {
				if (viewpoint[i] < aabb_min[i] || aabb_max[i] < viewpoint[i]) {
					return false;
				}
			}
		}
	}

	cnoid::Vector3 t1 = dir_inv.cwiseProduct(aabb_min - viewpoint);
	cnoid::Vector3 t2 = dir_inv.cwiseProduct(aabb_max - viewpoint);

	cnoid::Vector3 t_near = t1.cwiseMin(t2);
	cnoid::Vector3 t_far = t1.cwiseMax(t2);

	if (has_zero) {
		for (int i = 0; i < 3; i++) {
			if (zero_flag[i]) {
				t_near[i] = -std::numeric_limits<double>::max();
				t_far[i] = std::numeric_limits<double>::max();
			}
		}
	}

	double t_near_intersect = t_near.maxCoeff();
	double t_far_intersect = t_far.minCoeff();

	if (t_near_intersect > t_far_intersect) return false;

	cnoid::Vector3 inter_p1 = viewpoint + t_near_intersect * dir;
	cnoid::Vector3 inter_p2 = viewpoint + t_far_intersect * dir;

	region_p_min = inter_p1.cwiseMin(inter_p2);
	region_p_max = inter_p1.cwiseMax(inter_p2);

	return true;
}

// void PointCloudMerger::merge(PointCloudTPtr cloud, const cnoid::Vector3& viewpoint, double leafsize) {
// #ifdef SKIP_POINTCLOUDMERGE
// 	return;
// #endif
// 	models_.clear();
// 	model_colors_.clear();
// 	model_points_.clear();
// 	unchanged_cluster_ids_.clear();

// 	std::vector<cnoid::ColdetModelPtr> models;

// 	grasp::StopWatch timer;
// 	timer.start();
	
// 	// generate models
// 	generateMeshes(models, viewpoint);
// 	if (models.empty()) return;

// 	PointCloudClustersHolder* cluster_holder = PointCloudClustersHolder::instance();

// 	PointCloudT::const_iterator ite;

// 	// n_p is the number of points where a ray from viewpoint to a point in the cloud and cluster's surface intersect.
// 	// n_out is the number of points which exceeds threshold distance from cluster's surface.
// 	std::vector<int> n_p(models.size(), 0);
// 	std::vector<int> n_out(models.size(), 0);
// 	for (ite = cloud->begin(); ite != cloud->end(); ++ite) {
// 		double p[3] = {ite->x, ite->y, ite->z};
// 		cnoid::Vector3 dir = (cnoid::Vector3(ite->x, ite->y, ite->z) - viewpoint).normalized();
// 		double dp[3] = {dir.x(), dir.y(), dir.z()};
// 		double dn[3] = {-dir.x(), -dir.y(), -dir.z()};
// 		for (size_t i = 0; i < models.size(); i++) {
// 			if (cluster_holder->isPrevGraspTarget(i)) continue;
// 			double dist1 = models[i]->computeDistanceWithRay(p, dp);
// 			if (dist1 != 0.0) {
// 				if (dist1 > dist_th_) {
// 					n_out[i]++;
// 				}
// 				n_p[i]++;
// 				// std::cout << i << " " <<  dist1 << std::endl;
// 				continue;
// 			}
// 			double dist2 = models[i]->computeDistanceWithRay(p, dn);
// 			if (dist2 != 0.0) {
// 				if (dist2 > dist_th_) {
// 					n_out[i]++;
// 				}
// 				n_p[i]++;
// 				// std::cout << i << " " <<  dist2 << std::endl;
// 			}
// 		}
// 	}

// 	timer.stopAndPrintTime("merge");

// 	// pick clusters which rate of n_out is low or n_p is zero.
// 	for (size_t i = 0; i < models.size(); i++) {
// 		if (cluster_holder->isPrevGraspTarget(i)) continue;
// 		std::cout << "n_out:" << n_out[i] << " n_cross:" << n_p[i] << std::endl;
// 		if (n_p[i] != 0 && (static_cast<double>(n_out[i])/static_cast<double>(n_p[i]) > reject_ratio_th_)) {
// 			continue;
// 		}

// 		models_.push_back(models[i]);
// 		model_points_.push_back(PointCloudTPtr(new PointCloudT()));
// 		pcl::copyPointCloud(*(cluster_holder->getCloudPtr(i)), *(model_points_.back()));
// 		(*cloud) += *(cluster_holder->getCloudPtr(i));

// 		PointCloudTPtr cluster_cloud = PointCloudClustersHolder::instance()->getCloudPtr(i);
// 		// std::cout << "cloud_size" << cluster_cloud->size() << std::endl;
// 		if ((n_p[i] > 0.5 * cluster_cloud->size()) && (static_cast<double>(n_out[i])/static_cast<double>(n_p[i]) < 0.2)) {
// 			unchanged_cluster_ids_.push_back(i);
// 			model_colors_.push_back(cnoid::Vector3(1.0, 0, 0));
// 		} else {
// 			model_colors_.push_back(cnoid::Vector3(0, 0, 1.0));
// 		}
// 	}

// 	PointCloudHandler* pch = PointCloudHandler::instance();
// 	pch->leafsize = leafsize;
// 	PointCloudTPtr tmp(new PointCloudT);
// 	pch->VoxelFilter(cloud, tmp);
// 	pcl::copyPointCloud(*tmp, *cloud);
// }

std::vector<cnoid::ColdetModelPtr> PointCloudMerger::getModels() const {
	return models_;
}

std::vector<cnoid::Vector3> PointCloudMerger::getColors() const {
	return model_colors_;
}

std::vector<PointCloudTPtr> PointCloudMerger::getModelPoints() const {
	return model_points_;
}

void PointCloudMerger::getUnchangedClusterIDs(std::vector<int>& ids) const {
	ids = unchanged_cluster_ids_;
}

/**
 * @brief generate ColdetModels from clusters which are included in PointCloudClustersHolder.
 */
void PointCloudMerger::generateMeshes(std::vector<cnoid::ColdetModelPtr>& models, const cnoid::Vector3& viewpoint) const {
	models.clear();
	PointCloudHandler* pch = PointCloudHandler::instance();
	PointCloudClustersHolder* cluster_holder = PointCloudClustersHolder::instance();

	for (int i = 0; i < cluster_holder->size(); i++) {
		PointCloudTPtr output(new PointCloudT());
		pch->Smoothing(cluster_holder->getCloudPtr(i), output);
		pch->Triangulation(output, viewpoint);

		cnoid::ColdetModelPtr model(new cnoid::ColdetModel());
		model->setNumVertices(pch->cloud_xyz->size());
		model->setNumTriangles(2 * pch->triangles.polygons.size());
		int vid = 0;
		int tid = 0;
		for (size_t j = 0; j < pch->cloud_xyz->size(); j++) {
			pcl::PointXYZ at = pch->cloud_xyz->at(j);
			model->setVertex(vid++, at.x, at.y, at.z);
		}
		for (size_t j = 0; j < pch->triangles.polygons.size(); j++) {
			model->setTriangle(tid++,
												 pch->triangles.polygons[j].vertices[0],
												 pch->triangles.polygons[j].vertices[1],
												 pch->triangles.polygons[j].vertices[2]);
			model->setTriangle(tid++,
												 pch->triangles.polygons[j].vertices[2],
												 pch->triangles.polygons[j].vertices[1],
												 pch->triangles.polygons[j].vertices[0]);
		}
		model->setName("cluster");
		model->build();
		models.push_back(model);
	}
}

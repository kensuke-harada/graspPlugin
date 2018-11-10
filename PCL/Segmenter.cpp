#include "Segmenter.h"

#include <algorithm>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/segment_differences.h>

#include <cnoid/MessageView>

#include "PointCloudHolder.h"
#include "PointCloudUtility.h"

#include "lccp_segmentation.h"

#define NOOUTPUT_TIMELOG
#include "../GraspDataGen/Util/StopWatch.h"

AbstractSegmenter::AbstractSegmenter() :
	segmented_cloud(new PointCloudT),
	dist_th(0.005),
	num_plane_remove(1),
	rate_plane_remove(0.8),
	tolerance(0.004),
	vertical_tolerance_(0.004),
	use_rate_in_remove(false),
	clustered_cloud(new ColorPointCloud) {
	// nothing is done
}

AbstractSegmenter::~AbstractSegmenter() {
	// nothing is done
}

/**
 * Set a pointer to the target point cloud data.
 */
void AbstractSegmenter::setInputCloud(PointCloudTConstPtr _cloud) {
	cloud = _cloud;
}

/**
 * Get a pointer to the point cloud dataset aftersegmentation.
 */
PointCloudTPtr AbstractSegmenter::getSegmentedCloud() const {
	return segmented_cloud;
}


std::vector<pcl::PointIndices> AbstractSegmenter::getClusterPointIndices() const {
	return cluster_indices;
}

/**
 * Get a pointer to the point cloud dataset after clustering.
 */
ColorPointCloudPtr AbstractSegmenter::getClusteredCloud() const {
	return clustered_cloud;
}

void AbstractSegmenter::setDistThresholdPlaneRemovement(double dist) {
	dist_th = dist;
}

void AbstractSegmenter::setRemovePlaneNum(unsigned int num) {
	num_plane_remove = num;
}

void AbstractSegmenter::setRemovePlaneRate(double rate) {
	rate_plane_remove = rate;
	if (rate > 1.0 || rate < 0.0) {
		rate_plane_remove = 0.0;
	}
}

void AbstractSegmenter::setTolerance(double tol) {
	tolerance = tol;
}

void AbstractSegmenter::setVerticalTolerance(double v_tol) {
	vertical_tolerance_ = v_tol;
}

void AbstractSegmenter::setViewPoint(const cnoid::Vector3& viewpoint) {
	viewpoint_ = viewpoint;
}

void AbstractSegmenter::setClusterPointIndices(const std::vector<pcl::PointIndices>& indices) {
	cluster_indices = indices;
}

bool AbstractSegmenter::reSegment(const pcl::PointIndices& apply_indices, std::vector<pcl::PointIndices>& segmented_indices,
																	double reseg_radius, double boundary_radius) {
	grasp::StopWatch timer;
	timer.start();
	NormalCloudPtr normals = NormalCloudPtr(new NormalCloud());
	NormalEstimator normal_est;
	normal_est.setRadius(0.015);
	pcl::PointIndicesPtr indices(new pcl::PointIndices(apply_indices));
	pcl::IndicesPtr indices_ptr(new std::vector<int>(indices->indices));
	KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
	tree->setInputCloud(segmented_cloud, indices_ptr);
	normal_est.setSearchMethod(tree);
	normal_est.computeNormals(segmented_cloud, normals, indices, viewpoint_);

	timer.stopAndPrintTime("reseg(normal)");
	timer.start();

	// put a mark to a point that is located near a boundary
	std::vector<bool> is_near_boundary(segmented_cloud->size(), false);  // near boundary flag
	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	std::vector<int> nn2_indices;
	std::vector<float> nn2_distances;
	pcl::BoundaryEstimation<PointT, pcl::Normal, pcl::Boundary> bound_est;
	Eigen::Vector4f u = Eigen::Vector4f::Zero();
	Eigen::Vector4f v = Eigen::Vector4f::Zero();
	for (size_t idx = 0; idx < apply_indices.indices.size(); ++idx) {
		int cur_idx = apply_indices.indices[idx];
		if (!pcl::isFinite(segmented_cloud->points[cur_idx])) {
			continue;
		}

		if (tree->radiusSearch(segmented_cloud->points[cur_idx], boundary_radius, nn_indices, nn_distances) <= 1) {
			continue;
		}

		bound_est.getCoordinateSystemOnPlane(normals->points[idx], u, v);

		if (bound_est.isBoundaryPoint(*segmented_cloud, segmented_cloud->points[cur_idx],
																	nn_indices, u, v , bound_est.getAngleThreshold())) {
			if (tree->radiusSearch(segmented_cloud->points[cur_idx], reseg_radius, nn2_indices, nn2_distances) > 1) {
				for (size_t j = 1; j < nn2_indices.size(); ++j) {
					is_near_boundary[nn2_indices[j]] = true;
				}
			}
		}
	}

	timer.stopAndPrintTime("reseg(putmark)");
	timer.start();

	// adopt euclidean clustering to a pointcloud without points located near the boundary
	std::vector<std::vector<int> > clusters;
	std::vector<bool> processed(segmented_cloud->size(), false);
	for (size_t i = 0; i < apply_indices.indices.size(); ++i) {
		int cur_idx = apply_indices.indices[i];
		if (processed[cur_idx] || is_near_boundary[cur_idx]) continue;

		clusters.push_back(std::vector<int>());
		std::vector<int>& current_cluster = clusters.back();
		int cii = 0;

		current_cluster.push_back(cur_idx);
		processed[cur_idx] = true;

		while (cii < static_cast<int>(current_cluster.size())) {
			if (tree->radiusSearch(segmented_cloud->points[current_cluster[cii]], tolerance, nn_indices, nn_distances) < 1) {
				cii++;
				continue;
			}

			for (size_t nii = 0; nii < nn_indices.size(); ++nii) {
				if (processed[nn_indices[nii]] || is_near_boundary[nn_indices[nii]]) continue;
				current_cluster.push_back(nn_indices[nii]);
				processed[nn_indices[nii]] = true;
			}
			cii++;
		}
	}

	timer.stopAndPrintTime("reseg(clustering)");
	timer.start();

	pcl::IndicesPtr indices_incluster(new std::vector<int>());
	std::vector<int> cluster_id(segmented_cloud->size(), -1);

	// remove a cluster that number of points is too small or too large
	for (size_t i = 0; i < clusters.size(); ++i) {
		if (clusters[i].size() < 10 || clusters[i].size() > 25000) {
			for (size_t j = 0; j < clusters[i].size(); ++j) {
				processed[clusters[i][j]] = false;
			}
			clusters[i].clear();
		} else {
			for (size_t j = 0; j < clusters[i].size(); ++j) {
				indices_incluster->push_back(clusters[i][j]);
				cluster_id[clusters[i][j]] = i;
			}
		}
	}

	// bind cluster id to a point that does not belong to any clusters
	if (!indices_incluster->empty()) {
		KdTreePointTPtr tree_cluster = KdTreePointTPtr(new KdTreePointT(false));
		tree_cluster->setInputCloud(segmented_cloud, indices_incluster);
		std::vector<int> nn_cluster_indices(1);
		std::vector<float> nn_cluster_distances(1);
		for (size_t i = 0; i < apply_indices.indices.size(); ++i) {
			int cur_idx = apply_indices.indices[i];
			if (processed[cur_idx]) continue;
			if (tree_cluster->nearestKSearch(segmented_cloud->points[cur_idx], 1, nn_cluster_indices, nn_cluster_distances) < 1) {
				continue;
			}
			cluster_id[cur_idx] = cluster_id[nn_cluster_indices[0]];
		}
	}

	// // bind cluster id to a point that does not belong to any clusters
	// std::vector<float> distances(segmented_cloud->size(), 100);
	// std::vector<int> cluster_id(segmented_cloud->size(), -1);
	// double search_radius = 4*reseg_radius;
	// for (size_t i = 0; i < clusters.size(); ++i) {
	// 	for (size_t j = 0; j < clusters[i].size(); ++j) {
	// 		if (tree->radiusSearch(segmented_cloud->points[clusters[i][j]], search_radius, nn_indices, nn_distances) < 1) {
	// 			continue;
	// 		}
	// 		for (size_t nii = 0; nii < nn_indices.size(); ++nii) {
	// 			if (processed[nn_indices[nii]]) continue;
	// 			if (distances[nn_indices[nii]] > nn_distances[nii]) {
	// 				distances[nn_indices[nii]] = nn_distances[nii];
	// 				cluster_id[nn_indices[nii]] = i;
	// 			}
	// 		}
	// 	}
	// }

	timer.stopAndPrintTime("reseg(bind)");

	for (size_t i = 0; i < apply_indices.indices.size(); ++i) {
		int cur_idx = apply_indices.indices[i];
		if (cluster_id[cur_idx] != -1) {
			clusters[cluster_id[cur_idx]].push_back(cur_idx);
		}
	}

	for (size_t i = 0; i < clusters.size(); ++i) {
		if (clusters[i].empty()) continue;
		pcl::PointIndices r;
		r.indices.resize(clusters[i].size());
		for (size_t j = 0; j < clusters[i].size(); ++j) {
			r.indices[j] = clusters[i][j];
		}
		segmented_indices.push_back(r);
	}

	if (segmented_indices.size() < 2) return false;
	return true;
}

/**
 * Remove planes from the point cloud.
 * The result is stored in @c segmented_cloud.
 */
void AbstractSegmenter::removePlane() {
	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);  // Maximum number of iterations before giving up.
	seg.setDistanceThreshold(dist_th);  // Determine how close a point must be to the model in order to be considered an inlier.

	int nr_points = segmented_cloud->points.size();

	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);

	for (int i = 0; ; i++) {
		if ((!use_rate_in_remove) && (!(i < num_plane_remove))) break;
		if ((use_rate_in_remove) && !(segmented_cloud->points.size() > (1.0-rate_plane_remove) * nr_points)) break;

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(segmented_cloud);
		seg.segment(*inliers_plane, *coefficients_plane);
		if (inliers_plane->indices.size() == 0) {
			std::ostream& os = cnoid::MessageView::mainInstance()->cout();
			os << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(segmented_cloud);
		extract.setIndices(inliers_plane);

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*segmented_cloud);
	}
}

void AbstractSegmenter::generateClusteredCloud() {
	clustered_cloud->clear();
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		int r, g, b;
		r = rand() % 255;
		g = rand() % 255;
		b = rand() % 255;

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
			pcl::PointXYZRGBA p;
			p.x = segmented_cloud->points[*pit].x;
			p.y = segmented_cloud->points[*pit].y;
			p.z = segmented_cloud->points[*pit].z;
			p.r = r;
			p.g = g;
			p.b = b;
			clustered_cloud->push_back(p);
		}
	}
}

Segmenter::Segmenter() {
}

Segmenter::~Segmenter() {
}

/**
 * Segment objects from the point cloud data.
 */
bool Segmenter::segment() {
	pcl::copyPointCloud(*cloud, *segmented_cloud);

	removePlane();

	if (segmented_cloud->points.size() < 1) return false;

	if (!clustering()) return false;

	return true;
}

/**
 * Run Euclidean clustering to segment objects.
 * The clustering result is stored in @c clusterd_cloud.
 */
bool Segmenter::clustering() {
	cluster_indices.clear();
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(segmented_cloud);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance);  // Spatial cluster tolerance as a measure in the L2 Euclidean space.
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(segmented_cloud);
	ec.extract(cluster_indices);

	if (cluster_indices.size() < 1) return false;

	generateClusteredCloud();

	return true;
}

double ConditionalSegmenter::squared_dist;
double ConditionalSegmenter::squared_vdist;
cnoid::Vector3 ConditionalSegmenter::viewpoint;


ConditionalSegmenter::ConditionalSegmenter() {
}

ConditionalSegmenter::~ConditionalSegmenter() {
}

bool ConditionalSegmenter::segment() {
	ConditionalSegmenter::squared_dist = tolerance * tolerance;
	ConditionalSegmenter::squared_vdist = vertical_tolerance_ * vertical_tolerance_;
	ConditionalSegmenter::viewpoint = viewpoint_;

	BoxCloudHolder* bch = BoxCloudHolder::instance();
	if (bch->hasBox()) {
		// pcl::SegmentDifferences<pcl::PointXYZ> seg;
    // seg.setInputCloud(cloud);
    // seg.setTargetCloud(pch->getBoxModelPtr());
    // PointCloudTPtr tmp = PointCloudTPtr(new PointCloudT());
    // seg.setDistanceThreshold(0.0025 * 2.0 * 0.0025 * 2.0);
    // seg.segment(*segmented_cloud);

		KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
		PointCloudTConstPtr box_model = bch->getBoxModelPtr();
		std::vector<int> nn_indices(1);
		std::vector<float> nn_distances(1);

		std::vector<int> overlap_indices;
		PointCloudTPtr tmp_cloud = PointCloudTPtr(new PointCloudT());
		pcl::copyPointCloud(*cloud, *tmp_cloud);
		*tmp_cloud += *(bch->getBoxPtr());
		tree->setInputCloud(box_model);
		double dist_th = 0.0025 * 2.0 * 0.0025 * 2.0;
		for (size_t i = 0; i < tmp_cloud->points.size(); i++) {
			if (!isFinite(tmp_cloud->points[i]))
				continue;
			if (!tree->nearestKSearch(tmp_cloud->points[i], 1, nn_indices, nn_distances))
				continue;

			if (nn_distances[0] < dist_th)
				overlap_indices.push_back(i);
		}
		PointCloudTPtr box = PointCloudTPtr(new PointCloudT());
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(tmp_cloud);
		extract.setIndices(boost::make_shared<std::vector<int> >(overlap_indices));
		extract.setNegative(false);
		extract.filter(*box);
		bch->setBoxCloud(box);

		extract.setNegative(true);
		extract.filter(*segmented_cloud);
	} else {
		pcl::copyPointCloud(*cloud, *segmented_cloud);
	}

	removePlane();

	if (segmented_cloud->points.size() < 1) return false;

	if (!clustering()) return false;

	return true;
}

bool ConditionalSegmenter::clustering() {
	cluster_indices.clear();
	pcl::ConditionalEuclideanClustering<PointT> ec;
	// double max_tol = std::max(tolerance, vertical_tolerance_);
	double tol = std::sqrt(tolerance * tolerance + vertical_tolerance_ * vertical_tolerance_);
	ec.setConditionFunction(&ConditionalSegmenter::conditionFunction);
	ec.setInputCloud(segmented_cloud);
	ec.setClusterTolerance(tol);  // Spatial cluster tolerance as a measure in the L2 Euclidean space.
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.segment(cluster_indices);
	// ec.extract (cluster_indices);

	if (cluster_indices.size() < 1) return false;

	generateClusteredCloud();

	return true;
}

bool ConditionalSegmenter::conditionFunction(const PointT& seed_p, const PointT& cand_p, float squared_dist) {
	cnoid::Vector3 seed = cnoid::Vector3(seed_p.x, seed_p.y, seed_p.z);
	cnoid::Vector3 cand = cnoid::Vector3(cand_p.x, cand_p.y, cand_p.z);
	cnoid::Vector3 diff = seed - cand;
	cnoid::Vector3 dir = ConditionalSegmenter::viewpoint - (0.5 * (seed + cand));
	double dir_squared_norm = dir.squaredNorm();
	double v_len = diff.dot(dir);
	return (((diff.cross(dir)).squaredNorm() < ConditionalSegmenter::squared_dist * dir_squared_norm) &&
					(v_len * v_len < ConditionalSegmenter::squared_vdist * dir_squared_norm));
}

LccpSegmenter::LccpSegmenter() :
	voxel_resolution_(0.0025),
	seed_resolution_(0.01),
	spatial_importance_(0.4),
	normal_importance_(1.0),
	color_importance_(0.0),
	concavility_tolerance_(10),
	min_cluster_size_(20) {
}

LccpSegmenter::~LccpSegmenter() {
}

bool LccpSegmenter::segment() {
	BoxCloudHolder* bch = BoxCloudHolder::instance();
	if (bch->hasBox()) {
		KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
		PointCloudTConstPtr box_model = bch->getBoxModelPtr();
		std::vector<int> nn_indices(1);
		std::vector<float> nn_distances(1);

		std::vector<int> overlap_indices;
		PointCloudTPtr tmp_cloud = PointCloudTPtr(new PointCloudT());
		pcl::copyPointCloud(*cloud, *tmp_cloud);
		*tmp_cloud += *(bch->getBoxPtr());
		tree->setInputCloud(box_model);
		double dist_th = 0.0025 * 2.0 * 0.0025 * 2.0;
		for (size_t i = 0; i < tmp_cloud->points.size(); i++) {
			if (!isFinite(tmp_cloud->points[i]))
				continue;
			if (!tree->nearestKSearch(tmp_cloud->points[i], 1, nn_indices, nn_distances))
				continue;

			if (nn_distances[0] < dist_th)
				overlap_indices.push_back(i);
		}
		PointCloudTPtr box = PointCloudTPtr(new PointCloudT());
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(tmp_cloud);
		extract.setIndices(boost::make_shared<std::vector<int> >(overlap_indices));
		extract.setNegative(false);
		extract.filter(*box);
		bch->setBoxCloud(box);

		extract.setNegative(true);
		extract.filter(*segmented_cloud);
	} else {
		pcl::copyPointCloud(*cloud, *segmented_cloud);
	}

	removePlane();

	if (segmented_cloud->points.size() < 1) return false;

	if (!clustering()) return false;
	return true;
}

void LccpSegmenter::setSuperVoxelParameter(double voxel_resolution, double seed_resolution, double spatial, double normal, double color) {
	voxel_resolution_ = voxel_resolution;
	seed_resolution_ = seed_resolution;
	spatial_importance_ = spatial;
	normal_importance_ = normal;
	color_importance_ = color;
}

void LccpSegmenter::setConcavityTolerance(double angle_degree) {
	concavility_tolerance_ = angle_degree;
}

void LccpSegmenter::setMinClusterSize(int num_points) {
	min_cluster_size_ = num_points;
}

bool LccpSegmenter::clustering() {
	// super voxel clustering
	pcl::SupervoxelClustering<PointT> super(voxel_resolution_, seed_resolution_, false);
  super.setInputCloud(segmented_cloud);
  super.setColorImportance(color_importance_);
  super.setSpatialImportance(spatial_importance_);
  super.setNormalImportance(normal_importance_);
	super.setViewPoint(viewpoint_.cast<float>());

	std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
	super.extract(supervoxel_clusters);

	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	// segmentation
	pcl::LCCPSegmentation<PointT> seg;
	seg.setConcavityToleranceThreshold(concavility_tolerance_);
	seg.setMinSegmentSize(min_cluster_size_);
	seg.setKFactor(1);
	seg.setSanityCheck(true);
	seg.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	seg.segment();

	std::map<uint32_t, std::set<uint32_t> > seg_to_sv_map;
	seg.getSegmentToSupervoxelMap(seg_to_sv_map);

	clustered_cloud->clear();
	segmented_cloud->clear();
	cluster_indices.clear();
	int i = 0;
	std::map<uint32_t, std::set<uint32_t> >::iterator seg_ite;
	for (seg_ite = seg_to_sv_map.begin(); seg_ite != seg_to_sv_map.end(); ++seg_ite) {
		uint8_t r = static_cast<uint8_t>((rand() % 256));
    uint8_t g = static_cast<uint8_t>((rand() % 256));
    uint8_t b = static_cast<uint8_t>((rand() % 256));
		uint32_t color = static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b);
		pcl::PointIndices indices;
		std::set<uint32_t>::iterator sv_ite;
		for (sv_ite = (*seg_ite).second.begin(); sv_ite != (*seg_ite).second.end(); ++ sv_ite) {
			PointCloudT::Ptr tmp = supervoxel_clusters[(*sv_ite)]->voxels_;
			PointCloudT::iterator it;
			for (it = tmp->begin(); it != tmp->end(); ++it) {
				segmented_cloud->push_back(*it);
				indices.indices.push_back(i++);
			}
			ColorPointCloud rgb_copy;
			copyPointCloud(*tmp, rgb_copy);
			ColorPointCloud::iterator rgb_copy_itr = rgb_copy.begin();
			for ( ; rgb_copy_itr != rgb_copy.end (); ++rgb_copy_itr) {
				rgb_copy_itr->rgba = color;
			}
			*clustered_cloud += rgb_copy;
		}
		cluster_indices.push_back(indices);
	}
	if (clustered_cloud->size() < 1) return false;

	return true;
}

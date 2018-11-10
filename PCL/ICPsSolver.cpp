/**
 * @file   ICPsSolver.cpp
 * @author Akira Ohchi
*/

#include "ICPsSolver.h"

#include <limits>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <pcl/registration/icp.h>

ICPsSolver::ICPsSolver() :
	max_iteration_(50),
	trans_eps_(1e-6),
	fit_eps_(0.01) {
}

ICPsSolver::~ICPsSolver() {
}

void ICPsSolver::setSceneCloud(PointCloudTConstPtr scene_cloud) {
	scene_cloud_ = scene_cloud;
}

void ICPsSolver::setObjModelCloud(PointCloudTConstPtr obj_cloud) {
	obj_cloud_ = obj_cloud;
}

void ICPsSolver::setMaxIteration(int max_iteration) {
	max_iteration_ = max_iteration;
}

void ICPsSolver::setTransEps(double trans_eps) {
	trans_eps_ = trans_eps;
}

void ICPsSolver::setFitEps(double fit_eps) {
	fit_eps_ = fit_eps;
}

void ICPsSolver::getBestMatrix(Eigen::Matrix4f& trans_matrix) const {
	trans_matrix = trans_matrix_;
}

double ICPsSolver::getBestScore() const {
	return best_score_;
}

ICPsSolverSerial::ICPsSolverSerial() {
}

ICPsSolverSerial::~ICPsSolverSerial() {
}

bool ICPsSolverSerial::solveICPs(const InputDataVec& inputs) {
	return solveICPs(inputs.begin(), inputs.end());
}

bool ICPsSolverSerial::solveICPs(InputDataVecConstIte inputs_begin, InputDataVecConstIte inputs_end) {
	has_sol_ = false;
	score_ = std::numeric_limits<double>::max();

	const float outlier_coeff = 0.00001;
	const float outlier_ratio_th = 0.4;

	KdTreePointTPtr scene_tree = KdTreePointTPtr(new KdTreePointT(false));
	KdTreePointTPtr obj_tree = KdTreePointTPtr(new KdTreePointT(false));
	scene_tree->setInputCloud(scene_cloud_);
	obj_tree->setInputCloud(obj_cloud_);

	PointCloudTPtr aligned_cloud = PointCloudTPtr(new PointCloudT());

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputTarget(scene_cloud_);
	icp.setSearchMethodTarget(scene_tree, true);
	icp.setMaxCorrespondenceDistance(0.015);
	icp.setMaximumIterations(max_iteration_);
	icp.setTransformationEpsilon(trans_eps_);
	icp.setEuclideanFitnessEpsilon(fit_eps_);

	PointCloudTPtr aligned_scene_cloud = PointCloudTPtr(new PointCloudT());
	size_t scene_cloud_size = scene_cloud_->size();
	aligned_scene_cloud->resize(scene_cloud_size);

	Eigen::Matrix4f trans_mat;
	for (InputDataVecConstIte it = inputs_begin; it != inputs_end; ++it) {
		aligned_cloud->clear();

		icp.setInputSource(it->target_cloud);
		icp.align(*aligned_cloud, it->init_matrix);
		if (icp.hasConverged()) {
			trans_mat = icp.getFinalTransformation();
		} else {
			continue;
		}

		// sTo = sTc * cTo
		// s:scene (world) coordinate
		// c:camera coordinate
		// o:object coordinate
		Eigen::Matrix4f aligned_matrix = trans_mat * it->pose_matrix;

		// "alinged_scene_cloud" = oTs * "scnene_cloud"
		for (size_t i = 0; i < scene_cloud_size; ++i) {
			const PointT &src = scene_cloud_->points[i];
			PointT &tgt = aligned_scene_cloud->points[i];
			float x = src.x - aligned_matrix(0, 3);
			float y = src.y - aligned_matrix(1, 3);
			float z = src.z - aligned_matrix(2, 3);
			tgt.x = aligned_matrix(0, 0) * x + aligned_matrix(1, 0) * y + aligned_matrix(2, 0) * z;
			tgt.y = aligned_matrix(0, 1) * x + aligned_matrix(1, 1) * y + aligned_matrix(2, 1) * z;
			tgt.z = aligned_matrix(0, 2) * x + aligned_matrix(1, 2) * y + aligned_matrix(2, 2) * z;
		}

		double icp_score = calcScore(aligned_scene_cloud, obj_tree.get(), 0.1);

		if (icp_score > score_) {
			continue;
		}

		int out_nr = getNumOutlierPoints(aligned_cloud, scene_tree.get(), 0.005);
		double point_size = static_cast<double>(aligned_cloud->points.size());
		double point_size_inv = 1.0 / point_size;
		double outlier_score = outlier_coeff * static_cast<double>(out_nr) * point_size_inv;
		double total_score = icp_score + outlier_score;

		if (total_score > score_) {
			continue;
		}

		int out_nr2 = getNumOutlierPoints(scene_cloud_, aligned_cloud, 0.005);
		double outlier_ratio = static_cast<double>(out_nr2) * point_size_inv;

		if (outlier_ratio > outlier_ratio_th) {
			continue;
		}

		has_sol_ = true;
		score_ = total_score;
		best_score_ = score_;
		trans_matrix_ = aligned_matrix;
	}
	return has_sol_;
}

int ICPsSolverSerial::getNumOutlierPoints(const PointCloudTConstPtr& obj, const PointCloudTConstPtr& scene, double max_range) const {
	KdTreePointT tree(false);
	tree.setInputCloud(scene);
	return getNumOutlierPoints(obj, &tree, max_range);
}

int ICPsSolverSerial::getNumOutlierPoints(const PointCloudTConstPtr& obj, KdTreePointT* tree, double max_range) const {
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	int nr_out = 0;
	double max_range_squre = max_range * max_range;
	size_t point_size = obj->points.size();
	for (size_t i = 0; i < point_size; i++) {
		tree->nearestKSearch(obj->points[i], 1, nn_indices, nn_dists);
		if (nn_dists[0] > max_range_squre) {
			nr_out++;
		}
	}
	return nr_out;
}

double ICPsSolverSerial::calcScore(const PointCloudTConstPtr& obj, KdTreePointT* tree, double max_range) const {
	double fitness_score = 0.0;
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	int nr = 0;
	double max_range_squre = max_range;
	size_t point_size = obj->points.size();
	for (size_t i = 0; i < point_size; i++) {
		tree->nearestKSearch(obj->points[i], 1, nn_indices, nn_dists);
		if (nn_dists[0] < max_range_squre) {
			nr++;
			fitness_score += nn_dists[0];
		}
	}
	if (nr > 0)
		return (fitness_score / nr);
	else
		return std::numeric_limits<double>::max();
}

bool ICPsSolverSerial::hasSol() const {
	return has_sol_;
}

double ICPsSolverSerial::getScore() const {
	return score_;
}

void ICPsSolverSerial::clear() {
	has_sol_ = false;
}

ICPsSolverParallel::ICPsSolverParallel() {
}

ICPsSolverParallel::~ICPsSolverParallel() {
}

bool ICPsSolverParallel::solveICPs(const InputDataVec& inputs) {
	int num_threads = boost::thread::hardware_concurrency();

	boost::thread_group* thr_grp = new boost::thread_group();

	std::vector<ICPsSolverSerial*> icps(num_threads);
	std::vector<InputDataVecConstIte> begins(num_threads);
	std::vector<InputDataVecConstIte> ends(num_threads);

	for (int i = 0; i < num_threads; i++) {
		int n = inputs.size() / num_threads;
		int r = inputs.size() % num_threads;
		int offset = (i > r) ? r : i;
		int offset_n = (i+1 > r) ? r : i+1;
		int start = i * n + offset;
		int end = (i+1) * n + offset_n;
		begins[i] = inputs.begin() + start;
		ends[i] = inputs.begin() + end;
		icps[i] = new ICPsSolverSerial;
		icps[i]->setMaxIteration(max_iteration_);
		icps[i]->setTransEps(trans_eps_);
		icps[i]->setFitEps(fit_eps_);
		icps[i]->setSceneCloud(scene_cloud_);
		icps[i]->setObjModelCloud(obj_cloud_);
		thr_grp->create_thread(boost::bind(&ICPsSolverSerial::solveICPs, icps[i], begins[i], ends[i]));
	}
	thr_grp->join_all();
	delete thr_grp;

	bool ret = false;
	best_score_ = std::numeric_limits<double>::max();
	for (int i = 0; i < num_threads; i++) {
		if (!icps[i]->hasSol()) continue;
		if (best_score_ > icps[i]->getScore()) {
			ret = true;
			best_score_ = icps[i]->getScore();
			icps[i]->getBestMatrix(trans_matrix_);
		}
	}

	for (int i = 0; i < num_threads; i++) {
		delete icps[i];
	}

	return ret;
}

void ICPsSolverParallel::clear() {
}

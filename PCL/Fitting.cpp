/**
 * @file   Fitting.cpp
 * @author Akira Ohchi
*/

#include "Fitting.h"

#include <limits>
#include <string>
#include <algorithm>
#include <vector>

#include <QSettings>
#include <QString>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/extract_indices.h>
#undef PCL_NO_PRECOMPILE
#include <pcl/sample_consensus/ransac.h>
#include "sac_model_box.h"
// #include <pcl/features/rsd.h>
// use a copy of rsd.h in original PCL1.7.0 because rsd.h is not found
// in PCL ver 1.7.0-2+precise4.
#include "rsd.h"

#include <cnoid/ExecutablePath>

#include "CVFHDescriptorManipulator.h"
#include "PointCloudHandler.h"
#include "PointCloudDrawer.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/DrawUtility.h"
#include "../Grasp/VectorMath.h"
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

namespace {
	double _drand() {
#ifdef WIN32
		return static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
#else
		return drand48();
#endif
	}
}

void Fitting::fit() {
	init();
	readParams();

	PointCloudTPtr cloud(new PointCloudT());
	getModelPointCloud(cloud);

	PointCloudTPtr sampled_cloud(new PointCloudT());
	downSampling(cloud, sampled_cloud, param.voxel_size);

	NormalCloud::Ptr normal(new NormalCloud());
	computeNormal(sampled_cloud, normal);

	DrawUtility* drawer = DrawUtility::instance();
	drawer->clear();
	PointCloudDraw point_drawer;

	int n_points = sampled_cloud->points.size();

	std::vector<Vector3> pos;
	std::vector<Vector3> dir;
	std::vector<double> radius, len;
	PointCloudTPtr target_cloud(new PointCloudT());
	copyPointCloud(*sampled_cloud, *target_cloud);
	if (param.do_cylinder_fitting) {
		if (fittingCylinders(sampled_cloud, normal, pos, dir, radius, len)) {
			for (int i = 0; i < pos.size(); i++) {
				addCylinder(pos[i], dir[i], radius[i], len[i]);
				if (param.show_fittings_shape) {
					drawer->cylinders.push_back(Cylinders(pos[i], dir[i], radius[i], len[i], Vector3(1.0, 0, 0), 0.6));
				}
				std::cout << "cylinder:" << std::endl;
				std::cout << " pos " << pos[i].transpose() << std::endl;
				std::cout << " dir " << dir[i].transpose() << std::endl;
				std::cout << " radius " << radius[i] << std::endl;
				std::cout << " len " << len[i] << std::endl;

				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
				getCylinderInlier(target_cloud, pos[i], dir[i], radius[i], len[i], inliers);

				PointCloudTPtr cylinder_cloud(new PointCloudT());
				copyPointCloud(*target_cloud, inliers->indices, *cylinder_cloud);
				if (param.show_fittings) {
					point_drawer.addPointCloud(cylinder_cloud, Vector3(0, 1, 0));
				}

				pcl::ExtractIndices<pcl::PointXYZ> extract;
				extract.setInputCloud(target_cloud);
				extract.setIndices(inliers);
				extract.setNegative(true);
				extract.filter(*target_cloud);
				pcl::ExtractIndices<pcl::Normal> extract_n;
				extract_n.setInputCloud(normal);
				extract_n.setIndices(inliers);
				extract_n.setNegative(true);
				extract_n.filter(*normal);
			}
		}
	}

	PointCloudTPtr residual_cloud(new PointCloudT());

	if (param.do_box_fitting) {
		int iter_count = 0;
		while (target_cloud->points.size() > 0.1 * n_points) {
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
			tree->setInputCloud(target_cloud);

			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance(0.01);
			ec.setMinClusterSize(1);
			ec.setMaxClusterSize(100000);
			ec.setSearchMethod(tree);
			ec.setInputCloud(target_cloud);
			ec.extract(cluster_indices);

			PointCloudTPtr tmp_cloud(new PointCloudT());
			NormalCloud::Ptr tmp_normal(new NormalCloud());

			for (int i = 0; i < cluster_indices.size(); i++) {
				if (cluster_indices[i].indices.size() < 0.1 * n_points) {
					PointCloudTPtr tmp2_cloud(new PointCloudT());
					copyPointCloud(*target_cloud, cluster_indices[i], *tmp2_cloud);
					*residual_cloud += *tmp2_cloud;
					continue;
				}
				Vector3 center;
				Vector3 edge;
				Matrix3 rot;
				std::vector<int> inlier;
				PointCloudTPtr cluster_cloud(new PointCloudT());
				NormalCloud::Ptr cluster_normal(new NormalCloud());
				copyPointCloud(*target_cloud, cluster_indices[i], *cluster_cloud);
				copyPointCloud(*normal, cluster_indices[i], *cluster_normal);
				if (fittingBox(cluster_cloud, cluster_normal, inlier, center, rot, edge, iter_count)) {
					pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
					getBoxInlier(cluster_cloud, center, rot, edge, inliers, 0.0);
					optimizeBoxInlier(cluster_cloud, center, rot, edge, inliers);
					getBoxInlier(cluster_cloud, center, rot, edge, inliers);
					addBox(rot, center, edge);

					if (param.show_fittings_shape) {
						drawer->boxes.push_back(Boxes(center, rot, edge, Vector3(1, 0, 0), 0.6));
					}
					std::cout << "box:" << std::endl;
					std::cout << " center " << center.transpose() << std::endl;
					std::cout << " R " << rot << std::endl;
					std::cout << " length " << edge.transpose() << std::endl;

					PointCloudTPtr box_cloud(new PointCloudT());
					copyPointCloud(*cluster_cloud, inliers->indices, *box_cloud);
					if (param.show_fittings) {
						point_drawer.addPointCloud(box_cloud, Vector3(1, 0, 0));
					}

					PointCloudTPtr outlier_cloud(new PointCloudT());
					NormalCloud::Ptr outlier_normal(new NormalCloud());

					pcl::ExtractIndices<pcl::PointXYZ> extract;
					extract.setInputCloud(cluster_cloud);
					extract.setIndices(inliers);
					extract.setNegative(true);
					extract.filter(*outlier_cloud);
					pcl::ExtractIndices<pcl::Normal> extract_n;
					extract_n.setInputCloud(cluster_normal);
					extract_n.setIndices(inliers);
					extract_n.setNegative(true);
					extract_n.filter(*outlier_normal);

					*tmp_cloud += *outlier_cloud;
					*tmp_normal += *outlier_normal;
				} else {
					*residual_cloud += *target_cloud;
					continue;
				}
			}
			copyPointCloud(*tmp_cloud, *target_cloud);
			copyPointCloud(*tmp_normal, *normal);
			iter_count++;
		}
	}
	*residual_cloud += *target_cloud;

	if (param.show_fittings) {
		point_drawer.addPointCloud(residual_cloud, Vector3(0.5, 0.5, 0.5));
		point_drawer.draw();
	}
	if (param.show_fittings_shape) {
		drawer->displayShapes();
	}
}

void Fitting::getCylinders(std::vector<Vector3>& center, std::vector<cnoid::Vector3>& dir,
													 std::vector<double>& radius, std::vector<double>& len) const {
	center = cylinder_center_;
	dir = cylinder_dir_;
	radius = cylinder_radius_;
	len = cylinder_length_;
}

void Fitting::getBoxes(std::vector<Matrix3>& r, std::vector<Vector3>& center, std::vector<Vector3>& edge) const {
	r = box_r_;
	center = box_center_;
	edge = box_edge_;
}

void Fitting::addCylinder(const cnoid::Vector3& center, const cnoid::Vector3& dir, double radius, double len) {
	cylinder_center_.push_back(center);
	cylinder_dir_.push_back(dir);
	cylinder_radius_.push_back(radius);
	cylinder_length_.push_back(len);
}

void Fitting::addBox(const cnoid::Matrix3& r, const cnoid::Vector3& center, const cnoid::Vector3& edge) {
	box_r_.push_back(r);
	box_center_.push_back(center);
	box_edge_.push_back(edge);
}

void Fitting::init() {
	box_r_.clear();
	box_center_.clear();
	box_edge_.clear();
	cylinder_dir_.clear();
	cylinder_center_.clear();
	cylinder_radius_.clear();
	cylinder_length_.clear();
}

void Fitting::getModelPointCloud(PointCloudTPtr cloud) const {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cnoid::ColdetModelPtr coldet = grasp::PlanBase::instance()->object()->coldetModel();
#else
	cnoid::ColdetModelPtr coldet = grasp::ColdetConverter::ConvertFrom(grasp::PlanBase::instance()->object()->collisionShape());
#endif
	PointCloudHandler::instance()->convertColdetModel2PointCloud(coldet, cloud, 5 * 10e-7);
}

void Fitting::staticalRemove(const PointCloudTConstPtr in, PointCloudTPtr out) const {
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(in);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*out);
}

void Fitting::downSampling(const PointCloudTConstPtr in, PointCloudTPtr out, double leafsize) const {
	pcl::VoxelGrid<PointT> vg;
	vg.setLeafSize(leafsize, leafsize, leafsize);
	vg.setInputCloud(in);
	vg.filter(*out);
}

void Fitting::computeNormal(const PointCloudTConstPtr in, NormalCloud::Ptr normal) const {
	NormalEstimator normal_est;
	// normal_est.setRadius(0.01);
	normal_est.useKSearch(true);
	normal_est.setK(10);
	normal_est.computeNormals(in, normal, Eigen::Vector3d(0, 0, 0));
}

bool Fitting::fittingBox(const PointCloudTConstPtr cloud, const NormalCloud::ConstPtr normal, std::vector<int>& inlier,
												 cnoid::Vector3& center, cnoid::Matrix3& rot, cnoid::Vector3& edge, int iter_count) const {
	pcl::SampleConsensusModelBox<PointT, pcl::Normal>::Ptr sac_box(new pcl::SampleConsensusModelBox<PointT, pcl::Normal>(cloud));
	sac_box->setInputNormals(normal);
	sac_box->setRadiusLimits(0.01, 0.2);
	sac_box->setOrthoEps(cos(M_PI/2.0-param.box_ortho_angle_tolerance));
	sac_box->setVoxelSize(param.voxel_size);
	if (iter_count == 0) {
		sac_box->setAcceptPointRatio(0.1);
	} else {
		sac_box->setAcceptPointRatio(param.box_require_point_ratio);
	}
	pcl::RandomSampleConsensus<PointT> sac(sac_box, param.box_dist_tolerance);
	sac.setDistanceThreshold(param.box_dist_tolerance);
	sac.setMaxIterations(1000);


	if (!sac.computeModel()) {
		std::cout << "fitting failed" << std::endl;
		return false;
	}

	Eigen::VectorXf coeff;
	sac.getModelCoefficients(coeff);
	sac.getInliers(inlier);

	std::vector<double> min_len(3, std::numeric_limits<double>::max());
	std::vector<double> max_len(3, -std::numeric_limits<double>::max());
	std::vector<Vector3> n(3);
	n[0] << coeff[0], coeff[1], coeff[2];
	n[1] << coeff[3], coeff[4], coeff[5];
	n[2] << coeff[6], coeff[7], coeff[8];

	rot = grasp::v3(n[0], n[1], n[2]);

	center << coeff[9], coeff[10], coeff[11];
	edge << coeff[12], coeff[13], coeff[14];

	return true;
}

bool Fitting::fittingCylinders(const PointCloudTConstPtr cloud, const NormalCloud::Ptr normal,
															 std::vector<cnoid::Vector3>& pos, std::vector<cnoid::Vector3>& dir,
															 std::vector<double>& radius, std::vector<double>& len) const {
	std::vector<pcl::PointIndices> clusters;
	PointNormalCloud::Ptr pointnormal_cloud (new PointNormalCloud());
	cluster(cloud, normal, pointnormal_cloud, clusters);

	PointCloudTPtr clustered_cloud(new PointCloudT());
	NormalCloud::Ptr clustered_normal(new NormalCloud());
	copyPointCloud(*pointnormal_cloud, *clustered_cloud);
	copyPointCloud(*pointnormal_cloud, *clustered_normal);

	RSDCloud::Ptr radii(new RSDCloud());
	computeRSD(clustered_cloud, clustered_normal, clusters, radii);

	int n_points = clustered_cloud->points.size();
	bool has_cylinder = false;
	while (clustered_cloud->points.size() > 0.05 * n_points) {
		// fitting cylinder
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		Vector3 c_pos, c_dir;
		double c_radius, c_len;
		fittingCylinder(clustered_cloud, clustered_normal, inliers, c_pos, c_dir, c_radius, c_len);

		int count = 0;
		double error_sum = 0;
		for (int i = 0; i < inliers->indices.size(); i++) {
			double r_min = radii->points[inliers->indices[i]].r_min;
			if (r_min < 0) continue;
			double error = fabs(fabs(c_radius) - r_min);
			error_sum += error * error;
			count++;
		}

		if (sqrt(error_sum/count) < param.cylinder_rsd_tolerance && (!inliers->indices.empty())) {
			pos.push_back(c_pos);
			dir.push_back(c_dir);
			radius.push_back(c_radius);
			len.push_back(c_len);

			pcl::PointIndices::Ptr inlier(new pcl::PointIndices());
			getCylinderInlier(clustered_cloud, c_pos, c_dir, c_radius, c_len, inlier);
			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud(clustered_cloud);
			extract.setIndices(inlier);
			extract.setNegative(true);
			extract.filter(*clustered_cloud);
			pcl::ExtractIndices<pcl::Normal> extract_n;
			extract_n.setInputCloud(clustered_normal);
			extract_n.setIndices(inlier);
			extract_n.setNegative(true);
			extract_n.filter(*clustered_normal);
			pcl::ExtractIndices<pcl::PrincipalRadiiRSD> extract_r;
			extract_r.setInputCloud(radii);
			extract_r.setIndices(inlier);
			extract_r.setNegative(true);
			extract_r.filter(*radii);
			has_cylinder = true;
		} else {
			return has_cylinder;
		}
	}
	return has_cylinder;
}

void Fitting::fittingCylinder(const PointCloudTConstPtr cloud, const NormalCloud::Ptr normal, pcl::PointIndices::Ptr inliers, Vector3& c_pos, Vector3& c_dir, double& c_radius, double& c_length) const {
	double radiusTolerance = 0.01;
	double minRadius = 0.01;
	double maxRadius = 0.1;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.01);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(param.cylinder_radius_tolerance);
  seg.setRadiusLimits(minRadius, maxRadius);
	Eigen::Vector3f axis(0, 0, 0);
	seg.setAxis(axis);
	seg.setEpsAngle(90.0/180.0*3.1416);

	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients());
	// pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	seg.setInputCloud(cloud);
	seg.setInputNormals(normal);
	seg.segment(*inliers, *coefficients_cylinder);

	c_pos << coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2];
	c_dir << coefficients_cylinder->values[3], coefficients_cylinder->values[4], coefficients_cylinder->values[5];
	c_radius = coefficients_cylinder->values[6];
	c_dir = grasp::unit(c_dir);

	PointCloudTPtr target_cloud(new PointCloudT());
	copyPointCloud(*cloud, *inliers, *target_cloud);
	if (inliers->indices.empty()) {
		return;
	}
	std::vector<double> lengSet;
	double squre_error = 0;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator it = target_cloud->begin(); it != target_cloud->end(); ++it) {
		Vector3 r = Vector3(it->x, it->y, it->z);
		lengSet.push_back(Vector3(r - c_pos).dot(c_dir));
		double error = ((r - c_pos) - c_pos - (r - c_pos).dot(c_dir) * c_dir).norm() - fabs(c_radius);
		squre_error += error*error;
	}
	sort(lengSet.begin(), lengSet.end());

	double max_len = lengSet.back() - lengSet.front();
	double max_gap = max_len * 0.1;
	double lmax = 0.0, lmin = 0.0, tmp_lmax, tmp_lmin;

	bool is_start = true;
	for (size_t i = 0; i < lengSet.size()-1; i++) {
		if (is_start) {
			tmp_lmin = lengSet[i];
			tmp_lmax = lengSet[i];
			is_start = false;
		}
		if (lengSet[i+1]-lengSet[i] > max_gap) {
			if ((lmax-lmin) < (tmp_lmax-tmp_lmin)) {
				lmax = tmp_lmax;
				lmin = tmp_lmin;
			}
			is_start = true;
		} else {
			tmp_lmax = lengSet[i+1];
		}
	}
	if ((lmax-lmin) < (tmp_lmax-tmp_lmin)) {
		lmax = tmp_lmax;
		lmin = tmp_lmin;
	}
	c_length = max_len;

	c_pos = c_pos + c_dir*(lmax+lmin)*0.5;
}


void Fitting::cluster(const PointCloudTConstPtr cloud, const NormalCloud::Ptr normal,
											PointNormalCloud::Ptr normals_filtered_cloud,
											std::vector<pcl::PointIndices>& clusters) const {
	std::vector<int> indices_out;
	std::vector<int> indices_in;

	filterNormalsWithHighCurvature(*normal, indices_out, indices_in, param.high_curvature);

	if (param.show_curvature_result) {
		showHighCurvatureCloud(cloud, indices_out, indices_in);
	}

  normals_filtered_cloud->width = static_cast<uint32_t>(indices_in.size());
  normals_filtered_cloud->height = 1;
  normals_filtered_cloud->points.resize(normals_filtered_cloud->width);

	for (size_t i = 0; i < indices_in.size(); i++) {
		normals_filtered_cloud->points[i].x = cloud->points[indices_in[i]].x;
    normals_filtered_cloud->points[i].y = cloud->points[indices_in[i]].y;
    normals_filtered_cloud->points[i].z = cloud->points[indices_in[i]].z;
	}

	pcl::search::KdTree<pcl::PointNormal>::Ptr normals_tree_filtered(new pcl::search::KdTree<pcl::PointNormal>());
	normals_tree_filtered->setInputCloud(normals_filtered_cloud);
	pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> n3d;
	// n3d.setRadiusSearch(0.01);
	n3d.setKSearch(10);
	n3d.setSearchMethod(normals_tree_filtered);
	n3d.setInputCloud(normals_filtered_cloud);
	n3d.compute(*normals_filtered_cloud);

	pcl::search::KdTree<pcl::PointNormal>::Ptr normals_tree(new pcl::search::KdTree<pcl::PointNormal>());
	normals_tree->setInputCloud(normals_filtered_cloud);

	extractEuclideanClusterSmooth(*normals_filtered_cloud, *normals_filtered_cloud,
																normals_tree,
																clusters);
	if (param.show_cluster_result) {
		showClusterCloud(normals_filtered_cloud, clusters);
	}
}

void Fitting::filterNormalsWithHighCurvature(const NormalCloud& cloud,
																							std::vector<int>& out, std::vector<int>& in,
																							float threshold) const {
	out.clear();
	in.clear();

	for (int i = 0; i < cloud.points.size(); i++) {
		if (cloud.points[i].curvature > threshold) {
			out.push_back(i);
		} else {
			in.push_back(i);
		}
	}
}

void Fitting::showHighCurvatureCloud(const PointCloudTConstPtr cloud, const std::vector<int>& out, const std::vector<int>& in) const {
	PointCloudDraw draw;

	PointCloudTPtr cloud_in(new PointCloudT());
	PointCloudTPtr cloud_out(new PointCloudT());
	copyPointCloud(*cloud, in, *cloud_in);
	copyPointCloud(*cloud, out, *cloud_out);

	draw.addPointCloud(cloud_in, Vector3(0.5, 0.5, 0.5));
	draw.addPointCloud(cloud_out, Vector3(1, 0, 0));
	draw.draw();
}

void Fitting::showClusterCloud(const PointNormalCloud::ConstPtr cloud, const std::vector<pcl::PointIndices>& clusters) const {
	PointCloudDraw draw;

#ifdef WIN32
	srand(1);
#else
	srand48(1);
#endif

	for (size_t i = 0; i < clusters.size(); i++) {
		PointCloudTPtr cluster_cloud(new PointCloudT());
		copyPointCloud(*cloud, clusters[i], *cluster_cloud);
		draw.addPointCloud(cluster_cloud, Vector3(_drand(), _drand(), _drand()));
	}

	draw.draw();
}

void Fitting::extractEuclideanClusterSmooth(const PointNormalCloud& cloud, const PointNormalCloud& normals,
																						const pcl::search::KdTree<pcl::PointNormal>::Ptr& tree,
																						std::vector<pcl::PointIndices>& clusters) const {
	std::vector<bool> processed(cloud.points.size(), false);

	double tolerance = param.cluster_tolerance;
	double eps_angle = param.cluster_eps_angle;
	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	// Process all points in the indices vector
	for (int i = 0; i < static_cast<int>(cloud.points.size()); i++) {
		if (processed[i])
			continue;

		std::vector<unsigned int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back(i);

		processed[i] = true;

		while (sq_idx < static_cast<int>(seed_queue.size())) {
			if (!tree->radiusSearch(seed_queue[sq_idx], tolerance, nn_indices, nn_distances)) {
				sq_idx++;
				continue;
			}

			for (size_t j = 0; j < nn_indices.size(); j++) {
				if (processed[nn_indices[j]])
					continue;

				double dot_p = normals.points[seed_queue[sq_idx]].normal[0] * normals.points[nn_indices[j]].normal[0]
					+ normals.points[seed_queue[sq_idx]].normal[1] * normals.points[nn_indices[j]].normal[1]
					+ normals.points[seed_queue[sq_idx]].normal[2] * normals.points[nn_indices[j]].normal[2];

				if (fabs(acos(dot_p)) < eps_angle) {
					processed[nn_indices[j]] = true;
					seed_queue.push_back(nn_indices[j]);
				}
			}
			sq_idx++;
		}

		if (seed_queue.size() >= param.cluster_min_points && seed_queue.size() <= 100000) {
			pcl::PointIndices r;
			r.indices.resize(seed_queue.size());
			for (size_t j = 0; j < seed_queue.size(); j++)
				r.indices[j] = seed_queue[j];

			std::sort(r.indices.begin(), r.indices.end());
			r.indices.erase(std::unique(r.indices.begin(), r.indices.end()), r.indices.end());

			r.header = cloud.header;
			clusters.push_back(r);
		}
	}
}

void Fitting::computeRSD(const PointCloudTConstPtr cloud, const NormalCloud::ConstPtr normal, const std::vector<pcl::PointIndices>& clusters, RSDCloud::Ptr radii) const {
	radii->points.resize(cloud->points.size());
	radii->width = radii->points.size();
	radii->height = 1;
	for (size_t idx = 0; idx < radii->points.size(); idx++) {
		radii->points[idx].r_min = -1;
	}

	// compute RSD
	for (int i = 0; i < clusters.size(); i++) {
		pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr tmp_radii(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
		pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
		PointCloudTPtr target_cloud(new PointCloudT());
		NormalCloud::Ptr target_normal(new NormalCloud());
		copyPointCloud(*cloud, clusters[i], *target_cloud);
		copyPointCloud(*normal, clusters[i], *target_normal);
		rsd.setInputCloud(target_cloud);
		rsd.setSearchSurface(target_cloud);
		rsd.setInputNormals(target_normal);
		rsd.setRadiusSearch(param.rsd_radius);
		rsd.setNrSubdivisions(param.rsd_subdiv);
		rsd.compute(*tmp_radii);

		for (int j = 0; j < tmp_radii->points.size(); j++) {
			radii->points[clusters[i].indices[j]] = tmp_radii->points[j];
		}
	}
}

void Fitting::getBoxInlier(const PointCloudTConstPtr cloud, const cnoid::Vector3& center, const cnoid::Matrix3& rot,
													 const cnoid::Vector3& edge, pcl::PointIndices::Ptr inliers, double margin) const {
	std::vector<cnoid::Vector3> n(3);
	n[0] = rot.col(0).transpose();
	n[1] = rot.col(1).transpose();
	n[2] = rot.col(2).transpose();
	inliers->indices.clear();
	for (int i = 0; i < cloud->points.size(); i++) {
		Vector3 target_p(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		bool is_inlier = true;
		for (int j = 0; j < 3; j++) {
			if (fabs((target_p - center).dot(n[j])) > edge[j]/2.0 + margin) {
				is_inlier = false;
				break;
			}
		}
		if (is_inlier) {
			inliers->indices.push_back(i);
		}
	}
}

void Fitting::optimizeBoxInlier(const PointCloudTConstPtr cloud, cnoid::Vector3& center,
																const cnoid::Matrix3& rot, cnoid::Vector3& edge,
																const pcl::PointIndices::Ptr inlier) const {
	std::vector<Vector3> n(3);
	n[0] = rot.col(0).transpose();
	n[1] = rot.col(1).transpose();
	n[2] = rot.col(2).transpose();

	std::vector<std::vector<double> > c(3);
	for (int i = 0; i < inlier->indices.size(); i++) {
		Vector3 target_p(cloud->points[inlier->indices[i]].x, cloud->points[inlier->indices[i]].y, cloud->points[inlier->indices[i]].z);
		for (int j = 0; j < 3; j++) {
			c[j].push_back(n[j].dot(target_p - center));
		}
	}

	for (int i = 0; i < 3; i++) {
		int min_idx = 0;
		int max_idx = c[i].size() -1;
		double sec_len = 0.0025;
		int dens = floor(0.75 * c[i].size() * (0.0025/edge[i]));
		std::sort(c[i].begin(), c[i].end());
		for (int j = 0; j < c[i].size() - dens; j++) {
			if (c[i][j+dens] - c[i][j] < sec_len) {
				break;
			}
			min_idx = j;
		}
		for (int j = c[i].size()-1; j >= dens + min_idx; j--) {
			if(c[i][j] - c[i][j-dens] < sec_len) {
				break;
			}
			max_idx = j;
		}
		edge[i] = c[i][max_idx] - c[i][min_idx];
		center += (c[i][max_idx] + c[i][min_idx])/2.0 * n[i];
	}
}

void Fitting::getCylinderInlier(const PointCloudTConstPtr cloud, const cnoid::Vector3& pos, const cnoid::Vector3& dir,
																double radius, double len, pcl::PointIndices::Ptr inliers) const {
	double margin = 0.0025;
	for (int i = 0; i < cloud->points.size(); i++) {
		Vector3 target_p(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		bool is_inlier = true;
		double length = (target_p - pos).dot(dir);
		if (fabs(length) > (len/2.0 + margin)) {
			continue;
		}
		if (((pos + length * dir) - target_p).norm() > radius + margin) {
			continue;
		}
		inliers->indices.push_back(i);
	}
}

void Fitting::saveNormal(const PointCloudTConstPtr cloud, const NormalCloud::ConstPtr normal, const std::string& filename) const {
	pcl::PointCloud<pcl::PointNormal>::Ptr normalpoint_cloud(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields(*cloud, *normal, *normalpoint_cloud);
	pcl::io::savePCDFileASCII(filename, *normalpoint_cloud);
}

void Fitting::readParams() {
		QString default_path = QString::fromStdString(cnoid::executableTopDirectory()
																								+ "/extplugin/graspPlugin/PCL/");

	QSettings setting(default_path + "config.ini", QSettings::IniFormat);
	setting.beginGroup("Fitting");

	param.voxel_size                = setting.value("sampling_density", 0.001).toDouble();
	param.high_curvature            = setting.value("high_curvature", 1.0).toDouble();
	param.show_curvature_result     = setting.value("show_curvature_result", false).toBool();
	param.cluster_tolerance         = setting.value("cluster_tolerance", 0.015).toDouble();
	param.cluster_eps_angle         = setting.value("cluster_eps_angle", 0.125).toDouble();
	param.cluster_min_points        = setting.value("clsuter_min_points", 50).toInt();
	param.show_cluster_result       = setting.value("show_cluster_result", false).toBool();
	param.rsd_radius                = setting.value("rsd_radius", 0.03).toDouble();
	param.rsd_subdiv                = setting.value("rsd_subdiv", 5).toInt();
	param.cylinder_rsd_tolerance    = setting.value("cylinder_rsd_tolerance", 0.01).toDouble();
	param.cylinder_radius_tolerance = setting.value("cylinder_radius_tolerance", 0.01).toDouble();
	param.box_ortho_angle_tolerance = setting.value("box_ortho_angle_tolerance", 0.25).toDouble();
	param.box_require_point_ratio   = setting.value("box_require_point_ratio", 0.5).toDouble();
	param.box_dist_tolerance        = setting.value("box_dist_tolerance", 0.0025).toDouble();
	param.show_fittings             = setting.value("show_fitting_points", false).toBool();
	param.show_fittings_shape       = setting.value("show_fitting_shapes", true).toBool();
	param.do_box_fitting            = setting.value("do_box_fitting", true).toBool();
	param.do_cylinder_fitting       = setting.value("do_cylinder_fitting", true).toBool();
	setting.endGroup();
}

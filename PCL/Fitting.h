/**
 * @file   Fitting.h
 * @author Akira Ohchi
*/

#ifndef _PCL_FITTING_H_
#define _PCL_FITTING_H_

#include <vector>
#include <string>

#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

#include <cnoid/EigenTypes>

#include "PointCloudTypes.h"

class Fitting {
	public:
	/* typedef pcl::PointXYZ PointT; */
	/* typedef pcl::PointCloud<PointT> Cloud; */
	typedef pcl::PointCloud<pcl::Normal> NormalCloud;
	typedef pcl::PointCloud<pcl::PointNormal> PointNormalCloud;
	typedef pcl::PointCloud<pcl::PrincipalRadiiRSD> RSDCloud;

	void fit();
	void getCylinders(std::vector<cnoid::Vector3>& center, std::vector<cnoid::Vector3>& dir,
										std::vector<double>& radius, std::vector<double>& len) const;
	void getBoxes(std::vector<cnoid::Matrix3>& r, std::vector<cnoid::Vector3>& center, std::vector<cnoid::Vector3>& edge) const;

 private:
	std::vector<cnoid::Matrix3> box_r_;
	std::vector<cnoid::Vector3> box_center_;
	std::vector<cnoid::Vector3> box_edge_;
	std::vector<cnoid::Vector3> cylinder_dir_;
	std::vector<cnoid::Vector3> cylinder_center_;
	std::vector<double>         cylinder_radius_;
	std::vector<double>         cylinder_length_;

	void init();
	void addCylinder(const cnoid::Vector3& center, const cnoid::Vector3& dir, double radius, double len);
	void addBox(const cnoid::Matrix3& r, const cnoid::Vector3& center, const cnoid::Vector3& edge);
	void getModelPointCloud(PointCloudTPtr cloud) const;
	void staticalRemove(const PointCloudTConstPtr in, PointCloudTPtr out) const;
	void downSampling(const PointCloudTConstPtr in, PointCloudTPtr out, double leafsize) const;
	void computeNormal(const PointCloudTConstPtr in, NormalCloud::Ptr normal) const;
	bool fittingBox(const PointCloudTConstPtr cloud, const NormalCloud::ConstPtr normal, std::vector<int>& inlier,
									cnoid::Vector3& center, cnoid::Matrix3& rot, cnoid::Vector3& edge, int iter_count = 0) const;
	bool fittingCylinders(const PointCloudTConstPtr cloud, const NormalCloud::Ptr normal,
												std::vector<cnoid::Vector3>& pos, std::vector<cnoid::Vector3>& dir,
												std::vector<double>& radius, std::vector<double>& len) const;
	void fittingCylinder(const PointCloudTConstPtr cloud, const NormalCloud::Ptr normal,
												 pcl::PointIndices::Ptr inliers, cnoid::Vector3& pos, cnoid::Vector3& dir,
												 double& radius, double& len) const;
	void cluster(const PointCloudTConstPtr cloud, const NormalCloud::Ptr normal, PointNormalCloud::Ptr pointnormal,
							 std::vector<pcl::PointIndices>& clusters) const;
	void filterNormalsWithHighCurvature(const NormalCloud& cloud,
																			std::vector<int>& out, std::vector<int>& in,
																			float threshold) const;
	void showHighCurvatureCloud(const PointCloudTConstPtr cloud, const std::vector<int>& out, const std::vector<int>& in) const;
	void showClusterCloud(const PointNormalCloud::ConstPtr cloud, const std::vector<pcl::PointIndices>& clusters) const;
	void extractEuclideanClusterSmooth(const PointNormalCloud& cloud, const PointNormalCloud& normals,
																		 const pcl::search::KdTree<pcl::PointNormal>::Ptr& tree,
																		 std::vector<pcl::PointIndices>& clusters) const;
	void computeRSD(const PointCloudTConstPtr cloud, const NormalCloud::ConstPtr normal, const std::vector<pcl::PointIndices>& clusters, RSDCloud::Ptr radii) const;
	void getBoxInlier(const PointCloudTConstPtr cloud, const cnoid::Vector3& center, const cnoid::Matrix3& rot,
										const cnoid::Vector3& edge, pcl::PointIndices::Ptr inliers, double marign = 0.0025) const;
	void optimizeBoxInlier(const PointCloudTConstPtr cloud, cnoid::Vector3& center, const cnoid::Matrix3& rot,
												 cnoid::Vector3& edge, const pcl::PointIndices::Ptr) const;
	void getCylinderInlier(const PointCloudTConstPtr cloud, const cnoid::Vector3& pos, const cnoid::Vector3& dir,
												 double radius, double len, pcl::PointIndices::Ptr inliers) const;

	void saveNormal(const PointCloudTConstPtr cloud, const NormalCloud::ConstPtr normal, const std::string& filename) const;

	/* void fittingPlane(Cloud::Ptr cloud, pcl::PointIndices::Ptr inliers, cnoid::Vector3& edge, cnoid::Vector3& center, cnoid::Matrix3& Rot); */
	struct Params {
		double voxel_size;
		double high_curvature;
		bool   show_curvature_result;
		double cluster_tolerance;
		double cluster_eps_angle;
		int    cluster_min_points;
		bool   show_cluster_result;
		double rsd_radius;
		int    rsd_subdiv;
		double cylinder_rsd_tolerance;
		double cylinder_radius_tolerance;
		double box_ortho_angle_tolerance;
		double box_dist_tolerance;
		double box_require_point_ratio;
		bool   show_fittings;
		bool   show_fittings_shape;
		bool   do_box_fitting;
		bool   do_cylinder_fitting;
	};
	Params param;
	void readParams();
};

#endif /* _PCL_FITTING_H_ */

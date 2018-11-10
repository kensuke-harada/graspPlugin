#ifndef _PCL_POINTCLOUDUTILITY_H_
#define _PCL_POINTCLOUDUTILITY_H_

#include <cnoid/EigenTypes>
#include <cnoid/ColdetModel>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/SceneShape>
#endif

#include "PointCloudTypes.h"

class NormalEstimator {
 public:
	NormalEstimator();
	virtual ~NormalEstimator();

	void setRadius(double radius);
	void setK(unsigned int k);
	void useKSearch(bool use);
	void useRadiusSearch(bool use);
	void setSearchMethod(KdTreePointTPtr tree);

	double getRadius() const;
	double getK() const;

	void computeNormals(PointCloudTConstPtr input, NormalCloudPtr normals,
											const cnoid::Vector3& viewpoint = cnoid::Vector3(0, 0, 0));
	void computeNormals(PointCloudTConstPtr input, NormalCloudPtr normals,
											pcl::PointIndicesPtr indices,
											const cnoid::Vector3& viewpoint = cnoid::Vector3(0, 0, 0));

 private:
	double radius_;
	unsigned int k_;
	bool use_ksearch_;
	KdTreePointTPtr tree_;
};

class PointCloudUtil {
 public:
	static void modelToPointCloud(
		cnoid::ColdetModelPtr target_model, PointCloudTPtr cloud, double sampling_density = 1.0e-6);
	static void modelToPointCloudVisibleRegion(
		cnoid::ColdetModelPtr target_model, PointCloudTPtr cloud, 
		const cnoid::Vector3& viewpoint, double sampling_density = 1.0e-06);
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	static void modelToPointCloud(
		cnoid::SgNode* target_model, PointCloudTPtr cloud, double sampling_density = 1.0e-6);
	static void modelToPointCloudVisibleRegion(
		cnoid::SgNode* target_model, PointCloudTPtr cloud, 
		const cnoid::Vector3& viewpoint, double sampling_density = 1.0e-06);
#endif
	static void makeBoxCloud(PointCloudTPtr cloud, const cnoid::Vector3& edge, double interval);
	static void makeCylinderCloud(PointCloudTPtr cloud, double radius, double length, double interval);

	static cnoid::Matrix4f convM3VtoM4(const cnoid::Matrix3& rot, const cnoid::Vector3& trans);
	static void convM4toM3V(const cnoid::Matrix4f& mat, cnoid::Matrix3& rot, cnoid::Vector3& trans);

	static void getMinMax(const PointCloudTPtr& cloud, cnoid::Vector3& min_p, cnoid::Vector3& max_p);

	/**
	 * @brief Convert pointcloud to std::vector<cnoid::Vector3>
	 * @param[in] cloud a pointer to the target point cloud
	 * @param[out] vec_cloud a vector of points.
	 */
	static void cloudToVec(PointCloudTConstPtr cloud, std::vector<cnoid::Vector3>& vec);
	static void vecToCloud(const std::vector<cnoid::Vector3>& vec, PointCloudTPtr cloud);
	static void normalCloudToVec(NormalCloudConstPtr normal, std::vector<cnoid::Vector3>& vec);

	static void removeNan(PointCloudTPtr cloud);
	static void removeNan(ColorPointCloudPtr cloud);
	static void voxelFilter(PointCloudTConstPtr input, PointCloudTPtr output, double leaf_size);
	static void transCloud(PointCloudTPtr cloud, const cnoid::Matrix3& R, const cnoid::Vector3& p);
	static void transCloud(ColorPointCloudPtr cloud, const cnoid::Matrix3& R, const cnoid::Vector3& p);
	static void transCloud(PointCloudTConstPtr input, PointCloudTPtr output, const cnoid::Matrix3& R, const cnoid::Vector3& p);
	static void cropCloud(const PointCloudTConstPtr& input, PointCloudTPtr& output,
												const cnoid::Vector3& min_p, const cnoid::Vector3& max_p);

	/**
	 * @brief Compute bounding box of the point cloud.
	 * @param[in] cloud a pointer to the target point cloud
	 * @param[out] edge a edge of bounding box
	 * @param[out] center center position
	 * @param[out] Rot rotation matrix
	 */
	static void calcBoundingBox(PointCloudTConstPtr cloud, cnoid::Vector3& edge, cnoid::Vector3& center, cnoid::Matrix3& Rot);
};

#endif /* _PCL_POINTCLOUDUTILITY_H_ */

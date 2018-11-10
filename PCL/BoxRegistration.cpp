#include "BoxRegistration.h"

#include <cnoid/ColdetModel>

#include <pcl/registration/icp.h>

#include "PointCloudUtility.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

BoxRegistration::BoxRegistration() :
	leaf_size_(0.005) {
	icp_params_.resize(2);
	icp_params_[0].max_iteration = 20;
	icp_params_[0].max_correspond_dist = 0.05;
	icp_params_[0].trans_eps = 1e-6;
	icp_params_[0].fit_eps = 0.01;
	icp_params_[1].max_iteration = 30;
	icp_params_[1].max_correspond_dist = 0.02;
	icp_params_[1].trans_eps = 1e-6;
	icp_params_[1].fit_eps = 0.01;
}

BoxRegistration::~BoxRegistration() {
}

/**
 * @brief do registration box by using ICP
 * @param[in,out] body body pointer of the target box
 * @param[in] input point cloud
 * @param[in] viewpoint view point
 * @param[out] box_cloud point cloud of box
 */
bool BoxRegistration::registration(cnoid::BodyPtr body, PointCloudTConstPtr input, const cnoid::Vector3& viewpoint, PointCloudTPtr box_cloud) {
	cnoid::ColdetModelPtr model;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	model = body->link(0)->coldetModel();
#else
	model = grasp::ColdetConverter::ConvertFrom(body->link(0)->collisionShape());
#endif
	PointCloudTPtr box_model_cloud(new PointCloudT());
	// viewpoint in box coordinate system
	cnoid::Vector3 viewpoint_boxcoord = body->rootLink()->R().transpose() * (viewpoint - body->rootLink()->p());
	// convert box model to point cloud
	PointCloudUtil::modelToPointCloudVisibleRegion(model, box_model_cloud, viewpoint_boxcoord);

	// down sampling
	PointCloudTPtr filtered_box_cloud(new PointCloudT());
	PointCloudTPtr filtered_input(new PointCloudT());
	PointCloudUtil::voxelFilter(box_model_cloud, filtered_box_cloud, leaf_size_);
	PointCloudUtil::voxelFilter(input, filtered_input, leaf_size_);

	cnoid::Matrix4f trans_matrix;
	trans_matrix = PointCloudUtil::convM3VtoM4(body->rootLink()->R(), body->rootLink()->p());

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputSource(filtered_box_cloud);
	icp.setInputTarget(filtered_input);

	// registration using ICP step by step. from coarse grain to fine grain.
	for (size_t i = 0; i < icp_params_.size(); i++) {
		icp.setMaxCorrespondenceDistance(icp_params_[i].max_correspond_dist);
		icp.setMaximumIterations(icp_params_[i].max_iteration);
		icp.setTransformationEpsilon(icp_params_[i].trans_eps);
		icp.setEuclideanFitnessEpsilon(icp_params_[i].fit_eps);
		icp.align(*box_cloud, trans_matrix);

		if (!icp.hasConverged()) {
			return false;
		}

		trans_matrix = icp.getFinalTransformation();
	}

	cnoid::Vector3 trans_p;
	cnoid::Matrix3 trans_R;
	PointCloudUtil::convM4toM3V(trans_matrix, trans_R, trans_p);
	body->rootLink()->R() = trans_R;
	body->rootLink()->p() = trans_p;
	return true;
}

bool BoxRegistration::registration(cnoid::BodyPtr body, PointCloudTConstPtr input, const cnoid::Vector3& viewpoint) {
	PointCloudTPtr box_cloud(new PointCloudT());
	return registration(body, input, viewpoint, box_cloud);
}

void BoxRegistration::setLeafSize(double leaf_size) {
	leaf_size_ = leaf_size;
}

void BoxRegistration::setICPParams(const std::vector<ICPParams>& params) {
	icp_params_ = params;
}

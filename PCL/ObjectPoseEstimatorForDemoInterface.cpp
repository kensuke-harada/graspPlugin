#include "ObjectPoseEstimatorForDemoInterface.h"

#include "ObjectPoseEstimatorForDemoImpl.h"

#include "PointCloudUtility.h"

PoseEstimatorForDemo::PoseEstimatorForDemo() :
	PoseEstimator(new PoseEstimatorForDemoImpl()) {
}

PoseEstimatorForDemo::~PoseEstimatorForDemo() {
}

void PoseEstimatorForDemo::captureForDemo(std::vector<cnoid::Vector3>& points) {
	dynamic_cast<PoseEstimatorForDemoImpl*>(impl)->captureForDemo(points);
}

bool PoseEstimatorForDemo::estimationForDemo(const std::vector<cnoid::Vector3>& points, cnoid::Vector3& p, cnoid::Matrix3& R) {
	return dynamic_cast<PoseEstimatorForDemoImpl*>(impl)->estimationForDemo(points, p, R);
}

void PoseEstimatorForDemo::estimationForDemoWoICP(const std::vector<cnoid::Vector3>& points, std::vector<cnoid::Vector3>& p,
																							std::vector<cnoid::Matrix3>& R) {
	dynamic_cast<PoseEstimatorForDemoImpl*>(impl)->estimationForDemoWoICP(points, p, R);
}

bool PoseEstimatorForDemo::ICPforDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& init_ps, const std::vector<cnoid::Matrix3>& init_Rs,
															 cnoid::Vector3& p, cnoid::Matrix3& R) {
	return dynamic_cast<PoseEstimatorForDemoImpl*>(impl)->ICPforDemo(points, init_ps, init_Rs, p, R);
}

bool PoseEstimatorForDemo::twoStageRegistrationForDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& sface_points,
																								const std::string& box_path, const cnoid::Vector3& center,
																								cnoid::Vector3& p, cnoid::Matrix3& R) {
	return dynamic_cast<PoseEstimatorForDemoImpl*>(impl)->twoStageRegistrationForDemo(points, sface_points, box_path, center, p, R);
}

void PoseEstimatorForDemo::removeLinkPointCloud(std::vector<cnoid::Vector3>& points, cnoid::Link* target_link) {
	PointCloudTPtr link_cloud(new PointCloudT());
	PointCloudTPtr cap_cloud(new PointCloudT());
	PointCloudUtil::modelToPointCloud(target_link->shape(), link_cloud);
	PointCloudUtil::transCloud(link_cloud, target_link->R(), target_link->p());
	PointCloudUtil::vecToCloud(points, cap_cloud);

	points.clear();

	KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
	tree->setInputCloud(link_cloud);
	std::vector<int> nn_indices(1);
	std::vector<float> nn_distances(1);
	std::vector<int> overlap_indices;
	double dist_th = 0.0025 * 0.0025 * 2.0 * 2.0;
	for (size_t i = 0; i < cap_cloud->points.size(); i++) {
		if (!tree->nearestKSearch(cap_cloud->points[i], 1, nn_indices, nn_distances))
			continue;
		if (nn_distances[0] < dist_th)
			continue;
		points.push_back(cnoid::Vector3(cap_cloud->points[i].x, cap_cloud->points[i].y, cap_cloud->points[i].z));
	}
}


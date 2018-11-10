#include "ObjectPoseEstimatorForDemoImpl.h"

#include "ObjectPoseEstimatorForDemo.h"
#include "PointCloudUtility.h"

PoseEstimatorForDemoImpl::PoseEstimatorForDemoImpl() {
	ope.reset(new ObjectPoseEstimatorForDemo());
}

PoseEstimatorForDemoImpl::~PoseEstimatorForDemoImpl() {
}

void PoseEstimatorForDemoImpl::captureForDemo(std::vector<cnoid::Vector3>& points) {
	ope->readScene(param_.is_load_file, param_.is_save_file, param_.load_file_path, param_.save_file_path,
								 camera_p_, camera_R_, false);
	ope->downsampling(true);
	PointCloudUtil::cloudToVec(ope->getSampledSceneCloud(), points);
}

bool PoseEstimatorForDemoImpl::estimationForDemo(const std::vector<cnoid::Vector3>& points, cnoid::Vector3& p, cnoid::Matrix3& R) {
	return dynamic_cast<ObjectPoseEstimatorForDemo*>(ope.get())->estimationForDemo(points, p, R, camera_p_, camera_R_, param_.num_neighbor);
}

void PoseEstimatorForDemoImpl::estimationForDemoWoICP(const std::vector<cnoid::Vector3>& points, std::vector<cnoid::Vector3>& p,
																											std::vector<cnoid::Matrix3>& R) {
	dynamic_cast<ObjectPoseEstimatorForDemo*>(ope.get())->estimationForDemoWoICP(points, p, R, camera_p_, camera_R_, param_.num_neighbor);
}

bool PoseEstimatorForDemoImpl::ICPforDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& init_ps,
																					const std::vector<cnoid::Matrix3>& init_Rs,
																					cnoid::Vector3& p, cnoid::Matrix3& R) {
	return dynamic_cast<ObjectPoseEstimatorForDemo*>(ope.get())->ICPforDemo(points, init_ps, init_Rs, p, R, param_.iteration);
}

bool PoseEstimatorForDemoImpl::twoStageRegistrationForDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& sface_points,
																													 const std::string& box_path, const cnoid::Vector3& center,
																													 cnoid::Vector3& p, cnoid::Matrix3& R) {
	return dynamic_cast<ObjectPoseEstimatorForDemo*>(ope.get())->twoStageRegistrationForDemo(points, sface_points, box_path, center, p, R);
}

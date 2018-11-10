#include "BoxRegistrationInterface.h"

#include "PointCloudTypes.h"
#include "PointCloudUtility.h"
#include "BoxRegistration.h"

BoxRegistrationInterface::BoxRegistrationInterface() {
}

BoxRegistrationInterface::~BoxRegistrationInterface() {
}

/**
 * @brief do registration box by using ICP
 * @param[in,out] body body pointer of the target box
 * @param[in] input point cloud
 * @param[in] viewpoint view point
 */
bool BoxRegistrationInterface::registration(cnoid::BodyPtr body,
																						const std::vector<cnoid::Vector3>& input,
																						const cnoid::Vector3& viewpoint) const {
	PointCloudTPtr cloud(new PointCloudT);
	PointCloudUtil::vecToCloud(input, cloud);
	BoxRegistration box_reg;
	return box_reg.registration(body, cloud, viewpoint);
}

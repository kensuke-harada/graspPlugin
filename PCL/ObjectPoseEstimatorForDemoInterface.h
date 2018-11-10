#ifndef _PCL_OBJECTPOSEESTIMATORFORDEMOINTERFACE_H_
#define _PCL_OBJECTPOSEESTIMATORFORDEMOINTERFACE_H_

#include <vector>
#include <string>

#include <cnoid/EigenTypes>
#include <cnoid/Link>

#include "ObjectPoseEstimatorInterface.h"

#include "exportdef.h"

class EXCADE_API PoseEstimatorForDemo :
public PoseEstimator {
 public:
	PoseEstimatorForDemo();
	~PoseEstimatorForDemo();

	void captureForDemo(std::vector<cnoid::Vector3>& points);
	bool estimationForDemo(const std::vector<cnoid::Vector3>& points, cnoid::Vector3& p, cnoid::Matrix3& R);
	void estimationForDemoWoICP(const std::vector<cnoid::Vector3>& points, std::vector<cnoid::Vector3>& p, std::vector<cnoid::Matrix3>& R);
	bool ICPforDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& init_ps, const std::vector<cnoid::Matrix3>& init_Rs, cnoid::Vector3& p, cnoid::Matrix3& R);
	bool twoStageRegistrationForDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& sface_points,
																	 const std::string& box_path, const cnoid::Vector3& center,
																	 cnoid::Vector3& p, cnoid::Matrix3& R);
	void removeLinkPointCloud(std::vector<cnoid::Vector3>& points, cnoid::Link* target_link);
};

#endif /* _PCL_OBJECTPOSEESTIMATORFORDEMOINTERFACE_H_ */

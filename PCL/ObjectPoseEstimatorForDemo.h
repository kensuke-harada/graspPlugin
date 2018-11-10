#ifndef _PCL_OBJECTPOSEESTIMATORFORDEMO_H_
#define _PCL_OBJECTPOSEESTIMATORFORDEMO_H_

#include <vector>
#include <string>
#include <limits>

#include <cnoid/EigenTypes>

#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>

#include "ObjectPoseEstimator.h"

#include "exportdef.h"

class EXCADE_API ObjectPoseEstimatorForDemo :
public ObjectPoseEstimator {
 public:
	ObjectPoseEstimatorForDemo();
	~ObjectPoseEstimatorForDemo();

	bool estimationForDemo(const std::vector<cnoid::Vector3>& points, cnoid::Vector3& p, cnoid::Matrix3& R,
												 const cnoid::Vector3& view_point, const cnoid::Matrix3& cameraR, int nn);
	void estimationForDemoWoICP(const std::vector<cnoid::Vector3>& points,
															std::vector<cnoid::Vector3>& p, std::vector<cnoid::Matrix3>& R,
															const cnoid::Vector3& view_point, const cnoid::Matrix3& cameraR, int nn);
	bool ICPforDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& init_ps,
									const std::vector<cnoid::Matrix3>& init_Rs,
									cnoid::Vector3& p, cnoid::Matrix3& R, int num_ite);
	bool twoStageRegistrationForDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& sface_points,
																	 const std::string& box_path, const cnoid::Vector3& center,
																	 cnoid::Vector3& p, cnoid::Matrix3& R);
};

#endif /* _PCL_OBJECTPOSEESTIMATORFORDEMO_H_ */

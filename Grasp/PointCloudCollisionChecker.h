#ifndef _GRASP_POINTCLOUDCOLLISIONCHECKER_H_
#define _GRASP_POINTCLOUDCOLLISIONCHECKER_H_

#include <vector>
#include <string>
#include <limits>

#include <cnoid/BodyItem>
#include <cnoid/EigenTypes>

#include "exportdef.h"

namespace grasp {
	class PointCloudEnv;

	class EXCADE_API PointCloudCollisionChecker {
	public:
		static bool isCollidingPointCloud(const std::vector<cnoid::Vector3>& p,
																			const cnoid::BodyItemPtr& item,
																			double tol = 0.001);
		static bool isCollidingPointCloud(const PointCloudEnv* pc_env,
																			const cnoid::BodyItemPtr& item,
																			double tol = 0.001);
		static bool isCollidingPointCloud(const PointCloudEnv* pc_env,
																			const cnoid::BodyPtr& item,
																			double tol = 0.001);
		static bool isCollidingPointCloudSub(const PointCloudEnv* pc_env,
																				 cnoid::Link* link,
																				 double tol = 0.001);
		static bool getColPointCloud(std::vector<cnoid::Vector3>& colPoints,
																 const PointCloudEnv* pc_env,
																 const cnoid::BodyItemPtr& item,
																 double tol = 0.001);
		static bool getColPointCloudSub(std::vector<cnoid::Vector3>& colPoints,
																		const PointCloudEnv* pc_env,
																		cnoid::Link* link,
																		double tol = 0.001);
		static bool isCollidingPointCloudAllCheck(const PointCloudEnv* pc_env,
																							const cnoid::BodyItemPtr& item,
																							std::vector<std::string>& collink,
																							double tol = 0.001);
	};
}

#endif /* _GRASP_POINTCLOUDCOLLISIONCHECKER_H_ */

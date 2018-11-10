#ifndef _PCL_BOXREGISTRATOION_H_
#define _PCL_BOXREGISTRATOION_H_

#include <vector>

#include <cnoid/EigenTypes>
#include <cnoid/Body>

#include "PointCloudTypes.h"

class BoxRegistration {
 public:
	BoxRegistration();
	virtual ~BoxRegistration();

	struct ICPParams {
		int max_iteration;
		double max_correspond_dist;
		double trans_eps;
		double fit_eps;
	};

	bool registration(cnoid::BodyPtr body, PointCloudTConstPtr input, const cnoid::Vector3& viewpoint, PointCloudTPtr box_cloud);
	bool registration(cnoid::BodyPtr body, PointCloudTConstPtr input, const cnoid::Vector3& viewpoint);

	void setLeafSize(double leaf_size);
	void setICPParams(const std::vector<ICPParams>& params);

 private:
	double leaf_size_;
	std::vector<ICPParams> icp_params_;
};

#endif /* _PCL_BOXREGISTRATOION_H_ */

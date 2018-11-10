/**
 * @file ObjectPoseEstimatorImpl.h
 * @author Akira Ohchi
 */

#ifndef _PCL_OBJECTPOSEESTIMATORIMPL_H_
#define _PCL_OBJECTPOSEESTIMATORIMPL_H_

#include <vector>

#include <boost/scoped_ptr.hpp>

#include <cnoid/EigenTypes>
#include <cnoid/Body>

#include "ObjectPoseEstimateParams.h"
#include "PointCloudMerger.h"

class ObjectPoseEstimator;
class ConditionalSegmenter;
class LccpSegmenter;
class SAC_IAEstimator;
class ICPEstimator;
class PointCloudMerger;

class PoseEstimatorImpl {
 public:
	PoseEstimatorImpl();
 	virtual ~PoseEstimatorImpl();

 	void init(const ObjectPoseEstimateParams& param);
 	void readObject();
 	void capture();

 	bool boxRegistration(cnoid::BodyPtr box_body);
 	bool segmentation();
 	void estimationInit(int &size);
 	bool estimation(int i, cnoid::Vector3& p, cnoid::Matrix3& R, std::vector<int>& outlier_idx);
 	bool estimation();
 	void getEnvCloud(std::vector<cnoid::Vector3>& env_cloud);
 	void registerResultsToDrawer();
 	void displayEstimationResults();

 	cnoid::Vector3 camera_p_;
 	cnoid::Matrix3 camera_R_;
 	ObjectPoseEstimateParams param_;

 	boost::scoped_ptr<ObjectPoseEstimator> ope;
 	boost::scoped_ptr<ConditionalSegmenter> seg;
 	boost::scoped_ptr<LccpSegmenter> lccp_seg;
 	boost::scoped_ptr<SAC_IAEstimator> init_est;
 	boost::scoped_ptr<ICPEstimator> est;
 	PointCloudMerger merger_;
};

#endif /* _PCL_OBJECTPOSEESTIMATORIMPL_H_ */

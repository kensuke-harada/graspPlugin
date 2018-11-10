/**
 * @file   ObjectPoseEstimatorInterface.h
 * @author Akira Ohchi
 */

#ifndef _PCL_OBJECTPOSEESTIMATORINTERFACE_H_
#define _PCL_OBJECTPOSEESTIMATORINTERFACE_H_

#include <vector>
#include <string>

#include <cnoid/EigenTypes>
#include <cnoid/Body>

#ifdef CNOID_GE_15
#include <cnoid/PointSetItem>
#endif

#include "ObjectPoseEstimateParams.h"

#include "exportdef.h"

class PoseEstimatorImpl;

class EXCADE_API PoseEstimator {
 public:
	PoseEstimator();
	explicit PoseEstimator(PoseEstimatorImpl* impl_);
	~PoseEstimator();

	bool poseEstimate(const ObjectPoseEstimateParams& param, bool do_boxadjustment = false);
	bool poseEstimate(const ObjectPoseEstimateParams& param, cnoid::BodyPtr box_body);

	static void readParams(ObjectPoseEstimateParams* param);
	static void clearPointClouds();

	static void drawClusters();
	static void drawPrevAndCurrPointCloud();

	enum ErrorValue {
		SUCCESS,
		SEGMENTATION_ERROR,
		ESTIMATION_ERROR,
		SETTING_ERROR
	};

	ErrorValue getErrorCode() const;

	void setCameraMat(const cnoid::Vector3& view_point, const cnoid::Matrix3& ori);
	void init(const ObjectPoseEstimateParams& param);
	void readObject();
	void capture();
	bool boxRegistration(const cnoid::BodyPtr& box_body);
	bool segmentation();
	void estimationInit(int& size);
	bool estimation(int i, cnoid::Vector3& p, cnoid::Matrix3& R, std::vector<int>& outlier_idx);
	void registerResultsToDrawer();
	void displayEstimationResults();
	bool estimation();
	void getEnvCloud(std::vector<cnoid::Vector3>& env_cloud);
	static void showEnvCloud(const std::vector<cnoid::Vector3>& env_cloud);

	static void getPrevSampledCloud(std::vector<cnoid::Vector3>& prev_cloud);
	static void getCurrSampledCloud(std::vector<cnoid::Vector3>& curr_cloud);
	static void getPrevClusterCloud(std::vector<cnoid::Vector3>& prev_cluster_cloud);
	static void drawPointClouds(const std::vector<std::vector<cnoid::Vector3> >& points, const std::vector<cnoid::Vector3>& colors);

#ifdef CNOID_GE_15
	void getCurrPointCloudItem(cnoid::PointSetItemPtr& point_item);
	void getSampledCurrPointCloudItem(cnoid::PointSetItemPtr& point_item);
	void getClusteredCloudItem(cnoid::PointSetItemPtr& point_item);
	void getClusterPointCloudItems(std::vector<cnoid::PointSetItemPtr>& point_items, std::vector<int>& flags);
#endif

 protected:
	ErrorValue error_code_;
	cnoid::BodyPtr box_body_;

	PoseEstimatorImpl* impl;
};

#endif /* _PCL_OBJECTPOSEESTIMATORINTERFACE_H_ */

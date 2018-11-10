#ifndef _PCL_CALIBRATION_H_
#define _PCL_CALIBRATION_H_

#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include <cnoid/EigenTypes>
#include <cnoid/BodyItem>

#include "PointCloudTypes.h"

class Calibration {
 public:
	Calibration();
	virtual ~Calibration();

	bool registration();
	void acceptPointPairs();
	bool generateMatrixFile(const std::string& filepath) const;

	void displayPointCloud();

	bool goNext();

	static std::string getDefaultMatrixFilePath();

	static void capture();

	cnoid::BodyItemPtr calib_model;

	cnoid::Vector3 getTargetLinkPInCamera();

 private:
	typedef std::vector<cnoid::Vector3> pointVec;

	pointVec points_link_;
	pointVec points_camera_;

	pointVec tmp_points_link_;
	pointVec tmp_points_camera_;

	int current_id_;

	cnoid::Link* target_link_;
	bool is_calib_model_loaded_;

	PointCloudTPtr target_cloud_;

	void getCameraLinkT(cnoid::Vector3& p, cnoid::Matrix3& R) const;
	void getTargetLinkT(cnoid::Vector3& p, cnoid::Matrix3& R) const;
	bool getRegistrationT(cnoid::Vector3& p, cnoid::Matrix3& R);
	void loadRobotPose();
	bool loadPointCloud();
	bool initialSetting();

	static std::string getDataDirPath();
	static int getMaxFileID();

	static void calcRototranslation(const pointVec& y_points, const pointVec& x_points, cnoid::Matrix4f& yTx);
};

typedef boost::shared_ptr<Calibration> CalibrationPtr;

#endif /* _PCL_CALIBRATION_H_ */

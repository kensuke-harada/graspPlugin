#include "PointCloudHolder.h"

#include <QSettings>
#include <QString>

#include <pcl/io/pcd_io.h>

#include <cnoid/ExecutablePath>

#include "../Grasp/Camera.h"
#include "PointCloudHandler.h"
#include "PointCloudUtility.h"

using cnoid::Vector3;
using cnoid::Matrix3;

namespace {
	std::string getPluginPath() {
		return (cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PCL/");
	}
}

PointCloudHolder* PointCloudHolder::instance() {
	static PointCloudHolder* instance = new PointCloudHolder();
	return instance;
}

PointCloudTConstPtr PointCloudHolder::getCloudPtr() const {
	return cloud_;
}

ColorPointCloudConstPtr PointCloudHolder::getColorCloudPtr() const {
	return color_cloud_;
}

bool PointCloudHolder::capture(bool do_load_file, bool do_save_file,
															 const std::string& load_filepath, const std::string& save_filepath,
															 bool do_merge, bool do_trans) {
	cnoid::Vector3 p = cnoid::Vector3::Zero();
	cnoid::Matrix3 R = cnoid::Matrix3::Identity();
	if (do_trans) {
		grasp::CameraPtr camera = grasp::CameraHandler::instance()->getTargetCamera();
		p = camera->getCameraPosition();
		R = camera->getCameraRotation();
	}

	return capture(do_load_file, do_save_file, load_filepath, save_filepath, p, R, do_merge);
}

bool PointCloudHolder::capture(bool do_load_file, bool do_save_file,
															 const std::string& load_filepath, const std::string& save_filepath,
															 const cnoid::Vector3& p, const cnoid::Matrix3& R,
															 bool do_merge) {
	ColorPointCloudPtr grabbed_cloud = ColorPointCloudPtr(new ColorPointCloud());
	// get a point cloud
	//if (do_load_file) {
#if defined(READ_PCD)
		pcl::io::loadPCDFile<ColorPointT>(load_filepath, *grabbed_cloud);
#elif defined(READ_PLY)
		pcl::io::loadPLYFile<ColorPointT>(load_filepath, *grabbed_cloud);
#else
	//} else {
		PointCloudHandler* pch = PointCloudHandler::instance();
		pch->captureWithoutInit(true);
		pcl::copyPointCloud(*(pch->cloud_xyzrgba), *grabbed_cloud);
#endif
		//}

	// remove points having NaN value
	PointCloudUtil::removeNan(grabbed_cloud);

	// save the point cloud to the file
	if (do_save_file) {
		pcl::io::savePCDFile(save_filepath, *grabbed_cloud, true);
	}

	push(grabbed_cloud, do_merge, p, R);

	return true;
}

void PointCloudHolder::clear() {
	cloud_->clear();
	color_cloud_->clear();
}

PointCloudHolder::PointCloudHolder() :
	cloud_(new PointCloudT),
	color_cloud_(new ColorPointCloud) {
	initialSetting();
}

void PointCloudHolder::initialSetting() {
	// load parameters from INI file
	QString config_path = QString::fromStdString(getPluginPath() + "config.ini");
	QSettings setting(config_path, QSettings::IniFormat);
	setting.beginGroup("Capture");

	do_correction_ = setting.value("do_correction", false).toBool();
	k1_ = setting.value("k1").toDouble();
	k2_ = setting.value("k2").toDouble();
	d1_ = setting.value("d1").toDouble();
	d2_ = setting.value("d2").toDouble();

	setting.endGroup();
}

void PointCloudHolder::push(ColorPointCloudPtr cloud, bool do_append,
														const Vector3& p, const Matrix3& R) {
	if (!do_append) {
		clear();
	}

	if (do_correction_) {
		for (size_t i = 0; i < cloud->size(); ++i) {
			ColorPointT &src = cloud->points[i];
			float x2 = src.x * src.x;
			float y2 = src.y * src.y;
			float z2 = src.z * src.z;
			float r2 = (x2 + y2) / z2;
			src.z = src.z * (1 + k1_ * r2 + k2_ * r2 * r2);
			src.z = d1_ * src.z + d2_ * z2;
		}
	}

	PointCloudUtil::transCloud(cloud, R, p);
	if (do_append && !color_cloud_->empty()) {
		*color_cloud_ += *cloud;
	} else {
		color_cloud_ = cloud;
	}

	cloud_->clear();

	pcl::copyPointCloud(*color_cloud_, *cloud_);
}

BoxCloudHolder* BoxCloudHolder::instance() {
	static BoxCloudHolder* instance = new BoxCloudHolder();
	return instance;
}

PointCloudTConstPtr BoxCloudHolder::getBoxModelPtr() const {
	return box_model_cloud_;
}

void BoxCloudHolder::setBoxModelCloud(PointCloudTPtr box) {
	box_model_cloud_ = box;
	has_box_ = true;
}

PointCloudTPtr BoxCloudHolder::getBoxPtr() const {
	return box_cloud_;
}

void BoxCloudHolder::setBoxCloud(PointCloudTPtr box) {
	box_cloud_= box;
	has_box_ = true;
}

void BoxCloudHolder::clearBox() {
	box_model_cloud_ = PointCloudTPtr(new PointCloudT());
	box_cloud_ = PointCloudTPtr(new PointCloudT());
	has_box_ = false;
}

bool BoxCloudHolder::hasBox() const {
	return has_box_;
}

BoxCloudHolder::BoxCloudHolder() :
	box_model_cloud_(new PointCloudT),
	box_cloud_(new PointCloudT),
	has_box_(false) {
}

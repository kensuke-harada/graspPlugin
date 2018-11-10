#include "Calibration.h"

#include <fstream>
#include <sstream>

#include <iostream>

#include <QDir>
#include <QSettings>
#include <QString>
#include <QStringList>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <cnoid/ExecutablePath>
#include <cnoid/ColdetModel>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>

#include "../Grasp/VectorMath.h"
#include "../Grasp/TransformationMatrixReader.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/Camera.h"

#include "PointCloudUtility.h"
#include "PointCloudDrawer.h"
#include "PointCloudHolder.h"
#include "Registrator.h"

namespace {
	std::string getPluginPath() {
		return (cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PCL/");
	}
}

Calibration::Calibration() :
	current_id_(0) {
	is_calib_model_loaded_ = initialSetting();
}

Calibration::~Calibration() {
	if (is_calib_model_loaded_) {
		calib_model->detachFromParentItem();
	}
}

bool Calibration::goNext() {
	current_id_++;
	return (current_id_ <= getMaxFileID());
}

bool Calibration::registration() {
	tmp_points_link_.clear();
	tmp_points_camera_.clear();

	cnoid::Vector3 camlink_p;
	cnoid::Matrix3 camlink_R;
	cnoid::Vector3 trgtlink_p;
	cnoid::Matrix3 trgtlink_R;

	cnoid::Vector3 trans_p;
	cnoid::Matrix3 trans_R;

	loadRobotPose();
	if (!loadPointCloud()) {
		return false;
	}

	if (!getRegistrationT(trans_p, trans_R)) {
		return false;
	}

	getCameraLinkT(camlink_p, camlink_R);
	getTargetLinkT(trgtlink_p, trgtlink_R);

	pointVec points;
	points.push_back(cnoid::Vector3(0, 0, 0));
	// points.push_back(cnoid::Vector3(0.1, 0, 0));
	// points.push_back(cnoid::Vector3(-0.1, 0, 0));
	// points.push_back(cnoid::Vector3(0, 0.1, 0));
	// points.push_back(cnoid::Vector3(0, -0.1, 0));
	// points.push_back(cnoid::Vector3(0, 0, 0.1));
	// points.push_back(cnoid::Vector3(0, 0, -0.1));

	cnoid::Vector4f min_p;
	cnoid::Vector4f max_p;
	pcl::getMinMax3D(*target_cloud_, min_p, max_p);
	cnoid::Vector3 center_link = (0.5 * (min_p + max_p)).cast<double>().head(3);

	for (size_t i = 0; i < points.size(); i++) {
		tmp_points_camera_.push_back(trans_R * (points[i] + center_link) + trans_p);
		// tmp_points_link_.push_back(camlink_R.transpose() * (trgtlink_R * (points[i] + center_link) + trgtlink_p - camlink_p));
		tmp_points_link_.push_back(camlink_R.transpose() * (target_link_->attitude() * (points[i] + center_link) + trgtlink_p - camlink_p));
	}

	return true;
}

void Calibration::acceptPointPairs() {
	points_link_.insert(points_link_.end(), tmp_points_link_.begin(), tmp_points_link_.end());
	points_camera_.insert(points_camera_.end(), tmp_points_camera_.begin(), tmp_points_camera_.end());
	tmp_points_link_.clear();
	tmp_points_camera_.clear();
}

bool Calibration::generateMatrixFile(const std::string& filepath) const {
	if (points_link_.size() < 3) return false;

	cnoid::Matrix4f calib_mat = cnoid::Matrix4f::Identity();

// #ifdef DEBUG_MODE
	std::cout << "y_points(link):" << std::endl;
	for (size_t i = 0; i < points_link_.size(); i++) {
		std::cout << points_link_[i].transpose() << std::endl;
	}
	std::cout << "x_points:(camera)" << std::endl;
	for (size_t i = 0; i < points_camera_.size(); i++) {
		std::cout << points_camera_[i].transpose() << std::endl;
	}

// #endif

	pointVec y = points_link_;
	pointVec x = points_camera_;

	for (int i = 0; i < 10; i++) {
		cnoid::Matrix4f tmp_t;
		calcRototranslation(y, x, tmp_t);
		cnoid::Matrix3 R;
		cnoid::Vector3 p;
		PointCloudUtil::convM4toM3V(tmp_t, R, p);
		for (size_t j = 0; j < y.size(); j++) {
			x[j] = R * x[j] + p;
		}
		calib_mat = tmp_t * calib_mat;
	}


	grasp::TransMatReader::writeTransMatrix(filepath, calib_mat);

	return true;
}

void Calibration::displayPointCloud() {
	loadRobotPose();
	loadPointCloud();

	PointCloudDrawer* draw = PointCloudDrawer::instance();
	draw->clear();
	draw->addPointCloud(PointCloudHolder::instance()->getCloudPtr());
	draw->draw();

	calib_model->body()->rootLink()->p() = target_link_->p();
	calib_model->body()->rootLink()->R() = target_link_->attitude();
	calib_model->notifyKinematicStateChange();
}

std::string Calibration::getDefaultMatrixFilePath() {
	return getPluginPath() + "calibtools/calibmat.txt";
}

std::string Calibration::getDataDirPath() {
	return getPluginPath() + "calibrationdata/";
}

int Calibration::getMaxFileID() {
	int max_id = -1;
	QDir data_dir(QString::fromStdString(getDataDirPath()));
	if (!data_dir.exists()) {
		data_dir.mkpath(".");
	}
	QFileInfoList filelist = data_dir.entryInfoList(QDir::Files);
	QFileInfoList::const_iterator p;
	for (p = filelist.constBegin(); p != filelist.constEnd(); ++p) {
		QStringList splits = (*p).baseName().split("_");
		if (splits.size() != 2) continue;
		if (splits[1] != "angle") continue;

		int id = splits[0].toInt();
		if (id > max_id) max_id = id;
	}
	return max_id;
}

void Calibration::capture() {
	int id = getMaxFileID() + 1;
	QString data_path = QString::fromStdString(getDataDirPath());
#if defined(READ_PLY)
	QString pcd_path = QString("%1%2_scene.ply").arg(data_path).arg(id, 4, 10, QChar('0'));
#else
	QString pcd_path = QString("%1%2_scene.pcd").arg(data_path).arg(id, 4, 10, QChar('0'));
#endif
	QString angle_path = QString("%1%2_angle.txt").arg(data_path).arg(id, 4, 10, QChar('0'));
	PointCloudHolder* ph = PointCloudHolder::instance();
	ph->capture(false, true, "", pcd_path.toStdString(), false, true);

	std::ofstream fout(angle_path.toStdString().c_str());
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	for (int n = 0; n < pb->body()->numJoints(); n++) {
		fout << pb->body()->joint(n)->q() << " ";
	}

	PointCloudDrawer* draw = PointCloudDrawer::instance();
	draw->clear();
	draw->addPointCloud(ph->getCloudPtr());
	draw->draw();
}

cnoid::Vector3 Calibration::getTargetLinkPInCamera() {
	cnoid::Vector3 camera_p = grasp::CameraHandler::instance()->getTargetCamera()->getCameraPosition();
	cnoid::Matrix3 camera_R = grasp::CameraHandler::instance()->getTargetCamera()->getCameraRotation();
	cnoid::Vector3 target_p;
	cnoid::Matrix3 target_R;
	getTargetLinkT(target_p, target_R);

	return camera_R.transpose() * (target_p - camera_p);

}

void Calibration::getCameraLinkT(cnoid::Vector3& p, cnoid::Matrix3& R) const {
	if (grasp::CameraHandler::instance()->getTargetCamera()->isAttachedCamera()) {
		grasp::AttachedCameraPtr camera = grasp::CameraHandler::instance()->getTargetAttachedCamera();
		p = camera->getCameraLink()->p();
		R = camera->getCameraLink()->R();
	} else {
		p = cnoid::Vector3::Zero();
		R = cnoid::Matrix3::Identity();
	}
}

void Calibration::getTargetLinkT(cnoid::Vector3& p, cnoid::Matrix3& R) const {
	p = cnoid::Vector3::Zero();
	R = cnoid::Matrix3::Identity();

	grasp::PlanBase* pb = grasp::PlanBase::instance();
	if (pb->targetArmFinger == NULL) return;

	if (target_link_ == NULL) return;

	p = target_link_->p();
	R = target_link_->R();
}

bool Calibration::getRegistrationT(cnoid::Vector3& p, cnoid::Matrix3& R) {

	cnoid::Link* calib_link = calib_model->body()->rootLink();

	cnoid::Vector4f min_p;
	cnoid::Vector4f max_p;
	pcl::getMinMax3D(*target_cloud_, min_p, max_p);
	cnoid::Vector3 center_link = (0.5 * (min_p + max_p)).cast<double>().head(3);
	cnoid::Vector3 center = calib_link->attitude() * center_link + calib_link->p();

	PointCloudHolder* ph = PointCloudHolder::instance();

	PointCloudTPtr target_cloud = PointCloudTPtr(new PointCloudT());
	// cnoid::Matrix4f trans_mat = PointCloudUtil::convM3VtoM4(target_link_->attitude(), target_link_->p());

	// cnoid::Matrix4f trans_mat = PointCloudUtil::convM3VtoM4(calib_link->attitude(), calib_link->p());
	// pcl::transformPointCloud(*target_cloud_, *target_cloud, trans_mat);

	PointCloudTConstPtr scene_cloud = ph->getCloudPtr();
	PointCloudTPtr sampled_target_cloud = PointCloudTPtr(new PointCloudT());
	PointCloudTPtr sampled_scene_cloud = PointCloudTPtr(new PointCloudT());
	PointCloudTPtr tmp_scene_cloud = PointCloudTPtr(new PointCloudT());

	double margin = 0.1;
	cnoid::Vector3 link_p = target_link_->p();
	pcl::CropBox<PointT> crop;
	crop.setInputCloud(scene_cloud);
	// Eigen::Vector4f min_pt(link_p.x() - margin, link_p.y() - margin, link_p.z() - margin, 1);
	// Eigen::Vector4f max_pt(link_p.x() + margin, link_p.y() + margin, link_p.z() + margin, 1);
	Eigen::Vector4f min_pt(center.x() - margin, center.y() - margin, center.z() - margin, 1);
	Eigen::Vector4f max_pt(center.x() + margin, center.y() + margin, center.z() + margin, 1);
	crop.setMin(min_pt);
	crop.setMax(max_pt);
	crop.filter(*tmp_scene_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(0.001, 0.001, 0.001);
	vg.setInputCloud(target_cloud_);
	vg.filter(*sampled_target_cloud);
	vg.setInputCloud(tmp_scene_cloud);
	vg.filter(*sampled_scene_cloud);

	// *sampled_scene_cloud += *sampled_target_cloud;

	if (sampled_scene_cloud->empty()) {
		std::cout << "there are no points" << std::endl;
		return false;
	}

  FeatureCloudPtr obj_feature =  FeatureCloudPtr(new FeatureCloud());
	obj_feature->setInputCloud(sampled_target_cloud);
	obj_feature->setViewPoint(grasp::CameraHandler::instance()->getViewPoint());
	// obj_feature->setFeatureRadius(0.08);
	// obj_feature->setNormalRadius(0.04);
	// obj_feature->computeLocalFeatures();

	FeatureCloudPtr scene_feature =  FeatureCloudPtr(new FeatureCloud());
	scene_feature->setInputCloud(sampled_scene_cloud);
	scene_feature->setViewPoint(grasp::CameraHandler::instance()->getViewPoint());
	// scene_feature->setFeatureRadius(0.08);
	// scene_feature->setNormalRadius(0.04);
	// scene_feature->computeLocalFeatures();

	// SAC_IAEstimator sac_ia;
	// sac_ia.setObjFeatureCloud(obj_feature);
	// sac_ia.setSceneFeatureCloud(scene_feature);
	// sac_ia.setMaxCorrespondenceDist(0.01);
	// sac_ia.estimate();

	// PointCloudTPtr sac_aligned = sac_ia.getAlignedObjCloud();

	ICPEstimator icp;
	icp.setObjFeatureCloud(obj_feature);
	icp.setSceneFeatureCloud(scene_feature);
	icp.setIteration(20);
	icp.setMaxCorrespondenceDist(0.01);
	// icp.init_matrix = PointCloudUtil::convM3VtoM4(sac_ia.getRot(), sac_ia.getTrans());
	icp.init_matrix = PointCloudUtil::convM3VtoM4(calib_link->attitude(), calib_link->p());
	icp.estimate();

	icp.setObjFeatureCloud(obj_feature);
	icp.setSceneFeatureCloud(scene_feature);
	icp.setIteration(80);
	icp.setMaxCorrespondenceDist(0.005);
	// icp.init_matrix = PointCloudUtil::convM3VtoM4(sac_ia.getRot(), sac_ia.getTrans());
	icp.init_matrix = PointCloudUtil::convM3VtoM4(icp.getRot(), icp.getTrans());
	icp.estimate();
	PointCloudTPtr aligned = icp.getAlignedObjCloud();

	p = icp.getTrans();
	R = icp.getRot();

	// calib_model->body()->rootLink()->p() = R * target_link_->p() + p;
	// calib_model->body()->rootLink()->R() = R * target_link_->attitude();
	calib_model->body()->rootLink()->p() = p;
	calib_model->body()->rootLink()->R() = R;
	calib_model->notifyKinematicStateChange();

	cnoid::Vector3 camera_p = grasp::CameraHandler::instance()->getTargetCamera()->getCameraPosition();
	cnoid::Matrix3 camera_R = grasp::CameraHandler::instance()->getTargetCamera()->getCameraRotation();

	// p = camera_R.transpose() * (R * target_link_->p() + p - camera_p);
	// R = camera_R.transpose() * R * target_link_->attitude();
	p = camera_R.transpose() * (p - camera_p);
	R = camera_R.transpose() * R;


	// draw
	// PointCloudDrawer* draw = PointCloudDrawer::instance();

	// draw->clear();
	// draw->addPointCloud(sampled_target_cloud, cnoid::Vector3(0,0,1));
	// draw->addPointCloud(aligned, cnoid::Vector3(1,0,0));
	// draw->addPointCloud(sac_aligned, cnoid::Vector3(0,1,0));
	// draw->addPointCloud(sampled_scene_cloud);
	// draw->draw();

	return true;
}

void Calibration::loadRobotPose() {
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	QString data_path = QString::fromStdString(getDataDirPath());
	QString angle_path = QString("%1%2_angle.txt").arg(data_path).arg(current_id_, 4, 10, QChar('0'));
	std::ifstream ifs(angle_path.toStdString().c_str());
	std::string line;
	getline(ifs, line);
	std::stringstream buf(line);
	for (int i = 0; i < pb->body()->numJoints(); i++) {
		buf >> pb->body()->joint(i)->q();
	}
	pb->body()->calcForwardKinematics();
	pb->flush();
}

bool Calibration::loadPointCloud() {
	PointCloudHolder* ph = PointCloudHolder::instance();

	QString data_path = QString::fromStdString(getDataDirPath());
	QString pcd_path = QString("%1%2_scene.pcd").arg(data_path).arg(current_id_, 4, 10, QChar('0'));
	return ph->capture(true, false, pcd_path.toStdString(), "", false);
}

bool Calibration::initialSetting() {
	calib_model = new cnoid::BodyItem();

	// load parameters from INI file
	QString config_path = QString::fromStdString(getPluginPath() + "config.ini");
	QSettings setting(config_path, QSettings::IniFormat);
	setting.beginGroup("Calibration");

	std::string link_name = setting.value("target_link_name").toString().toStdString();
	std::string calib_model_filepath = setting.value("calib_model_file").toString().toStdString();

	setting.endGroup();

	// set target link which has a calibration marker
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	target_link_ = pb->body()->link(link_name);
	if (target_link_ == NULL) {
		return false;
	}

	// load the marker model
	if (!calib_model->loadModelFile(getPluginPath() + calib_model_filepath)) {
		return false;
	}
	calib_model->setName("Calib");
	cnoid::RootItem::mainInstance()->addChildItem(calib_model);
	cnoid::ItemTreeView::mainInstance()->checkItem(calib_model, true);

	// generate marker model point cloud
	target_cloud_ = PointCloudTPtr(new PointCloudT());
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cnoid::ColdetModelPtr model = calib_model->body()->rootLink()->coldetModel();
#else
	cnoid::SgNode* model = calib_model->body()->rootLink()->collisionShape();
#endif
	PointCloudUtil::modelToPointCloud(model, target_cloud_);

	return true;
}

void Calibration::calcRototranslation(const pointVec& y_points, const pointVec& x_points, cnoid::Matrix4f& yTx) {
	if (y_points.size() != x_points.size()) return;
	int size = y_points.size();

	cnoid::Vector3 ave_y = cnoid::Vector3::Zero();
	cnoid::Vector3 ave_x = cnoid::Vector3::Zero();

	for (int i = 0; i < size; i++) {
		ave_y += y_points[i];
		ave_x += x_points[i];
	}
	ave_y /= size;
	ave_x /= size;

	cnoid::MatrixXd X(3, size);
	cnoid::MatrixXd Y(3, size);

	for (int i = 0; i < size; i++) {
		X.col(i) = x_points[i] - ave_x;
		Y.col(i) = y_points[i] - ave_y;
	}

	cnoid::Matrix3 XY = X * Y.transpose();

	Eigen::JacobiSVD<cnoid::Matrix3> svd(XY, Eigen::ComputeFullU | Eigen::ComputeFullV);

	cnoid::Matrix3 V = svd.matrixV();
	cnoid::Matrix3 U = svd.matrixU();

	cnoid::Matrix3 H = cnoid::Vector3(1, 1, (V*U.transpose()).determinant()).asDiagonal();

	cnoid::Matrix3 R = V * H * U.transpose();

	cnoid::Vector3 t = ave_y - R * ave_x;

	yTx = PointCloudUtil::convM3VtoM4(R, t);
}

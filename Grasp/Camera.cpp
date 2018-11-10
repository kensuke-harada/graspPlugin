#include "Camera.h"

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>

#include "TransformationMatrixReader.h"
#include "UtilFunction.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/ValueTree>
#endif

using namespace grasp;
using namespace cnoid;
using std::string;


CameraHandler::CameraHandler() {
	// add default camera and set it as a target camera.
	addDefaultFixCamera();
	target_camera_ = cameras_[0];
	target_camera_id_ = 0;
}

CameraHandler::~CameraHandler() {
}

CameraHandler* CameraHandler::instance() {
	static CameraHandler* instance = new CameraHandler();
	return instance;
}

void CameraHandler::load(BodyPtr body, const string& filepath) {
	// clear cameras and add default camera
	addDefaultFixCamera();

	// load camera information and append cameras
	CameraParameterReader cpr;
	cpr.readRobotYamlFile(body, filepath);
	CameraArray tmp_cameras = cpr.getCameras();
	cameras_.insert(cameras_.end(), tmp_cameras.begin(), tmp_cameras.end());
	target_camera_ = cameras_[0];
	target_camera_id_ = 0;
}

void CameraHandler::init(BodyPtr body, const string& filepath) {
	load(body, filepath);
}

bool CameraHandler::setTargetCamera(int id) {
	if (id >= size()) return false;
	target_camera_ = cameras_[id];
	target_camera_id_ = id;
	return true;
}

CameraPtr CameraHandler::getCamera(int id) const {
	return cameras_[id];
}

CameraPtr CameraHandler::getTargetCamera() const {
	return target_camera_;
}

int CameraHandler::getTargetCameraID() const {
	return target_camera_id_;
}

cnoid::Vector3 CameraHandler::getViewPoint() const {
	return target_camera_->getCameraPosition();
}

AttachedCameraPtr CameraHandler::getTargetAttachedCamera() const {
	return boost::dynamic_pointer_cast<AttachedCamera>(target_camera_);
}

int CameraHandler::size() const {
	return cameras_.size();
}

void CameraHandler::addDefaultFixCamera() {
	cameras_.clear();
	CameraPtr camera = boost::make_shared<FixedCamera>();
	camera->setType("FIXED");
	camera->setFocalDistance(0.5);
	camera->setName("DEFAULT_FIXEDCAMERA");
	Vector3 p;
	Matrix3 R;
	TransMatReader::getTransMatrix(getDefaultFixCameraTransMatrixFilePath(), R, p);
	camera->setCameraPosition(R, p);
	cameras_.push_back(camera);
}

std::string CameraHandler::getDefaultFixCameraTransMatrixFilePath() const {
	std::string filepath = cnoid::executableTopDirectory() + std::string("/extplugin/graspPlugin/PCL/calibtools/calibmat.txt");
	return filepath;
}


CameraParameterReader::CameraParameterReader() {
}

CameraParameterReader::~CameraParameterReader() {
}

void CameraParameterReader::readRobotYamlFile(BodyPtr body, const string& filepath) {
	cameras_.clear();
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	if (body->info()->find("camera")->type() != ValueNode::SEQUENCE) return;
#else
	if (body->info()->find("camera")->nodeType() != ValueNode::LISTING) return;
#endif
	const Listing& glist = *(*body->info())["camera"].toListing();

	for (int i = 0; i < glist.size(); i++) {
		const Mapping& gSettings =  *glist[i].toMapping();
		if (!gSettings.isValid() || gSettings.empty()) {
			continue;
		}

		string str_type;
		bool is_fix = false;
		if (gSettings.read("type", str_type)) {
			is_fix = (str_type == "FIXED");
		} else {
			std::cerr << "ERROR (Robot YAML): there is a camera that type is not specified" << std::endl;
			continue;
		}

		CameraPtr camera;

		if (is_fix) {
			camera = boost::make_shared<FixedCamera>();
		} else {
			// set base link and camera link
			string str_base_link, str_camera_link;
			Link* base_link = NULL;
			Link* camera_link = NULL;

			if (gSettings.read("base", str_base_link)) {
				base_link = body->link(str_base_link);
			}
			if (gSettings.read("cameraLink", str_camera_link)) {
				camera_link = body->link(str_camera_link);
			}
			if (base_link == NULL || camera_link == NULL) {
				std::cerr << "ERROR (Robot YAML): there is a camera that base_link and/or camera_link is not specified" << std::endl;
				continue;
			}
			camera = boost::make_shared<AttachedCamera>(body, base_link, camera_link);
		}

		camera->setType(str_type);

		double dist;
		if (gSettings.read("focal_distance", dist)) {
			camera->setFocalDistance(dist);
		} else {
			std::cerr << "INFO (Robot YAML): there is a camera that focal_distance is not specified" << std::endl;
		}

		if (gSettings.read("minDistance", dist)) {
			camera->setMinDistance(dist);
		} else {
			camera->setMinDistance(camera->getFocalDistance());
		}

		if (gSettings.read("maxDistance", dist)) {
			camera->setMaxDistance(dist);
		} else {
			camera->setMaxDistance(camera->getFocalDistance());
		}

		string str_name;
		if (gSettings.read("name", str_name)) {
			camera->setName(str_name);
		}

		///// to be removed //////
		const Listing& dir_list = *gSettings.findListing("direction");
		if (dir_list.isValid() && dir_list.size() == 3) {
			camera->setDirection(Vector3(dir_list[0].toDouble(), dir_list[1].toDouble(), dir_list[2].toDouble()));
		}

		const Listing& pos_list = *gSettings.findListing("pos");
		if (pos_list.isValid() && pos_list.size() == 3) {
			camera->setPositionLocal(Vector3(pos_list[0].toDouble(), pos_list[1].toDouble(), pos_list[2].toDouble()));
		}
		/////////////////////////////

		string str_transmat_filepath;
		if (gSettings.read("transMatFile", str_transmat_filepath)) {
			boost::filesystem::path transmat_path(str_transmat_filepath);
			string transmat_filepath;
			if (transmat_path.has_root_path()) {
				transmat_filepath = getNativePathString(transmat_path);
			} else {
				boost::filesystem::path orgpath(filepath);
				transmat_filepath = getNativePathString(orgpath.parent_path() / transmat_path);
			}
			Vector3 p;
			Matrix3 R;
			if (!TransMatReader::getTransMatrix(transmat_filepath, R, p)) {
				std::cerr << "ERROR (Robot YAML): loading error (" << transmat_filepath << std::endl;
			}
			camera->setCameraPosition(R, p);
		} else {
			std::cerr << "ERROR (Robot YAML): there is a camera that transMatFile is not specified" << std::endl;
		}

		cameras_.push_back(camera);
	}
}

const CameraArray& CameraParameterReader::getCameras() const {
	return cameras_;
}

Camera::Camera() :
	focal_distance_(0.5),
	min_distance_(0.5),
	max_distance_(0.5),
	camera_R_(Matrix3::Identity()),
	camera_p_(Vector3::Zero()) {
}

Camera::~Camera() {
}

void Camera::setName(const string& name) {
	name_ = name;
}

string Camera::getName() const {
	return name_;
}

void Camera::setType(const string& type) {
	if (type == "HEAD") {
		type_ = HEAD_CAMERA;
	} else if (type == "HAND") {
		type_ = HANDEYE_CAMERA;
	} else if (type == "FIXED") {
		type_ = FIXED_CAMERA;
	} else {
		type_ = UNKNOWN;
	}
}

Camera::CameraType Camera::getType() const {
	return type_;
}

bool Camera::isFixedCamera() const {
	return (type_ == FIXED_CAMERA);
}

bool Camera::isAttachedCamera() const {
	return (type_ == HANDEYE_CAMERA || type_ == HEAD_CAMERA);
}

void Camera::setFocalDistance(double distance) {
	focal_distance_ = distance;
}

double Camera::getFocalDistance() const {
	return focal_distance_;
}

void Camera::setMinDistance(double distance) {
	min_distance_ = distance;
}

double Camera::getMinDistance() const {
	return min_distance_;
}

void Camera::setMaxDistance(double distance) {
	max_distance_ = distance;
}

double Camera::getMaxDistance() const {
	return max_distance_;
}

void Camera::setCameraPosition(const Matrix3& R, const Vector3& p) {
	camera_R_ = R;
	camera_p_ = p;
}

Vector3 Camera::getOffset() const {
	return camera_p_;
}

FixedCamera::FixedCamera() {
	type_ = FIXED_CAMERA;
}

FixedCamera::~FixedCamera() {
}

Vector3 FixedCamera::getCameraPosition() const {
	return camera_p_;
}

Matrix3 FixedCamera::getCameraRotation() const {
	return camera_R_;
}

Vector3 FixedCamera::getViewDirection() const {
	return camera_R_ * Vector3::UnitZ();
}

AttachedCamera::AttachedCamera(cnoid::BodyPtr body, cnoid::Link* base, cnoid::Link* camera) {
	camera_path_ = createJointPath(base, camera);
	body_ = body;
}

AttachedCamera::~AttachedCamera() {
}

AttachedCameraPtr AttachedCamera::copy(cnoid::BodyPtr body) const {
	AttachedCameraPtr ret =
		boost::make_shared<AttachedCamera>(body,
																			 body->link(camera_path_->baseLink()->name()),
																			 body->link(camera_path_->endLink()->name()));
	ret->name_ = name_;
	ret->type_ = type_;
	ret->focal_distance_ = focal_distance_;
	ret->camera_R_ = camera_R_;
	ret->camera_p_ = camera_p_;
	ret->dir = dir;
	return ret;
}

Vector3 AttachedCamera::getCameraPosition() const {
	return (getCameraLink()->R() * camera_p_ + getCameraLink()->p());
}

Matrix3 AttachedCamera::getCameraRotation() const {
	return getCameraLink()->R() * camera_R_;
}

Vector3 AttachedCamera::getViewDirection() const {
	return getCameraLink()->R() * camera_R_ * Vector3::UnitZ();
}

cnoid::JointPathPtr AttachedCamera::getPath() const {
	return camera_path_;
}

cnoid::Link* AttachedCamera::getBaseLink() const {
	return camera_path_->baseLink();
}

cnoid::Link* AttachedCamera::getCameraLink() const {
	return camera_path_->endLink();
}

cnoid::BodyPtr AttachedCamera::getBody() const {
	return body_;
}

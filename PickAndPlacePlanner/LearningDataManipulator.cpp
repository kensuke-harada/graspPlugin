#include "LearningDataManipulator.h"

#include "../Grasp/PlanBase.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"
#include "../Grasp/GraspController.h"

#include "SweptVolume.h"

namespace fs = boost::filesystem;

void AngleFileManipulator::write(const std::string& filepath, const std::vector<double>& q) {
	std::ofstream ofs(filepath.c_str());
	for (size_t i = 0; i < q.size(); i++) {
		ofs << q[i];
		if (i != (q.size() -1)) ofs << " ";
	}
}

void AngleFileManipulator::read(const std::string& filepath, cnoid::BodyPtr& body) {
	std::ifstream ifs(filepath.c_str());
	std::string line;
	getline(ifs, line);
	std::stringstream buf(line);
	for (int i = 0; i < body->numJoints(); i++) {
		buf >> body->joint(i)->q();
	}
	body->calcForwardKinematics();
}

void ObjectPosFileManipulator::write(const std::string& filepath, cnoid::Link* object_link) {
	std::ofstream ofs(filepath.c_str());
	for (int i = 0; i < 3; i++) {
		ofs << object_link->p()(i);
		if (i!=2) ofs << " ";
	}
	ofs << std::endl;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ofs << object_link->R()(i, j);
			if (!(i == 2 && j == 2))
				ofs << " ";
		}
	}
}

void ObjectPosFileManipulator::read(const std::string& filepath, cnoid::Link* object_link) {
	cnoid::Vector3 p;
	cnoid::Matrix3 R;
	std::ifstream ifs(filepath.c_str());
	std::string line;
	getline(ifs, line);
	std::stringstream buf_p(line);
	for (int i = 0; i < 3; i++) {
		buf_p >> p(i);
	}
	getline(ifs, line);
	std::stringstream buf_R(line);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			buf_R >> R(i, j);
		}
	}
	object_link->p() = p;
	object_link->R() = R;
}

void PointCloudFileManipulator::write(const std::string& point_filepath, const std::string& idx_filepath, const grasp::ObjPoseEstimateSol& sol) {
	std::ofstream p_ofs(point_filepath.c_str());
	std::vector<cnoid::Vector3>& points = sol.env_point_cloud->p();
	for (size_t i = 0; i < points.size(); i++) {
		p_ofs << points[i].transpose() << std::endl;
	}

	std::ofstream i_ofs(idx_filepath.c_str());
	const std::vector<int>& indices = sol.outlier_indices;
	for (size_t i = 0; i < indices.size(); i++) {
		i_ofs << indices[i] << " ";
	}
}

void PointCloudFileManipulator::read(const std::string& point_filepath, const std::string& idx_filepath, grasp::ObjPoseEstimateSol& sol) {
	std::ifstream p_ifs(point_filepath.c_str());
	std::string line;
	std::vector<cnoid::Vector3>& points = sol.env_point_cloud->p();
	points.clear();
	if (p_ifs.eof()) return;
	while (getline(p_ifs, line)) {
		std::stringstream buf(line);
		cnoid::Vector3 vec;
		if (buf.eof()) break;
		for (int i = 0; i < 3; i++) {
			buf >> vec(i);
		}
		points.push_back(vec);
	}

	sol.outlier_indices.clear();
	std::ifstream i_ifs(idx_filepath.c_str());
	while(!i_ifs.eof()) {
		int index;
		i_ifs >> index;
		sol.outlier_indices.push_back(index);
	}
}

GraspReconstractor* GraspReconstractor::instance() {
	static GraspReconstractor* instance = new GraspReconstractor();
	return instance;
}

GraspReconstractor::GraspReconstractor() :
	data_dir_("./data/"),
	angle_suffix_("_angle.txt"),
	obj_suffix_("_object.txt"),
	idx_suffix_("_idx.txt"),
	pcd_suffix_("_pcd.txt"),
	done_init_(false) {

	cnoid::Item* ro_item = cnoid::RootItem::instance();
	point_cloud_ = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	point_cloud_->setName("PointCloud");
	ro_item->addChildItem(point_cloud_);
	cnoid::ItemTreeView::instance()->checkItem(point_cloud_);
}

bool GraspReconstractor::reconstract(int id) const {
	std::string id_str = (boost::format("%d") % id).str();
	if (!checkFiles(id_str)) {
		std::cout << "files does not exist: id" << id << std::endl;
		return false;
	}
	loadObjectPose(id_str);
	loadRobotPose(id_str);
	loadPointCloud(id_str);

	grasp::PlanBase::instance()->flush();

	if (grasp::ObjPoseEstimateSolHolder::instance()->at(0).env_point_cloud->p().empty()) {
		return false;
	}
	return true;
}

bool GraspReconstractor::checkFiles(const std::string& id_str) const {
	const fs::path dir(data_dir_);
	if (!fs::exists(dir / fs::path(id_str + angle_suffix_))) return false;
	if (!fs::exists(dir / fs::path(id_str + obj_suffix_))) return false;
	if (!fs::exists(dir / fs::path(id_str + idx_suffix_))) return false;
	if (!fs::exists(dir / fs::path(id_str + pcd_suffix_))) return false;

	return true;
}

void GraspReconstractor::loadObjectPose(const std::string& id_str) const {
	std::string object_filepath = data_dir_ + id_str + obj_suffix_;
	cnoid::Link* obj_link = grasp::PlanBase::instance()->targetObject->object;
	ObjectPosFileManipulator::read(object_filepath, obj_link);
}

void GraspReconstractor::loadRobotPose(const std::string& id_str) const {
	std::string angle_filepath = data_dir_ + id_str + angle_suffix_;
	cnoid::BodyPtr body = grasp::PlanBase::instance()->body();
	AngleFileManipulator::read(angle_filepath, body);
}

void GraspReconstractor::loadPointCloud(const std::string& id_str) const {
	std::string pcd_filepath = data_dir_ + id_str + pcd_suffix_;
	std::string idx_filepath = data_dir_ + id_str + idx_suffix_;

	grasp::ObjPoseEstimateSol sol;
	sol.env_point_cloud = new grasp::PointCloudEnv();
	grasp::ObjPoseEstimateSolHolder::instance()->clear();

	PointCloudFileManipulator::read(pcd_filepath, idx_filepath, sol);

	grasp::ObjPoseEstimateSolHolder::instance()->push_back(sol);
}

void GraspReconstractor::showPointCloud() {
	grasp::ObjPoseEstimateSol& sol = grasp::ObjPoseEstimateSolHolder::instance()->at(0);

	cnoid::SgVertexArrayPtr vertices = new cnoid::SgVertexArray();
	for (size_t i = 0; i < sol.outlier_indices.size(); i++) {
		vertices->push_back((sol.env_point_cloud->p()[sol.outlier_indices[i]]).cast<float>());
	}
	point_cloud_->pointSet()->setVertices(vertices);
	point_cloud_->notifyUpdate();
}

void GraspReconstractor::computeInsidePoints(int id, bool output_points, bool show_sv) {
	if (!done_init_) {
		done_init_ = true;
		initialSetting();
	}
	const double len = 0.1;
	grasp::PlanBase* pb = grasp::PlanBase::instance();

	cnoid::Vector3 app_vec = pb->palm()->R() * GRCR_ * cnoid::Vector3(0, len, 0);

	SweptVolume sv;
	sv.setGRCR(GRCR_);
	sv.setNumGraspStep(3);
	sv.setAppLength(len);
	sv.makeSweptVolume();

	SweptVolumeChecker svc(&sv);
	svc.setObjName(pb->targetObject->name());
	svc.setMargin(0.002);
	grasp::ObjPoseEstimateSol& sol = grasp::ObjPoseEstimateSolHolder::instance()->at(0);
	svc.setObjPoseEstimateSol(&(sol));

	svc.check(pb->palm()->p()-app_vec, pb->palm()->R());

	std::vector<double> f;
	svc.getFeatureVector(f);
	
	if (output_points) {
		std::string points_filename = data_dir_ + (boost::format("sv_points_%04d") % id).str();
		svc.outputPointsGRC(points_filename);
	}

	if (show_sv) {
		SweptVolumeDrawer sv_drawer(&sv);
		sv_drawer.clear();
		std::vector<int> indices;
		svc.getInlierIndices(indices);
		sv_drawer.addPointsPtr(&(sol.env_point_cloud->p()), &indices);
		sv_drawer.draw(-app_vec);
	}

}

class GraspControllerProxy : public grasp::GraspController {
public:
	void settingPrehensionParam(const grasp::PrehensionParameter& param) {settingPrehension(param);}
};

void GraspReconstractor::initialSetting() {
	grasp::PrehensionPtr target_prehension = grasp::PlanBase::instance()->targetArmFinger->getTargetPrehension()[0];
	grasp::PrehensionParameter pp;
	target_prehension->convertToPrehensionParameter(&pp);

	GraspControllerProxy gcp;
	gcp.initial(grasp::PlanBase::instance()->targetObject, grasp::PlanBase::instance()->targetArmFinger);
	gcp.settingPrehensionParam(pp);

	GRCR_ = grasp::rotFromRpy(pp.GRCmax_rpy);
}

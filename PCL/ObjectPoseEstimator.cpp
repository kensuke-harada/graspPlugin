#define PCL_NO_PRECOMPILE
#include "ObjectPoseEstimator.h"

#include <limits>
#include <flann/flann.h>

#include <pcl/registration/icp.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "../Grasp/VectorMath.h"
#include "../GraspDataGen/Util/Logger.h"
#include "../Grasp/Camera.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include "CVFHDescriptorManipulator.h"
#endif

#include "PointCloudHolder.h"
#include "PointCloudUtility.h"

#include <Eigen/StdVector>

#include "ICPsSolver.h"

#include "Registrator.h"
#include "Segmenter.h"

#include "PoseEstimationResultMatcher.h"

#include "../GraspDataGen/Util/StopWatch.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/ColdetConverter.h"
#endif

#define THREAD


using namespace std;
namespace fs = boost::filesystem;

//#define DEBUG_MODE

ObjectPointCloudGenerator::ObjectPointCloudGenerator() {
}

ObjectPointCloudGenerator::~ObjectPointCloudGenerator() {
}

bool ObjectPointCloudGenerator::loadVRMLFile(const std::string& filepath, cnoid::BodyItemPtr& body_item) {
	// loda VRML model
	if (!body_item->load(filepath.c_str(), "OpenHRP-VRML-MODEL")) {
		return false;
	}
	// add model to itemtreeview
	cnoid::RootItem::instance()->addChildItem(body_item);
	cnoid::ItemTreeView::instance()->checkItem(body_item, true);

	return true;
}

void ObjectPointCloudGenerator::generatePointCloud(const cnoid::BodyItemPtr& obj_item, PointCloudTPtr& cloud) const {
	std::string pcd_filepath;
	// check whether pcd fiel path is specified in yaml file
	if (getPCDFilePath(obj_item, pcd_filepath)) {
		loadFromPCD(pcd_filepath, cloud);
	} else {
		generateFromBodyItem(obj_item, cloud);
	}
}

bool ObjectPointCloudGenerator::getPCDFilePath(const cnoid::BodyItemPtr& obj_item, std::string& path) const {
	const Mapping& info = *(obj_item->body()->info()->toMapping());
	ValueNode* dir_path = info.find("descriptorDir");

	if (dir_path->isString()) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		fs::path yaml_path(obj_item->lastAccessedFilePath());
#else
		fs::path yaml_path(obj_item->filePath());
#endif
		fs::path whole_pcd_path = yaml_path.parent_path() / dir_path->toString() / "whole.pcd";
		if (fs::exists(whole_pcd_path)) {
			path = whole_pcd_path.string();
			return true;
		}		
	}
	return false;
}

void ObjectPointCloudGenerator::loadFromPCD(const std::string& path, PointCloudTPtr& cloud) const {
	pcl::io::loadPCDFile<PointT>(path, *cloud);
}

void ObjectPointCloudGenerator::generateFromBodyItem(const cnoid::BodyItemPtr& obj_item, PointCloudTPtr& cloud) const {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ColdetModelPtr target_model = obj_item->body()->link(0)->coldetModel();
#else
	cnoid::ColdetModelPtr target_model = grasp::ColdetConverter::ConvertFrom(obj_item->body()->link(0)->collisionShape());
#endif
	PointCloudUtil::modelToPointCloud(target_model, cloud);
}

/**
* Constructor
*/
ObjectPoseEstimator::ObjectPoseEstimator() :
    obj_cloud(new PointCloudT),
    scene_cloud(new PointCloudT),
    sampled_obj_cloud(new PointCloudT),
    sampled_scene_cloud(new PointCloudT),
    env_cloud(new PointCloudT),
    env_normal(new NormalCloud),
    segmenter(NULL),
    estimator(NULL),
    initial_estimator(NULL),
    object_link(NULL),
    do_downsampling(true),
    voxel_leaf(0.005),
		region_max_(cnoid::Vector3(0.3, 0.3, 0.3)),
		region_min_(cnoid::Vector3(-0.3, -0.3, -0.3)),
    num_candidate(1),
    feature_radius(0.02),
    normal_radius(0.005),
    normal_k(10),
    feature_k(10),
    use_normal_ksearch(false),
    use_feature_ksearch(false),
		use_prev_result_(false),
		result_matcher_(new PoseEstimationResultMatcher) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	cvfh_est_ = new CVFHPoseEstimator();
#endif
}

/**
* Destructor
*/
ObjectPoseEstimator::~ObjectPoseEstimator() {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	delete cvfh_est_;
#endif
	delete result_matcher_;
}

bool ObjectPoseEstimator::readScene(bool is_load_file, bool is_save_file, const string& load_file_path, const string& save_file_path,
                                                                        bool is_merge, bool is_handcamera) {
	return readScene(is_load_file, is_save_file, load_file_path, save_file_path, is_merge);
}

/**
* Load the scene point cloud data.
* @param[in] is_load_file true for loading the point cloud from the file.
* @param[in] is_save_file true for saving the point cloud to the file.
* @param[in] load_file_path file path for loading
* @param[in] save_file_path file path for saving
* @param[in] is_merge true for merging previous point cloud
*/
bool ObjectPoseEstimator::readScene(bool is_load_file, bool is_save_file, const string& load_file_path, const string& save_file_path,
																		bool is_merge) {
	bool ret;
	PointCloudHolder* ph = PointCloudHolder::instance();
	ret = ph->capture(is_load_file, is_save_file, load_file_path, save_file_path, is_merge);
	scene_cloud = ph->getCloudPtr();
	return ret;
}

/**
* @brief Load the scene point cloud data. View point is set via parameter p and R.
* @param[in] is_load_file true for loading the point cloud from the file.
* @param[in] is_save_file true for saving the point cloud to the file.
* @param[in] load_file_path file path for loading
* @param[in] save_file_path file path for saving
* @param[in] p translation vector
* @param[in] R rotation matrix
* @param[in] is_merge true for merging previous point cloud
*/
bool ObjectPoseEstimator::readScene(bool is_load_file, bool is_save_file, const string& load_file_path, const string& save_file_path,
																		const cnoid::Vector3& p, const cnoid::Matrix3& R, bool is_merge) {
	bool ret;
	PointCloudHolder* ph = PointCloudHolder::instance();
	ret = ph->capture(is_load_file, is_save_file, load_file_path, save_file_path, p, R, is_merge);
	scene_cloud = ph->getCloudPtr();
	return ret;
}

/**
* Load the point cloud data of target object from the VRML file.
* @param[in] file_path the path of object VRML file.
*/
bool ObjectPoseEstimator::readObjectVRML(const string file_path) {
	BodyItemPtr bodyitem(new BodyItem);
	// laod VRML model
	if (!ObjectPointCloudGenerator::loadVRMLFile(file_path, bodyitem)) {
		return false;
	}

	// set object
	grasp::PlanBase::instance()->SetGraspedObject(bodyitem);

	// generate object cloud
	readObject();

	return true;
}

/**
* Make the point cloud data of targeted object.
*/
void ObjectPoseEstimator::readObject() {
	// target object bodyitem
	BodyItemPtr bodyitem = grasp::PlanBase::instance()->targetObject->bodyItemObject;

	// set object link
	object_link = bodyitem->body()->link(0);

	// generate object cloud
	ObjectPointCloudGenerator obj_cloud_gen;
	obj_cloud_gen.generatePointCloud(bodyitem, obj_cloud);
}

void ObjectPoseEstimator::getViewPoint(bool is_handcamera, cnoid::Vector3& view_point) const {
	getViewPoint(view_point);
}

void ObjectPoseEstimator::getViewPoint(cnoid::Vector3& view_point) {
	view_point = grasp::CameraHandler::instance()->getViewPoint();
}

void ObjectPoseEstimator::saveEnv(){
  pcl::io::savePCDFileASCII("./env.pcd",*env_cloud);
}

void ObjectPoseEstimator::readEnv(){
  pcl::io::loadPCDFile<pcl::PointXYZ>("./env.pcd",*env_cloud);
}

void ObjectPoseEstimator::setEnvCloud(PointCloudTPtr env) {
	pcl::copyPointCloud(*env, *env_cloud);
}

/**
* Get a pointer to the object cloud point.
* @return  a pointer to the object cloud point
*/
PointCloudTPtr ObjectPoseEstimator::getObjCloud() const {
    return obj_cloud;
}

/**
* Get a pointer to the scene cloud point.
* @return  a pointer to the scene cloud point.
*/
PointCloudTConstPtr ObjectPoseEstimator::getSceneCloud() const {
    return scene_cloud;
}

/**
* Get a pointer to the sampled object cloud point.
* @return a pointer to the sampled object cloud point
*/
PointCloudTPtr ObjectPoseEstimator::getSampledObjCloud() const {
    return sampled_obj_cloud;
}

/**
* Get a pointer to the sampled scene cloud point.
* @return a pointer to the sampled scene cloud point
*/
PointCloudTPtr ObjectPoseEstimator::getSampledSceneCloud() const {
    return sampled_scene_cloud;
}

/**
* Get a pointer to the scene cloud point(color).
* @return a pointer tot the scene cloud pointer.
*/
ColorPointCloudConstPtr ObjectPoseEstimator::getSceneColorCloud() const {
	return PointCloudHolder::instance()->getColorCloudPtr();
}

ColorPointCloudPtr ObjectPoseEstimator::getClusteredCloud() {
	return segmenter->getClusteredCloud();
}

/**
* Set a pointer to the posture estimater object.
* @param[in] _estiamtor a pointer to the posture estimater object
*/
void ObjectPoseEstimator::setEstimator(AbstractPoseEstimator* _estimator) {
    estimator = _estimator;
}

/**
* Set a pointer to the initial posture estimater object.
* @param[in] _estiamtor a pointer to the intial posture estimater object
*/
void ObjectPoseEstimator::setInitialEstimator(AbstractPoseEstimator* _estimator) {
    initial_estimator = _estimator;
}

/**
* Set a pointer to segemeter object.
* @param[in] _segmenter a pointer to segemeter object
*/
void ObjectPoseEstimator::setSegmenter(AbstractSegmenter* _segmenter) {
    segmenter = _segmenter;
}

/**
* Set a leaf size of Voxel filter.
* @param[in] _leaf leaf size
*/
void ObjectPoseEstimator::setVoxelLeaf(double _leaf) {
    voxel_leaf = _leaf;
}

/**
* Set search range.
* @param[in] _x_max,_x_min,_y_max,_y_min,_z_max,_z_min range
*/
void ObjectPoseEstimator::setTargetRegion(double _x_max, double  _x_min, double _y_max, double _y_min, double _z_max, double _z_min) {
	region_max_ = cnoid::Vector3(_x_max, _y_max, _z_max);
	region_min_ = cnoid::Vector3(_x_min, _y_min, _z_min);
}

void ObjectPoseEstimator::setTargetRegion(const cnoid::Vector3& min, const cnoid::Vector3& max) {
	region_max_ = max;
	region_min_ = min;
}

/**
* Set the number of taget cluster to be estimated.
* @param[in] num number of candidates
*/
void ObjectPoseEstimator::setCandidateNum(unsigned int num) {
    num_candidate = num;
}

/**
* Set a radius which is used for determing the neighbors in local feature estimation.
* @param[in] _radius radius
*/
void ObjectPoseEstimator::setFeatureRadius(double _radius) {
    feature_radius = _radius;
}

/**
* Set a radius which is used for determing the neighbors in normal estimation.
* @param[in] _radius radius
*/
void ObjectPoseEstimator::setNormalRadius(double _radius) {
    normal_radius = _radius;
}

/**
* Set the number of k-nearest neighbors in local feature estimation.
* @param[in] _k number of k-nearest neighbors
*/
void ObjectPoseEstimator::setFeatureK(unsigned int _k) {
    feature_k = _k;
}

/**
* Set the number of k-nearest neighbors in normal estimation.
* @param[in] _k number of k-nearest neighbors
*/
void ObjectPoseEstimator::setNormalK(unsigned int _k) {
    normal_k = _k;
}

/**
* Select a method to determine nearest neighbors in local feature estimation.
* @param[in] _use ture: use k-search false: use radius search
*/
void ObjectPoseEstimator::useFeatureKSearch(bool _use) {
    use_feature_ksearch = _use;
}

/**
* Select a method to determine nearest neighbors in normal estimation.
* @param[in] _use ture: use k-search false: use radius search
*/
void ObjectPoseEstimator::useNormalKSearch(bool _use) {
    use_feature_ksearch = _use;
}

void ObjectPoseEstimator::setICPMaxIteration(int _ite) {
	icp_max_iteration = _ite;
}

void ObjectPoseEstimator::setSegmentResegmentRadius(double radius) {
	segment_reseg_radius_ = radius;
}

void ObjectPoseEstimator::setSegmentBoundaryRadius(double radius) {
	segment_bound_radius_ = radius;
}

/**
* @brief Downsample the point clouds.
* @param[in] do_narrow narrow the search range if it is set to true
*/
bool ObjectPoseEstimator::downsampling(bool do_narrow) {
	PointCloudTPtr clipped_scene_cloud = PointCloudTPtr(new PointCloudT);

	// narrow the search range
	if (do_narrow) {
		PointCloudUtil::cropCloud(scene_cloud, clipped_scene_cloud, region_min_, region_max_);
	} else {
		pcl::copyPointCloud(*scene_cloud, *clipped_scene_cloud);
	}

	if (!do_downsampling) {
		// use the original point clouds
		pcl::copyPointCloud(*obj_cloud, *sampled_obj_cloud);
		pcl::copyPointCloud(*clipped_scene_cloud, *sampled_scene_cloud);
	} else {
		// downsampling by using voxel filter
		PointCloudUtil::voxelFilter(obj_cloud, sampled_obj_cloud, voxel_leaf);
		PointCloudUtil::voxelFilter(clipped_scene_cloud, sampled_scene_cloud, voxel_leaf);
	}
	pcl::copyPointCloud(*sampled_scene_cloud, *env_cloud);
	return true;
}

/**
* Segment objects from the point cloud of the scene.
*/
bool ObjectPoseEstimator::segment() {
    segmenter->setInputCloud(sampled_scene_cloud);
    return segmenter->segment();
}


void ObjectPoseEstimator::initCvfhEstimate(int& n_cand) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	grasp::ScopeStopWatch timer("recog(init)");
	initProc();

	// initial setting
	grasp::CameraPtr camera = grasp::CameraHandler::instance()->getTargetCamera();
	// cvfh_est_->setCameraR(camera->getCameraRotation());
	// cvfh_est_->setViewPoint(view_point);
	// cvfh_est_->setNN(nn);
	// cvfh_est_->doRejectByBoundingBoxSize(use_bb);
	cvfh_est_->setMaxDistance(max_distance_);
	cvfh_est_->setICPMaxIteration(icp_max_iteration);
	cvfh_est_->setObjectCloud(sampled_obj_cloud);
	cvfh_est_->setCandidates(&candidates_);

	cvfh_est_->initCvfhEstimate(grasp::PlanBase::instance()->targetObject->bodyItemObject);

	n_cand = getNumTargetCandidates();
#else
	std::cout << "CVFH is not supported. Please upgrade PCL" << std::endl;
#endif
}

bool ObjectPoseEstimator::cvfhEstimateOneCluster(int i, cnoid::Vector3& p, cnoid::Matrix3& R, std::vector<int>& outlier_idx,
																								 const cnoid::Vector3& view_point, const cnoid::Matrix3& cameraR, int nn, bool use_bb) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	int n_reuse = result_matcher_->getReuseSolSize();

	if (n_reuse > i) {
		registerReuseSol(i);
		PoseSolution& sol = estimation_sols.back();
		p = sol.p;
		R = sol.R;
		outlier_idx.clear();
		outlier_idx.insert(outlier_idx.end(), sol.outlier_indices.begin(), sol.outlier_indices.end());

		PointCloudClustersHolder* pcch = PointCloudClustersHolder::instance();
		grasp::ObjPoseEstimateSol pose_sol;
		grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
		opes->push_back(pose_sol);
		opes->back().p = sol.p;
		opes->back().R = sol.R;
		pcch->getCluster(pcch->size()-1).sol = &(opes->back());
		
		return true;
	}

	int cand_i = i - n_reuse;
	grasp::ScopeStopWatch timer("recog(main)");
	// TODO: following codes will be moved to initCvfhEstimate
	cvfh_est_->setCameraR(cameraR);
	cvfh_est_->setViewPoint(view_point);
	cvfh_est_->setNN(nn);
	cvfh_est_->doRejectByBoundingBoxSize(use_bb);

	PoseSolution sol;
	sol.score = std::numeric_limits<double>::max();
	ClusterInfo target_cluster;
	if (isSimilarCloudToPreviousResult(candidates_[cand_i].cloud, &target_cluster)) {
		sol.p = target_cluster.p;
		sol.R = target_cluster.R;
	} else{
		if (!cvfh_est_->cvfhEstimateOneCluster((cand_i), sol)) {
			return false;
		}
	}
	p = sol.p;
	R = sol.R;

	estimation_sols.push_back(sol);

	PointCloudClustersHolder* pcch = PointCloudClustersHolder::instance();
	pcch->addCluster().cloud = candidates_[cand_i].cloud;
	pcch->getCluster(pcch->size()-1).p = sol.p;
	pcch->getCluster(pcch->size()-1).R = sol.R;
	pcch->getCluster(pcch->size()-1).prev_match =
		(candidates_[cand_i].is_match_prev_cluster ? ClusterInfo::PARTIAL : ClusterInfo::NONE);

	grasp::ObjPoseEstimateSol pose_sol;
	grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
	opes->push_back(pose_sol);
	opes->back().p = sol.p;
	opes->back().R = sol.R;
	pcch->getCluster(pcch->size()-1).sol = &(opes->back());

	setOutlierIdx(p, R, outlier_idx);

	return true;
#else
	std::cout << "CVFH is not supported. Please upgrade PCL" << std::endl;
	return false;
#endif
}

/**
* Estimate the object position and posture by using CVFH feature.
* @param[in] do_move no use
* @param[in] view_point camera viewpoint
* @param[in] file_path not use
* @param[in] nn the maximum number of candidate to be applied ICP
* @param[in] b_selectbest only use the top estimation or not
* @param[in] is_binpicking
*/
bool ObjectPoseEstimator::cvfhEstimate(bool do_move, const cnoid::Vector3& view_point,
																				std::string file_path, int nn,
																				bool use_bb, bool b_selectbest, bool is_binpicking) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  grasp::StopWatch timer;
	timer.start();
	initProc();

	// initial setting
	grasp::CameraPtr camera = grasp::CameraHandler::instance()->getTargetCamera();
	cvfh_est_->setCameraR(camera->getCameraRotation());
	cvfh_est_->setViewPoint(view_point);
	cvfh_est_->setNN(nn);
	cvfh_est_->doRejectByBoundingBoxSize(use_bb);
	cvfh_est_->setMaxDistance(max_distance_);
	cvfh_est_->setICPMaxIteration(icp_max_iteration);
	cvfh_est_->setObjectCloud(sampled_obj_cloud);
	cvfh_est_->setCandidates(&candidates_);

	cvfh_est_->initCvfhEstimate(grasp::PlanBase::instance()->targetObject->bodyItemObject);

	timer.stopAndPrintTime("recog(init)");

	int n = getNumTargetCandidates();
	int n_reuse = result_matcher_->getReuseSolSize();
	for (int i = 0; i < n_reuse; i++) {
		registerReuseSol(i);
	}
	for (int i = 0; i < n - n_reuse; i++) {
		grasp::ScopeStopWatch timer("recog(main)");
		PoseSolution sol;
		sol.score = std::numeric_limits<double>::max();
		ClusterInfo target_cluster;
		bool has_sol = false;
		if (isSimilarCloudToPreviousResult(candidates_[i].cloud, &target_cluster)) {
			sol.p = target_cluster.p;
			sol.R = target_cluster.R;
			sol.score = target_cluster.score;
			has_sol = true;
			std::cout << "similar" << std::endl;
		} else {
			if (cvfh_est_->cvfhEstimateOneCluster(i, sol)) {
				has_sol = true;
				std::cout << "recog_suc:" << std::endl;
			} else {
				std::cout << "recog_fail:" << std::endl;
			}
		}
		if (has_sol) {
			setOutlierIdx(sol.p, sol.R, sol.outlier_indices);
			estimation_sols.push_back(sol);

			PointCloudClustersHolder* pcch = PointCloudClustersHolder::instance();
			pcch->addCluster().cloud = candidates_[i].cloud;
			pcch->getCluster(pcch->size()-1).p = sol.p;
			pcch->getCluster(pcch->size()-1).R = sol.R;
			pcch->getCluster(pcch->size()-1).sol = NULL;
			pcch->getCluster(pcch->size()-1).prev_match =
		(candidates_[i].is_match_prev_cluster ? ClusterInfo::PARTIAL : ClusterInfo::NONE);
		}
	}
	if (estimation_sols.empty()) {
		cout << "object recoginiton failed: cannot find object" << endl;
    return false;
	}

	cout << "sols_size:" << estimation_sols.size() << std::endl;
	
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
	if (b_selectbest) {
		object_link->p() = estimation_sols[0].p;
		object_link->R() = estimation_sols[0].R;
		grasp::ObjPoseEstimateSol sol;
		opes->push_back(sol);
		opes->at(0).p = estimation_sols[0].p;
		opes->at(0).R = estimation_sols[0].R;
		opes->at(0).score = estimation_sols[0].score;
		grasp::TargetObject* target_object = new grasp::TargetObject(pb->targetObject->bodyItemObject);
		opes->at(0).targetObject = target_object;
	} else {
		opes->resize(estimation_sols.size());
		std::vector<BodyItemPtr> bodyitems;
		getBodyItems(estimation_sols.size(), bodyitems);
		for (size_t i = 0; i < bodyitems.size(); i++) {
			bodyitems[i]->body()->link(0)->p() = estimation_sols[i].p;
			bodyitems[i]->body()->link(0)->R() = estimation_sols[i].R;
			bodyitems[i]->notifyKinematicStateChange();
			grasp::ObjPoseEstimateSol sol;
			opes->at(i) = sol;
			opes->at(i).p = estimation_sols[i].p;
			opes->at(i).R = estimation_sols[i].R;
			opes->at(i).score = estimation_sols[i].score;
			grasp::TargetObject* target_object = new grasp::TargetObject(bodyitems[i]);
			opes->at(i).targetObject = target_object;
		}
	}
	pb->flush();
	for (size_t i = 0; i < opes->size(); i++) {
		PointCloudClustersHolder::instance()->getCluster(i).sol = &(opes->at(i));
	}

	for (size_t i = 0; i < estimation_sols.size(); i++) {
		opes->at(i).outlier_indices = estimation_sols[i].outlier_indices;
	}

	if (!is_binpicking) {
		//index filter
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		// Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		// Extract the inliers
		extract.setInputCloud(env_cloud);
		extract.setIndices(boost::make_shared<std::vector<int> >(opes->at(0).outlier_indices));
		extract.setNegative (false);
		extract.filter(*env_cloud);
	}
	return true;
#else
	std::cout << "CVFH is not supported. Please upgrade PCL" << std::endl;
	return false;
#endif
}

void ObjectPoseEstimator::registerPrevCloudToResultMatcher() {
	result_matcher_->setPrevObjectCloud(&prev_clusters, sampled_obj_cloud);
}

void ObjectPoseEstimator::setUsePrevResult(bool flag) {
	use_prev_result_ = flag;
}

void ObjectPoseEstimator::setOutlierIdx(const cnoid::Vector3& p, const cnoid::Matrix3& R, std::vector<int>& outlier_idx) const {
	PointCloudTPtr sol_cloud = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::transCloud(sampled_obj_cloud, sol_cloud, R, p);

	KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
	tree->setInputCloud(sol_cloud);

	std::vector<int> nn_indices(1);
	std::vector<float> nn_distances(1);

	Eigen::Vector4f min_p, max_p;
	pcl::getMinMax3D(*sol_cloud, min_p, max_p);
	min_p -= 2.0 * voxel_leaf * Eigen::Vector4f::Ones();
	max_p += 2.0 * voxel_leaf * Eigen::Vector4f::Ones();
	int count = 0;
	outlier_idx.clear();
	outlier_idx.reserve(env_cloud->points.size());
	double dist_th = voxel_leaf * voxel_leaf * 4.0;
	for (size_t j = 0; j < env_cloud->points.size(); j++) {
		if (!isFinite(env_cloud->points[j]))
			continue;
		if ((min_p.x() > env_cloud->points[j].x) || (min_p.y() > env_cloud->points[j].y) || (min_p.z() > env_cloud->points[j].z)) {
			outlier_idx.push_back(j);
			continue;
		}
		if ((max_p.x() < env_cloud->points[j].x) || (max_p.y() < env_cloud->points[j].y) || (max_p.z() < env_cloud->points[j].z)) {
			outlier_idx.push_back(j);
			continue;
		}
		if (!tree->nearestKSearch(env_cloud->points[j], 1, nn_indices, nn_distances))
			continue;

		if (nn_distances[0] > dist_th)
			outlier_idx.push_back(j);
	}
}

void ObjectPoseEstimator::initProc() {
	// move the target object to origin
	object_link->p() = Vector3::Zero();
	object_link->R() = Matrix3::Identity();

	makeCandidates();

	PointCloudClustersHolder::instance()->clear();

	estimation_sols.clear();

	grasp::ObjPoseEstimateSolHolder::instance()->clear();
}

int ObjectPoseEstimator::getNumTargetCandidates() const {
	return std::min(static_cast<int>(num_candidate), static_cast<int>(candidates_.size()));
}

void ObjectPoseEstimator::makeCandidates() {
	if (use_prev_result_) {
		result_matcher_->matchPrevResults(sampled_scene_cloud);
	}
	// make point cloud of each cluster
	vector<pcl::PointIndices> cluster_indices = segmenter->getClusterPointIndices();
	vector<PointCloudTPtr> clusters;

	// obtain a bounding box of the target object
	Vector3 obj_center;
	Vector3 obj_edge;
	Matrix3 obj_rot;
	calcBoundingBox(sampled_obj_cloud, obj_edge, obj_center, obj_rot);
	max_distance_ = obj_edge.norm();
	std::vector<double> edge_vec(3);
	grasp::setVector3(obj_edge, edge_vec);
	std::sort(edge_vec.begin(), edge_vec.end());
	double obj_bb_face_area = edge_vec[1] * edge_vec[2];

	std::vector<Candidate> match_candidate;
	std::vector<Candidate> not_match_candidate;

	// determine the priority of each cluster
	candidates_.clear();
	vector<pcl::PointIndices> updated_cluster_indices;
	int id = 0;
	for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
			 it != cluster_indices.end(); ++it) {
		PointCloudTPtr tmp_cloud(new PointCloudT());
		pcl::copyPointCloud(*segmenter->getSegmentedCloud(), *it, *tmp_cloud);

		Vector3 edge;
		Vector3 center;
		Matrix3 rot;
		PointCloudUtil::calcBoundingBox(tmp_cloud, edge, center, rot);

		std::vector<double> cluster_edge_vec(3);
		grasp::setVector3(edge, cluster_edge_vec);
		std::sort(cluster_edge_vec.begin(), cluster_edge_vec.end());
		double cluster_bb_face_area = cluster_edge_vec[1] * cluster_edge_vec[2];
		bool done_resegment = false;
		if (obj_bb_face_area < cluster_bb_face_area) {
			std::vector<pcl::PointIndices> recluster_indices;
			done_resegment = segmenter->reSegment(*it, recluster_indices, segment_reseg_radius_, segment_bound_radius_);
			if (done_resegment) {
				for (vector<pcl::PointIndices>::const_iterator rit = recluster_indices.begin();
						 rit != recluster_indices.end(); ++rit) {
					PointCloudTPtr rtmp_cloud(new PointCloudT());
					pcl::copyPointCloud(*segmenter->getSegmentedCloud(), *rit, *rtmp_cloud);

					PointCloudUtil::calcBoundingBox(rtmp_cloud, edge, center, rot);

					double error = getDiffRatio(obj_edge, edge);

					bool is_match, is_reuse_result;
					result_matcher_->matchPrevResultsToPointcloud(rtmp_cloud, is_match, is_reuse_result);
					if (is_reuse_result) continue;

					Candidate candidate;
					candidate.id = id++;
					candidate.cloud = rtmp_cloud;
					candidate.error = error;
					candidate.edge = edge;
					candidate.center = center;
					candidate.rot = rot;
					candidate.is_match_prev_cluster = is_match;
					if (is_match) {
						match_candidate.push_back(candidate);
					} else {
						not_match_candidate.push_back(candidate);
					}

					updated_cluster_indices.push_back(*rit);
				}
			}
		}
		if (!done_resegment) {
			double error = getDiffRatio(obj_edge, edge);

			bool is_match, is_reuse_result;
			result_matcher_->matchPrevResultsToPointcloud(tmp_cloud, is_match, is_reuse_result);
			if (is_reuse_result) continue;

			Candidate candidate;
			candidate.id = id++;
			candidate.cloud = tmp_cloud;
			candidate.error = error;
			candidate.edge = edge;
			candidate.center = center;
			candidate.rot = rot;
			candidate.is_match_prev_cluster = is_match;
			if (is_match) {
				match_candidate.push_back(candidate);
			} else {
				not_match_candidate.push_back(candidate);
			}

			updated_cluster_indices.push_back(*it);
		}
	}
	segmenter->setClusterPointIndices(updated_cluster_indices);
	segmenter->generateClusteredCloud();

	std::sort(match_candidate.begin(), match_candidate.end(), Candidate::Less);
	std::sort(not_match_candidate.begin(), not_match_candidate.end(), Candidate::Less);
	candidates_.insert(candidates_.end(), match_candidate.begin(), match_candidate.end());
	candidates_.insert(candidates_.end(), not_match_candidate.begin(), not_match_candidate.end());
}

bool ObjectPoseEstimator::isSimilarCloudToPreviousResult(PointCloudTConstPtr cloud, ClusterInfo* cluster) const {
	return false;
	for (size_t i = 0; i < prev_clusters.size(); i++) {
		PointCloudTPtr prev_result_cloud = PointCloudTPtr(new PointCloudT());
		PointCloudUtil::transCloud(sampled_obj_cloud, prev_result_cloud, prev_clusters[i].R, prev_clusters[i].p);

		KdTreePointTPtr tree = KdTreePointTPtr(new KdTreePointT(false));
		tree->setInputCloud(prev_result_cloud);

		std::vector<int> nn_indices(1);
		std::vector<float> nn_distances(1);

		Eigen::Vector4f min_p, max_p;
		pcl::getMinMax3D(*prev_result_cloud, min_p, max_p);
		min_p -= 2.0 * voxel_leaf * Eigen::Vector4f::Ones();
		max_p += 2.0 * voxel_leaf * Eigen::Vector4f::Ones();

		int count = 0;
		for (size_t j = 0; j < cloud->points.size(); j++) {
			if (!isFinite(cloud->points[j]))
				continue;
			if ((min_p.x() > cloud->points[j].x) || (min_p.y() > cloud->points[j].y) || (min_p.z() > cloud->points[j].z)) {
				continue;
			}
			if ((max_p.x() < cloud->points[j].x) || (max_p.y() < cloud->points[j].y) || (max_p.z() < cloud->points[j].z)) {
				continue;
			}
			if (!tree->nearestKSearch(cloud->points[j], 1, nn_indices, nn_distances))
				continue;

			if (nn_distances[0] < voxel_leaf * voxel_leaf * 4.0)
				count++;
		}

		if (count > 0.8 * cloud->points.size()) {
			cluster->p = prev_clusters[i].p;
			cluster->R = prev_clusters[i].R;
			cluster->score = prev_clusters[i].score;
			return true;
		}
	}
	return false;
}

void ObjectPoseEstimator::displayEstimationResults() {
    grasp::PlanBase* pb = grasp::PlanBase::instance();
		grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
    std::vector<BodyItemPtr> bodyitems;
    getBodyItems(opes->size(), bodyitems);
    for (size_t i = 0; i < bodyitems.size(); i++) {
        bodyitems[i]->body()->rootLink()->p() = opes->at(i).p;
				bodyitems[i]->body()->rootLink()->R() = opes->at(i).R;
        bodyitems[i]->notifyKinematicStateChange();
    }
    pb->flush();
}



void ObjectPoseEstimator::getBodyItems(int n, std::vector<cnoid::BodyItemPtr>& items) const {
	items.clear();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	ItemPtr ro_item = RootItem::mainInstance()->findItem<Item>("RecognizedObjects");
#else
	std::string name = "RecognizedObjects";
	Item* ro_item = RootItem::mainInstance()->findItem<RecognizedRootItem>(name);
#endif
	if (ro_item == NULL) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ro_item = new Item();
#else
		ro_item = new RecognizedRootItem();
#endif
		ro_item->setName("RecognizedObjects");
		RootItem::mainInstance()->addChildItem(ro_item);
	}

	grasp::PlanBase* pb = grasp::PlanBase::instance();

	Item* target_item = ro_item->childItem();
	while (target_item != NULL) {
		bool is_target = false;
		if (static_cast<int>(items.size()) < n) {
			if (target_item->name() == pb->targetObject->bodyItemObject->name()) {
				BodyItem* bodyitem = dynamic_cast<BodyItem*>(target_item);
				if (bodyitem != NULL) {
					is_target = true;
					items.push_back(bodyitem);
				}
			}
		}
		ItemTreeView::mainInstance()->checkItem(target_item, is_target);
		target_item = target_item->nextItem();
	}

	for (int i = items.size(); i < n; i++) {
		BodyItemPtr bodyitem(new BodyItem(*(grasp::PlanBase::instance()->targetObject->bodyItemObject)));
		ro_item->addChildItem(bodyitem);
		ItemTreeView::mainInstance()->checkItem(bodyitem, true);
		items.push_back(bodyitem);
	}
}

/**
* Estimate the object position and posture.
* @param[in] do_move if it is true, the point cloud and the taraget object are translated and rotated using Matirx in calibmat.txt
*/
bool ObjectPoseEstimator::estimate(bool do_move) {
    // move the target object to origin
    object_link->p() = Vector3::Zero();
    object_link->R() = Matrix3::Identity();

    // make point cloud of each cluster
    vector<pcl::PointIndices> cluster_indices = segmenter->getClusterPointIndices();
    vector<PointCloudTPtr> clusters;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloudTPtr tmp_cloud(new PointCloudT());
        pcl::copyPointCloud(*segmenter->getSegmentedCloud(),*it,*tmp_cloud);
        clusters.push_back(tmp_cloud);
    }

    // obtain a bounding box of the target object
    Vector3 obj_center;
    Vector3 obj_edge;
    Matrix3 obj_rot;
    calcBoundingBox(sampled_obj_cloud, obj_edge, obj_center, obj_rot);

    // compute local features of the target object
    FeatureCloudPtr obj_feature = FeatureCloudPtr(new FeatureCloud());
    createFeatureCloud(sampled_obj_cloud, Vector3(0, 0, 2.0), obj_feature);

//#ifdef DEBUG_MODE
//	pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_obj(new pcl::PointCloud<pcl::PointNormal>());
//	pcl::PointCloud<pcl::PointNormal>::Ptr point_normal_scene(new pcl::PointCloud<pcl::PointNormal>());
//	pcl::concatenateFields(*obj_cloud_dsample,*obj_feature.getNormals(),*point_normal_obj);
//	pcl::concatenateFields(*scene_cloud_dsample,*scene_feature.getNormals(),*point_normal_scene);
//	pcl::io::savePCDFileASCII(cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PCL/obj.pcd", *point_normal_obj);
//	pcl::io::savePCDFileASCII(cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PCL/scene.pcd", *point_normal_scene);
//#endif

    // determin the priority of each cluster
    std::vector<Candidate> candidates;
    int id=0;
    for (vector<PointCloudTPtr>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
        Vector3 edge;
        Vector3 center;
        Matrix3 rot;
        calcBoundingBox(*it, edge, center, rot);

        double error = getDiffRatio(obj_edge,edge);

        Candidate candidate;
        candidate.id = id++;
        candidate.cloud = *it;
        candidate.error = error;
        candidate.edge = edge;
        candidate.center = center;
        candidate.rot = rot;
				candidate.is_match_prev_cluster = false;
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(), Candidate::Less);

    vector<Vector3> p_list;
    vector<Matrix3> r_list;
    vector<double> scores;

// #if defined(DEBUG_MODE) && defined(ENABLE_OSG)
//     DrawUtility::instance()->boxes.clear();
// #endif

    for (int i = 0; i < num_candidate; i++) {
        if (i >= clusters.size()) break;
// #if defined(DEBUG_MODE) && defined(ENABLE_OSG)
//         DrawUtility::instance()->boxes.push_back(Boxes(candidates[i].center,candidates[i].rot,candidates[i].edge,Vector3(1,0,0),0.2));
// #endif
        object_link->p() = Vector3::Zero();
        object_link->R() = Matrix3::Identity();

        // compute local features of the cluster
        FeatureCloudPtr cluster_feature = FeatureCloudPtr(new FeatureCloud());
        createFeatureCloud(candidates[i].cloud, Vector3(0, 0, 0), cluster_feature);

        if (initial_estimator != NULL) {
            // initial pose estimation
            initial_estimator->setObjFeatureCloud(obj_feature);
            initial_estimator->setSceneFeatureCloud(cluster_feature);
            initial_estimator->setMaxCorrespondenceDist(voxel_leaf * 2.0);
            if (initial_estimator->estimate()) {
                moveObject(initial_estimator->getRot(), initial_estimator->getTrans());
                pcl::copyPointCloud(*initial_estimator->getAlignedObjCloud(), *obj_cloud);
            } else {
                p_list.push_back(Vector3::Zero());
                r_list.push_back(Matrix3::Identity());
                scores.push_back(DBL_MAX);
                continue;
            }
        }

        // pose estimation
        FeatureCloudPtr moved_obj_feature = FeatureCloudPtr(new FeatureCloud());
        moved_obj_feature->setInputCloud(obj_cloud);
        estimator->setObjFeatureCloud(moved_obj_feature);
        estimator->setSceneFeatureCloud(cluster_feature);
        estimator->setMaxCorrespondenceDist(voxel_leaf * 2.0);
        if (estimator->estimate()) {
            moveObject(estimator->getRot(), estimator->getTrans());
        } else {
            p_list.push_back(Vector3::Zero());
            r_list.push_back(Matrix3::Identity());
            scores.push_back(DBL_MAX);
            continue;
        }

        p_list.push_back(object_link->p());
        r_list.push_back(object_link->R());
        scores.push_back(estimator->getScore());
#ifdef DEBUG_MODE
        cout << "estimation score:" << estimator->getScore() << endl;
#endif
    }

    // move the object to the best position
    int idx = grasp::argmin(scores);
    object_link->p() = p_list[idx];
    object_link->R() = r_list[idx];

    pcl::SegmentDifferences<pcl::PointXYZ> seg;
    seg.setInputCloud(sampled_scene_cloud);
    seg.setTargetCloud(estimator->getAlignedObjCloud());
    PointCloudTPtr tmp = PointCloudTPtr(new PointCloudT());
    seg.setDistanceThreshold(voxel_leaf * 2.0 * voxel_leaf * 2.0);
    seg.segment(*tmp);
    pcl::copyPointCloud(*tmp, *env_cloud);

    return true;
}



/**
* Get the difference ratio between the bounding boxes.
* @param[in] ori_edge edge length of the bounding box of the target object
* @param[in] edge     edge length of the bounding box of the cluster
*/
double ObjectPoseEstimator::getDiffRatio(const Vector3& ori_edge, const Vector3& edge) const {
    std::vector<double> ori_edge_(3), edge_(3);
    grasp::setVector3(ori_edge, ori_edge_);
    grasp::setVector3(edge, edge_);
    std::sort(ori_edge_.begin(), ori_edge_.end());
    std::sort(edge_.begin(), edge_.end());

    double error = 0.0;
    for (int i = 1; i < 3; i++) {
        error += fabs((edge_[i]/ori_edge_[i]) - 1);
    }

    return error;
}

/**
* Create a feature cloud of the point cloud.
* @param[in] cloud target point cloud
* @param[in] view_point view point which is used in normal estimation
* @param[out] a pointer to feature cloud
*/
void ObjectPoseEstimator::createFeatureCloud(PointCloudTConstPtr cloud, const cnoid::Vector3& view_point, FeatureCloudPtr feature, bool only_normal) {
    feature->setInputCloud(cloud);
    feature->useNormalKSearch(use_normal_ksearch);
    if (use_normal_ksearch) {
        feature->setNormalK(normal_k);
    } else {
        feature->setNormalRadius(normal_radius);
    }
    feature->useFeatureKSearch(use_feature_ksearch);
    if (use_feature_ksearch) {
        feature->setFeatureK(feature_k);
    } else {
        feature->setFeatureRadius(feature_radius);
    }
    feature->setViewPoint(view_point);
    if (only_normal) {
        feature->computeNormals();
    } else {
        feature->computeLocalFeatures();
    }
}

double ObjectPoseEstimator::getScore(PointCloudTConstPtr obj, PointCloudTConstPtr scene, double max_range) const {
    pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> tve;
    tve.setMaxRange (max_range);
    return tve.validateTransformation (scene, obj, Matrix4f::Identity());
}

int ObjectPoseEstimator::getNumOutlierPoints(PointCloudTConstPtr obj, PointCloudTConstPtr scene, double max_range) const {
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(scene);
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    int nr_out = 0;
    for (size_t i = 0; i < obj->points.size(); i++) {
        tree.nearestKSearch(obj->points[i], 1, nn_indices, nn_dists);
        if (nn_dists[0] > max_range * max_range) {
            nr_out++;
            continue;
        }
    }
    return nr_out;
}

void ObjectPoseEstimator::calcBoundingBox(PointCloudTConstPtr cloud, Vector3& edge, Vector3& center, Matrix3& Rot) {
	PointCloudUtil::calcBoundingBox(cloud, edge, center, Rot);
}

void ObjectPoseEstimator::cloud2Vec(PointCloudTConstPtr cloud, vector<Vector3>& vec_cloud) {
	PointCloudUtil::cloudToVec(cloud, vec_cloud);
}

void ObjectPoseEstimator::normal2Vec(NormalCloudPtr normal, vector<Vector3>& vec_normal) {
	PointCloudUtil::normalCloudToVec(normal, vec_normal);
}

/**
* Move and rotate the object.
* @param[in] rot rotation matrix
* @param[in] trans translation vector
*/
void ObjectPoseEstimator::moveObject(const Matrix3& rot, const Vector3& trans) {
    object_link->p() = rot * object_link->p() + trans;
    object_link->R() = rot * object_link->R();
#ifdef DEBUG_MODE
    cout << "rpy" << grasp::rpyFromRot(object_link->R()) * (180.0/3.14) << endl;
    cout << "pos" << object_link->p() << endl;
#endif
}

void ObjectPoseEstimator::registerReuseSol(int i ) {
	PoseSolution sol;
	sol.score = std::numeric_limits<double>::max();
	PointCloudTPtr sol_cloud = PointCloudTPtr(new PointCloudT());
	result_matcher_->getReuseSol(i, sol.p, sol.R, sol_cloud);
	setOutlierIdx(sol.p, sol.R, sol.outlier_indices);
	estimation_sols.push_back(sol);

	ClusterInfo& cluster  = PointCloudClustersHolder::instance()->addCluster();
	cluster.cloud = sol_cloud;
	cluster.p = sol.p;
	cluster.R = sol.R;
	cluster.sol = NULL;
	cluster.prev_match = ClusterInfo::FULL;
}

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
CVFHPoseEstimator::CVFHPoseEstimator() :
	nn_(42),
	reject_by_bb_(false) {
}

CVFHPoseEstimator::~CVFHPoseEstimator() {
}

/**
 * @brief initial procedure
 * @param[in] bodyitem a bodyitem pointer of target object
 */
void CVFHPoseEstimator::initCvfhEstimate(cnoid::BodyItemPtr bodyitem) {
	std::string des_dir = getDescriptorDir(bodyitem);
	CVFHDescriptorCache::instance()->getCVFHDescriptors(des_dir, des_);

	PartialPointCloudCache* pc_cache = PartialPointCloudCache::instance();
	pc_cache->init(des_.size(), des_dir);

	pcl::PointCloud<DescriptorPair::CVFHDescriptor>::Ptr data_sigs = pcl::PointCloud<DescriptorPair::CVFHDescriptor>::Ptr(new pcl::PointCloud<DescriptorPair::CVFHDescriptor>());
	for (size_t i = 0; i < des_.size(); i++) {
		data_sigs->push_back(des_[i].des_pair.cvfh_descriptor);
	}

	MyPointRepresentation::ConstPtr point_rep(new MyPointRepresentation);
	match_search_.setPointRepresentation(point_rep);
	match_search_.setInputCloud(data_sigs);
}

void CVFHPoseEstimator::setViewPoint(const cnoid::Vector3& view_point) {
	view_point_ = view_point;
}

void CVFHPoseEstimator::setCameraR(const cnoid::Matrix3& cameraR) {
	cameraR_ = cameraR;
}

void CVFHPoseEstimator::setNN(int nn) {
	nn_ = nn;
}

void CVFHPoseEstimator::doRejectByBoundingBoxSize(bool use_bb) {
	reject_by_bb_ = use_bb;
}

void CVFHPoseEstimator::setMaxDistance(double max_distance) {
	max_distance_ = max_distance;
}

void CVFHPoseEstimator::setICPMaxIteration(int icp_max_iteration) {
	icp_max_iteration_ = icp_max_iteration;
}

void CVFHPoseEstimator::setObjectCloud(PointCloudTPtr obj_cloud) {
	sampled_obj_cloud_ = obj_cloud;
}

void CVFHPoseEstimator::setCandidates(std::vector<ObjectPoseEstimator::Candidate>* candidates) {
	candidates_ = candidates;
}

/**
 * @brief estimation object position and pose
 * @param[in] i target candidate index
 * @param[out] sol estimation result
 * @return true if a fesible solution is obtained
 */
bool CVFHPoseEstimator::cvfhEstimateOneCluster(int i, ObjectPoseEstimator::PoseSolution& sol)  {
	bool has_sol = false;
	const double score_th = 5.0e-05;
	const double angle_eps = 10.0;
	// maximu number of candidate descriptors
	int NN = (reject_by_bb_) ? 42 : nn_;

	CVFHDescriptorManipulator::CVFHDescriptors cluster_des;
	// compute cvfh descriptor of target candidate
	CVFHDescriptorManipulator cvfh;
	NormalEstimator normal_est;
	normal_est.setRadius(0.015);
	cvfh.setNormalEstimator(&normal_est);
	cvfh.computeDescriptor(candidates_->at(i).cloud, view_point_, max_distance_, cluster_des);

	ICPsSolver::InputDataVec icp_inputs;
	pcl::CRHAlignment<pcl::PointXYZ, 90> crha;
	PartialPointCloudCache* pc_cache = PartialPointCloudCache::instance();

	for (size_t ds = 0; ds < cluster_des.size(); ds++) {
		std::vector<int> neigh_indices(NN);
		std::vector<float> neigh_sqr_dists(NN);

		int found_neighs = match_search_.nearestKSearch(cluster_des[ds].des_pair.cvfh_descriptor, NN, neigh_indices, neigh_sqr_dists);
		Vector3 cent_cluster = cluster_des[ds].des_pair.centroid.cast<double>();

		for (int j = 0, k = 0; (j < NN && k < nn_); j++) {
			int neigh_idx = neigh_indices[j];
			Vector3 cent = des_[neigh_idx].des_pair.centroid.cast<double>();
			Matrix4f pose = des_[neigh_idx].pose;

			if (reject_by_bb_) {
				if (!isAcceptableBoundingBox(i, neigh_idx)) continue;
			}
			k++;

			std::vector<float> peaks;
			pcl::PointCloud<pcl::Histogram<90> > hist1, hist2;
			hist1.push_back(des_[neigh_idx].des_pair.crh_descriptor);
			hist2.push_back(cluster_des[ds].des_pair.crh_descriptor);
			crha.computeRollAngle(hist1, hist2, peaks);

			std::vector<float> peaksext;
			pcl::PointCloud<pcl::Histogram<90> > hist1ext, hist2ext;
			hist1ext.push_back(des_[neigh_idx].des_pair.crhext_descriptor);
			hist2ext.push_back(cluster_des[ds].des_pair.crhext_descriptor);
			crha.computeRollAngle(hist1ext, hist2ext, peaksext);

			// merge peaks and peaksext
			for (size_t l = 0; l < peaksext.size(); l++) {
				bool do_add = true;
				for (size_t m = 0; m < peaks.size(); m++) {
					if (fabs(peaks[m] - peaksext[l]) < angle_eps) {
						do_add = false;
						break;
					}
				}
				if (do_add) peaks.push_back(peaksext[l]);
			}

			Vector3 view_vec_cluster = cent_cluster - view_point_;
			cnoid::Vector3 view_vec_cluster_cam = cameraR_.transpose() * view_vec_cluster;
			Matrix3 R_target_cluster = grasp::rotFromTwoVecs(cent, view_vec_cluster_cam);
			// Vector3 axis = cent.normalized();
			Vector3 axis(0,0,1);

			PointCloudTPtr partialcloud = pc_cache->getPartialCloud(des_[neigh_idx]);

			for (size_t l = 0; l < peaks.size(); l++) {
				Matrix3 init_R = grasp::rotFromTwoVecs(cent, view_vec_cluster) * grasp::rodrigues(axis, peaks[l] * M_PI/180.0);
				// Matrix3 init_R = cameraR_ * R_target_cluster * grasp::rodrigues(axis, peaks[l] * M_PI/180.0);
				Vector3 init_p = cent_cluster - init_R * cent;
				icp_inputs.push_back(ICPsSolver::InputData());
				icp_inputs.back().init_matrix = PointCloudUtil::convM3VtoM4(init_R, init_p);
				icp_inputs.back().pose_matrix = pose;
				icp_inputs.back().target_cloud = partialcloud;
			}
		}
	}

	// solve icp for candidate poses
	Eigen::Matrix4f trans_matrix;
#ifdef THREAD
	ICPsSolver* icps = new ICPsSolverParallel;
#else
	ICPsSolver* icps = new ICPsSolverSerial;
#endif
	icps->setMaxIteration(icp_max_iteration_);
	icps->setSceneCloud(candidates_->at(i).cloud);
	icps->setObjModelCloud(sampled_obj_cloud_);
	if (icps->solveICPs(icp_inputs)) {
		icps->getBestMatrix(trans_matrix);
		sol.p = trans_matrix.col(3).head(3).cast<double>();
		sol.R = trans_matrix.topLeftCorner(3,3).cast<double>();
		sol.score = icps->getBestScore();
		if (sol.score < score_th) {
			has_sol = true;
		}
	}
	delete icps;

	return has_sol;
}

void CVFHPoseEstimator::cvfhEstimateOneClusterWoICP(int i, std::vector<ObjectPoseEstimator::PoseSolution>& sol)  {
	bool has_sol = false;
	const double score_th = 5.0e-05;
	const double angle_eps = 10.0;
	// maximu number of candidate descriptors
	int NN = (reject_by_bb_) ? 42 : nn_;

	CVFHDescriptorManipulator::CVFHDescriptors cluster_des;
	// compute cvfh descriptor of target candidate
	CVFHDescriptorManipulator cvfh;
	NormalEstimator normal_est;
	normal_est.setRadius(0.015);
	cvfh.setNormalEstimator(&normal_est);
	cvfh.computeDescriptor(candidates_->at(i).cloud, view_point_, max_distance_, cluster_des);

	ICPsSolver::InputDataVec icp_inputs;
	pcl::CRHAlignment<pcl::PointXYZ, 90> crha;
	PartialPointCloudCache* pc_cache = PartialPointCloudCache::instance();

	for (size_t ds = 0; ds < cluster_des.size(); ds++) {
		std::vector<int> neigh_indices(NN);
		std::vector<float> neigh_sqr_dists(NN);

		int found_neighs = match_search_.nearestKSearch(cluster_des[ds].des_pair.cvfh_descriptor, NN, neigh_indices, neigh_sqr_dists);
		Vector3 cent_cluster = cluster_des[ds].des_pair.centroid.cast<double>();

		for (int j = 0, k = 0; (j < NN && k < nn_); j++) {
			int neigh_idx = neigh_indices[j];
			Vector3 cent = des_[neigh_idx].des_pair.centroid.cast<double>();
			Matrix4f pose = des_[neigh_idx].pose;

			if (reject_by_bb_) {
				if (!isAcceptableBoundingBox(i, neigh_idx)) continue;
			}
			k++;

			std::vector<float> peaks;
			pcl::PointCloud<pcl::Histogram<90> > hist1, hist2;
			hist1.push_back(des_[neigh_idx].des_pair.crh_descriptor);
			hist2.push_back(cluster_des[ds].des_pair.crh_descriptor);
			crha.computeRollAngle(hist1, hist2, peaks);

			std::vector<float> peaksext;
			pcl::PointCloud<pcl::Histogram<90> > hist1ext, hist2ext;
			hist1ext.push_back(des_[neigh_idx].des_pair.crhext_descriptor);
			hist2ext.push_back(cluster_des[ds].des_pair.crhext_descriptor);
			crha.computeRollAngle(hist1ext, hist2ext, peaksext);

			// merge peaks and peaksext
			for (size_t l = 0; l < peaksext.size(); l++) {
				bool do_add = true;
				for (size_t m = 0; m < peaks.size(); m++) {
					if (fabs(peaks[m] - peaksext[l]) < angle_eps) {
						do_add = false;
						break;
					}
				}
				if (do_add) peaks.push_back(peaksext[l]);
			}

			Vector3 view_vec_cluster = cent_cluster - view_point_;
			cnoid::Vector3 view_vec_cluster_cam = cameraR_.transpose() * view_vec_cluster;
			Matrix3 R_target_cluster = grasp::rotFromTwoVecs(cent, view_vec_cluster_cam);
			// Vector3 axis = cent.normalized();
			Vector3 axis(0,0,1);

			for (size_t l = 0; l < peaks.size(); l++) {
				Matrix3 init_R = grasp::rotFromTwoVecs(cent, view_vec_cluster) * grasp::rodrigues(axis, peaks[l] * M_PI/180.0);
				// Matrix3 init_R = cameraR_ * R_target_cluster * grasp::rodrigues(axis, peaks[l] * M_PI/180.0);
				Vector3 init_p = cent_cluster - init_R * cent;
				sol.push_back(ObjectPoseEstimator::PoseSolution());
				sol.back().R = init_R * pose.topLeftCorner(3,3).cast<double>();
				sol.back().p = init_R * pose.col(3).head(3).cast<double>() + init_p;
				// std::cout << "peaks:" << peaks[l] << std::endl;
				// std::cout << "init_p:" << sol.back.transpose() << std::endl;
				// std::cout << "init_R:" << (grasp::rpyFromRot((PointCloudUtil::convM3VtoM4(init_R, init_p) * pose).topLeftCorner(3,3).cast<double>())/3.14*180).transpose() << std::endl;
				// std::cout << "pose_R:" << (grasp::rpyFromRot((pose).topLeftCorner(3,3).cast<double>())/3.14*180).transpose() << std::endl << std::endl;
			}
		}
	}

}

/**
 * @brief check if target descriptor is acceptable judging by bounding box size.
 * @param[in] cand_id candidate index
 * @param[in] neigh_idx target descriptor index
 */
bool CVFHPoseEstimator::isAcceptableBoundingBox(int cand_id, int neigh_idx) const {
	double likelihood;
	double cluster_bb_area = 1.0;
	int minid, maxid;
	candidates_->at(cand_id).edge.minCoeff(&minid);
	candidates_->at(cand_id).edge.maxCoeff(&maxid);
	for (int l = 0; l < 3; l++) {
		if (l != minid) cluster_bb_area *= candidates_->at(cand_id).edge[l];
	}
	double target_bb_area = des_[neigh_idx].bounding_box[0] * des_[neigh_idx].bounding_box[1];
	likelihood = (cluster_bb_area > target_bb_area) ? (target_bb_area / cluster_bb_area) : (cluster_bb_area / target_bb_area);
	double cluster_depth = candidates_->at(cand_id).edge[minid];
	double model_depth = des_[neigh_idx].bounding_box[2];
	if (cluster_depth < 0.01) cluster_depth = 0.01;
	if (model_depth < 0.01) model_depth = 0.01;
	double likelihood_depth = (cluster_depth < model_depth) ? (cluster_depth/model_depth) :(model_depth/cluster_depth);
	double max_bb_len, mid_bb_len, max_cluster_len, mid_cluster_len;
	if (des_[neigh_idx].bounding_box[0] > des_[neigh_idx].bounding_box[1]) {
		max_bb_len = des_[neigh_idx].bounding_box[0];
		mid_bb_len = des_[neigh_idx].bounding_box[1];
	} else {
		max_bb_len = des_[neigh_idx].bounding_box[1];
		mid_bb_len = des_[neigh_idx].bounding_box[0];
	}
	max_cluster_len = candidates_->at(cand_id).edge[maxid];
	if ((minid == 1 && maxid == 2) || (maxid == 1 && minid == 2)) {
		mid_cluster_len = candidates_->at(cand_id).edge[0];
	} else if ((minid == 0 && maxid == 2) || (maxid == 0 && minid == 2)) {
		mid_cluster_len = candidates_->at(cand_id).edge[1];
	} else {
		mid_cluster_len = candidates_->at(cand_id).edge[2];
	}

	double likelihood_first = (max_bb_len < max_cluster_len) ? (max_bb_len/max_cluster_len) : (max_cluster_len/max_bb_len);
	double likelihood_second = (mid_bb_len < mid_cluster_len) ? (mid_bb_len/mid_cluster_len) : (mid_cluster_len/mid_bb_len);

	if (likelihood_first < likelihood_th_) return false;
	if (likelihood_second < likelihood_th_) return false;
	if (likelihood_depth < likelihood_th_depth_) return false;

	return true;
}

/**
* @brief Load descriptor directory from YAML file.
*
* likelihood_th_ and likelihood_th_depth_ are set in this method.
* @param[in] bodyitem BodyItem pointer to the target model
* @return descriptor directory path
*/
std::string CVFHPoseEstimator::getDescriptorDir(cnoid::BodyItemPtr bodyitem) {
	const Mapping& info = *(bodyitem->body()->info()->toMapping());
	ValueNode* dir_path = info.find("descriptorDir");
  ValueNode* like_th = info.find("likelihoodTh");
  if (like_th->isValid()) {
    likelihood_th_ = like_th->toDouble();
  } else {
    likelihood_th_ = 0.75;
  }
  ValueNode* like_th_depth = info.find("likelihoodThDepth");
  if (like_th_depth->isValid()) {
    likelihood_th_depth_ = like_th_depth->toDouble();
  } else {
    likelihood_th_depth_ = 0.2;
  }
	if (dir_path->isString()){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		fs::path yaml_path(bodyitem->lastAccessedFilePath());
#else
		fs::path yaml_path(bodyitem->filePath());
#endif
		fs::path des_dir_path = yaml_path.parent_path() / dir_path->toString();
		return des_dir_path.string();
	}
	return "";
}

#endif

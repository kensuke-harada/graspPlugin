/**
 * @file   ObjectPoseEstimatorInterface.cpp
 * @author Akira Ohchi
 */

#include "ObjectPoseEstimatorInterface.h"

#include <QDir>
#include <QFileInfo>
#include <QSettings>

#include <boost/scoped_ptr.hpp>

#include <pcl/pcl_config.h>
#include <pcl/filters/voxel_grid.h>

#include <cnoid/ExecutablePath>

#include "ObjectPoseEstimator.h"
#include "Registrator.h"
#include "Segmenter.h"
#include "PointCloudDrawer.h"
#include "BoxRegistration.h"
#include "PointCloudMerger.h"
#include "PointCloudHolder.h"
#include "MergerResultDrawer.h"
#include "PointCloudUtility.h"
#ifdef CNOID_GE_15
#include "PointSetItemDrawer.h"
#endif
#include "ObjectPoseEstimatorImpl.h"
#include "../Grasp/PlanBase.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"

//#define OBJPOSEESTIMATE_SAVE_CAPTUREDPOINTCLOUD
#ifdef OBJPOSEESTIMATE_SAVE_CAPTUREDPOINTCLOUD
#include <string>
#include <boost/lexical_cast.hpp>
#endif

// for measuring the time
// #define NOOUTPUT_TIMELOG
#include "../GraspDataGen/Util/StopWatch.h"

using namespace std;

#ifdef OBJPOSEESTIMATE_SAVE_CAPTUREDPOINTCLOUD
namespace {
	std::string getSavePCDFilePath() {
		static int file_no = 0;
		std::string str_no = boost::lexical_cast<std::string>(file_no);
		file_no++;
		return "./capturedpcd_" + str_no + ".pcd";
	}
}
#endif


PoseEstimator::PoseEstimator() {
	impl = new PoseEstimatorImpl();
}

PoseEstimator::PoseEstimator(PoseEstimatorImpl* impl_) {
	impl = impl_;
}

PoseEstimator::~PoseEstimator() {
	if (impl != NULL) {
		delete impl;
	}
}

void PoseEstimator::setCameraMat(const cnoid::Vector3& view_point, const cnoid::Matrix3& ori) {
	impl->camera_p_ = view_point;
	impl->camera_R_ = ori;
}

void PoseEstimator::init(const ObjectPoseEstimateParams& param) {
	impl->init(param);
}

void PoseEstimator::readObject() {
	impl->readObject();
}

void PoseEstimator::capture() {
	impl->capture();
}

#ifdef CNOID_GE_15
void PoseEstimator::getCurrPointCloudItem(cnoid::PointSetItemPtr& point_item) {
	PointCloudTConstPtr cloud = PointCloudHolder::instance()->getCloudPtr();
	PointSetItemDrawer::cloud2PointItem(cloud, point_item, cnoid::Vector3(0.1, 0.1, 0.1));
}

void PoseEstimator::getSampledCurrPointCloudItem(cnoid::PointSetItemPtr& point_item) {
	PointCloudTConstPtr cloud = MergerResultDrawer::instance()->getCurrSampledCloud();
	PointSetItemDrawer::cloud2PointItem(cloud, point_item, cnoid::Vector3(0.1, 0.1, 0.1));
}

void PoseEstimator::getClusteredCloudItem(cnoid::PointSetItemPtr& point_item) {
	ColorPointCloudConstPtr cloud = impl->ope->getClusteredCloud();
	PointSetItemDrawer::cloud2PointItem(cloud, point_item);
}

void PoseEstimator::getClusterPointCloudItems(std::vector<cnoid::PointSetItemPtr>& point_items, std::vector<int>& flags) {
	point_items.clear();
	flags.clear();
	PointCloudClustersHolder* pcch = PointCloudClustersHolder::instance();
	for (int i = 0; i < pcch->size(); i++) {
		point_items.push_back(cnoid::PointSetItemPtr(new cnoid::PointSetItem()));
		PointCloudTConstPtr cloud = pcch->getCloudPtr(i);
		PointSetItemDrawer::cloud2PointItem(cloud, point_items.back());
		ClusterInfo::PrevResultMatch f = pcch->getCluster(i).prev_match;
		if (f == ClusterInfo::FULL) {
			flags.push_back(2);
		} else if (f == ClusterInfo::PARTIAL) {
			flags.push_back(1);
		} else {
			flags.push_back(0);
		}
	}
}

#endif

bool PoseEstimator::boxRegistration(const cnoid::BodyPtr& box_body) {
	grasp::ScopeStopWatch timer("recog(box_reg)");
	bool ret = impl->boxRegistration(box_body);
	return ret;
}

bool PoseEstimator::segmentation() {
	grasp::ScopeStopWatch timer("recog(segmentation)");
	bool ret = impl->segmentation();
	return ret;
}

void PoseEstimator::estimationInit(int& size) {
	impl->estimationInit(size);
}

bool PoseEstimator::estimation(int i, cnoid::Vector3& p, cnoid::Matrix3& R, std::vector<int>& outlier_idx) {
	return impl->estimation(i, p, R, outlier_idx);
}

bool PoseEstimator::estimation() {
	return impl->estimation();
}

void PoseEstimator::registerResultsToDrawer() {
	impl->registerResultsToDrawer();
}

void PoseEstimator::displayEstimationResults() {
    impl->displayEstimationResults();
}

void PoseEstimator::getEnvCloud(std::vector<cnoid::Vector3>& env_cloud) {
	impl->getEnvCloud(env_cloud);
}

void PoseEstimator::showEnvCloud(const std::vector<cnoid::Vector3>& env_cloud) {
	PointCloudDrawer* draw = PointCloudDrawer::instance();
	draw->clear();
	PointCloudTPtr cloud(new PointCloudT());
	PointCloudUtil::vecToCloud(env_cloud, cloud);
	draw->addPointCloud(cloud);
	draw->draw();
}

void PoseEstimator::getPrevSampledCloud(std::vector<cnoid::Vector3>& prev_cloud) {
	PointCloudUtil::cloudToVec(MergerResultDrawer::instance()->getPrevSampledCloud(), prev_cloud);
}

void PoseEstimator::getCurrSampledCloud(std::vector<cnoid::Vector3>& curr_cloud) {
	PointCloudUtil::cloudToVec(MergerResultDrawer::instance()->getCurrSampledCloud(), curr_cloud);
}

void PoseEstimator::getPrevClusterCloud(std::vector<cnoid::Vector3>& prev_cluster_cloud) {
	PointCloudTPtr tmp_cloud(new PointCloudT());
	std::vector<PointCloudTPtr> clouds;
	MergerResultDrawer::instance()->getClusterClouds(clouds);
	for (size_t i = 0; i < clouds.size(); i++) {
		*tmp_cloud += *(clouds[i]);
	}
	PointCloudUtil::cloudToVec(tmp_cloud, prev_cluster_cloud);
}

void PoseEstimator::drawPointClouds(const std::vector<std::vector<cnoid::Vector3> >& points, const std::vector<cnoid::Vector3>& colors) {
	PointCloudDrawer* draw = PointCloudDrawer::instance();
	draw->clear();
	for (size_t i = 0; i < points.size(); i++) {
		PointCloudTPtr tmp_cloud(new PointCloudT());
		PointCloudUtil::vecToCloud(points[i], tmp_cloud);
		draw->addPointCloud(tmp_cloud, colors[i]);
	}
	draw->draw();
}

bool PoseEstimator::poseEstimate(const ObjectPoseEstimateParams& param, bool do_boxadjustment) {
	boost::scoped_ptr<ObjectPoseEstimator> ope(new ObjectPoseEstimator());
	// boost::scoped_ptr<Segmenter> seg(new Segmenter());
	boost::scoped_ptr<ConditionalSegmenter> seg(new ConditionalSegmenter());
	boost::scoped_ptr<LccpSegmenter> lccp_seg(new LccpSegmenter());
	boost::scoped_ptr<SAC_IAEstimator> init_est(new SAC_IAEstimator());
	// RobustPoseEstimator* init_est = new RobustPoseEstimator();
	// RobustRejectivePoseEstimator* init_est = new RobustRejectivePoseEstimator();
	boost::scoped_ptr<ICPEstimator> est(new ICPEstimator());

	cnoid::Vector3 view_point;
	ope->getViewPoint(view_point);

	seg->setDistThresholdPlaneRemovement(param.plane_remove_dist);
	seg->setRemovePlaneNum(param.num_plane_remove);
	seg->setTolerance(param.segment_tolerance);
	seg->setVerticalTolerance(param.segment_vtolerance);
	seg->setViewPoint(view_point);

	lccp_seg->setRemovePlaneNum(param.num_plane_remove);
	lccp_seg->setViewPoint(view_point);
	lccp_seg->setSuperVoxelParameter(param.sampling_density, param.segment_sv_seed_resolution,
																	 param.segment_sv_spatial_coeff, param.segment_sv_normal_coeff, param.segment_sv_color_coeff);
	lccp_seg->setConcavityTolerance(param.segment_lccp_concavitytolerance);
	lccp_seg->setMinClusterSize(param.segment_lccp_minsize);

	est->setIteration(param.iteration);
	ope->setICPMaxIteration(param.iteration);
	ope->readScene(param.is_load_file, param.is_save_file, param.load_file_path, param.save_file_path,
								 param.is_mergecloud);
	if (param.is_load_obj) {
		ope->readObjectVRML(param.obj_file_path);
	} else {
		ope->readObject();
	}

	if (do_boxadjustment) {
		BoxCloudHolder* bch = BoxCloudHolder::instance();
		cnoid::Vector4f min_p, max_p;
		grasp::PlanBase* pb = grasp::PlanBase::instance();
		BoxRegistration reg;
		PointCloudTPtr box_cloud(new PointCloudT());
		PointCloudTPtr target_cloud(new PointCloudT());
		pcl::copyPointCloud(*(ope->getSceneCloud()), *target_cloud);
		if (bch->hasBox()) {
			*target_cloud += *(bch->getBoxPtr());
		}
		reg.registration(box_body_, target_cloud, view_point, box_cloud);
		pcl::getMinMax3D(*box_cloud, min_p, max_p);
		BoxCloudHolder::instance()->setBoxModelCloud(box_cloud);
		ope->setTargetRegion(max_p.x(), min_p.x(), max_p.y(), min_p.y(), param.zmax, param.zmin);
	} else {
		ope->setTargetRegion(param.xmax, param.xmin, param.ymax, param.ymin, param.zmax, param.zmin);
	}
	ope->setCandidateNum(param.num_candidate);
	ope->setVoxelLeaf(param.sampling_density);
	ope->useNormalKSearch(param.use_ksearch);
	ope->useFeatureKSearch(param.use_ksearch);
	if (param.use_ksearch) {
		ope->setNormalK(param.normal_k);
		ope->setFeatureK(param.feature_k);
	} else {
		ope->setNormalRadius(param.normal_radius);
		ope->setFeatureRadius(param.feature_radius);
	}
	if (param.use_lccp_segmentation) {
		ope->setSegmenter(lccp_seg.get());
	} else {
		ope->setSegmenter(seg.get());
	}
	ope->setInitialEstimator(init_est.get());
	ope->setEstimator(est.get());
	ope->setSegmentResegmentRadius(param.segment_resegment_radius);
	ope->setSegmentBoundaryRadius(param.segment_boundary_radius);
	ope->downsampling(param.is_do_narrow);

	PointCloudMerger merger;
	merger.merge(ope->getSampledSceneCloud(), view_point, param.sampling_density);
	ope->setEnvCloud(ope->getSampledSceneCloud());
	ope->prev_clusters.clear();
	std::vector<int> ids;
	merger.getUnchangedClusterIDs(ids);
	for (size_t i = 0; i < ids.size(); i++) {
		ope->prev_clusters.push_back(PointCloudClustersHolder::instance()->getCluster(ids[i]));
	}

	if (!(ope->segment())) {
		error_code_ = SEGMENTATION_ERROR;
		return false;
	}

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	if (param.do_recog_cvfh) {
		if (!ope->cvfhEstimate(param.is_show_moved, view_point, param.obj_file_path, param.num_neighbor, param.use_bbsimilarity, false, true)) {
			error_code_ = ESTIMATION_ERROR;
			return false;
		}
	} else {
#endif
		if (!(ope->estimate(param.is_show_moved))) {
			error_code_ = ESTIMATION_ERROR;
			return false;
		}
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	}
#endif
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	pb->RemoveAllPointCloudEnvironment();
	pb->pointCloudEnv.clear();
	vector<Vector3> cloud_vec;
	ObjectPoseEstimator::cloud2Vec(ope->env_cloud, cloud_vec);
	vector<Vector3> normal_vec;
	// ObjectPoseEstimator::normal2Vec(ope->env_normal, normal_vec);
	pb->SetPointCloudEnvironment(cloud_vec, normal_vec);

	grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
	for (size_t i = 0; i < opes->size(); i++) {
		opes->at(i).env_point_cloud = pb->pointCloudEnv.back();
	}

	MergerResultDrawer::instance()->updatePointCloud(ope->getSceneCloud(), ope->getClusteredCloud(), ope->getSampledSceneCloud(), merger.getModels(), merger.getColors(), merger.getModelPoints());

	PointCloudDrawer* draw = PointCloudDrawer::instance();
	if (param.is_show_segment) {
		draw->addPointCloud(ope->getClusteredCloud());
		// draw->addPointCloud(seg->getSegmentedCloud(), Vector3(1.0, 0, 0));
		draw->addPointCloud(ope->getSceneCloud());
	} else if (param.is_show_moved) {
		draw->addPointCloud(ope->env_cloud);
		// ope->saveEnv();
		grasp::PlanBase::instance()->flush();
	} else {
		if (param.is_color) {
			draw->addPointCloud(ope->getSceneColorCloud());
		} else {
			draw->addPointCloud(ope->getSceneCloud());
		}
	}
	draw->draw();

	error_code_ = SUCCESS;
	return true;
}

bool PoseEstimator::poseEstimate(const ObjectPoseEstimateParams& param, cnoid::BodyPtr box_body) {
	box_body_ = box_body;
	return poseEstimate(param, true);
}

void PoseEstimator::readParams(ObjectPoseEstimateParams* param) {
	if (param == NULL) return;
	QString default_path = QString::fromStdString(cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PCL/");

	QSettings setting(default_path + "config.ini", QSettings::IniFormat);
	setting.beginGroup("ObjPoseEstimate");

	QString show = setting.value("show_scene", "RESULT").toString();

	param->is_load_file     = setting.value("pcd_loadfile", false).toBool();
	param->is_save_file     = setting.value("pcd_savefile", false).toBool();
	param->is_load_obj      = setting.value("pcd_loadobj", false).toBool();
	param->is_color         = setting.value("show_color", true).toBool();
	param->is_show_segment  = (show.toStdString() == "SEGMENT");
	param->is_show_moved    = (show.toStdString() == "RESULT");
	param->is_show_scene    = (show.toStdString() == "SCENE");
	param->sampling_density = setting.value("sampling_density", 0.005).toDouble();
	QDir::setCurrent(default_path);
	param->load_file_path   = QFileInfo(setting.value("pcd_loadfilepath", default_path + "PCL/test_pcd.pcd").toString()).absoluteFilePath().toStdString();
	param->save_file_path   = QFileInfo(setting.value("pcd_savefilepath", default_path + "PCL/test_pcd.pcd").toString()).absoluteFilePath().toStdString();
	param->obj_file_path    = QFileInfo(setting.value("pcd_objfilepath", default_path + "PCL/test.yaml").toString()).absoluteFilePath().toStdString();
	param->normal_radius    = setting.value("SACIA_radius_normal", 0.01).toDouble();
	param->feature_radius   = setting.value("SACIA_radius_feature", 0.02).toDouble();
	param->normal_k         = setting.value("SACIA_knearest_normal", 10).toInt();
	param->feature_k        = setting.value("SACIA_knearest_feature", 10).toInt();
	param->use_ksearch      = setting.value("SACIA_use_knearest", false).toBool();
	param->iteration        = setting.value("ICP_iteration", 100).toInt();
	param->plane_remove_dist = setting.value("segmentation_distth", 0.005).toDouble();
	param->num_plane_remove = setting.value("segmentation_numplane", 1).toInt();
	param->num_candidate    = setting.value("segmentation_searchcluster", 1).toInt();
	param->segment_tolerance = setting.value("segmentation_tolerance", 0.004).toDouble();
	param->segment_vtolerance = setting.value("segmentation_vtolerance", 0.004).toDouble();
	param->segment_resegment_radius = setting.value("segmentation_resegment_radius", 0.01).toDouble();
	param->segment_boundary_radius = setting.value("segmentation_boundary_radius", 0.01).toDouble();
	param->is_do_narrow     = setting.value("region_enable", true).toBool();
	param->xmax             = setting.value("region_xmax", 1.0).toDouble();
	param->xmin             = setting.value("region_xmin", -1.0).toDouble();
	param->ymax             = setting.value("region_ymax", 1.0).toDouble();
	param->ymin             = setting.value("region_ymin", -1.0).toDouble();
	param->zmax             = setting.value("region_zmax", 1.0).toDouble();
	param->zmin             = setting.value("region_zmin", -1.0).toDouble();
	param->use_bbsimilarity = setting.value("use_boundingbox_similarity", false).toBool();
	param->is_handcamera    = setting.value("hand_camera", false).toBool();
	param->handcamera_arm_id = setting.value("hand_camera_armid", 0).toInt();
	param->do_recog_cvfh    = setting.value("method_useCVFH", false).toBool();
	QString viewpoint_str  = setting.value("viewpoint", "0.0 0.0 0.0").toString();
	QStringList viewpoint_strlist = viewpoint_str.split(" ");
	for (int i = 0; i < 3; i++) {
		param->view_point[i] = viewpoint_strlist.at(i).toDouble();
	}
	param->num_neighbor     = setting.value("CVFH_NN", 42).toInt();
	QDir::setCurrent(QString::fromStdString(cnoid::executableTopDirectory()));
	param->segment_sv_seed_resolution = setting.value("segmentation_lccp_seedsize", 0.01).toDouble();
	param->segment_sv_spatial_coeff = setting.value("segmentation_lccp_spatialcoeff", 0.4).toDouble();
	param->segment_sv_normal_coeff = setting.value("segmentation_lccp_normalcoeff", 1.0).toDouble();
	param->segment_sv_color_coeff = setting.value("segmentation_lccp_colorcoeff", 0.0).toDouble();
	param->segment_lccp_minsize = setting.value("segmentation_lccp_minclustersize", 20).toInt();
	param->segment_lccp_concavitytolerance = setting.value("segmentation_lccp_concavitytolerance", 10).toDouble();
	param->use_lccp_segmentation = setting.value("use_lccp_segmentation", false).toBool();
	setting.endGroup();
#if PCL_VERSION_COMPARE(<, 1, 7, 0)
	param->do_recog_cvfh = false;
#endif
}

// PoseEstimator::ErrorValue PoseEstimator::getErrorCode() const {
// 	return error_code_;
// }


void PoseEstimator::clearPointClouds() {
	PointCloudHolder::instance()->clear();
	BoxCloudHolder::instance()->clearBox();
	PointCloudClustersHolder::instance()->clear();
}

void PoseEstimator::drawClusters() {
	MergerResultDrawer::instance()->drawClusters();
}

void PoseEstimator::drawPrevAndCurrPointCloud() {
	MergerResultDrawer::instance()->drawPrevAndCurrPointCloud();
}

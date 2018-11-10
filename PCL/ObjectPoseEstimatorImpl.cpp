#include "ObjectPoseEstimatorImpl.h"

#include "../Grasp/PlanBase.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"

#include "ObjectPoseEstimator.h"
#include "BoxRegistration.h"
#include "Registrator.h"
#include "Segmenter.h"
#include "PointCloudHolder.h"
#include "PointCloudDrawer.h"
#include "MergerResultDrawer.h"

PoseEstimatorImpl::PoseEstimatorImpl() :
	ope(new ObjectPoseEstimator),
	seg(new ConditionalSegmenter),
	lccp_seg(new LccpSegmenter),
	init_est(new SAC_IAEstimator),
	est(new ICPEstimator) {
}

PoseEstimatorImpl::~PoseEstimatorImpl() {
}

void PoseEstimatorImpl::init(const ObjectPoseEstimateParams& param) {
	seg->setDistThresholdPlaneRemovement(param.plane_remove_dist);
	seg->setRemovePlaneNum(param.num_plane_remove);
	seg->setTolerance(param.segment_tolerance);
	seg->setVerticalTolerance(param.segment_vtolerance);
	seg->setViewPoint(camera_p_);

	lccp_seg->setRemovePlaneNum(param.num_plane_remove);
	lccp_seg->setViewPoint(camera_p_);
	lccp_seg->setSuperVoxelParameter(param.sampling_density, param.segment_sv_seed_resolution,
																	 param.segment_sv_spatial_coeff, param.segment_sv_normal_coeff, param.segment_sv_color_coeff);
	lccp_seg->setConcavityTolerance(param.segment_lccp_concavitytolerance);
	lccp_seg->setMinClusterSize(param.segment_lccp_minsize);

	est->setIteration(param.iteration);
	ope->setICPMaxIteration(param.iteration);
	ope->setTargetRegion(param.xmax, param.xmin, param.ymax, param.ymin, param.zmax, param.zmin);

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
	ope->setSegmentResegmentRadius(param.segment_resegment_radius);
	ope->setSegmentBoundaryRadius(param.segment_boundary_radius);
	if (param.use_lccp_segmentation) {
		ope->setSegmenter(lccp_seg.get());
	} else {
		ope->setSegmenter(seg.get());
	}
	ope->setInitialEstimator(init_est.get());
	ope->setEstimator(est.get());

	param_ = param;
}

void PoseEstimatorImpl::readObject() {
	if (param_.is_load_obj) {
		ope->readObjectVRML(param_.obj_file_path);
	} else {
		param_.obj_file_path = grasp::PlanBase::instance()->targetObject->bodyItemObject->filePath();
		ope->readObject();
	}
}

void PoseEstimatorImpl::capture() {
#ifdef OBJPOSEESTIMATE_SAVE_CAPTUREDPOINTCLOUD
	std::string save_pcd_file = getSavePCDFilePath();
	ope->readScene(param_.is_load_file, true, param_.load_file_path, save_pcd_file,
								 camera_p_, camera_R_, param_.is_mergecloud);
#else
	// ope->readScene(param_.is_load_file, param_.is_save_file, param_.load_file_path, param_.save_file_path,
	// 							 camera_p_, camera_R_, param_.is_mergecloud);
		ope->readScene(param_.is_load_file, param_.is_save_file, param_.load_file_path, param_.save_file_path,
									 cnoid::Vector3::Zero(), cnoid::Matrix3::Identity(), param_.is_mergecloud);
#endif
}

bool PoseEstimatorImpl::boxRegistration(cnoid::BodyPtr box_body) {
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
	bool ret = reg.registration(box_body, target_cloud, camera_p_, box_cloud);
	pcl::getMinMax3D(*box_cloud, min_p, max_p);
	bch->setBoxModelCloud(box_cloud);
	ope->setTargetRegion(max_p.x(), min_p.x(), max_p.y(), min_p.y(), param_.zmax, param_.zmin);
	return ret;
}

bool PoseEstimatorImpl::segmentation() {
	ope->downsampling(param_.is_do_narrow);

	if (param_.use_prev_result) {
		PointCloudClustersHolder* pcch = PointCloudClustersHolder::instance();
		for (size_t i = 0; i < pcch->size(); i++) {
			if (pcch->getCluster(i).sol != NULL) {
				pcch->getCluster(i).sol->is_target = (i == param_.prev_grasped_cluster_id);
			}
		}

		grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
		for (int i = 0; i < opes->size(); i++) {
			pcch->getCluster(i).is_feasible = opes->at(i).is_feasible;
		}

		merger_.merge(ope->getSampledSceneCloud(), camera_p_, param_.sampling_density);
		ope->prev_clusters.clear();
		std::vector<int> ids;
		merger_.getUnchangedClusterIDs(ids);
		for (size_t i = 0; i < ids.size(); i++) {
			ope->prev_clusters.push_back(PointCloudClustersHolder::instance()->getCluster(ids[i]));
		}
		ope->registerPrevCloudToResultMatcher();
	}
	ope->setUsePrevResult(param_.use_prev_result);

	if (!(ope->segment())) {
		return false;
	}
	return true;
}

void PoseEstimatorImpl::estimationInit(int& size) {
	ope->initCvfhEstimate(size);
}

bool PoseEstimatorImpl::estimation(int i, cnoid::Vector3& p, cnoid::Matrix3& R, std::vector<int>& outlier_idx) {
	return ope->cvfhEstimateOneCluster(i, p, R, outlier_idx, camera_p_, camera_R_, param_.num_neighbor);
}

bool PoseEstimatorImpl::estimation() {
	if (!ope->cvfhEstimate(param_.is_show_moved, camera_p_, param_.obj_file_path, param_.num_neighbor, param_.use_bbsimilarity, false, true)) {
		return false;
	}
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	pb->RemoveAllPointCloudEnvironment();
	pb->pointCloudEnv.clear();
	std::vector<Vector3> cloud_vec;
	ObjectPoseEstimator::cloud2Vec(ope->env_cloud, cloud_vec);
	std::vector<Vector3> normal_vec;
	pb->SetPointCloudEnvironment(cloud_vec, normal_vec);

	grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
	for (size_t i = 0; i <  opes->size(); i++) {
		opes->at(i).env_point_cloud = pb->pointCloudEnv.back();
	}

	MergerResultDrawer::instance()->updatePointCloud(ope->getSceneCloud(), ope->getClusteredCloud(), ope->getSampledSceneCloud(), merger_.getModels(), merger_.getColors(), merger_.getModelPoints());

	PointCloudDrawer* draw = PointCloudDrawer::instance();
	if (param_.is_show_segment) {
		draw->addPointCloud(ope->getClusteredCloud());
		draw->addPointCloud(ope->getSceneCloud());
	} else if (param_.is_show_moved) {
		draw->addPointCloud(ope->env_cloud);
		grasp::PlanBase::instance()->flush();
	} else {
		if (param_.is_color) {
			draw->addPointCloud(ope->getSceneColorCloud());
		} else {
			draw->addPointCloud(ope->getSceneCloud());
		}
	}
	draw->draw();

	return true;
}

void PoseEstimatorImpl::getEnvCloud(std::vector<cnoid::Vector3>& env_cloud) {
	ObjectPoseEstimator::cloud2Vec(ope->env_cloud, env_cloud);
}

void PoseEstimatorImpl::registerResultsToDrawer() {
	MergerResultDrawer::instance()->updatePointCloud(ope->getSceneCloud(), ope->getClusteredCloud(), ope->getSampledSceneCloud(), merger_.getModels(), merger_.getColors(), merger_.getModelPoints());
}

void PoseEstimatorImpl::displayEstimationResults() {
	ope->displayEstimationResults();
}


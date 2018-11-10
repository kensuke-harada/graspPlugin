#include "./ObjectPoseEstimatorForDemo.h"

#include "Registrator.h"

ObjectPoseEstimatorForDemo::ObjectPoseEstimatorForDemo() {
}

ObjectPoseEstimatorForDemo::~ObjectPoseEstimatorForDemo() {
}

bool ObjectPoseEstimatorForDemo::estimationForDemo(const std::vector<cnoid::Vector3>& points, cnoid::Vector3& p, cnoid::Matrix3& R,
																						const cnoid::Vector3& view_point, const cnoid::Matrix3& camera_R, int nn) {
	PointCloudTPtr scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::vecToCloud(points, scene);
	PointCloudTPtr sampled_scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::voxelFilter(scene, sampled_scene, voxel_leaf);

	Candidate candidate;
	candidate.id = 0;
	candidate.cloud = sampled_scene;

	candidates_.push_back(candidate);

	// obtain a bounding box of the target object
	Vector3 obj_center;
	Vector3 obj_edge;
	Matrix3 obj_rot;
	calcBoundingBox(sampled_obj_cloud, obj_edge, obj_center, obj_rot);
	max_distance_ = obj_edge.norm();

	cvfh_est_->setCameraR(camera_R);
	cvfh_est_->setViewPoint(view_point);
	cvfh_est_->setNN(nn);
	cvfh_est_->setMaxDistance(max_distance_);
	cvfh_est_->setICPMaxIteration(icp_max_iteration);
	cvfh_est_->setObjectCloud(sampled_obj_cloud);
	cvfh_est_->setCandidates(&candidates_);

	cvfh_est_->initCvfhEstimate(grasp::PlanBase::instance()->targetObject->bodyItemObject);
	PoseSolution sol;
	bool ret = cvfh_est_->cvfhEstimateOneCluster(0, sol);
	if (ret) {
		p = sol.p;
		R = sol.R;
	}
	return ret;
}

void ObjectPoseEstimatorForDemo::estimationForDemoWoICP(const std::vector<cnoid::Vector3>& points,
																								 std::vector<cnoid::Vector3>& p, std::vector<cnoid::Matrix3>& R,
																								 const cnoid::Vector3& view_point, const cnoid::Matrix3& camera_R, int nn) {
	PointCloudTPtr scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::vecToCloud(points, scene);
	PointCloudTPtr sampled_scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::voxelFilter(scene, sampled_scene, voxel_leaf);

	Candidate candidate;
	candidate.id = 0;
	candidate.cloud = sampled_scene;

	candidates_.clear();
	candidates_.push_back(candidate);

	// obtain a bounding box of the target object
	Vector3 obj_center;
	Vector3 obj_edge;
	Matrix3 obj_rot;
	calcBoundingBox(sampled_obj_cloud, obj_edge, obj_center, obj_rot);
	max_distance_ = obj_edge.norm();

	cvfh_est_->setCameraR(camera_R);
	cvfh_est_->setViewPoint(view_point);
	cvfh_est_->setNN(nn);
	cvfh_est_->setMaxDistance(max_distance_);
	cvfh_est_->setICPMaxIteration(icp_max_iteration);
	cvfh_est_->setObjectCloud(sampled_obj_cloud);
	cvfh_est_->setCandidates(&candidates_);

	cvfh_est_->initCvfhEstimate(grasp::PlanBase::instance()->targetObject->bodyItemObject);
	std::vector<PoseSolution> sols;
	cvfh_est_->cvfhEstimateOneClusterWoICP(0, sols);

	for (size_t i = 0; i < sols.size(); i++) {
		p.push_back(sols[i].p);
		R.push_back(sols[i].R);
	}
}

bool ObjectPoseEstimatorForDemo::ICPforDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& init_ps, const std::vector<cnoid::Matrix3>& init_Rs,
																		 cnoid::Vector3& p, cnoid::Matrix3& R, int num_ite) {
	PointCloudTPtr scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::vecToCloud(points, scene);
	PointCloudTPtr sampled_scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::voxelFilter(scene, sampled_scene, voxel_leaf);

	// KdTreePointTPtr scene_tree = KdTreePointTPtr(new KdTreePointT(false));
	KdTreePointTPtr obj_tree = KdTreePointTPtr(new KdTreePointT(false));
	// scene_tree->setInputCloud(sampled_scene);
	obj_tree->setInputCloud(sampled_obj_cloud);

	PointCloudTPtr aligned_cloud = PointCloudTPtr(new PointCloudT());

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputTarget(sampled_obj_cloud);
	icp.setSearchMethodTarget(obj_tree, true);
	icp.setMaxCorrespondenceDistance(0.01);
	icp.setMaximumIterations(num_ite);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.01);

	bool ret = false;
	double best_score = std::numeric_limits<double>::max();
	int best_inlier = 0;
	for (size_t i = 0; i < init_ps.size(); i++) {
		aligned_cloud->clear();

		Eigen::Matrix4f init_matrix;
		init_matrix.topLeftCorner(3, 3) = init_Rs[i].transpose().cast<float>();
		init_matrix.col(3).head(3) = -(init_Rs[i].transpose() * init_ps[i]).cast<float>();
		icp.setInputSource(sampled_scene);
		icp.align(*aligned_cloud, init_matrix);

		if (icp.hasConverged()) {
			double score = icp.getFitnessScore();

			std::vector<int> nn_indices(1);
			std::vector<float> nn_dists(1);
			int nr_in = 0;
			double max_range_square = voxel_leaf * voxel_leaf;
			size_t point_size = aligned_cloud->points.size();
			for (int j = 0; j < point_size; j++) {
				obj_tree->nearestKSearch(aligned_cloud->points[j], 1, nn_indices, nn_dists);
				if (nn_dists[0] < max_range_square) {
					nr_in++;
				}
			}

			if ((nr_in > best_inlier) || (nr_in == best_inlier && best_score > score)) {
				ret = true;
				best_score = score;
				best_inlier = nr_in;
				Eigen::Matrix4f trans_mat = icp.getFinalTransformation();
				R = trans_mat.topLeftCorner(3, 3).transpose().cast<double>();
				p = -R * trans_mat.col(3).head(3).cast<double>();
			}
		}
	}

	return ret;
}

bool ObjectPoseEstimatorForDemo::twoStageRegistrationForDemo(const std::vector<cnoid::Vector3>& points, const std::vector<cnoid::Vector3>& sface_points, const std::string& box_path, const cnoid::Vector3& center, cnoid::Vector3& p, cnoid::Matrix3& R) {
	// pcl::io::savePCDFile(box_path, *sampled_obj_cloud);

	/* fist regsitration */
	//// calc init sol by using sac-ia
	//// source: box
	//// target: captured point cloud
	PointCloudTPtr box_cloud = PointCloudTPtr(new PointCloudT());
	pcl::io::loadPCDFile<PointT>(box_path, *box_cloud);
	FeatureCloudPtr box_feature = FeatureCloudPtr(new FeatureCloud());
	box_feature->setInputCloud(box_cloud);
	box_feature->setFeatureRadius(8 * voxel_leaf);
	box_feature->setNormalRadius(5 * voxel_leaf);
	box_feature->computeLocalFeatures();

	PointCloudTPtr scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::vecToCloud(points, scene);
	PointCloudTPtr sampled_scene = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::voxelFilter(scene, sampled_scene, voxel_leaf);
	FeatureCloudPtr scene_feature = FeatureCloudPtr(new FeatureCloud());
	scene_feature->setInputCloud(sampled_scene);
	scene_feature->setViewPoint(center);
	scene_feature->setFeatureRadius(8 * voxel_leaf);
	scene_feature->setNormalRadius(5 * voxel_leaf);
	scene_feature->computeLocalFeatures();

	SAC_IAEstimator sac;
	sac.setObjFeatureCloud(box_feature);
	sac.setSceneFeatureCloud(scene_feature);
	sac.setMaxCorrespondenceDist(0.01);
	sac.estimate();

	cnoid::Matrix4f init_sol = PointCloudUtil::convM3VtoM4(sac.getRot(), sac.getTrans());

	//// refinment
	ICPEstimator icp;
	icp.setObjFeatureCloud(box_feature);
	icp.setSceneFeatureCloud(scene_feature);
	icp.setMaxCorrespondenceDist(0.005);
	icp.init_matrix = init_sol;
	icp.setIteration(100);
	icp.estimate();

	cnoid::Vector3 init_p = icp.getTrans();
	cnoid::Matrix3 init_R = icp.getRot();

	// std::cout << "iniR::" << (grasp::rpyFromRot(init_R)/3.14*180).transpose() << std::endl;
	// std::cout << "inip::" << init_p.transpose() << std::endl;

	PointCloudTPtr sface = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::vecToCloud(sface_points, sface);
	PointCloudTPtr sampled_sface = PointCloudTPtr(new PointCloudT());
	PointCloudUtil::voxelFilter(sface, sampled_sface, voxel_leaf);

	double max_range_square = voxel_leaf * voxel_leaf;
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);

	//// second registration ////
	std::vector<cnoid::Matrix3> rot(6);
	rot[0] << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	rot[1] << 1, 0, 0, 0, 0, -1, 0, 1, 0;
	rot[2] << 1, 0, 0, 0, -1, 0, 0, 0, -1;
	rot[3] << 1, 0, 0, 0, 0, 1, 0, -1, 0;
	rot[4] << 0, 0, 1, 0, 1, 0, -1, 0, 0;
	rot[5] << 0, 0, -1, 0, 1, 0, 1, 0, 0;

	PointCloudTPtr tmp_cloud = PointCloudTPtr(new PointCloudT());
	pcl::IterativeClosestPoint<PointT, PointT> icp2d;
	icp2d.setInputTarget(sampled_scene);
	icp2d.setMaxCorrespondenceDistance(0.005);
	icp2d.setMaximumIterations(10);
	pcl::registration::TransformationEstimation2D<PointT, PointT>::Ptr transest2D = pcl::registration::TransformationEstimation2D<PointT, PointT>::Ptr(new pcl::registration::TransformationEstimation2D<PointT, PointT>());
	icp2d.setTransformationEstimation(transest2D);

	pcl::IterativeClosestPoint<PointT, PointT> icp_topface;
	icp_topface.setInputTarget(sampled_sface);
	icp_topface.setMaxCorrespondenceDistance(0.005);
	icp_topface.setMaximumIterations(50);
	int best_nr_in = 0;
	double best_score = 100;
	for (int j = 0; j < 6; j++) {
		for (int i = 0; i < 4; i++) {
			cnoid::Matrix3 tmp_R = init_R * rot[j] * grasp::rotFromRpy(0, 0, M_PI*0.5*i);

			// regsitration rotated box to top face
			cnoid::Matrix4f trans_matrix = PointCloudUtil::convM3VtoM4(tmp_R, init_p);
			icp_topface.setInputSource(box_cloud);
			icp_topface.align(*tmp_cloud, trans_matrix);
			if (!icp_topface.hasConverged()) continue;
			// cnoid::Vector3 sol2_p;
			// cnoid::Matrix3 sol2_R;
			// PointCloudUtil::convM4toM3V(icp_topface.getFinalTransformation(), sol2_R, sol2_p);
			// std::cout << "R2::" << (grasp::rpyFromRot(sol2_R)/3.14*180).transpose() << std::endl;
			// std::cout << "p2::" << sol2_p.transpose() << std::endl;

			// regsitration rotated box to captured point cloud
			icp2d.setInputSource(box_cloud);
			PointCloudTPtr aligned_cloud = PointCloudTPtr(new PointCloudT());
			icp2d.align(*tmp_cloud, icp_topface.getFinalTransformation());
			if (!icp2d.hasConverged()) continue;
			double score = icp2d.getFitnessScore();

			cnoid::Vector3 sol_p;
			cnoid::Matrix3 sol_R;
			PointCloudUtil::convM4toM3V(icp2d.getFinalTransformation(), sol_R, sol_p);
			PointCloudUtil::transCloud(sampled_obj_cloud, aligned_cloud, sol_R, sol_p);

			// reject if the face with pin is upward
			double z_dot = (sol_R * cnoid::Vector3(0, 0, -1)).dot(cnoid::Vector3(0, 0, 1));
			if (z_dot > 0.25) continue;

			// calc nubmer of inlier points
			KdTreePointTPtr obj_tree = KdTreePointTPtr(new KdTreePointT(false));
			obj_tree->setInputCloud(aligned_cloud);
			int nr_in = 0;
			size_t point_size = sampled_scene->points.size();
			for (int j = 0; j < point_size; j++) {
				obj_tree->nearestKSearch(sampled_scene->points[j], 1, nn_indices, nn_dists);
				if (nn_dists[0] < 4*max_range_square) {
					nr_in++;
				}
			}

			// std::cout << "R::" << (grasp::rpyFromRot(sol_R)/3.14*180).transpose() << std::endl;
			// std::cout << "p::" << sol_p.transpose() << std::endl;
			// std::cout << "nr:" << nr_in << "/" << point_size << std::endl;
			if (nr_in > best_nr_in || (nr_in == best_nr_in && best_score > score)) {
				p = sol_p;
				R = sol_R;
				best_nr_in = nr_in;
				best_score = score;
			}
		}
	}

	return true;
}

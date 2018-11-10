#include "Registrator.h"

#include <pcl/registration/icp.h>

#include "ObjectPoseEstimator.h"

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include <pcl/registration/sample_consensus_prerejective.h>
#include "sample_consensus_rejective.h"
#endif

AbstractPoseEstimator::AbstractPoseEstimator() :
	obj_cloud(new PointCloudT),
	scene_cloud(new PointCloudT),
	aligned_obj_cloud(new PointCloudT),
	score(0),
	max_correspondence_dist(0.001),
	init_matrix(Eigen::Matrix4f::Identity()) {
}

AbstractPoseEstimator::~AbstractPoseEstimator() {
}

/**
 * Set a pointer to the point cloud dataset of the object.
 */
void AbstractPoseEstimator::setObjFeatureCloud(FeatureCloudPtr _obj) {
	obj_feature = _obj;
}

/**
 * Set a pointer to the point cloud dataset of the scene.
 */
void AbstractPoseEstimator::setSceneFeatureCloud(FeatureCloudPtr _scene) {
	scene_feature = _scene;
}

/**
 * Get a pointer to the point cloud of the posture estimated object.
 * @return a pointer to the point cloud of the posture estimated object
 */
PointCloudTPtr AbstractPoseEstimator::getAlignedObjCloud() const {
	return aligned_obj_cloud;
}

/**
 * Get the fitness score
 * @return fitness score
 */
double AbstractPoseEstimator::getScore() const {
	return score;
}

/**
 * Get the rotation matrix of the estimated object.
 * @return rotation matrix
 */
Eigen::Matrix3d AbstractPoseEstimator::getRot() const {
	return trans_matrix.block<3, 3>(0, 0);
}

/**
 * Get the translation vector of the estimated object.
 * @return translation vector
 */
Eigen::Vector3d AbstractPoseEstimator::getTrans() const {
	return trans_matrix.block<3, 1>(0, 3);
}

void AbstractPoseEstimator::setMaxCorrespondenceDist(double dist) {
	max_correspondence_dist = dist;
}

ICPEstimator::ICPEstimator() :
	iteration(50),
	trans_eps_(1e-6),
	fit_eps_(0.01) {
	// nothing is done
}

ICPEstimator::~ICPEstimator() {
	// nothing is done
}

/**
 * Set maximum iterations.
 * @param[in] _ite the number of iterations
 */
void ICPEstimator::setIteration(unsigned int _ite) {
	iteration = _ite;
}

void ICPEstimator::setTransEps(double trans_eps) {
	trans_eps_ = trans_eps;
}

void ICPEstimator::setFitEps(double fit_eps) {
	fit_eps_ = fit_eps;
}

void ICPEstimator::setTree(pcl::search::KdTree<pcl::PointXYZ>::Ptr tree) {
	tree_ = tree;
}
/**
 * Estimate the object position and posture by using ICP(Iterative Closest Point)
 */
bool ICPEstimator::estimate() {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	obj_cloud = obj_feature->getCloud();
	scene_cloud = scene_feature->getCloud();
	icp.setInputSource(obj_cloud);
	icp.setInputTarget(scene_cloud);

	if (tree_) {
		icp.setSearchMethodTarget(tree_, true);
	}
	icp.setMaxCorrespondenceDistance(max_correspondence_dist);
	icp.setMaximumIterations(iteration);
	icp.align(*aligned_obj_cloud, init_matrix);
	if (!icp.hasConverged()) {
		return false;
	}
	score = icp.getFitnessScore(max_correspondence_dist);
	trans_matrix = icp.getFinalTransformation().cast<double>();
	return true;
}


SAC_IAEstimator::SAC_IAEstimator() :
	min_sample_distance(0.005),
	max_iteration(1000) {
	// nothing is done
}

SAC_IAEstimator::~SAC_IAEstimator() {
	// nothing is done
}

/**
 * Estimate the object position and posture by using SAC-IA
 */
bool SAC_IAEstimator::estimate() {
	// create the SAC_IA object and set the parameters
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setMinSampleDistance(min_sample_distance);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist*10);
	sac_ia.setMaximumIterations(max_iteration);

	sac_ia.setInputTarget(scene_feature->getCloud());
	sac_ia.setTargetFeatures(scene_feature->getLocalFeatures());
	sac_ia.setInputSource(obj_feature->getCloud());
	sac_ia.setSourceFeatures(obj_feature->getLocalFeatures());

	sac_ia.align(*aligned_obj_cloud, init_matrix);
	score = sac_ia.getFitnessScore(max_correspondence_dist);
	trans_matrix = sac_ia.getFinalTransformation().cast<double>();

	return true;
}


#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
bool RobustPoseEstimator::estimate() {
  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> align;
  align.setInputSource(obj_feature->getCloud());
  align.setSourceFeatures(obj_feature->getLocalFeatures());
  align.setInputTarget(scene_feature->getCloud());
  align.setTargetFeatures(scene_feature->getLocalFeatures());
	align.setMaximumIterations(1000);
  align.setNumberOfSamples(3);  // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(2);  // Number of nearest features to use
  align.setSimilarityThreshold(0.6f);  // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(max_correspondence_dist);  // Set inlier threshold
  align.setInlierFraction(0.25f);  // Set required inlier fraction
  align.align(*aligned_obj_cloud);
	if (!align.hasConverged()) {
		return false;
	}
	score = align.getFitnessScore();
	trans_matrix = align.getFinalTransformation().cast<double>();
	return true;
}

bool RobustRejectivePoseEstimator::estimate() {
	CylinderParam cylinder = CylinderParam(Vector3(0, 0, 0), Vector3(0, 0, 1), 0.0328, 0.0416);
	pcl::SampleConsensusRejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33, pcl::Normal> align;
	align.setInputSource(obj_feature->getCloud());
  align.setSourceFeatures(obj_feature->getLocalFeatures());
	align.setSourceNormals(obj_feature->getNormals());
  align.setInputTarget(scene_feature->getCloud());
  align.setTargetFeatures(scene_feature->getLocalFeatures());
	align.setTargetNormals(scene_feature->getNormals());
	align.setMaximumIterations(5000);
  align.setNumberOfSamples(3);  // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(2);  // Number of nearest features to use
  align.setSimilarityThreshold(0.6f);  // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(max_correspondence_dist);  // Set inlier threshold
	align.setMaxCorrespondenceDistance(0.0025);  // Set inlier threshold
  align.setInlierFraction(0.1f);  // Set required inlier fraction
	align.addCylinder(cylinder);
  align.align(*aligned_obj_cloud);
	if (!align.hasConverged()) {
		return false;
	}
	score = align.getFitnessScore();
	trans_matrix = align.getFinalTransformation().cast<double>();
	return true;
}
#endif

FeatureCloud::FeatureCloud() :
method(new SearchMethod),
cloud(new PointCloudT),
normals(new NormalCloud),
features(new LocalFeatures),
view_point(Vector3(0, 0, 0)),
feature_radius(0.02),
normal_radius(0.005),
normal_k(10),
feature_k(10),
use_normal_ksearch(false),
use_feature_ksearch(false) {
    // nothing is done
}

FeatureCloud::~FeatureCloud() {
    // nothing is done
}

/**
* Set a pointer to the target point cloud.
* @param[in] _cloud a pointer to the target point cloud
*/
void FeatureCloud::setInputCloud(PointCloudTConstPtr _cloud) {
    cloud = _cloud;
}

/**
* Get a pointer to the target point cloud.
* @return a pointer to the target point cloud
*/
PointCloudTConstPtr FeatureCloud::getCloud() const {
    return cloud;
}

/**
* Get a pointer to the local features of target point cloud.
* @return a pointer to the local features
*/
LocalFeaturesPtr FeatureCloud::getLocalFeatures() const {
    return features;
}

/**
* Get a pointer to the normals of target point cloud.
* @return a pointer to the normals
*/
NormalCloudPtr FeatureCloud::getNormals() const {
    return normals;
}

/**
* Set a view point which is used in noraml estimation.
* @param[in] _viewpoint view point
*/
void FeatureCloud::setViewPoint(const Vector3& _viewpoint) {
    view_point = _viewpoint;
}

/**
* Set a radius which is used for determing the neighbors in local feature estimation.
* @param[in] _radius radius
*/
void FeatureCloud::setFeatureRadius(double _radius) {
    feature_radius = _radius;
}

/**
* Set a radius which is used for determing the neighbors in normal estimation.
* @param[in] _radius radius
*/
void FeatureCloud::setNormalRadius(double _radius) {
    normal_radius = _radius;
}

/**
* Set the number of k-nearest neighbors in local feature estimation.
* @param[in] _k number of k-nearest neighbors
*/
void FeatureCloud::setFeatureK(unsigned int _k) {
    feature_k = _k;
}

/**
* Set the number of k-nearest neighbors in normal estimation.
* @param[in] _k number of k-nearest neighbors
*/
void FeatureCloud::setNormalK(unsigned int _k) {
    normal_k = _k;
}

/**
* Select a method to determine nearest neighbors in local feature estimation.
* @param[in] _use ture: use k-search false: use radius search
*/
void FeatureCloud::useFeatureKSearch(bool _use) {
    use_feature_ksearch = _use;
}

/**
* Select a method to determine nearest neighbors in normal estimation.
* @param[in] _use ture: use k-search false: use radius search
*/
void FeatureCloud::useNormalKSearch(bool _use) {
    use_feature_ksearch = _use;
}

/**
* Compute the local features of target point cloud.
*/
void FeatureCloud::computeLocalFeatures() {
    computeNormals();

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(method);
    if (use_feature_ksearch) {
        fpfh_est.setKSearch(feature_k);
    } else {
        fpfh_est.setRadiusSearch(feature_radius);
    }
    fpfh_est.compute(*features);
}

/**
* Compute the normals of target point cloud.
*/
void FeatureCloud::computeNormals() {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setViewPoint(view_point(0), view_point(1), view_point(2));
    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(method);
    if (use_normal_ksearch) {
        norm_est.setKSearch(normal_k);
    } else {
        norm_est.setRadiusSearch(normal_radius);
    }
    norm_est.compute(*normals);
}


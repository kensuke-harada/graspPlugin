#ifndef _PCL_OBJECTPOSEESTIMATOR_H_
#define _PCL_OBJECTPOSEESTIMATOR_H_

#include <Eigen/Core>
#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(<, 1, 7, 0) && EIGEN_VERSION_AT_LEAST(3,2,0)
#include "EigenInternalMath.h"
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/transformation_validation_euclidean.h>


#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include <pcl/features/cvfh.h>
/* #include <pcl/recognition/crh_alignment.h>*/
// We use modified crh_alignment.h because computeRobllAngle
// function in orignal crh_alignment.h has memory leaks.
#include "crh_alignment.h"
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include <flann/flann.h>
#endif
#include "CVFHDescriptorManipulator.h"
#endif

#include <cnoid/ExecutablePath>
#include <cnoid/ItemTreeView>
/* #ifdef ENABLE_OSG */
/* #include "../Grasp/DrawUtility.h" */
/* #else */
#include "PCLBar.h"
/* #endif */
#include "../Grasp/VectorMath.h"
#include "../Grasp/PlanBase.h"
#include "PointCloudHandler.h"
#include "PointCloudTypes.h"
#include "PointCloudMerger.h"
/* #include "PointCloudDrawer.h" */

#include "exportdef.h"

typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef LocalFeatures::Ptr                    LocalFeaturesPtr;

class AbstractPoseEstimator;
class AbstractSegmenter;
class FeatureCloud;
class PoseEstimationResultMatcher;

typedef boost::shared_ptr<FeatureCloud>       FeatureCloudPtr;

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
	class RecognizedRootItem : public cnoid::Item {};
#endif

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
class CVFHPoseEstimator;
#endif

class ObjectPointCloudGenerator {
 public:
	ObjectPointCloudGenerator();
	virtual ~ObjectPointCloudGenerator();

	static bool loadVRMLFile(const std::string& filepath, cnoid::BodyItemPtr& body_item);
	void generatePointCloud(const cnoid::BodyItemPtr& obj_item, PointCloudTPtr& cloud) const;

 private:
	bool getPCDFilePath(const cnoid::BodyItemPtr& obj_item, std::string& path) const;
	void loadFromPCD(const std::string& path, PointCloudTPtr& cloud) const;
	void generateFromBodyItem(const cnoid::BodyItemPtr& obj_item, PointCloudTPtr& cloud) const;
};

class EXCADE_API ObjectPoseEstimator {
public:
    ObjectPoseEstimator();
    virtual ~ObjectPoseEstimator();

    struct PoseSolution {
        double score;
        cnoid::Vector3 p;
        cnoid::Matrix3 R;
        double outlier_ratio;
			std::vector<int> outlier_indices;
    };

		/** @deprecated */
    bool readScene(bool is_load_file, bool is_save_file, const std::string& load_file_path, const std::string& save_file_path,
                                 bool is_merge, bool is_handcamera);
		bool readScene(bool is_load_file, bool is_save_file, const std::string& load_file_path, const std::string& save_file_path,
									 bool is_merge);
		bool readScene(bool is_load_file, bool is_save_file, const std::string& load_file_path, const std::string& save_file_path,
									 const cnoid::Vector3& p, const cnoid::Matrix3& R, bool is_merge);
    bool readObjectVRML(const std::string file_path);
		void readObject();
		/** @deprecated */
    void getViewPoint(bool is_handcamera, cnoid::Vector3& view_point) const;
		static void getViewPoint(cnoid::Vector3& view_point);
		/** @deprecated */
		void saveEnv();
		/** @deprecated */
		void readEnv();
		void setEnvCloud(PointCloudTPtr env);

    PointCloudTPtr getObjCloud() const;
    PointCloudTConstPtr getSceneCloud() const;
    PointCloudTPtr getSampledObjCloud() const;
    PointCloudTPtr getSampledSceneCloud() const;
    ColorPointCloudConstPtr getSceneColorCloud() const;
		ColorPointCloudPtr getClusteredCloud();

    void setEstimator(AbstractPoseEstimator* _estimator);
    void setInitialEstimator(AbstractPoseEstimator* _estimator);
    void setSegmenter(AbstractSegmenter* _segmenter);
    void setVoxelLeaf(double _leaf);
    void setTargetRegion(double _x_max, double _x_min, double _y_max, double _y_min, double _z_max, double _z_min);
		void setTargetRegion(const cnoid::Vector3& min, const cnoid::Vector3& max);
    void setCandidateNum(unsigned int num);
    void setFeatureRadius(double _radius);
    void setNormalRadius(double _radisu);
    void setFeatureK(unsigned int _k);
    void setNormalK(unsigned int _k);
    void useFeatureKSearch(bool _use);
    void useNormalKSearch(bool _use);
		void setICPMaxIteration(int _ite);
		void setSegmentResegmentRadius(double radius);
		void setSegmentBoundaryRadius(double radius);

    bool downsampling(bool do_narrow = true);
    bool segment();
    bool estimate(bool do_move = false);

		void initCvfhEstimate(int& n_cand);
		bool cvfhEstimateOneCluster(int i, cnoid::Vector3& p, cnoid::Matrix3& R, std::vector<int>& outlier_idx,
																const cnoid::Vector3& view_point, const cnoid::Matrix3& cameraR, int nn = 42, bool use_bb = false);
    bool cvfhEstimate(bool do_move, const cnoid::Vector3& view_poin,
                      std::string file_path = "", int nn = 42,
                      bool use_bb = false, bool b_selectbest = false, bool is_binpicking = false);

		void registerPrevCloudToResultMatcher();
		void setUsePrevResult(bool flag);

		void displayEstimationResults();
		std::vector<ClusterInfo> prev_clusters;

    PointCloudTPtr env_cloud;
		/** @deprecated */
    NormalCloudPtr env_normal;

		/** @deprecated */
    static void calcBoundingBox(PointCloudTConstPtr cloud, cnoid::Vector3& edge, cnoid::Vector3& center, cnoid::Matrix3& Rot);
		/** @deprecated */
    static void cloud2Vec(PointCloudTConstPtr cloud, std::vector<cnoid::Vector3>& vec_cloud);
		/** @deprecated */
    static void normal2Vec(NormalCloudPtr normal, std::vector<cnoid::Vector3>& vec_normal);
 protected:
    void moveObject(const cnoid::Matrix3& rot, const cnoid::Vector3& trans);
    double getDiffRatio(const cnoid::Vector3& ori_edge, const cnoid::Vector3& edge) const;
    void createFeatureCloud(PointCloudTConstPtr cloud, const cnoid::Vector3& view_point, FeatureCloudPtr feature, bool only_normal = false);
    double getScore(PointCloudTConstPtr obj, PointCloudTConstPtr scene, double max_range = 0.1) const;
    int getNumOutlierPoints(PointCloudTConstPtr obj, PointCloudTConstPtr scene, double max_range = 0.1) const;
		void setOutlierIdx(const cnoid::Vector3& p, const cnoid::Matrix3& R, std::vector<int>& outlier_idx) const;
		void initProc();
		void makeCandidates();
		int getNumTargetCandidates() const;
		bool isSimilarCloudToPreviousResult(PointCloudTConstPtr cloud, ClusterInfo* cluster) const;
		void getBodyItems(int n, std::vector<cnoid::BodyItemPtr>& items) const;

		void registerReuseSol(int i);

 public:
    struct Candidate {
        int id;
        PointCloudTPtr cloud;
        cnoid::Vector3 edge;
        cnoid::Vector3 center;
        cnoid::Matrix3 rot;
        double error;
			bool is_match_prev_cluster;
        static bool Less(const Candidate& l, const Candidate& r) {
					return (l.error < r.error);
        }
    };
 protected:
		std::vector<Candidate> candidates_;
		double max_distance_;
		std::vector<PoseSolution> estimation_sols;

    AbstractSegmenter* segmenter;
    AbstractPoseEstimator* initial_estimator;
    AbstractPoseEstimator* estimator;
		PoseEstimationResultMatcher* result_matcher_;
    PointCloudTPtr obj_cloud; ///< point cloud that is made from target object
    PointCloudTPtr sampled_obj_cloud; ///< sampled obj_cloud
    PointCloudTConstPtr scene_cloud; ///< coloreless captured point cloud
    PointCloudTPtr sampled_scene_cloud; ///< sampled and clipped scene_cloud
		bool use_prev_result_;

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
		CVFHPoseEstimator* cvfh_est_;
#endif

    Link* object_link; ///< link of target object

    bool do_downsampling;
    double voxel_leaf;
		cnoid::Vector3 region_max_;
		cnoid::Vector3 region_min_;
    unsigned int num_candidate;
    double normal_radius;
    double feature_radius;
    unsigned int normal_k;
    unsigned int feature_k;
    bool use_normal_ksearch;
    bool use_feature_ksearch;
		int icp_max_iteration;
		double segment_reseg_radius_;
		double segment_bound_radius_;
};

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
class CVFHPoseEstimator {
 public:
	CVFHPoseEstimator();
	virtual ~CVFHPoseEstimator();

	void initCvfhEstimate(cnoid::BodyItemPtr bodyitem);
	bool cvfhEstimateOneCluster(int i, ObjectPoseEstimator::PoseSolution& sol);
	void cvfhEstimateOneClusterWoICP(int i, std::vector<ObjectPoseEstimator::PoseSolution>& sol);

	void setViewPoint(const cnoid::Vector3& view_point);
	void setCameraR(const cnoid::Matrix3& cameraR);
	void setNN(int nn);
	void doRejectByBoundingBoxSize(bool use_bb);
	void setMaxDistance(double max_distance);
	void setICPMaxIteration(int icp_max_iteration);
	void setObjectCloud(PointCloudTPtr obj_cloud);
	void setCandidates(std::vector<ObjectPoseEstimator::Candidate>* candidates);

 private:
	cnoid::Vector3 view_point_;
	int nn_;
	bool reject_by_bb_;
	cnoid::Matrix3 cameraR_;
	std::vector<ObjectPoseEstimator::Candidate>* candidates_;
	double max_distance_;
	int icp_max_iteration_;
	PointCloudTPtr sampled_obj_cloud_; ///< sampled obj_cloud
	double likelihood_th_;
	double likelihood_th_depth_;

	CVFHDescriptorManipulator::CVFHDescriptors des_;
	pcl::KdTreeFLANN<DescriptorPair::CVFHDescriptor, flann::ChiSquareDistance<float> > match_search_;

	bool isAcceptableBoundingBox(int cand_id, int neigh_idx) const;
	std::string getDescriptorDir(cnoid::BodyItemPtr bodyitem);
};
#endif

#endif /* _PCL_OBJECTPOSEESTIMATOR_H_ */

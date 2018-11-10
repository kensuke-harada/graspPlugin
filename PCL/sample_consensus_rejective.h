/**
 * @file   sample_consensus_rejective.h
 * @author Akira Ohchi
*/

#ifndef _PCL_SAMPLE_CONSENSUS_REJECTIVE_H_
#define _PCL_SAMPLE_CONSENSUS_REJECTIVE_H_

#include <vector>

#include <pcl/registration/sample_consensus_prerejective.h>

#include "PrimitiveShapeParameter.h"

namespace pcl {
  template<typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
		class SampleConsensusRejective : public SampleConsensusPrerejective<PointSource, PointTarget, FeatureT> {
	public:
		typedef typename Registration<PointSource, PointTarget>::Matrix4 Matrix4;
		typedef typename SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::PointCloudSource PointCloudSource;

		using Registration<PointSource, PointTarget>::input_;
		using Registration<PointSource, PointTarget>::target_;
		using Registration<PointSource, PointTarget>::tree_;
		using Registration<PointSource, PointTarget>::transformation_;
		using Registration<PointSource, PointTarget>::final_transformation_;
		using Registration<PointSource, PointTarget>::transformation_estimation_;
		using Registration<PointSource, PointTarget>::max_iterations_;
		using Registration<PointSource, PointTarget>::corr_dist_threshold_;
		using Registration<PointSource, PointTarget>::converged_;
		using SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::input_features_;
		using SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::target_features_;
		using SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::inlier_fraction_;
		using SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::nr_samples_;
		using SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::correspondence_rejector_poly_;
		using SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::inliers_;
		typedef pcl::PointCloud<NormalT> NormalCloud;
		typedef typename NormalCloud::Ptr NormalCloudPtr;
		typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

		SampleConsensusRejective()
			: input_normals_(new NormalCloud)
			, target_normals_(new NormalCloud)
			, reject_flip_normal_fraction_(0.5)
			, reject_inside_inlier_ratio_(3.0) {
		}

		virtual ~SampleConsensusRejective() {
		}

		void setSourceNormals(const NormalCloudConstPtr& normals);
		inline const NormalCloudConstPtr getSourceNormals() const {return input_normals_;}

		void setTargetNormals(const NormalCloudConstPtr& normals);
		inline const NormalCloudConstPtr getTargetNormals() const {return target_normals_;}

		inline void setCylinders(const std::vector<CylinderParam>& cylinders) {cylinders_ = cylinders;}
		inline void setBoxes(const std::vector<BoxParam>& boxes) {boxes_ = boxes;}
		inline void addCylinder(const CylinderParam& cylinder) {cylinders_.push_back(cylinder);}
		inline void addBox(const BoxParam& box) {boxes_.push_back(box);}
		inline void clearPimitiveShapes() {clearCylinders(); clearBoxes();}
		inline void clearCylinders() {cylinders_.clear();}
		inline void clearBoxes() {boxes_.clear();}

		void computeTransformation(PointCloudSource &output, const Eigen::Matrix4f& guess);
		void getFitness(std::vector<int>& inliers, std::vector<int>& insiders, bool& is_flip);

		inline const std::vector<int>& getInsiders() const {
			return insiders_;
		}
	protected:
		void getFitnessCylinder(std::vector<int>& inliers, std::vector<int>& insiders, bool& is_flip);
		void getFitnessBox(std::vector<int>& inliers, std::vector<int>& insiders, bool& is_flip);
		
		NormalCloudConstPtr input_normals_;
		NormalCloudConstPtr target_normals_;

		float reject_flip_normal_fraction_;
		float reject_inside_inlier_ratio_;

		std::vector<CylinderParam> cylinders_;
		std::vector<BoxParam> boxes_;
		std::vector<int> insiders_;
	};
}

#include <./sample_consensus_rejective.hpp>

#endif /* _PCL_SAMPLE_CONSENSUS_REFJECTIVE_H_ */






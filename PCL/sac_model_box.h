/**
 * @file   sac_model_box.h
 * @author Akira Ohchi
 */

#ifndef _PCL_SAC_MODEL_BOX_H_
#define _PCL_SAC_MODEL_BOX_H_

#include <cstddef>
#include <vector>
#include <set>

#include <Eigen/Geometry>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/common/common.h>

namespace pcl {

template<typename PointT, typename PointNT>
class SampleConsensusModelBox :
      public SampleConsensusModel<PointT>,
      public SampleConsensusModelFromNormals<PointT, PointNT> {
 public:
  using SampleConsensusModel<PointT>::input_;
  using SampleConsensusModel<PointT>::indices_;
  using SampleConsensusModel<PointT>::radius_min_;
  using SampleConsensusModel<PointT>::radius_max_;
  using SampleConsensusModelFromNormals<PointT, PointNT>::normals_;
  using SampleConsensusModelFromNormals<PointT, PointNT>::normal_distance_weight_;
  using SampleConsensusModel<PointT>::error_sqr_dists_;

  typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
  typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
  typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

  typedef boost::shared_ptr<SampleConsensusModelBox> Ptr;

  SampleConsensusModelBox(const PointCloudConstPtr &cloud, bool random = false)
      : SampleConsensusModel<PointT>(cloud, random)
      , SampleConsensusModelFromNormals<PointT, PointNT>()
      , ortho_eps_(0.5)
      , voxel_size_(0.005)
      , accept_point_ratio_(0.0) {
  }

    SampleConsensusModelBox(const PointCloudConstPtr &cloud,
                            const std::vector<int> &indices,
                            bool random = false)
        : SampleConsensusModel<PointT>(cloud, indices, random)
        , SampleConsensusModelFromNormals<PointT, PointNT>()
        , ortho_eps_(0.5)
        , voxel_size_(0.005)
        , accept_point_ratio_(0.0) {
    }

      SampleConsensusModelBox(const SampleConsensusModelBox &source)
          : SampleConsensusModel<PointT>()
          , SampleConsensusModelFromNormals<PointT, PointNT>()
          , ortho_eps_(0.5)
          , voxel_size_(0.005)
          , accept_point_ratio_(0.0) {
        *this = source;
      }

        virtual ~SampleConsensusModelBox() {;}

  inline SampleConsensusModelBox&
  operator = (const SampleConsensusModelBox &source) {
    SampleConsensusModel<PointT>::operator=(source);
    ortho_eps_ = source.ortho_eps_;
    voxel_size_ = source.voxel_size_;
    accept_point_ratio_ = source.accept_point_ratio_;
    return (*this);
  }

  void getSamples(int &iterations, std::vector<int> &samples);

  bool computeModelCoefficients(const std::vector<int> &samples,
                                Eigen::VectorXf &model_coefficients) const;

  bool computeModelCoefficients(const std::vector<int> &samples,
                                Eigen::VectorXf &model_coefficients) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).computeModelCoefficients(samples, model_coefficients);
  }

  void optimizeModelCoefficients(const std::vector<int> &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients) const;

  void optimizeModelCoefficients(const std::vector<int> &inliers,
                                 const Eigen::VectorXf &model_coefficients,
                                 Eigen::VectorXf &optimized_coefficients) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).optimizeModelCoefficients(inliers, model_coefficients, optimized_coefficients);
  }

  void getDistancesToModel(const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) const;

  void getDistancesToModel(const Eigen::VectorXf &model_coefficients,
                           std::vector<double> &distances) {
    static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).getDistancesToModel(model_coefficients, distances);
  }

  void selectWithinDistance(const Eigen::VectorXf &model_coefficients,
                            const double threshold,
                            std::vector<int> &inliers) const;

  void selectWithinDistance(const Eigen::VectorXf &model_coefficients,
                            const double threshold,
                            std::vector<int> &inliers) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).selectWithinDistance(model_coefficients, threshold, inliers);
  }

#if PCL_VERSION_COMPARE(<, 1, 10, 0)
  int countWithinDistance(const Eigen::VectorXf &model_coefficients,
                          const double threshold) const;

  int countWithinDistance(const Eigen::VectorXf &model_coefficients,
                          const double threshold) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).countWithinDistance(model_coefficients, threshold);
  }
#else
  std::size_t countWithinDistance(const Eigen::VectorXf &model_coefficients,
                          const double threshold) const;

  std::size_t countWithinDistance(const Eigen::VectorXf &model_coefficients,
                          const double threshold) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).countWithinDistance(model_coefficients, threshold);
  }
#endif

  void projectPoints(const std::vector<int> &inliers,
                     const Eigen::VectorXf &model_coefficients,
                     PointCloud &projected_points,
                     bool copy_data_fields = true) const;

  void projectPoints(const std::vector<int> &inliers,
                     const Eigen::VectorXf &model_coefficients,
                     PointCloud &projected_points,
                     bool copy_data_fields = true) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).projectPoints(inliers, model_coefficients, projected_points, copy_data_fields);
  }

  bool doSamplesVerifyModel(const std::set<int> &indices,
                            const Eigen::VectorXf &model_coefficients,
                            const double threshold) const;

  bool doSamplesVerifyModel(const std::set<int> &indices,
                            const Eigen::VectorXf &model_coefficients,
                            const double threshold) {
    return static_cast<const SampleConsensusModelBox<PointT, PointNT> &>(*this).doSamplesVerifyModel(indices, model_coefficients, threshold);
  }

  void setOrthoEps(double eps) {ortho_eps_ = eps;}
  void setVoxelSize(double size) {voxel_size_ = size;}
  void setAcceptPointRatio(double ratio) {accept_point_ratio_ = ratio;}

  inline pcl::SacModel getModelType() const {return pcl::SacModel(0);}
 protected:
  bool isModelValid(const Eigen::VectorXf &model_coefficients);

  bool isSampleGood(const std::vector<int> &samples) const;

  void reComputeModelCoefficients(Eigen::VectorXf &model_coefficients) const;
 private:
  double ortho_eps_;
  double voxel_size_;
  double accept_point_ratio_;
};
}

#include "sac_model_box.hpp"

#endif

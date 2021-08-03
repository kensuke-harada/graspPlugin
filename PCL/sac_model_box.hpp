/**
 * @file   sac_model_box.hpp
 * @author Akira Ohchi
 */

#ifndef _PCL_SAC_MODEL_BOX_HPP_
#define _PCL_SAC_MODEL_BOX_HPP_

#include <vector>
#include <set>
#include <limits>
#include <algorithm>

#include <pcl/sample_consensus/eigen.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/common/concatenate.h>

template<typename PointT, typename PointNT>
void pcl::SampleConsensusModelBox<PointT, PointNT>::getSamples(int &iterations,
                                                               std::vector<int> &samples) {
  if (indices_->size() < 3) {
    PCL_ERROR("[pcl::SampleConsensusModelBox::getSamples] Can not select %zu unique points out of %zu!\n",
              samples.size(), indices_->size());
    // one of these will make it stop :)
    samples.clear();
    iterations = INT_MAX - 1;
    return;
  }

  // Get a second point which is different than the first
  samples.resize(3);
  for (unsigned int iter = 0; iter < SampleConsensusModel<PointT>::max_sample_checks_; ++iter) {
    // Choose the random indices
    if (SampleConsensusModel<PointT>::samples_radius_ < std::numeric_limits<double>::epsilon())
      SampleConsensusModel<PointT>::drawIndexSample(samples);
    else
      SampleConsensusModel<PointT>::drawIndexSampleRadius(samples);

    // If it's a good sample, stop here
    if (isSampleGood(samples)) {
      PCL_DEBUG("[pcl::SampleConsensusModelBox::getSamples] Selected %zu samples.\n", samples.size());
      return;
    }
  }
  PCL_DEBUG("[pcl::SampleConsensusModelBox::getSamples] WARNING: Could not select %d sample points in %d iterations!\n", 3, SampleConsensusModel<PointT>::max_sample_checks_);
  samples.clear();
}

template<typename PointT, typename PointNT>
bool pcl::SampleConsensusModelBox<PointT, PointNT>::computeModelCoefficients(
    const std::vector<int> &samples, Eigen::VectorXf &model_coefficients) {
  // Need 3 samples
  if (samples.size() != 3) {
    PCL_ERROR("[pcl::SampleConsensusModelBox::computeModelCoefficients] Invalid set of samples given (%zu)!\n", samples.size());
    return false;
  }

  if (!normals_) {
    PCL_ERROR("[pcl::SampleConsensusModelBox::computeModelCoefficients] No input dataset containing normals was given!\n");
    return false;
  }

  Eigen::Vector3f p0(input_->points[samples[0]].x, input_->points[samples[0]].y, input_->points[samples[0]].z);
  Eigen::Vector3f p1(input_->points[samples[1]].x, input_->points[samples[1]].y, input_->points[samples[1]].z);
  Eigen::Vector3f p2(input_->points[samples[2]].x, input_->points[samples[2]].y, input_->points[samples[2]].z);

  Eigen::Vector3f n0(normals_->points[samples[0]].normal[0], normals_->points[samples[0]].normal[1], normals_->points[samples[0]].normal[2]);
  Eigen::Vector3f n1(normals_->points[samples[1]].normal[0], normals_->points[samples[1]].normal[1], normals_->points[samples[1]].normal[2]);
  Eigen::Vector3f n2(normals_->points[samples[2]].normal[0], normals_->points[samples[2]].normal[1], normals_->points[samples[2]].normal[2]);

  Eigen::Vector3f avg_n01 = ((n0 + n1) / 2.0).normalized();
  Eigen::Vector3f avg_n012 = ((avg_n01 + n2) / 2.0).normalized();
  Eigen::Vector3f axis = avg_n01.cross(n2).normalized();
  Eigen::Vector3f adjusted_n2 = Eigen::AngleAxisf(M_PI/4.0, axis) * avg_n012;
  Eigen::Vector3f adjusted_avgn01 = Eigen::AngleAxisf(-M_PI/4.0, axis) * avg_n012;
  Eigen::Vector3f tmp0 = Eigen::AngleAxisf(M_PI/4.0, adjusted_n2) * adjusted_avgn01;
  Eigen::Vector3f tmp1 = Eigen::AngleAxisf(-M_PI/4.0, adjusted_n2) * adjusted_avgn01;
  Eigen::Vector3f adjusted_n0;
  Eigen::Vector3f adjusted_n1;
  if (fabs(tmp0.dot(n0)) > fabs(tmp0.dot(n1))) {
    adjusted_n0 = tmp0;
    adjusted_n1 = tmp1;
  } else {
    adjusted_n0 = tmp1;
    adjusted_n1 = tmp0;
  }

  Eigen::Vector3f corner = (p0.dot(adjusted_n0)*adjusted_n1.cross(adjusted_n2) + p1.dot(adjusted_n1)*adjusted_n2.cross(adjusted_n0) + p2.dot(adjusted_n2)*n0.cross(adjusted_n1)) /
                           ((adjusted_n0.cross(adjusted_n1)).dot(adjusted_n2));

  std::vector<Eigen::Vector3f> len;
  double k = 1.25;
  len.push_back(k*((corner - p1).dot(adjusted_n0) * adjusted_n0 + (corner - p2).dot(adjusted_n0) * adjusted_n0)/2.0);
  len.push_back(k*((corner - p0).dot(adjusted_n1) * adjusted_n1 + (corner - p2).dot(adjusted_n1) * adjusted_n1)/2.0);
  len.push_back(k*((corner - p1).dot(adjusted_n2) * adjusted_n2 + (corner - p0).dot(adjusted_n2) * adjusted_n2)/2.0);

  for (int i = 0; i < 3; i++) {
    double tmp_len = len[i].norm();
    if (tmp_len < radius_min_) {
      len[i] *= radius_min_ / tmp_len;
    } else if (tmp_len > radius_max_) {
      len[i] *= radius_max_ / tmp_len;
    }
  }

  Eigen::Vector3f center = corner - (len[0] + len[1] + len[2]);

  if ((adjusted_n0.cross(adjusted_n1)).dot(adjusted_n2) < 0) {
    adjusted_n2 *= -1;
  }

  model_coefficients.resize(15);
  // normal of planes
  model_coefficients[0] = adjusted_n0[0];
  model_coefficients[1] = adjusted_n0[1];
  model_coefficients[2] = adjusted_n0[2];
  model_coefficients[3] = adjusted_n1[0];
  model_coefficients[4] = adjusted_n1[1];
  model_coefficients[5] = adjusted_n1[2];
  model_coefficients[6] = adjusted_n2[0];
  model_coefficients[7] = adjusted_n2[1];
  model_coefficients[8] = adjusted_n2[2];

  // center
  model_coefficients[9] = center[0];
  model_coefficients[10] = center[1];
  model_coefficients[11] = center[2];

  // length
  model_coefficients[12] = len[0].norm() * 2.0;
  model_coefficients[13] = len[1].norm() * 2.0;
  model_coefficients[14] = len[2].norm() * 2.0;

  reComputeModelCoefficients(model_coefficients);
  return true;
}

template<typename PointT, typename PointNT>
void pcl::SampleConsensusModelBox<PointT, PointNT>::optimizeModelCoefficients(
    const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
    Eigen::VectorXf &optimized_coefficients) {
  PCL_ERROR("[pcl::SampleConsensusModelBox::optimizeModelCoefficients] Not implemented!\n");
}

template<typename PointT, typename PointNT>
void pcl::SampleConsensusModelBox<PointT, PointNT>::getDistancesToModel(
    const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) {
  PCL_ERROR("[pcl::SampleConsensusModelBox::getDistancesToModel] Not implemented!\n");
}

template<typename PointT, typename PointNT>
void pcl::SampleConsensusModelBox<PointT, PointNT>::selectWithinDistance(
    const Eigen::VectorXf &model_coefficients, const double threshold, std::vector<int> &inliers) {
  int nr_p = 0;

  inliers.resize(indices_->size());

  std::vector<Eigen::Vector3f> plane_n;
  std::vector<Eigen::Vector3f> plane_p;
  Eigen::Vector3f center(model_coefficients[9], model_coefficients[10], model_coefficients[11]);
  for (int i = 0; i < 3; i++) {
    plane_n.push_back(Eigen::Vector3f(model_coefficients[3*i], model_coefficients[3*i+1], model_coefficients[3*i+2]));
    plane_p.push_back(center + ((model_coefficients[12+i] * plane_n[i*2]) / 2.0));
    plane_n.push_back(-Eigen::Vector3f(model_coefficients[3*i], model_coefficients[3*i+1], model_coefficients[3*i+2]));
    plane_p.push_back(center + ((model_coefficients[12+i] * plane_n[i*2+1]) / 2.0));
  }

  for (size_t i = 0; i < indices_->size(); i++) {
    Eigen::Vector3f target_p(input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z);
    std::vector<double> dist;
    std::vector<bool> is_inrange;
    std::vector<Eigen::Vector3f> target_normals;
    for (int j = 0; j < 3; j++) {
      bool is_in0 = plane_n[2*j].dot(target_p-plane_p[2*j]) < 0;
      bool is_in1 = plane_n[2*j+1].dot(target_p-plane_p[2*j+1]) < 0;
      is_inrange.push_back(is_in0 && is_in1);
      dist.push_back(fabs((target_p-plane_p[2*j]).dot(plane_n[2*j])));
      dist.push_back(fabs((target_p-plane_p[2*j+1]).dot(plane_n[2*j+1])));
    }
    for (int j = 0; j < 3; j++) {
      double dist_from_box = 0;
      dist_from_box += std::min(dist[2*j], dist[2*j+1]);
      if (!is_inrange[(j+1)%3]) {
        dist_from_box += std::min(dist[2*((j+1)%3)], dist[2*((j+1)%3)+1]);
      }
      if (!is_inrange[(j+2)%3]) {
        dist_from_box += std::min(dist[2*((j+2)%3)], dist[2*((j+2)%3)+1]);
      }
      if (dist_from_box < threshold && fabs(plane_n[2*j].dot(target_p-plane_p[2*j])) < ortho_eps_) {
        inliers[nr_p] = (*indices_)[i];
        nr_p++;
        break;
      }
    }
  }
  inliers.resize(nr_p);
}

template<typename PointT, typename PointNT>
int pcl::SampleConsensusModelBox<PointT, PointNT>::countWithinDistance(
    const Eigen::VectorXf &model_coefficients, const double threshold) {
  int nr_p = 0;
  std::vector<Eigen::Vector3f> plane_n;
  std::vector<Eigen::Vector3f> plane_p;
  Eigen::Vector3f center = Eigen::Vector3f(model_coefficients[9], model_coefficients[10], model_coefficients[11]);
  for (int i = 0; i < 3; i++) {
    plane_n.push_back(Eigen::Vector3f(model_coefficients[3*i], model_coefficients[3*i+1], model_coefficients[3*i+2]));
    plane_p.push_back(center + ((model_coefficients[12+i] * plane_n[i*2]) / 2.0));
    plane_n.push_back(-Eigen::Vector3f(model_coefficients[3*i], model_coefficients[3*i+1], model_coefficients[3*i+2]));
    plane_p.push_back(center + ((model_coefficients[12+i] * plane_n[i*2+1]) / 2.0));
  }
  for (size_t i = 0; i < indices_->size(); i++) {
    Eigen::Vector3f target_p(input_->points[(*indices_)[i]].x, input_->points[(*indices_)[i]].y, input_->points[(*indices_)[i]].z);
    std::vector<double> dist;
    std::vector<bool> is_inrange;
    std::vector<Eigen::Vector3f> target_normals;
    for (int j = 0; j < 3; j++) {
      bool is_in0 = plane_n[2*j].dot(target_p-plane_p[2*j]) < 0;
      bool is_in1 = plane_n[2*j+1].dot(target_p-plane_p[2*j+1]) < 0;
      is_inrange.push_back(is_in0 && is_in1);
      dist.push_back(fabs((target_p-plane_p[2*j]).dot(plane_n[2*j])));
      dist.push_back(fabs((target_p-plane_p[2*j+1]).dot(plane_n[2*j+1])));
    }
    for (int j = 0; j < 3; j++) {
      double dist_from_box = 0;
      dist_from_box += std::min(dist[2*j], dist[2*j+1]);
      if (!is_inrange[(j+1)%3]) {
        dist_from_box += std::min(dist[2*((j+1)%3)], dist[2*((j+1)%3)+1]);
      }
      if (!is_inrange[(j+2)%3]) {
        dist_from_box += std::min(dist[2*((j+2)%3)], dist[2*((j+2)%3)+1]);
      }
      if (dist_from_box < threshold && fabs(plane_n[2*j].dot(target_p-plane_p[2*j])) < ortho_eps_) {
        nr_p++;
        break;
      }
    }
  }
  double area = 0;
  for (int i = 0; i < 3; i++) {
    area += 2 * model_coefficients[12+i] * model_coefficients[12+((i+1)%3)];
  }
  if (nr_p < accept_point_ratio_ * area/(voxel_size_ * voxel_size_)) {
    return -INT_MAX;
  }
  return nr_p;
}

template<typename PointT, typename PointNT>
void pcl::SampleConsensusModelBox<PointT, PointNT>::projectPoints(
    const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients,
    PointCloud &projected_points, bool copy_data_fields) {
  PCL_ERROR("[pcl::SampleConsensusModelBox::projectPoints] Not implemented!\n");
}

template<typename PointT, typename PointNT>
bool pcl::SampleConsensusModelBox<PointT, PointNT>::doSamplesVerifyModel(
    const std::set<int> &indices, const Eigen::VectorXf &model_coefficients,
    const double threshold) {
  PCL_ERROR("[pcl::SampleConsensusModelBox::doSamplesVerifyModel] Not implemented!\n");
  return true;
}

template<typename PointT, typename PointNT>
bool pcl::SampleConsensusModelBox<PointT, PointNT>::isModelValid(
    const Eigen::VectorXf &model_coefficients) {
  return true;
}

template<typename PointT, typename PointNT>
bool pcl::SampleConsensusModelBox<PointT, PointNT>::isSampleGood(const std::vector<int> &samples) const {
  if (samples.size() != 3) return false;
  Eigen::Vector3f n0(normals_->points[samples[0]].normal[0], normals_->points[samples[0]].normal[1], normals_->points[samples[0]].normal[2]);
  Eigen::Vector3f n1(normals_->points[samples[1]].normal[0], normals_->points[samples[1]].normal[1], normals_->points[samples[1]].normal[2]);
  Eigen::Vector3f n2(normals_->points[samples[2]].normal[0], normals_->points[samples[2]].normal[1], normals_->points[samples[2]].normal[2]);
  if (fabs(n0.dot(n1)) > ortho_eps_) return false;
  if (fabs(n0.dot(n2)) > ortho_eps_) return false;
  if (fabs(n1.dot(n2)) > ortho_eps_) return false;

  return true;
}

template<typename PointT, typename PointNT>
void pcl::SampleConsensusModelBox<PointT, PointNT>::reComputeModelCoefficients(Eigen::VectorXf &model_coefficients) {
  std::vector<Eigen::Vector3f> n(3);
  for (int i = 0; i < 3; i++) {
    n[i] << model_coefficients[3*i], model_coefficients[3*i+1], model_coefficients[3*i+2];
  }

  std::vector<int> inliers;
  selectWithinDistance(model_coefficients, 0.0025, inliers);
  std::vector<float> min_len(3, std::numeric_limits<float>::max());
  std::vector<float> max_len(3, -std::numeric_limits<float>::max());
  for (int i = 0; i < inliers.size(); i++) {
    Eigen::Vector3f target_p(input_->points[inliers[i]].x, input_->points[inliers[i]].y, input_->points[inliers[i]].z);
    for (int j = 0; j < 3; j++) {
      double l = n[j].dot(target_p);
      if (l < min_len[j]) {
        min_len[j] = l;
      }
      if (l > max_len[j]) {
        max_len[j] = l;
      }
    }
  }
  Eigen::Vector3f center = n[0] * ((min_len[0] + max_len[0])/2.0) + n[1] * ((min_len[1] + max_len[1])/2.0) + n[2] * ((min_len[2] + max_len[2])/2.0);
  for (int i = 0; i < 3; i++) {
    model_coefficients[9+i] = center[i];
    model_coefficients[12+i] = max_len[i] - min_len[i];
  }
}

#endif

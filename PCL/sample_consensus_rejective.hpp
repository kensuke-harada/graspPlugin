/**
 * @file   sample_consensus_rejective.hpp
 * @author Akira Ohchi
*/

#ifndef _PCL_SAMPLE_CONSENSUS_REJECTIVE_HPP_
#define _PCL_SAMPLE_CONSENSUS_REJECTIVE_HPP_

template<typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
void pcl::SampleConsensusRejective<PointSource, PointTarget, FeatureT, NormalT>::setSourceNormals(const NormalCloudConstPtr& normals) {
	if (normals == NULL || normals->empty()) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::setSourceNormals] Invalid or empty point cloud dataset given!\n");
	}
	input_normals_ = normals;
}

template<typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
void pcl::SampleConsensusRejective<PointSource, PointTarget, FeatureT, NormalT>::setTargetNormals(const NormalCloudConstPtr& normals) {
	if (normals == NULL || normals->empty()) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::setTargetNormals] Invalid or empty point cloud dataset given!\n");
	}
	target_normals_ = normals;
}

template <typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
void pcl::SampleConsensusRejective<PointSource, PointTarget, FeatureT, NormalT>::computeTransformation(PointCloudSource &output, const Eigen::Matrix4f& guess) {
  // Some sanity checks first
  if (!input_features_) {
    PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("No source features were given! Call setSourceFeatures before aligning.\n");
    return;
  }
  if (!target_features_) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("No target features were given! Call setTargetFeatures before aligning.\n");
    return;
  }

	if (!input_normals_) {
    PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("No source normals were given! Call setSourceNormals before aligning.\n");
    return;
  }
  if (!target_normals_) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("No target normals were given! Call setTargetNormals before aligning.\n");
    return;
  }

  if (input_->size () != input_features_->size ()) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("The source points and source feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               input_->size (), input_features_->size ());
    return;
  }

  if (target_->size () != target_features_->size ()) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("The target points and target feature points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               target_->size (), target_features_->size ());
    return;
  }

	if (input_->size () != input_normals_->size ()) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("The source points and source normal points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               input_->size (), input_normals_->size ());
    return;
  }

  if (target_->size () != target_normals_->size ()) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("The target points and target normal points need to be in a one-to-one relationship! Current input cloud sizes: %ld vs %ld.\n",
               target_->size (), target_normals_->size ());
    return;
  }

  if (inlier_fraction_ < 0.0f || inlier_fraction_ > 1.0f) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("Illegal inlier fraction %f, must be in [0,1]!\n",
               inlier_fraction_);
    return;
  }
  
  const float similarity_threshold = correspondence_rejector_poly_->getSimilarityThreshold ();
  if (similarity_threshold < 0.0f || similarity_threshold >= 1.0f) {
		PCL_ERROR ("[pcl::SampleConsensusRejective::computeTransformation] ");
    PCL_ERROR ("Illegal prerejection similarity threshold %f, must be in [0,1[!\n",
               similarity_threshold);
    return;
  }
  
  // Initialize prerejector (similarity threshold already set to default value in constructor)
  correspondence_rejector_poly_->setInputSource(input_);
  correspondence_rejector_poly_->setInputTarget(target_);
  correspondence_rejector_poly_->setCardinality(nr_samples_);
  int num_rejections = 0; // For debugging
  
  // Initialize results
  final_transformation_ = guess;
  inliers_.clear();
  float highest_inlier_fraction = inlier_fraction_;
  converged_ = false;
  
  // Temporaries
  std::vector<int> inliers, insiders, flip_inliers;
  float inlier_fraction;
  float error;
  
  // If guess is not the Identity matrix we check it
  if (!guess.isApprox(Eigen::Matrix4f::Identity (), 0.01f)) {
    //getFitness(inliers, insiders, flip_inliers);
		bool is_flip;
		getFitness(inliers, insiders, is_flip);
    inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
    
    if (inlier_fraction > highest_inlier_fraction) {
      inliers_ = inliers;
			insiders_ = insiders;
      highest_inlier_fraction = inlier_fraction;
      converged_ = true;
    }
  }

#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
	std::vector<std::vector<int> > similar_features(input_->size());
#endif
  // Start
  for (int i = 0; i < max_iterations_; ++i) {
    // Temporary containers
    std::vector<int> sample_indices(nr_samples_);
    std::vector<int> corresponding_indices(nr_samples_);
    
    // Draw nr_samples_ random samples
    SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::
            selectSamples(*input_, nr_samples_, sample_indices);

    // Find corresponding features in the target cloud
#if PCL_VERSION_COMPARE(>=, 1, 7, 2)
		SampleConsensusPrerejective<PointSource, PointTarget, FeatureT>::findSimilarFeatures(sample_indices, similar_features, corresponding_indices);
#else
    findSimilarFeatures(*input_features_, sample_indices, corresponding_indices);
#endif
    
    // Apply prerejection
    if (!correspondence_rejector_poly_->thresholdPolygon(sample_indices, corresponding_indices)) {
      ++num_rejections;
      continue;
    }

    // Estimate the transform from the correspondences, write to transformation_
    transformation_estimation_->estimateRigidTransformation(*input_, sample_indices, *target_, corresponding_indices, transformation_);

		// Get inliers (points on surface of the model), insiders (points inside the model), and flip_inliers (points with normal vector whose direction is the opposite to the model's normal)
    // getFitness(inliers, insiders, flip_inliers);
		bool is_flip;
		getFitness(inliers, insiders, is_flip);
		if (inliers.empty()) {
			continue;
		}
		// if (static_cast<float>(flip_inliers.size()) / static_cast<float>(inliers.size()) > reject_flip_normal_fraction_) {
		// 	++num_rejections;
		// 	continue;
		// }
		// if (is_flip) {
		// 	++num_rejections;
		// 	continue;
		// }

		if (static_cast<float>(insiders.size()) / static_cast<float>(inliers.size()) > reject_inside_inlier_ratio_) {
			++num_rejections;
			continue;
		}
		
    // Take a backup of previous result
    const Matrix4 final_transformation_prev = final_transformation_;
    
    // Set final result to current transformation
    final_transformation_ = transformation_;



				std::cout << "inliers:" << inliers.size() << " insider:" << insiders.size()  << std::endl;
    // If the new fit is better, update results
    const float inlier_fraction = static_cast<float> (inliers.size ()) / static_cast<float> (input_->size ());
    if (inlier_fraction > highest_inlier_fraction) {
      inliers_ = inliers;
			insiders_ = insiders;
      highest_inlier_fraction = inlier_fraction;
      converged_ = true;
    } else {
      // Restore previous result
      final_transformation_ = final_transformation_prev;
    }
  }

  // Apply the final transformation
  if (converged_)
    transformPointCloud (*input_, output, final_transformation_);
  
  // Debug output
  PCL_DEBUG("[pcl::SampleConsensusRejective::computeTransformation] Rejected %i out of %i generated pose hypotheses.\n",
            num_rejections, max_iterations_);
}

template <typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
void  pcl::SampleConsensusRejective<PointSource, PointTarget, FeatureT, NormalT>::getFitness(std::vector<int>& inliers, std::vector<int>& insiders,
																																														 bool& is_flip) {
  // Initialize variables
  inliers.clear();
  inliers.reserve(input_->size());
	insiders.clear();
	insiders.reserve(input_->size());
	// flip_inliers.clear();
	// flip_inliers.reserve(input_->size());

	is_flip = false;
	// Transform the input dataset using the transformation
  PointCloudSource input_transformed;
  input_transformed.resize(input_->size ());
  transformPointCloud(*input_, input_transformed, transformation_);
	
	getFitnessCylinder(inliers, insiders, is_flip);
	getFitnessBox(inliers, insiders, is_flip);
}

template <typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
void  pcl::SampleConsensusRejective<PointSource, PointTarget, FeatureT, NormalT>::
getFitnessCylinder(std::vector<int>& inliers, std::vector<int>& insiders, bool& is_flip) {
	const double flip_criterion = cos(3.0/4.0 * M_PI);
	for (size_t i = 0; i < cylinders_.size(); i++) {
		Eigen::Matrix4f trans = transformation_;
		cnoid::Vector3 trans_dir = trans.block<3, 3>(0, 0).cast<double>() * cylinders_[i].dir;
		cnoid::Vector3 trans_pos = trans.block<3, 3>(0, 0).cast<double>() * cylinders_[i].pos + trans.block<3, 1>(0, 3).cast<double>();
		
		cnoid::Vector3 sum_n(0.0, 0.0, 0.0);
		cnoid::Vector3 sum_model_n(0.0, 0.0, 0.0);
		for (size_t j = 0; j < target_->size(); j++) {
			// inside check
			cnoid::Vector3 p(target_->at(j).x, target_->at(j).y, target_->at(j).z);
			double len_dir = (p - trans_pos).dot(trans_dir);
			if (fabs(len_dir) > cylinders_[i].length/2.0 + corr_dist_threshold_) continue;
			double len_rad = ((trans_pos + len_dir * trans_dir) - p).norm();
			if (len_rad > cylinders_[i].radius + corr_dist_threshold_) continue;
			insiders.push_back(j);

			// inlier check
			cnoid::Vector3 n(target_normals_->at(j).normal_x, target_normals_->at(j).normal_y, target_normals_->at(j).normal_z);
			bool on_side = (fabs(fabs(len_dir) - cylinders_[i].length/2.0) < corr_dist_threshold_);
			bool on_plane = (fabs(len_rad - cylinders_[i].radius) < corr_dist_threshold_);
			double th = cos(0.50*M_PI);
			bool is_inlier = false;
			if (on_side) {
				cnoid::Vector3 n_side = (p - (trans_pos + len_dir * trans_dir)).normalized();
				if (n_side.dot(n) > th) is_inlier = true;
			}
			if (on_plane) {
				int sign = (len_dir > 0) ? 1 : -1;
				if ((sign * trans_dir).dot(n) > th) is_inlier = true;
			}
			if (is_inlier) {
				inliers.push_back(j);
			}
			// if (on_side || on_plane) {
			// 	inliers.push_back(j);
			// } else {
			// 	continue;
			// }

			// cnoid::Vector3 n(target_normals_->at(j).normal_x, target_normals_->at(j).normal_y, target_normals_->at(j).normal_z);
			// sum_n += n;

			// if (on_side) {
			// 	sum_model_n += (p - (trans_pos + len_dir * trans_dir)).normalized();
			// } else if (on_plane) {
			// 	int sign = (len_dir > 0) ? 1 : -1;
			// 	sum_model_n += sign * trans_dir;
			// }
			// // flip check
			// bool is_flip = true;
			// cnoid::Vector3 n(target_normals_->at(j).normal_x, target_normals_->at(j).normal_y, target_normals_->at(j).normal_z);
			// if (on_side) {
			// 	cnoid::Vector3 n_side = (p - (trans_pos + len_dir * trans_dir)).normalized();
			// 	is_flip &= ((n_side).dot(n) < flip_criterion) ;
			// }
			// if (on_plane) {
			// 	int sign = (len_dir > 0) ? 1 : -1;
			// 	is_flip &= ((sign * trans_dir).dot(n) < flip_criterion);
			// }
			// if (is_flip) {
			// 	flip_inliers.push_back(j);
			// }
		}
		// if (!inliers.empty()) {
		// 	sum_n /= inliers.size();
		// 	sum_model_n /= inliers.size();
		// 	sum_n.normalize();
		// 	sum_model_n.normalize();
		// 	if (sum_n.dot(sum_model_n) < 0) is_flip = true;
		// }
	}
}

template <typename PointSource, typename PointTarget, typename FeatureT, typename NormalT>
void  pcl::SampleConsensusRejective<PointSource, PointTarget, FeatureT, NormalT>::
getFitnessBox(std::vector<int>& inliers, std::vector<int>& insiders, bool& is_flip) {

}


#endif /* _PCL_SAMPLE_CONSENSUS_REJECTIVE_HPP_ */

















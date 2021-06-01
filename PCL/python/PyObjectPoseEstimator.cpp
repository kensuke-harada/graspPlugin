#include "../ObjectPoseEstimator.h"
#include "../Segmenter.h"

#include <cnoid/PyBase>
#include <boost/python.hpp>

namespace {
	bool capture(ObjectPoseEstimator& self) {
		return self.readScene(false, false, "", "", false);
	}

	bool capture_no_trans(ObjectPoseEstimator& self) {
		return self.readScene(false, false, "", "", cnoid::Vector3::Zero(), cnoid::Matrix3::Identity(), false);
	}

	bool capture_and_save(ObjectPoseEstimator& self, const std::string& save_filepath) {
		return self.readScene(false, true, "", save_filepath, false);
	}

	bool load(ObjectPoseEstimator& self, const std::string& load_filepath) {
		return self.readScene(true, false, load_filepath, "", false);
	}

	bool load_no_trans(ObjectPoseEstimator& self, const std::string& load_filepath) {
		return self.readScene(true, false, load_filepath, "", cnoid::Vector3::Zero(), cnoid::Matrix3::Identity(), false);
	}

	void downsample(ObjectPoseEstimator& self) {
		self.downsampling(true);
	}

	void downsample_without_cropping(ObjectPoseEstimator& self) {
		self.downsampling(false);
	}

	void set_target_region1(ObjectPoseEstimator& self, double x_max, double x_min, double y_max, double y_min, double z_max, double z_min) {
		self.setTargetRegion(x_max, x_min, y_max, y_min, z_max, z_min);
	}

	void set_target_region2(ObjectPoseEstimator& self, const cnoid::Vector3& min, const cnoid::Vector3& max) {
		self.setTargetRegion(min, max);
	}
}

void exportObjectPoseEstimator() {
	using namespace boost::python;

	class_<ObjectPoseEstimator>("ObjectPoseEstimator")
		.def("capture", capture)
		.def("capture_no_trans", capture_no_trans)
		.def("capture_and_save", capture_and_save)
		.def("load", load)
		.def("load_no_trans", load_no_trans)
		.def("load_object", &ObjectPoseEstimator::readObject)
		.def("set_target_region", set_target_region1)
		.def("set_target_region", set_target_region2)
		.def("set_voxel_leaf", &ObjectPoseEstimator::setVoxelLeaf)
		.def("downsample", downsample)
		.def("downsample_without_cropping", downsample_without_cropping)
		.def("get_scene_cloud", &ObjectPoseEstimator::getSceneCloud)
		.def("get_sampled_scene_cloud", &ObjectPoseEstimator::getSampledSceneCloud)
		.def("get_object_cloud", &ObjectPoseEstimator::getObjCloud)
		.def("get_sampled_object_cloud", &ObjectPoseEstimator::getSampledObjCloud)
		.def("set_segmenter", &ObjectPoseEstimator::setSegmenter)
		.def("segment", &ObjectPoseEstimator::segment)
		.def("get_clustered_cloud", &ObjectPoseEstimator::getClusteredCloud)
		;
}

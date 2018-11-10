#include "../LearningDataManipulator.h"

#include <cnoid/PyBase>

using namespace boost::python;
using namespace grasp;

void exportLearningDataManipulator() {
	class_<GraspReconstractor, GraspReconstractor*>("GraspReconstractor", no_init)
		.def("instance", &GraspReconstractor::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
		.def("reconstract", &GraspReconstractor::reconstract)
		.def("showPointCloud", &GraspReconstractor::showPointCloud)
		.def("computeInsidePoints", &GraspReconstractor::computeInsidePoints)
		;
}

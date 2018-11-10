#include "../Segmenter.h"

#include <cnoid/PyBase>

void exportSegmenter() {
	using namespace boost::python;

	class_<AbstractSegmenter, boost::noncopyable>("AbstractSegmenter", no_init)
		.def("set_view_point", &AbstractSegmenter::setViewPoint)
		.def("set_remove_plane_num", &AbstractSegmenter::setRemovePlaneNum)
		.def("set_tolerance", &AbstractSegmenter::setTolerance)
		.def("set_vertical_tolerance", &AbstractSegmenter::setVerticalTolerance)
		;

	class_<Segmenter, bases<AbstractSegmenter> >("Segmenter");

	class_<ConditionalSegmenter, bases<AbstractSegmenter> >("ConditionalSegmenter");

	class_<LccpSegmenter, bases<AbstractSegmenter> >("LccpSegmenter")
		.def("set_super_voxel_parameter", &LccpSegmenter::setSuperVoxelParameter)
		.def("set_min_cluster_size", &LccpSegmenter::setMinClusterSize)
		.def("set_concavity_tolerance", &LccpSegmenter::setConcavityTolerance)
		;
}

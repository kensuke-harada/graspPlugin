#include "../PointCloudHolder.h"

#include <cnoid/PyBase>

namespace {
	bool capture(PointCloudHolder& self) {
		return self.capture(false, false, "", "", false, false);
	}

	bool capture_and_save(PointCloudHolder& self, const std::string& save_filepath) {
		return self.capture(false, true, "", save_filepath, false, false);
	}

	bool load(PointCloudHolder& self, const std::string& load_filepath) {
		return self.capture(true, false, load_filepath, "", false, false);
	}
}

void exportPointCloudHolder() {
	using namespace boost::python;

	class_<PointCloudT>("PointCloudT");
	class_<ColorPointCloud>("ColorPointCloud");

	register_ptr_to_python<PointCloudTPtr>();
	register_ptr_to_python<PointCloudTConstPtr>();
	register_ptr_to_python<ColorPointCloudPtr>();
	register_ptr_to_python<ColorPointCloudConstPtr>();

	class_<PointCloudHolder, boost::noncopyable>("PointCloudHolder", no_init)
		.def("instance", &PointCloudHolder::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
		.def("capture", capture)
		.def("capture_and_save", capture_and_save)
		.def("load", load)
		.def("get_cloud", &PointCloudHolder::getCloudPtr)
		.def("get_color_cloud", &PointCloudHolder::getColorCloudPtr)
		;
}

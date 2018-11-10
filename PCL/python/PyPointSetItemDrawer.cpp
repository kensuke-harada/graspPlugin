#include "../PointSetItemDrawer.h"

#include <cnoid/PyBase>

#include <iostream>

namespace {
	void add_pointset(PointSetItemDrawer& self, PointCloudTPtr cloud, const std::string& item_name) {
		self.addPointSet(cloud, item_name);
	}

	void add_pointset_color(PointSetItemDrawer& self, ColorPointCloudPtr cloud, const std::string& item_name) {
		self.addPointSet(cloud, item_name);
	}
}

void exportPointSetItemDrawer() {
	using namespace boost::python;

	class_<PointSetItemDrawer>("PointSetItemDrawer", no_init)
		.def("instance", &PointSetItemDrawer::instance, return_value_policy<reference_existing_object>()).staticmethod("instance")
		.def("add_pointset", add_pointset)
		.def("add_pointset", add_pointset_color)
		.def("clear", &PointSetItemDrawer::clear)
		;
}

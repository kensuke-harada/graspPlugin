#include "../PickingTaskResultDrawer.h"

#include <cnoid/PyBase>

using namespace boost::python;
using namespace grasp;

void exportPickingTaskResultDrawer() {
	class_<PickingTaskResultDrawer, PickingTaskResultDrawer*>("PickingTaskResultDrawer")
		.def("showCurrGrid", &PickingTaskResultDrawer::showCurrGrid).staticmethod("showCurrGrid")
		.def("showMergePoints", &PickingTaskResultDrawer::showMergePoints).staticmethod("showMergePoints")
		.def("clearAll", &PickingTaskResultDrawer::clearAll).staticmethod("clearAll")
		;
}

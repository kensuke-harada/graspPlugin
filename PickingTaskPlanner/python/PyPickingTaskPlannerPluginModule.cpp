#include <cnoid/PyUtil>

void exportPickingTaskResultDrawer();
void exportSceneCapture();
void exportResultImporter();

BOOST_PYTHON_MODULE(PickingTaskPlannerPlugin) {
	boost::python::import("cnoid.Base");

	exportPickingTaskResultDrawer();
	exportSceneCapture();
	exportResultImporter();
}

#include <cnoid/PyUtil>

void exportPointCloudHolder();
void exportObjectPoseEstimator();
void exportSegmenter();
void exportPointSetItemDrawer();

BOOST_PYTHON_MODULE(GraspPCLPlugin) {
	boost::python::import("cnoid.Base");

	exportPointCloudHolder();
	exportObjectPoseEstimator();
	exportSegmenter();
	exportPointSetItemDrawer();
}

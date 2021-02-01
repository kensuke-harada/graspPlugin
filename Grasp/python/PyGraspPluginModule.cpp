#include <cnoid/PyUtil>
#include <boost/python.hpp>

void exportPlanBase();
void exportGrasp();
void exportAssemblyObject();

BOOST_PYTHON_MODULE(GraspPlugin) {
	boost::python::import("cnoid.Base");
	boost::python::import("cnoid.Body");
	boost::python::import("cnoid.BodyPlugin");

	exportPlanBase();
	exportGrasp();
	exportAssemblyObject();
}

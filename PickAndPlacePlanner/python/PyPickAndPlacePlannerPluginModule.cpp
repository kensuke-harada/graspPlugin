#include <cnoid/PyUtil>

void exportLearningDataManipulator();

BOOST_PYTHON_MODULE(PickAndPlacePlannerPlugin){
	boost::python::import("cnoid.Base");

	exportLearningDataManipulator();
}

#include <cnoid/PyUtil>
#include <boost/python.hpp>

void exportLearningDataManipulator();

BOOST_PYTHON_MODULE(PickAndPlacePlannerPlugin){
	boost::python::import("cnoid.Base");

	exportLearningDataManipulator();
}

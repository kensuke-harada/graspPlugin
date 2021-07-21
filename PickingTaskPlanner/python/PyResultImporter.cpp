#include "../ResultDataManipulator.h"

#include <cnoid/PyBase>

using namespace grasp;

void exportResultImporter() {
	using namespace boost::python;
	class_<ResultDataImporter>("ResultDataImporter")
		.def("import_data", &ResultDataImporter::importLastView)
		;
}

#include "../AssemblyObject.h"

#include <cnoid/PyBase>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

using namespace boost::python;
using namespace grasp;

void exportAssemblyObject() {
	class_<AssemblyObject, AssemblyObjectPtr, boost::noncopyable>("AssemblyObject", no_init)
		;

	class_<std::vector<AssemblyObject*> >("assembly_object_vector")
		.def(vector_indexing_suite<std::vector<AssemblyObject*> >())
		;
}

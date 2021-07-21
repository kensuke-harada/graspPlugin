#include "../SceneCapture.h"

#include <cnoid/PyBase>

using namespace boost::python;
using namespace grasp;

void exportSceneCapture() {
	def("captureScene", captureScene);
}

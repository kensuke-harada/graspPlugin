/*! @file
  @author Tokuo Tsuji
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/MenuManager>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/SceneView>	/* modified by qtconv.rb 0th rule*/
//#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/

#include "RobotInterfaceBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
#ifdef OPENRTM112
#include "ArmControllerRtc1_1_2/ArmController.h"
#else
#include "ArmControllerRtc/ArmController.h"
#endif
#include "ArmControllerRtc.h"
using namespace grasp;

class RobotInterfacePlugin : public cnoid::Plugin
{
private:

//	SceneView* sceneView;

	bool onTimeout() {

		return true;
	}

public:

	RobotInterfacePlugin() : Plugin("RobotInterface") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Grasp");
#else
		require("Grasp");
#endif
	}

	virtual bool initialize() {
		addToolBar(RobotInterfaceBar::instance());
		ArmControllerRtc::instance()->RtcStart();

		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(RobotInterfacePlugin);

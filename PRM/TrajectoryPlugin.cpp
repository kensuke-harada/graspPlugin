/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include "TrajectoryBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;


class TrajectoryPlugin : public cnoid::Plugin
{
private:

//	SceneView* sceneView;
	
	bool onTimeout() {
	   
		return true;
	}

public:
	
	TrajectoryPlugin() : Plugin("Trajectory") { 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Grasp");
#else
		require("Grasp");
#endif
	}
	
	virtual bool initialize() {
	
		addToolBar(TrajectoryBar::instance());

			
		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(TrajectoryPlugin);

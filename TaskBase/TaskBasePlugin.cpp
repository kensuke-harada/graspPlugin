/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include "TaskBaseBar.h"
#include "Action.h"
#include "GraspPoints.h"

using namespace std;
using namespace boost;
using namespace cnoid;
//using namespace grasp;


class TaskBasePlugin : public cnoid::Plugin
{
private:

//	SceneView* sceneView;
	bool onTimeout() {
	   
		return true;
	}

public:
	
	TaskBasePlugin() : Plugin("TaskBase") { 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		depend("Grasp");
		depend("Trajectory");
#else
		require("Grasp");
		require("Trajectory");
#endif
	}
	
	virtual bool initialize() {
		
        grasp::ActionItem::initializeClass(this);
        grasp::TaskItem::initializeClass(this);
				grasp::GraspPointsItem::initializeClass(this);
		addToolBar(grasp::TaskBaseBar::instance());
			
		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(TaskBasePlugin);

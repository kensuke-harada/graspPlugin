// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
  c) Kensuke Harada (AIST)
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MenuManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SceneView>	/* modified by qtconv.rb 0th rule*/  

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneView>
#endif
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <osg/Camera>
#include <osg/Geode>
#include <osg/GLExtensions>
#include <osgDB/ReadFile>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  
#endif

#include "ManipBar.h"
#include "ManipController.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

//#include "ManipRtc.h"

Vector3 objVisPos;
Matrix3 objVisRot;
double objMass;
string bodywrlname;

class ManipPlugin : public cnoid::Plugin
{
private:

	//SceneView* sceneView;
	
	bool onTimeout() {
		
		return true;
	}
	
public:
	
	ManipPlugin() : Plugin("PickAndPlacePlanner") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Grasp");
#else
		require("Grasp");
#endif
	}
	
	virtual bool initialize() {
		//Robotics::SceneBodyManager* manager = Robotics::SceneBodyManager::instance();
		grasp::GraspController::instance(ManipController::instance());
		//ManipRtc::instance()->RtcStart();
		//manage(manager->addSceneBodyFactory(create));
		
		addToolBar(ManipBar::instance());
		
		return true;
	}
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ManipPlugin);

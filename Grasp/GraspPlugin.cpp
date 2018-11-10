/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  

#include "PlanBase.h"
#ifdef CNOID_ENABLE_OSG
#include "GraspSceneBody.h"
#endif
#include "GraspBar.h"
#include "PlaceController.h"
#include "PrehensionFileMenu.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "GraspEventEater.h"
#include "SceneWidgetEventEaterManager.h"
#endif


using namespace std;
using namespace cnoid;
using namespace grasp;



/*
bool GraspSceneBody::onKeyPressEvent(const SceneViewEvent& event)
{
	bool handled = true;

	switch(std::toupper(event.key())){
	case 'G':
		cout << "test " << endl;

	default:
		handled = false;
	return cnoid::SceneBody::onKeyPressEvent(event);
		break;
	}
		
	return handled;
}
*/

#ifdef CNOID_ENABLE_OSG

bool GraspSceneBody::onButtonPressEvent(const OSGSceneViewEvent& event) {
    bool handled = false;

	if(event.button() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON){	/* modified by qtconv.rb 3rd rule*/  
		
		Vector3 pressPos = cnoid::Vector3(event.point().x(),event.point().y(),event.point().z());
		Vector3 normal = cnoid::Vector3(event.normal().x(),event.normal().y(),event.normal().z());
		
		PlaceController::instance()->setTargetPose(pressPos,normal);
			
		cout << pressPos << endl;
		cout << normal << endl;
	}
    return cnoid::OSGSceneBody::onButtonPressEvent(event);
}


cnoid::OSGSceneBody* GraspSceneBody::create(cnoid::BodyItem* item)
{
	return new GraspSceneBody(item);
}

#endif

class GrasplotPlugin : public cnoid::Plugin
{
private:

#ifdef CNOID_ENABLE_OSG
    OSGSceneView* sceneView;
#endif

	bool onTimeout() {
	   
		return true;
	}

public:
	
	GrasplotPlugin() : Plugin("Grasp") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Body");
#else
		require("Body");
#endif
	}
	
	virtual bool initialize() {

		Prehension::initializeClass(this);
		ArmFingers::initializeClass(this);
#ifdef CNOID_ENABLE_OSG
        cnoid::OSGSceneBodyManager* manager = cnoid::OSGSceneBodyManager::instance();
		manage(manager->addSceneBodyFactory(GraspSceneBody::create));
#endif
		addToolBar(GraspBar::instance());
		initializePrehensionFileLoader(*this);

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
		SceneWidgetEventEaterManager::instance()->initialize();
		GraspEventEater* eater = new GraspEventEater();
		SceneWidgetEventEaterManager::instance()->addEater(eater);
#endif

		return true;
	}

	virtual bool finalize() {
		PlanBase::instance()->finalize();
		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(GrasplotPlugin);

/**
   c) Tokuo Tsuji (Kyushu univ./AIST)
*/

#include "VisionTriggerBar.h"
//#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  

//#include <ExcadePlugins/Grasp/GraspController.h>
//#include <ExcadePlugins/Grasp/TrajectoryPlanner.h>

#include <VisionTriggerRtcController.h>

#include "ObjectRecognitionResultManipulator.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

VisionTriggerBar* VisionTriggerBar::instance()
{
	static VisionTriggerBar* instance = new VisionTriggerBar();
	return instance;
}

VisionTriggerBar::VisionTriggerBar()
	: ToolBar("VisionTriggerBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=VisionRTC="));

	addButton(("Start"), ("RTC start"))->
		sigClicked().connect(bind(&VisionTriggerBar::onStartButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Stop"), ("RTC stop"))->
		sigClicked().connect(bind(&VisionTriggerBar::onStopButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addSeparator();

	addButton(("Ext ON/OFF"), ("switch extended mode"))->
		sigClicked().connect(bind(&VisionTriggerBar::onExtStartButtonClicked, this));

	// show_all_children();	/* modified by qtconv.rb 7th rule*/  

}


VisionTriggerBar::~VisionTriggerBar()
{
//	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}

void VisionTriggerBar::onStartButtonClicked()
{
	bool success = VisionTriggerRtcController::instance()->VisionRecoginitionStart();
	if(!success){
		os << "ERROR: Settings " << endl;
	}
}

void VisionTriggerBar::onStopButtonClicked()
{
	VisionTriggerRtcController::instance()->stopFlag = true;
	os <<  "VisionTriggerStopped" << endl;	
}

void VisionTriggerBar::onExtStartButtonClicked()
{
	bool state = VisionTriggerRtcController::instance()->switchExtModeFlag();
	os << "VisionTriger Ext mode " << ((state) ? "ON" : "OFF") << endl;
}

bool VisionTriggerBar::storeState(Archive& archive)
{
	return true;
}


bool VisionTriggerBar::restoreState(const Archive& archive)
{
	return true;
}

/**
   @author Shin'ichiro Nakaoka
*/

#include "GraspRtcBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  

//#include <ExcadePlugins/Grasp/GraspController.h>
//#include <ExcadePlugins/Grasp/TrajectoryPlanner.h>

#include <GraspRtcController.h>
#include "GraspRtcBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

GraspRtcBar* GraspRtcBar::instance()
{
	static GraspRtcBar* instance = new GraspRtcBar();
	return instance;
}

GraspRtcBar::GraspRtcBar()
	: ToolBar("GraspRtcBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=GraspRTC="));

	addButton(("Grasp"), ("RTC start"))->
		sigClicked().connect(bind(&GraspRtcBar::onStartButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Release"), ("RTC start"))->
		sigClicked().connect(bind(&GraspRtcBar::onStartButtonClicked2, this));	/* modified by qtconv.rb 6th rule*/  
	
	addButton(("Stop"), ("RTC stop"))->
		sigClicked().connect(bind(&GraspRtcBar::onStopButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	// show_all_children();	/* modified by qtconv.rb 7th rule*/  

}


GraspRtcBar::~GraspRtcBar()
{

}


void GraspRtcBar::onStartButtonClicked()
{
	bool success = GraspRtcController::instance()->graspPlanStart(1);
	if(!success){
		os << "ERROR: Settings " << endl;
	}
}

void GraspRtcBar::onStartButtonClicked2()
{
	bool success = GraspRtcController::instance()->graspPlanStart(0);
	if(!success){
		os << "ERROR: Settings " << endl;
	}
}

void GraspRtcBar::onStopButtonClicked()
{
//	GraspController::instance()->stopFlag = true;
	os <<  "NOT implemented" << endl;	
}



bool GraspRtcBar::storeState(Archive& archive)
{
	return true;
}


bool GraspRtcBar::restoreState(const Archive& archive)
{
	return true;
}

/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "TrajectoryBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  

#include "../Grasp/GraspController.h"
#include "ParamDialog.h"
//#include "TrajectoryPlanner.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

TrajectoryBar* TrajectoryBar::instance()
{
	static TrajectoryBar* instance = new TrajectoryBar();
	return instance;
}

TrajectoryBar::TrajectoryBar()
	: ToolBar("TrajectoryBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{
	
	addSeparator();
	
	addLabel(("=PathPlan="));

	addButton(("Start"), ("Path planning start"))->
		sigClicked().connect(bind(&TrajectoryBar::onTrajectoryPlanButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Reset"), ("motion states reset"))->
		sigClicked().connect(bind(&TrajectoryBar::onResetButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
	
	addButton(("setStartState"), ("Set start Motion"))->
		sigClicked().connect(bind(&TrajectoryBar::onSetStartMotionStateButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  
	
	addButton(("setEndState"), ("Set End Motion"))->
		sigClicked().connect(bind(&TrajectoryBar::onSetEndMotionStateButtonClicked, this));	/* modified by qtconv.rb 6th rule*/  

	addButton(("Param"), ("parameter setting"))->
		sigClicked().connect(bind(&TrajectoryBar::onParameterButtonClicked, this));
	
	addSeparator();

	// show_all_children();	/* modified by qtconv.rb 7th rule*/  

}


TrajectoryBar::~TrajectoryBar()
{
//	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}




void TrajectoryBar::onTrajectoryPlanButtonClicked()
{
	if(!PlanBase::instance()->targetArmFinger){
		os << "error: Set Robot" << endl;
		return;
	}
	trajectoryPlanner_ = new TrajectoryPlanner();

	bool ret = trajectoryPlanner_->doTrajectoryPlanning();

	if(ret)
			os <<  "Trajectory Planning is finished" << endl;	
	else
			os <<  "Trajectory Planning is failed" << endl;	
}

void TrajectoryBar::onResetButtonClicked()
{
	PlanBase::instance()->graspMotionSeq.clear();
	PlanBase::instance()->startMotionState.id = -1;
	PlanBase::instance()->endMotionState.id = -1;
	os << "Motion states clear" << endl;
}


void TrajectoryBar::onSetStartMotionStateButtonClicked(){
	if(!PlanBase::instance()->targetArmFinger){
		os << "error: Set startMotionState" << endl;
		return;
	}
	PlanBase::instance()->graspMotionSeq.clear();
	PlanBase::instance()->startMotionState = PlanBase::instance()->getMotionState();
	PlanBase::instance()->startMotionState.id = 1;
}
void TrajectoryBar::onSetEndMotionStateButtonClicked(){
	if(!PlanBase::instance()->targetArmFinger){
		os << "error: Set endMotionState" << endl;
		return;
	}
	PlanBase::instance()->graspMotionSeq.clear();
	PlanBase::instance()->endMotionState = PlanBase::instance()->getMotionState();
	PlanBase::instance()->endMotionState.id = 2;
}

void TrajectoryBar::onParameterButtonClicked() {
	PRMParamDialog* paramDialog = new PRMParamDialog();
	paramDialog->exec();
	delete paramDialog;
}


bool TrajectoryBar::storeState(Archive& archive)
{
//	if(currentBodyItem_){
//	/	archive.writeItemId("current", currentBodyItem_);
//	}
	return true;
}


bool TrajectoryBar::restoreState(const Archive& archive)
{
//	if(!currentBodyItem_){
	//	currentBodyItem_ = archive.findItem<BodyItem>("current");
		//if(currentBodyItem_){
		//	if(targetBodyItems.empty()){
		//		targetBodyItems.push_back(currentBodyItem_);
			//}
			//sigCurrentBodyItemChanged_(currentBodyItem_.get());
		//}
//	}
	return true;
}

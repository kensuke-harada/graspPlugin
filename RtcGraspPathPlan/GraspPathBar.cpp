/**
   @author Shin'ichiro Nakaoka
*/

#include <time.h>

#include "GraspPathBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include "GraspPathController.h"
#include "GraspPathBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

GraspPathBar* GraspPathBar::instance()
{
	static GraspPathBar* instance = new GraspPathBar();
	return instance;
}

GraspPathBar::GraspPathBar()
	: ToolBar("GraspPathBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() )
{

	addSeparator();

	addLabel(("=PlanGraspPath="));

	addButton(("Grasp"), ("Grasp Path Plan start"))->
		sigClicked().connect(bind(&GraspPathBar::onStartButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("Release"), ("RTC start"))->
		sigClicked().connect(bind(&GraspPathBar::onStartButtonClicked2, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("Stop"), ("RTC stop"))->
		sigClicked().connect(bind(&GraspPathBar::onStopButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("createObjectTag"), ("caeateObjectTag"))->
		sigClicked().connect(bind(&GraspPathBar::onCreateRecordButtonClicked, this));	/* modified by qtconv.rb 6th rule*/
	// show_all_children();	/* modified by qtconv.rb 7th rule*/

	addButton(("deleteObjectTag"), ("deleteObjectTag"))->
		sigClicked().connect(bind(&GraspPathBar::onDeleteRecordButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("AppearObject"), ("appear"))->
		sigClicked().connect(bind(&GraspPathBar::onAppearButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("DisappearObject"), ("disappear"))->
		sigClicked().connect(bind(&GraspPathBar::onDisappearButtonClicked, this));	/* modified by qtconv.rb 6th rule*/


}


GraspPathBar::~GraspPathBar()
{

}

void GraspPathBar::onStartButtonClicked()
{

  clock_t start_ = clock();

	PlanBase *pb = PlanBase::instance();
	std::vector<pathInfo> trajectory;
	int state;
	std::vector<double> begin;
	std::vector<double> end;

	if( !pb->targetObject || !pb->robTag2Arm.size()) {
		os <<  "set object and robot" << endl;
		return;
	}
	for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as
        double q = pb->bodyItemRobot()->body()->joint(i)->q();
		begin.push_back(q);
		end.push_back(q);
	}
	GraspPathController::instance()->setTolerance(0.05);
	GraspPathController::instance()->graspPathPlanStart(0, begin, end, pb->targetArmFinger->name, pb->targetObject->name(), 50, &trajectory, &state);
//	GraspPathController::instance()->graspPathPlanStart(0, begin, end, (pb->robTag2Arm.begin())->first, pb->targetObject->name(), 50, &trajectory, &state);

	clock_t end_ = clock();
	cout << "It took "<< (double)(end_-start_)/CLOCKS_PER_SEC << "(s)" << endl;

	cout << "test" << endl;

}

void GraspPathBar::onStartButtonClicked2()
{
	PlanBase *pb = PlanBase::instance();
	std::vector<pathInfo> trajectory;
	int state;
	std::vector<double> begin;
	std::vector<double> end;

	if( !pb->targetObject || !pb->robTag2Arm.size()) {
		os <<  "set object and robot" << endl;
		return;
	}
	for(int i=0;i<pb->bodyItemRobot()->body()->numJoints();i++){ // If initial position is not collided, it is stored as
        double q = pb->bodyItemRobot()->body()->joint(i)->q();
		begin.push_back(q);
		end.push_back(q);
	}
	GraspPathController::instance()->setTolerance(0.05);
	GraspPathController::instance()->releasePathPlanStart(0, begin, end,  pb->targetArmFinger->name, pb->targetObject->name(), 50, &trajectory, &state);
}

void GraspPathBar::onStopButtonClicked()
{
//	GraspController::instance()->stopFlag = true;
	os <<  "NOT implemented" << endl;
}

void GraspPathBar::onCreateRecordButtonClicked(){

	GraspPathController::instance()->createRecord(1,"testTagCan");

}

void GraspPathBar::onDeleteRecordButtonClicked(){
	GraspPathController::instance()->deleteRecord("ALLID");
}

void GraspPathBar::onAppearButtonClicked(){
	GraspPathController::instance()->appear("testTagCan");
	GraspPathController::instance()->setPos("testTagCan",Vector3(1,0,1), Matrix3::Identity());
}

void GraspPathBar::onDisappearButtonClicked(){
	GraspPathController::instance()->disappear("ALLID");
}

bool GraspPathBar::storeState(Archive& archive)
{
	return true;
}


bool GraspPathBar::restoreState(const Archive& archive)
{
	return true;
}

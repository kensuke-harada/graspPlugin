/**
 * @file   PickingTaskPlannerBar.cpp
 * @author Akira Ohchi
*/

#include "PickingTaskPlannerBar.h"

#include <boost/bind.hpp>

#include <cnoid/MessageView>

#include "PickingTaskPlanner.h"
#include "PickingTaskResultDrawer.h"
#include "../PCL/ObjectPoseEstimatorInterface.h"

using namespace grasp;

PickingTaskPlannerBar* PickingTaskPlannerBar::instance() {
	static PickingTaskPlannerBar* instance = new PickingTaskPlannerBar();
	return instance;
}

PickingTaskPlannerBar::PickingTaskPlannerBar()
	: ToolBar("PickingTaskPlanner"),
	  mes(*cnoid::MessageView::mainInstance()),
		os(cnoid::MessageView::mainInstance()->cout()) {
	addSeparator();

	addLabel(("=PickingTaskPlanner="));

	addButton(("Start"), ("start bin picking"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onStartPickingClicked, this));

	addButton(("StartNoReject"), ("start bin picking without rejection"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onStartPickingNoRejectionClicked, this));

	addButton(("StartMultiPC"), ("start bin picking on multi PC"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onStartPickingMultiPCClicked, this));

	addButton(("Init"), ("initialize"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onInitClicked, this));

	addButton(("ShowCluster"), ("show cluster point clouds"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onShowClusterClicked, this));

	addButton(("ShowPrevCloud"), ("show previous captured point cloud"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onShowPrevClicked, this));

	addButton(("ShowCurGrid"), ("show current grid"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onShowCurrGridClicked, this));

	addButton(("ShowPrevGrid"), ("show previous grid"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onShowPrevGridClicked, this));

	addButton(("ShowMergedPoints"), ("show current points"))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onShowMergePointsClicked, this));

	addButton(("TriggerStart"), (""))->
		sigClicked().connect(boost::bind(&PickingTaskPlannerBar::onTriggerStartClicked, this));
}


PickingTaskPlannerBar::~PickingTaskPlannerBar() {
}

void PickingTaskPlannerBar::onStartPickingClicked() {
	PickingTaskPlanner::instance()->startPicking(false);
	os << "End Picking Task" << std::endl;
}

void PickingTaskPlannerBar::onStartPickingNoRejectionClicked() {
	PickingTaskPlanner::instance()->startPicking(false, true);
	os << "End Picking Task" << std::endl;
}

void PickingTaskPlannerBar::onStartPickingMultiPCClicked() {
	PickingTaskPlanner::instance()->startPicking(true);
	os << "End Picking Task" << std::endl;
}

void PickingTaskPlannerBar::onInitClicked() {
	PickingTaskPlanner::instance()->initialize();
}

void PickingTaskPlannerBar::onShowClusterClicked() {
	PoseEstimator::drawClusters();
}

void PickingTaskPlannerBar::onShowPrevClicked() {
	PoseEstimator::drawPrevAndCurrPointCloud();
}

void PickingTaskPlannerBar::onShowPrevGridClicked() {
	PickingTaskResultDrawer::showPrevGrid();
}

void PickingTaskPlannerBar::onShowCurrGridClicked() {
	PickingTaskResultDrawer::showCurrGrid();
}

void PickingTaskPlannerBar::onShowMergePointsClicked() {
	PickingTaskResultDrawer::showMergePoints();
}

void PickingTaskPlannerBar::onTriggerStartClicked() {
	PickingTaskPlanner::instance()->visionTriggerStart();
}


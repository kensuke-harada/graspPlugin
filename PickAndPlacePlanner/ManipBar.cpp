// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Kensuke Harada (AIST)
*/

#include "ManipBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/

#include "ManipController.h"
#include "ManipFailureAnalyzer.h"
#include "DualArmManipulation.h"
#include "SweptVolumeDialog.h"
#include "LearningDialog.h"
#include "../PRM/TrajectoryPlanner.h"


using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp::PickAndPlacePlanner;

StartDialog::StartDialog() : QDialog(cnoid::MainWindow::instance()){
	setWindowTitle("Select mode");

	QButtonGroup* bgrp = new QButtonGroup(this);
	bgrp->setExclusive(true);

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hbox = new QHBoxLayout();
	rb_default.setText("Default grasp planning");
	rb_default.setChecked(true);
	bgrp->addButton(&rb_default);
	hbox->addWidget(&rb_default);
	hbox->addStretch();
	vbox->addLayout(hbox);
	vbox->addStretch();

	if(PlanBase::instance()->armsList.size() > 1){
		hbox = new QHBoxLayout();
		rb_right.setText("Grasp with right hand");
		bgrp->addButton(&rb_right);
		hbox->addWidget(&rb_right);
		hbox->addStretch();
		vbox->addLayout(hbox);

		hbox = new QHBoxLayout();
		rb_left.setText("Grasp with left hand");
		bgrp->addButton(&rb_left);
		hbox->addWidget(&rb_left);
		hbox->addStretch();
		vbox->addLayout(hbox);
	}

	hbox = new QHBoxLayout();
	rb_recog.setText("Grasp recognized object");
	bgrp->addButton(&rb_recog);
	hbox->addWidget(&rb_recog);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	rb_cylinder.setText("Cylinder grasping mode");
	bgrp->addButton(&rb_cylinder);
	hbox->addWidget(&rb_cylinder);
	hbox->addStretch();
	vbox->addLayout(hbox);

#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Target object"));
	hbox->addWidget(&sb_targetobject);
	sb_targetobject.setMinimum(0);
	sb_targetobject.setValue(0);
	hbox->addStretch();
	vbox->addLayout(hbox);
#endif

	hbox = new QHBoxLayout();
	rb_bothhand.setText("grasp with both hands");
	bgrp->addButton(&rb_bothhand);
	hbox->addWidget(&rb_bothhand);
	hbox->addStretch();
	vbox->addLayout(hbox);
	rb_bothhand.sigToggled().connect(boost::bind(&StartDialog::rbBothhandToggled,this));

	hbox = new QHBoxLayout();
	cb_sameDirection = new QCheckBox("only same approach direction");
	cb_sameDirection->setEnabled(false);
	hbox->addSpacing(20);
	hbox->addWidget(cb_sameDirection);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addSpacing(20);
	l_keyposes = new QLabel("Set the number of keyposes between liftup and release pose");
	l_keyposes->setEnabled(false);
	hbox->addWidget(l_keyposes);
	sb_keyposes.setAlignment(Qt::AlignCenter);
	sb_keyposes.setMinimum(0);
	sb_keyposes.setValue(0);
	sb_keyposes.setEnabled(false);
	hbox->addWidget(&sb_keyposes);
	vbox->addLayout(hbox);

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&StartDialog::okClicked, this));

	vbox->addWidget(okButton);
}

void StartDialog::okClicked(){
	this->hide();

	if(rb_bothhand.isChecked()){
			DualArmManipulation::instance()->graspWithBothHand(cb_sameDirection->isChecked(),sb_keyposes.value());
	}
	else if(rb_cylinder.isChecked()){
		ManipController::instance()->strategy = ManipController::RIGHT_RIGHT;
		// ManipController::instance()->doCylinderGraspPlanning();
		ManipController::instance()->clearTrajectories();
		bool col_check_flag = PlanBase::instance()->doCheckCollisionPointCloudFinger;
		PlanBase::instance()->doCheckCollisionPointCloudFinger = false;
#if defined(EXPERIMENT_DATA_OUTPUT) && !defined(LEARNING_DATA_OUTPUT)
		if (ManipController::instance()->doBinPickingGraspPlanning(sb_targetobject.value())) {
#else
		if (ManipController::instance()->doBinPickingGraspPlanning()) {
#endif
				TrajectoryPlanner tp;
				tp.doTrajectoryPlanning();
		}
		PlanBase::instance()->doCheckCollisionPointCloudFinger = col_check_flag;
	}
	else if(rb_recog.isChecked()){
		ObjectPosReader opr;
		int i=0;
		string pluginPath = cnoid::executableTopDirectory() + string("/extplugin/graspPlugin/");
		while (opr.doReadFromFile(pluginPath + "RobotInterface/data/data_cap.mat", pluginPath + "PCL/calibtools/calibmat.txt", i++)){
			cout << "object pose " << i << endl;
			ManipController::instance()->strategy = ManipController::RIGHT_LEFT;
			PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[0];
			ManipController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
			if(ManipController::instance()->doGraspPlanning()){
				TrajectoryPlanner tp;
				tp.doTrajectoryPlanning();
				break;
			}
		}
	}
	else{
		if(rb_default.isChecked()){
			ManipController::instance()->strategy = ManipController::NOT_SELECTED;
		}
		else if(rb_right.isChecked()){
			ManipController::instance()->strategy = ManipController::RIGHT_RIGHT;
			PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[0];
		}
		else if(rb_left.isChecked()){
			ManipController::instance()->strategy = ManipController::LEFT_LEFT;
			PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[1];
		}

		ManipController::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
		if(ManipController::instance()->doGraspPlanning()){
//			RobotLocalFunctions::instance()->insertPipe();
				TrajectoryPlanner tp;
				tp.doTrajectoryPlanning();
		}
	}
	ManipController::instance()->strategy = ManipController::NOT_SELECTED;
}

void StartDialog::rbBothhandToggled(){
	cb_sameDirection->setEnabled(rb_bothhand.isChecked());
	sb_keyposes.setEnabled(rb_bothhand.isChecked());
	l_keyposes->setEnabled(rb_bothhand.isChecked());
}


DebugDialog::DebugDialog() : QDialog(cnoid::MainWindow::instance()){
	setWindowTitle("Select mode");

	QButtonGroup* bgrp = new QButtonGroup();
	bgrp->setExclusive(true);

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hbox = new QHBoxLayout();
	rb_default.setText("Default grasp planning");
	rb_default.setChecked(true);
	bgrp->addButton(&rb_default);
	hbox->addWidget(&rb_default);
	hbox->addStretch();
	vbox->addLayout(hbox);
	vbox->addStretch();

	if(PlanBase::instance()->armsList.size() > 1){
		hbox = new QHBoxLayout();
		rb_right.setText("Grasp with right hand");
		bgrp->addButton(&rb_right);
		hbox->addWidget(&rb_right);
		hbox->addStretch();
		vbox->addLayout(hbox);

		hbox = new QHBoxLayout();
		rb_left.setText("Grasp with left hand");
		bgrp->addButton(&rb_left);
		hbox->addWidget(&rb_left);
		hbox->addStretch();
		vbox->addLayout(hbox);
	}

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&DebugDialog::okClicked, this));

	vbox->addWidget(okButton);
}

void DebugDialog::okClicked(){
	this->hide();

	if(rb_default.isChecked()){
		ManipFailureAnalyzer::instance()->strategy = ManipController::NOT_SELECTED;
	}
	else if(rb_right.isChecked()){
		ManipFailureAnalyzer::instance()->strategy = ManipController::RIGHT_RIGHT;
		PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[0];
	}
	else if(rb_left.isChecked()){
		ManipFailureAnalyzer::instance()->strategy = ManipController::LEFT_LEFT;
		PlanBase::instance()->targetArmFinger = PlanBase::instance()->armsList[1];
	}
	PlanBase::instance()->jointSeq.clear();
	ManipFailureAnalyzer::instance()->initial(PlanBase::instance()->targetObject,  PlanBase::instance()->targetArmFinger);
	ManipFailureAnalyzer::instance()->analyze();
	ManipFailureAnalyzer::instance()->strategy = ManipController::NOT_SELECTED;
}


ManipBar* ManipBar::instance()
{
	static ManipBar* instance = new ManipBar();
	return instance;
}

ManipBar::ManipBar()
	: ToolBar("ManipBar"),
	  mes(*MessageView::mainInstance()),
	  os (MessageView::mainInstance()->cout() )
{

	addSeparator();

	addLabel(("=Manip="));

	addButton(("Start"), ("Pick and Place Planning"))->
		sigClicked().connect(bind(&ManipBar::onStartButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addLabel(("=INTENTION="));
	addButton(("1"), ("First Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onFirstButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("2"), ("Second Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onSecondButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("3"), ("Third Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onThirdButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("4"), ("Fourth Part of Object"))->
		sigClicked().connect(bind(&ManipBar::onFourButtonClicked, this));	/* modified by qtconv.rb 6th rule*/

	addButton(("Debug"), ("Failure analysis"))->
		sigClicked().connect(bind(&ManipBar::onDebugButtonClicked, this));

	addButton(("SweptVolume"), ("Display swept volume"))->
		sigClicked().connect(bind(&ManipBar::onSweptVolumeButtonClicked, this));

	addButton(("Learn"), ("Register learning data"))->
		sigClicked().connect(bind(&ManipBar::onLearnClicked, this));

	addSeparator();

	// show_all_children();	/* modified by qtconv.rb 7th rule*/
}

ManipBar::~ManipBar()
{
}

void ManipBar::onStartButtonClicked()
{
	StartDialog* SDialog = new StartDialog();
	SDialog->exec();
	delete SDialog;
}

void ManipBar::onAllButtonClicked()
{
}

void ManipBar::onFirstButtonClicked()
{
	ManipController::instance()->intention = 0;
	os <<  "Intention is set to 0" << endl;
}

void ManipBar::onSecondButtonClicked()
{
	PlacePlanner::instance()->putPos.assigned = 1;
	os << "Pot pos database at 1 " << endl;
}

void ManipBar::onThirdButtonClicked()
{
	ManipController::instance()->intention = 2;
	os <<  "Intention is set to 2" << endl;
}

void ManipBar::onFourButtonClicked()
{
	ManipController::instance()->intention = 3;
	os <<  "Intention is set to 3" << endl;
}

void ManipBar::onDebugButtonClicked() {
	DebugDialog* DDialog = new DebugDialog();
	DDialog->show();
}

void ManipBar::onSweptVolumeButtonClicked()
{
	SweptVolumeDialog* svd = new SweptVolumeDialog();
	svd->exec();
	delete svd;
}

void ManipBar::onLearnClicked()
{
	LearningDialog* ld = new LearningDialog();
	ld->show();
}

bool ManipBar::storeState(Archive& archive)
{
	return true;
}

bool ManipBar::restoreState(const Archive& archive)
{
	return true;
}

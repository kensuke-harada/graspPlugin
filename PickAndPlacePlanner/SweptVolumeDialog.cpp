/**
 * @file   SweptVolumeDialog.cpp
 * @author Akira Ohchi
*/

#include "SweptVolumeDialog.h"

#include <QtGui>

#include <boost/bind.hpp>

#include "../Grasp/ObjectPoseEstimationSolution.h"
#include "ManipController.h"

SweptVolumeDialog::SweptVolumeDialog()
	: QDialog(cnoid::MainWindow::instance()) {
	setWindowTitle(tr("Display swept volume"));

	sp_target_sol_id = new cnoid::SpinBox();

	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
	cnoid::PushButton* clearButton = new cnoid::PushButton(tr("&Clear"));

	grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
	int max_num = opes->empty() ? 0 : opes->size() - 1;
	sp_target_sol_id->setMaximum(max_num);
	sp_target_sol_id->setEnabled(opes->empty());
	okButton->setDefault(true);

	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	connect(clearButton, SIGNAL(clicked()), this, SLOT(accept()));

	okButton->sigClicked().connect(boost::bind(&SweptVolumeDialog::okClicked, this));
	clearButton->sigClicked().connect(boost::bind(&SweptVolumeDialog::clearClicked, this));

	QVBoxLayout* vbox = new QVBoxLayout();

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("Target obj estimation ID")));
	hbox->addWidget(sp_target_sol_id);
	vbox->addLayout(hbox);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(clearButton);
	hbox_button->addWidget(okButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

SweptVolumeDialog::~SweptVolumeDialog() {
}

void SweptVolumeDialog::okClicked() {
	grasp::PickAndPlacePlanner::ManipController::instance()->displaySweptVolume(sp_target_sol_id->value());
}

void SweptVolumeDialog::clearClicked() {
	grasp::PickAndPlacePlanner::ManipController::instance()->clearSweptVolume();
}


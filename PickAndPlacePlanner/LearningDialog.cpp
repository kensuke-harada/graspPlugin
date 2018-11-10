/**
 * @file   LearningDialog.cpp
 * @author Akira Ohchi
*/

#include "LearningDialog.h"

#include <iostream>

#include <QtGui>

#include <boost/bind.hpp>

#include <cnoid/MessageView>

#include "PenaltyFunction.h"

LearningDialog::LearningDialog() :
	QDialog(cnoid::MainWindow::instance()) {
	setWindowTitle(tr("Register data"));

	cnoid::PushButton* successButton = new cnoid::PushButton(tr("&Succeeded"));
	cnoid::PushButton* failButton = new cnoid::PushButton(tr("Failed"));

	connect(successButton, SIGNAL(clicked()), this, SLOT(accept()));
	connect(failButton, SIGNAL(clicked()), this, SLOT(accept()));

	successButton->sigClicked().connect(boost::bind(&LearningDialog::successClicked, this));
	failButton->sigClicked().connect(boost::bind(&LearningDialog::failClicked, this));

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(successButton);
	hbox->addWidget(failButton);

	setLayout(hbox);
}

LearningDialog::~LearningDialog() {
}

void LearningDialog::successClicked() {
	addData(true);
}

void LearningDialog::failClicked() {
	addData(false);
}

void LearningDialog::addData(bool is_succeed) {
	PenaltyFunctionUpdater* updater = PenaltyFunctionUpdater::instance();
	std::ostream& os = cnoid::MessageView::mainInstance()->cout();
	if (updater->registerData(is_succeed)) {
		os << "Add data as a " << (is_succeed ? "successful" : "failure")  << " example" << std::endl;
		// if (!is_succeed) {
			if (updater->update()) os << "Updated penalty function" << std::endl;
		// }
	} else {
		os << "Fail to register data!" << std::endl;
	}
}

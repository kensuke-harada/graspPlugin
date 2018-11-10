#include "CalibrationDialog.h"

#include <boost/bind.hpp>

#include <cnoid/MainWindow>

#include "Calibration.h"

CalibrationDialog::CalibrationDialog() :
	cnoid::Dialog(cnoid::MainWindow::instance()) {
	calib_ = new Calibration();

	msgLabel = new QLabel("message");

	doneButton = new cnoid::PushButton(tr("&OK"));
	skipButton = new cnoid::PushButton(tr("&Skip"));

	acceptButton = new cnoid::PushButton(tr("&Accept"));
	retryButton = new cnoid::PushButton(tr("&Retry"));
	rejectButton = new cnoid::PushButton(tr("Re&ject"));

	doneButton->sigClicked().connect(boost::bind(&CalibrationDialog::doneClicked, this));
	skipButton->sigClicked().connect(boost::bind(&CalibrationDialog::skipClicked, this));
	acceptButton->sigClicked().connect(boost::bind(&CalibrationDialog::acceptClicked, this));
	retryButton->sigClicked().connect(boost::bind(&CalibrationDialog::retryClicked, this));
	rejectButton->sigClicked().connect(boost::bind(&CalibrationDialog::rejectClicked, this));

	QVBoxLayout* vbox = new QVBoxLayout();
	QHBoxLayout* hbox = new QHBoxLayout();

	vbox->addWidget(msgLabel);
	hbox->addWidget(doneButton);
	hbox->addWidget(skipButton);
	hbox->addWidget(acceptButton);
	hbox->addWidget(retryButton);
	hbox->addWidget(rejectButton);
	vbox->addLayout(hbox);

	setLayout(vbox);

	calib_->displayPointCloud();

	adjustDialogMode();
}

CalibrationDialog::~CalibrationDialog() {
	delete calib_;
}

void CalibrationDialog::setMatFilePath(const std::string& path) {
	output_path_ = path;
}

void CalibrationDialog::acceptClicked() {
	calib_->acceptPointPairs();
	goNextPose();
}

void CalibrationDialog::rejectClicked() {
	goNextPose();
}

void CalibrationDialog::retryClicked() {
	hide();
	calib_->registration();
	show();
}

void CalibrationDialog::doneClicked() {
	hide();
	calib_->registration();
	decisionDialogMode();
	show();
}

void CalibrationDialog::skipClicked() {
	goNextPose();
}

void CalibrationDialog::adjustDialogMode() {
	msgLabel->setText("move Calib object");
	msgLabel->show();
	doneButton->show();
	skipButton->show();
	acceptButton->hide();
	retryButton->hide();
	rejectButton->hide();
}

void CalibrationDialog::decisionDialogMode() {
	msgLabel->hide();
	doneButton->hide();
	skipButton->hide();
	acceptButton->show();
	retryButton->show();
	rejectButton->show();
}

void CalibrationDialog::goNextPose() {
	if (!calib_->goNext()) {
		calib_->generateMatrixFile(output_path_);
		accept();
	} else {
		adjustDialogMode();
		calib_->displayPointCloud();
	}
}

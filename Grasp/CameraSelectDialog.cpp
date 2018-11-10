#include "CameraSelectDialog.h"

#include <boost/bind.hpp>

#include <cnoid/MessageView>

#include "PlanBase.h"
#include "Camera.h"

using namespace grasp;

CameraSelectDialog::CameraSelectDialog() {
	setWindowTitle("Select camera");

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("select camera"));

	PlanBase* tc = PlanBase::instance();
	CameraHandler* camera_handler = CameraHandler::instance();

	int current_camera_id = camera_handler->getTargetCameraID();
	if (tc->targetArmFinger != NULL) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		camera_handler->init(tc->body(), tc->bodyItemRobot()->lastAccessedFilePath());
#else
		camera_handler->init(tc->body(), tc->bodyItemRobot()->filePath());
#endif
	}
	if (current_camera_id < camera_handler->size()) {
		camera_handler->setTargetCamera(current_camera_id);
	}

	for (int i = 0; i < camera_handler->size(); i++) {
		cameras.insertItem(i, camera_handler->getCamera(i)->getName().c_str());
	}
	if (current_camera_id < camera_handler->size()) {
		cameras.setCurrentIndex(current_camera_id);
	}

	hbox->addWidget(&cameras);
	hbox->addStretch();
	vbox->addLayout(hbox);

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&CameraSelectDialog::okClicked, this));

	vbox->addWidget(okButton);
}

CameraSelectDialog::~CameraSelectDialog() {
}

void CameraSelectDialog::okClicked() {
	if (cameras.count() < 1) return;
	bool is_success;
	is_success = CameraHandler::instance()->setTargetCamera(cameras.currentIndex());
	if (!is_success) return;
	cnoid::MessageView::mainInstance()->cout() << "set camera ID: "
		<< cameras.currentIndex()
		<<" Name: "
		<< CameraHandler::instance()->getTargetCamera()->getName()
		<< std::endl;
}

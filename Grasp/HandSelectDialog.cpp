#include "HandSelectDialog.h"

#include <QString>

#include <boost/bind.hpp>

#include <cnoid/MessageView>

#include "PlanBase.h"
#include "RobotBody.h"

using namespace grasp;

HandSelectDialog::HandSelectDialog() {
	attach_mode_ = true;

	setWindowTitle("attach/detach hand");

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hboxarm = new QHBoxLayout();
	hboxarm->addWidget(new QLabel("select arm"));
	hboxarm->addWidget(&armNumber);
	hboxarm->addStretch();
	vbox->addLayout(hboxarm);

	QHBoxLayout* hboxhand = new QHBoxLayout();
	hboxhand->addWidget(new QLabel("select hand"));
	hboxhand->addWidget(&hands);
	hboxhand->addStretch();
	vbox->addLayout(hboxhand);

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&HandSelectDialog::okClicked, this));

	armNumber.sigCurrentIndexChanged().connect(boost::bind(&HandSelectDialog::armChanged, this));

	vbox->addWidget(okButton);
}

HandSelectDialog::~HandSelectDialog() {
}

bool HandSelectDialog::hasAttachHandList() const {
	RobotBodyPtr body = PlanBase::instance()->robotBody();
	for (int i = 0; i < body->armSize(); i++) {
		if (body->handListSize(i) > 0) {
			if (!body->isAttached(i)) return true;
		}
	}
	return false;
}

bool HandSelectDialog::hasDetachHandList() const {
	RobotBodyPtr body = PlanBase::instance()->robotBody();
	for (int i = 0; i < body->armSize(); i++) {
		if (body->handListSize(i) > 0) {
			if (body->isAttached(i)) return true;
		}
	}
	return false;
}

bool HandSelectDialog::setAttachMode() {
	attach_mode_ = true;
	armNumber.clear();
	RobotBodyPtr body = PlanBase::instance()->robotBody();
	int index = 0;
	for (int i = 0; i < body->armSize(); i++) {
		if (body->handListSize(i) > 0) {
			if (!body->isAttached(i)) {
				armNumber.insertItem(index++, QString::number(i));
			}
		}
	}
	return true;
}

bool HandSelectDialog::setDetachMode() {
	attach_mode_ = false;
	armNumber.clear();
	RobotBodyPtr body = PlanBase::instance()->robotBody();
	int index = 0;
	for (int i = 0; i < body->armSize(); i++) {
		if (body->handListSize(i) > 0) {
			if (body->isAttached(i)) {
				armNumber.insertItem(index++, QString::number(i));
			}
		}
	}
	return true;
}

void HandSelectDialog::okClicked() {
	RobotBodyPtr body = PlanBase::instance()->robotBody();
	int arm_number = armNumber.currentText().toInt();
	if (attach_mode_) {
		body->attachHand(arm_number, hands.currentIndex());
	} else {
		body->detachHand(arm_number);
	}
	body->calcForwardKinematics();
	// PlanBase::instance()->setArmState(body->getArmsState());
}

void HandSelectDialog::armChanged() {
	hands.clear();
	if (!attach_mode_) {
		hands.setEnabled(false);
	} else {
		hands.setEnabled(true);
		RobotBodyPtr body = PlanBase::instance()->robotBody();
		int arm_number = armNumber.currentText().toInt();
		for (int i = 0; i < body->handListSize(arm_number); i++) {
			hands.insertItem(i, body->getRobotArmFingers(arm_number)->getRobotHand(i)->handName.c_str());
		}
	}
}

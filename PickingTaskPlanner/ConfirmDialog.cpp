#include "ConfirmDialog.h"

#include <boost/bind.hpp>

#include <cnoid/MainWindow>

using namespace grasp;

ConfirmDialog::ConfirmDialog() :
	QDialog(cnoid::MainWindow::instance()) {
	state = 0;
	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
	cnoid::PushButton* cancelButton = new cnoid::PushButton(tr("&Cancel"));

	okButton->sigClicked().connect(boost::bind(&ConfirmDialog::okClicked, this));
	cancelButton->sigClicked().connect(boost::bind(&ConfirmDialog::cancelClicked, this));

	QVBoxLayout* vbox = new QVBoxLayout();
	QHBoxLayout* hbox_button = new QHBoxLayout();
	vbox->addWidget(new QLabel("do move?"));
	hbox_button->addStretch();
	hbox_button->addWidget(cancelButton);
	hbox_button->addWidget(okButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

ConfirmDialog::~ConfirmDialog() {
}

int ConfirmDialog::getState() const {
	return state;
}

void ConfirmDialog::closeEvent(QCloseEvent* ce) {
	ce->ignore();
	if (state == 0) {
		state = -1;
	}
}

void ConfirmDialog::okClicked() {
	state = 1;
}

void ConfirmDialog::cancelClicked() {
	state = -1;
}

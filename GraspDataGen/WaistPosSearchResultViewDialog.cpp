#include "WaistPosSearchResultViewDialog.h"

#include <QtGui>

#include <boost/bind.hpp>

#include "WaistPositionSearcher.h"

using namespace grasp;

WaistPosSearchResultViewDialog::WaistPosSearchResultViewDialog() {
	WaistPosSearchResult* res = WaistPosSearchResult::instance();

	BoxInfoArray boxes = res->getBoxes();

	for (size_t i = 0; i < boxes.size(); i++) {
		displayBoxRightCheck.push_back(new cnoid::CheckBox());
		displayBoxLeftCheck.push_back(new cnoid::CheckBox());
	}
	displayRouteCheck = new cnoid::CheckBox();



	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));

	for (size_t i = 0; i < boxes.size(); i++) {
		displayBoxRightCheck[i]->setChecked(true);
		displayBoxLeftCheck[i]->setChecked(true);
	}
	displayRouteCheck->setChecked(true);
	displayRouteCheck->setText(tr("display route"));

	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&WaistPosSearchResultViewDialog::okClicked, this));


	QVBoxLayout* vbox = new QVBoxLayout();
	QHBoxLayout* hbox_box = new QHBoxLayout();
	QGridLayout* grid_box = new QGridLayout();
	grid_box->addWidget(new QLabel(tr("box")), 0, 0);
	grid_box->addWidget(new QLabel(tr("right")), 0, 1);
	grid_box->addWidget(new QLabel(tr("left")), 0, 2);
	for (size_t i = 0; i < boxes.size(); i++) {
		grid_box->addWidget(new QLabel(QString::fromStdString(boxes[i]->bodyItem()->name())), i+1, 0);
		grid_box->addWidget(displayBoxRightCheck[i], i+1, 1);
		grid_box->addWidget(displayBoxLeftCheck[i], i+1, 2);
	}
	hbox_box->addLayout(grid_box);
	hbox_box->addStretch();
	vbox->addLayout(hbox_box);

	QHBoxLayout* hbox_route = new QHBoxLayout();
	hbox_route->addWidget(displayRouteCheck);
	hbox_route->addStretch();
	vbox->addLayout(hbox_route);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(okButton);
	vbox->addLayout(hbox_button);
	vbox->addStretch();

	setLayout(vbox);
}

void WaistPosSearchResultViewDialog::okClicked() {
	WaistPosSearchResult* res = WaistPosSearchResult::instance();

	BoxInfoArray boxes = res->getBoxes();
	std::vector<bool> right, left;
	for (size_t i = 0; i < boxes.size(); i++) {
		right.push_back(displayBoxRightCheck[i]->isChecked());
		left.push_back(displayBoxLeftCheck[i]->isChecked());
	}

	res->showResult(right, left, displayRouteCheck->isChecked());
}

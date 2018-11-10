#include "WaistPosSearchDialog.h"

#include <QtGui>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ItemList>

#include "WaistPositionSearcher.h"
#include "../Grasp/PlanBase.h"

using namespace grasp;

WaistPosSearchDialog::WaistPosSearchDialog() {
	readBoxInfos();

	QTableWidget* boxObjPairTable = new QTableWidget();
	useCameraCheck = new cnoid::CheckBox();
	if (boxinfos.size() > 1) {
		gridIntervalSpin = new cnoid::DoubleSpinBox();
		displayGraspablePointCheck = new cnoid::CheckBox();
		if (PlanBase::instance()->armsList.size() > 1) {
			useRightArmCheck = new cnoid::CheckBox();
			useLeftArmCheck = new cnoid::CheckBox();
		}
	}
	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
#ifdef DEBUG_MODE
	cornerIDSpin = new cnoid::SpinBox();
	cnoid::PushButton* debugButton = new cnoid::PushButton(tr("&Debug"));
#endif

	// setting
	boxObjPairTable->setRowCount(boxinfos.size());
	boxObjPairTable->setColumnCount(2);

	boxObjPairTable->setHorizontalHeaderLabels(QString("Box;Object").split(";"));
	boxObjPairTable->verticalHeader()->hide();
	boxObjPairTable->horizontalHeader()->setStretchLastSection(true);
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	boxObjPairTable->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#else
	boxObjPairTable->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#endif
	boxObjPairTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  boxObjPairTable->setSelectionMode(QAbstractItemView::NoSelection);

	for (size_t i = 0; i < boxinfos.size(); i++) {
		QTableWidgetItem* box_item = new QTableWidgetItem();
		box_item->setText(QString::fromStdString(boxinfos[i]->body()->name()));
		box_item->setFlags(box_item->flags() & ~Qt::ItemIsEditable);

		cnoid::ComboBox* obj_item = new cnoid::ComboBox();
		object_items.push_back(obj_item);
		for (int j = 0; j < boxinfos[i]->getObjectSize(); j++){
			obj_item->insertItem(j, QString::fromStdString(boxinfos[i]->getObjInfo(j)->object_name));
		}

		boxObjPairTable->setItem(i, 0, box_item);
		boxObjPairTable->setCellWidget(i, 1, obj_item);
	}

	useCameraCheck->setChecked(false);
	useCameraCheck->setText(tr("with camera"));

	if (boxinfos.size() > 1) {
		gridIntervalSpin->setRange(0.001, 1.000);
		gridIntervalSpin->setDecimals(3);
		gridIntervalSpin->setValue(0.05);

		displayGraspablePointCheck->setChecked(false);
		displayGraspablePointCheck->setText(tr("show graspable points"));
	}

	if ((boxinfos.size() > 1) && (PlanBase::instance()->armsList.size() > 1)) {
		useRightArmCheck->setText(tr("right hand"));
		useRightArmCheck->setChecked(PlanBase::instance()->targetArmFinger->arm == PlanBase::instance()->arm(0));
		useLeftArmCheck->setText(tr("left hand"));
		useLeftArmCheck->setChecked(PlanBase::instance()->targetArmFinger->arm == PlanBase::instance()->arm(1));
	}

	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&WaistPosSearchDialog::okClicked, this));
#ifdef DEBUG_MODE
	cornerIDSpin->setRange(0, 3);
	connect(debugButton, SIGNAL(clicked()), this, SLOT(accept()));
	debugButton->sigClicked().connect(boost::bind(&WaistPosSearchDialog::debugClicked, this));
#endif

	// layout
	setWindowTitle(tr("Search optimal waist position"));
	QVBoxLayout* vbox = new QVBoxLayout();

	vbox->addWidget(boxObjPairTable);

	QVBoxLayout* vbox_param = new QVBoxLayout();
	vbox_param->addWidget(useCameraCheck);
	if (boxinfos.size() > 1) {
		QHBoxLayout* hbox_gridinterval = new QHBoxLayout();
		hbox_gridinterval->addWidget(new QLabel(tr("grid interval[m]")));
		hbox_gridinterval->addWidget(gridIntervalSpin);
		vbox_param->addLayout(hbox_gridinterval);
		vbox_param->addWidget(displayGraspablePointCheck);
	}
	vbox_param->addStretch();
	vbox->addLayout(vbox_param);
	
	if ((boxinfos.size() > 1) && (PlanBase::instance()->armsList.size() > 1)) {
		QGroupBox* gbox = new QGroupBox(tr("target hand"));
		QVBoxLayout* vbox_hand = new QVBoxLayout();
		vbox_hand->addWidget(useRightArmCheck);
		vbox_hand->addWidget(useLeftArmCheck);
		gbox->setLayout(vbox_hand);
		vbox->addWidget(gbox);
	}

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(okButton);
	vbox->addLayout(hbox_button);
	vbox->addStretch();

#ifdef DEBUG_MODE
	vbox->addWidget(cornerIDSpin);
	vbox->addWidget(debugButton);
#endif

	setLayout(vbox);
}

void WaistPosSearchDialog::okClicked() {
	for (size_t i = 0; i < boxinfos.size(); i++) {
		boxinfos[i]->setTargetObj(object_items[i]->currentIndex());
	}

	WaistSearchParam param;
	param.with_camera = useCameraCheck->isChecked();
	if (boxinfos.size() > 1) {
		param.grid_interval = gridIntervalSpin->value();
		param.display_graspable_points = displayGraspablePointCheck->isChecked();
		if (PlanBase::instance()->armsList.size() > 1) {
			if (useRightArmCheck->isChecked()) param.target_arm_ids.push_back(0);
			if (useLeftArmCheck->isChecked()) param.target_arm_ids.push_back(1);
		} else {
			param.target_arm_ids.push_back(WaistPositionSearcher::getTargetArmID());
		}
	} else {
		param.target_arm_ids.push_back(WaistPositionSearcher::getTargetArmID());
	}

	WaistPositionSearcher searcher;
	searcher.searchWaistPos(boxinfos, param);
}

#ifdef DEBUG_MODE
void WaistPosSearchDialog::debugClicked() {
	for (size_t i = 0; i < boxinfos.size(); i++) {
		boxinfos[i]->setTargetObj(object_items[i]->currentIndex());
	}

	WaistSearchParam param;
	param.with_camera = useCameraCheck->isChecked();
	param.target_arm_ids.push_back(WaistPositionSearcher::getTargetArmID());

	WaistPositionSearcher searcher;
	searcher.graspCornerObj(boxinfos, param, cornerIDSpin->value());
}
#endif

void WaistPosSearchDialog::readBoxInfos() {
	cnoid::ItemList<cnoid::BodyItem> bodyItems = cnoid::ItemTreeView::mainInstance()->selectedItems<cnoid::BodyItem>();
	for (size_t i = 0; i < bodyItems.size(); i++) {
		cnoid::BodyItemPtr trgt_bodyitem = bodyItems[i].get();
		BoxInfoPtr boxinfo = boost::make_shared<BoxInfo>(trgt_bodyitem);
		if (boxinfo->getObjectSize() > 0) {
			boxinfos.push_back(boxinfo);
		}
	}
}

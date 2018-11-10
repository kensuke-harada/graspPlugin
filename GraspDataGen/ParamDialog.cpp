#include "ParamDialog.h"

#include <string>

#include <QtGui>
#include <QFile>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

#include "../Grasp/PlanBase.h"
#include "PrehensionParamHandler.h"
using namespace grasp;

PrehensionParamTab::PrehensionParamTab(int id_) {
	id = id_;

	nameLine = new cnoid::LineEdit();
	GRCposXSpin = new cnoid::DoubleSpinBox();
	GRCposYSpin = new cnoid::DoubleSpinBox();
	GRCposZSpin = new cnoid::DoubleSpinBox();
	GRCrpyRSpin = new cnoid::DoubleSpinBox();
	GRCrpyPSpin = new cnoid::DoubleSpinBox();
	GRCrpyYSpin = new cnoid::DoubleSpinBox();
	GRCedgeXSpin = new cnoid::DoubleSpinBox();
	GRCedgeYSpin = new cnoid::DoubleSpinBox();
	GRCedgeZSpin = new cnoid::DoubleSpinBox();
	loadMinSpin = new cnoid::DoubleSpinBox();
	loadMaxSpin = new cnoid::DoubleSpinBox();
	usePalmCheck = new cnoid::CheckBox();
	palmCloseXSpin = new cnoid::DoubleSpinBox();
	palmCloseYSpin = new cnoid::DoubleSpinBox();
	palmCloseZSpin = new cnoid::DoubleSpinBox();

	PlanBase* tc = PlanBase::instance();

	const int num_finger = tc->nFing();
	for (int i = 0; i < num_finger; i++) {
		FingerComponent fing;
		const int num_joint = tc->fingers(i)->nJoints;
		for (int j = 0; j < num_joint; j++) {
			fing.fingerAngleSpins.push_back(new cnoid::DoubleSpinBox());
			fing.fingerContactChecks.push_back(new cnoid::CheckBox());
			fing.fingerCompLinkCombo.push_back(new cnoid::ComboBox());
			fing.fingerCloseSpins.push_back(new cnoid::DoubleSpinBox());
		}
		finger_components.push_back(fing);
	}

	cnoid::PushButton* displayButton = new cnoid::PushButton(tr("&Display"));
	cnoid::PushButton* graspButton = new cnoid::PushButton(tr("&Grasp"));
	cnoid::PushButton* removeButton = new cnoid::PushButton(tr("&Remove"));

	// Setting
	GRCposXSpin->setRange(-100, 100);
	GRCposXSpin->setDecimals(4);
	GRCposYSpin->setRange(-100, 100);
	GRCposYSpin->setDecimals(4);
	GRCposZSpin->setRange(-100, 100);
	GRCposZSpin->setDecimals(4);
	GRCrpyRSpin->setRange(-360, 360);
	GRCrpyRSpin->setDecimals(2);
	GRCrpyPSpin->setRange(-360, 360);
	GRCrpyPSpin->setDecimals(2);
	GRCrpyYSpin->setRange(-360, 360);
	GRCrpyYSpin->setDecimals(2);
	GRCedgeXSpin->setRange(-100, 100);
	GRCedgeXSpin->setDecimals(4);
	GRCedgeYSpin->setRange(-100, 100);
	GRCedgeYSpin->setDecimals(4);
	GRCedgeZSpin->setRange(-100, 100);
	GRCedgeZSpin->setDecimals(4);
	loadMinSpin->setRange(0, 100);
	loadMinSpin->setDecimals(2);
	loadMaxSpin->setRange(0, 100);
	loadMaxSpin->setDecimals(2);
	palmCloseXSpin->setRange(-1, 1);
	palmCloseXSpin->setDecimals(2);
	palmCloseXSpin->setEnabled(false);
	palmCloseYSpin->setRange(-1, 1);
	palmCloseYSpin->setDecimals(2);
	palmCloseYSpin->setEnabled(false);
	palmCloseZSpin->setRange(-1, 1);
	palmCloseZSpin->setDecimals(2);
	palmCloseZSpin->setEnabled(false);

	for (int i = 0; i < num_finger; i++) {
		const int num_joint = tc->fingers(i)->nJoints;
		for (int j = 0; j < num_joint; j++) {
			finger_components[i].fingerAngleSpins[j]->setRange(-180, 180);
			finger_components[i].fingerAngleSpins[j]->setDecimals(2);
			finger_components[i].fingerCloseSpins[j]->setRange(-180, 180);
			finger_components[i].fingerCloseSpins[j]->setDecimals(3);
			for (int m = 0; m < num_joint; m++) {
				finger_components[i].fingerCompLinkCombo[j]->setEditable(true);
				finger_components[i].fingerCompLinkCombo[j]->addItem(QString::number(m));
			}
		}
	}

	usePalmCheck->sigToggled().connect(boost::bind(&PrehensionParamTab::usePalmcloseToggled, this));
	displayButton->sigClicked().connect(boost::bind(&PrehensionParamTab::displayClicked, this));
	graspButton->sigClicked().connect(boost::bind(&PrehensionParamTab::graspClicked, this));
	removeButton->sigClicked().connect(boost::bind(&PrehensionParamTab::removeClicked, this));

	// layout
	QVBoxLayout* vbox = new QVBoxLayout();
	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("Name:")));
	hbox->addWidget(nameLine);
	hbox->addStretch();
	vbox->addLayout(hbox);

	QHBoxLayout* hbox_grc = new QHBoxLayout();
	QGridLayout* grid_grc = new QGridLayout();
	grid_grc->addWidget(new QLabel(tr("GRC Position:")), 0, 0);
	grid_grc->addWidget(GRCposXSpin, 0, 1);
	grid_grc->addWidget(GRCposYSpin, 0, 2);
	grid_grc->addWidget(GRCposZSpin, 0, 3);
	grid_grc->addWidget(new QLabel(tr("GRC RPY[deg]:")), 1, 0);
	grid_grc->addWidget(GRCrpyRSpin, 1, 1);
	grid_grc->addWidget(GRCrpyPSpin, 1, 2);
	grid_grc->addWidget(GRCrpyYSpin, 1, 3);
	grid_grc->addWidget(new QLabel(tr("GRC max Edge:")), 2, 0);
	grid_grc->addWidget(GRCedgeXSpin, 2, 1);
	grid_grc->addWidget(GRCedgeYSpin, 2, 2);
	grid_grc->addWidget(GRCedgeZSpin, 2, 3);
	hbox_grc->addLayout(grid_grc);
	hbox_grc->addStretch();
	vbox->addLayout(hbox_grc);

	QFrame* hline = new QFrame();
	hline->setFrameShape(QFrame::HLine);
	hline->setFrameShadow(QFrame::Sunken);
	vbox->addWidget(hline);

	QHBoxLayout* hbox_fing = new QHBoxLayout();
	QGridLayout* grid_fing = new QGridLayout();
	for (size_t i = 0; i < finger_components.size(); i++) {
		grid_fing->addWidget(new QLabel(QString("Finger%1:").arg(i)), 5*i, 0);
		grid_fing->addWidget(new QLabel(tr("")), 5*i, 1);
		grid_fing->addWidget(new QLabel(tr("Angle[deg/m]")), 5*i+1, 1);
		grid_fing->addWidget(new QLabel(tr("Contact")), 5*i+2, 1);
		grid_fing->addWidget(new QLabel(tr("CompLink")), 5*i+3, 1);
		grid_fing->addWidget(new QLabel(tr("Close[deg/m]")), 5*i+4, 1);
		const int num_joint =  finger_components[i].fingerAngleSpins.size();

		for (int j = 0; j < num_joint; j++) {
			grid_fing->addWidget(new QLabel(QString("[%1:%2]").arg(j).arg(tc->fingers(i)->joint(j)->name().c_str())), 5*i, j+2, Qt::AlignCenter);
			grid_fing->addWidget(finger_components[i].fingerAngleSpins[j], 5*i+1, j+2);
			grid_fing->addWidget(finger_components[i].fingerContactChecks[j], 5*i+2, j+2, Qt::AlignCenter);
			grid_fing->addWidget(finger_components[i].fingerCompLinkCombo[j], 5*i+3, j+2);
			grid_fing->addWidget(finger_components[i].fingerCloseSpins[j], 5*i+4, j+2);
		}
	}
	hbox_fing->addLayout(grid_fing);
	hbox_fing->addStretch();
	vbox->addLayout(hbox_fing);

	hline = new QFrame();
	hline->setFrameShape(QFrame::HLine);
	hline->setFrameShadow(QFrame::Sunken);
	vbox->addWidget(hline);

	QHBoxLayout* hbox_param = new QHBoxLayout();
	QGridLayout* grid_param = new QGridLayout();
	grid_param->addWidget(new QLabel(tr("Min/Max Load:")), 0, 0);
	QHBoxLayout* hbox_load = new QHBoxLayout();
	hbox_load->addWidget(new QLabel(tr("min")));
	hbox_load->addWidget(loadMinSpin);
	hbox_load->addWidget(new QLabel(tr("max")));
	hbox_load->addWidget(loadMaxSpin);
	hbox_load->addStretch();
	grid_param->addLayout(hbox_load, 0, 1);
	grid_param->addWidget(new QLabel(tr("Use Palm Close:")), 1, 0);
	QHBoxLayout* hbox_palm = new QHBoxLayout();
	hbox_palm->addWidget(usePalmCheck);
	hbox_palm->addWidget(new QLabel(tr(" close direction(")));
	hbox_palm->addWidget(palmCloseXSpin);
	hbox_palm->addWidget(palmCloseYSpin);
	hbox_palm->addWidget(palmCloseZSpin);
	hbox_palm->addWidget(new QLabel(tr(")")));
	hbox_palm->addStretch();
	grid_param->addLayout(hbox_palm, 1, 1);
	hbox_param->addLayout(grid_param);
	hbox_param->addStretch();
	vbox->addLayout(hbox_param);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(displayButton);
	hbox_button->addWidget(graspButton);
	hbox_button->addWidget(removeButton);
	vbox->addLayout(hbox_button);
	vbox->addStretch();

	setLayout(vbox);
}

void PrehensionParamTab::usePalmcloseToggled() {
	palmCloseXSpin->setEnabled(usePalmCheck->isChecked());
	palmCloseYSpin->setEnabled(usePalmCheck->isChecked());
	palmCloseZSpin->setEnabled(usePalmCheck->isChecked());
}

void PrehensionParamTab::displayClicked() {
	PrehensionParameterExt param;
	PrehensionParamHandler pphandler;
	makePrehensionParam(&param);
	pphandler.display(param);
}

void PrehensionParamTab::graspClicked() {
	PrehensionParameterExt param;
	PrehensionParamHandler pphandler;
	makePrehensionParam(&param);
	pphandler.grasp(param);
}

void PrehensionParamTab::removeClicked() {
	QMessageBox msgBox;
	msgBox.setText("Are you sure you want to delete this parameter?");
	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::No);
  if (msgBox.exec() == QMessageBox::Yes) this->deleteLater();
}

void PrehensionParamTab::makePrehensionParam(PrehensionParameterExt* param) {
	param->coeff_GRCdes = 0.5;
	param->coeff_GRCmin = 0.1;

	param->name = nameLine->text().toStdString();
	param->GRCmax_pos(0) = GRCposXSpin->value();
	param->GRCmax_pos(1) = GRCposYSpin->value();
	param->GRCmax_pos(2) = GRCposZSpin->value();
	param->GRCmax_rpy(0) = deg2rad(GRCrpyRSpin->value());
	param->GRCmax_rpy(1) = deg2rad(GRCrpyPSpin->value());
	param->GRCmax_rpy(2) = deg2rad(GRCrpyYSpin->value());
	param->GRCmax_edge(0) = GRCedgeXSpin->value();
	param->GRCmax_edge(1) = GRCedgeYSpin->value();
	param->GRCmax_edge(2) = GRCedgeZSpin->value();


	param->GRCmin_pos = param->GRCmax_pos;
	param->GRCdes_pos = param->GRCmax_pos;
	param->GRCmin_rpy = param->GRCmax_rpy;
	param->GRCdes_rpy = param->GRCmax_rpy;
	param->GRCmin_edge = param->coeff_GRCmin * param->GRCmax_edge;
	param->GRCdes_edge = param->coeff_GRCdes * param->GRCmax_edge;

	int LR_flag=0;
	if(PlanBase::instance()->targetArmFinger == PlanBase::instance()->armsList[1]) LR_flag = 1;
	

	param->fingers.clear();
	size_t n_fing = finger_components.size();
	for (size_t i = 0; i < n_fing; i++) {
		PrehensionParameter::FingerParameter fing;
		size_t n_joint = finger_components[i].fingerAngleSpins.size();
		param->angles.push_back(std::vector<double>(n_joint, 0.0));
		for (size_t j = 0; j < n_joint; j++) {
			if(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->isRotationalJoint() ){
				param->angles[i][j] = deg2rad(finger_components[i].fingerAngleSpins[j]->value());
				fing.close.push_back(deg2rad(finger_components[i].fingerCloseSpins[j]->value()));
			}
			else{
				param->angles[i][j] = finger_components[i].fingerAngleSpins[j]->value();
				fing.close.push_back(finger_components[i].fingerCloseSpins[j]->value());
			}
			fing.contact.push_back(finger_components[i].fingerContactChecks[j]->isChecked());
			fing.complink.push_back(finger_components[i].fingerCompLinkCombo[j]->currentText().toInt());

		}
		param->fingers.push_back(fing);
	}

	param->load_min = loadMinSpin->value();
	param->load_max = loadMaxSpin->value();
	param->use_palmclose = usePalmCheck->isChecked();

	param->palmclose_dir(0) = palmCloseXSpin->value();
	param->palmclose_dir(1) = palmCloseYSpin->value();
	param->palmclose_dir(2) = palmCloseZSpin->value();

	param->ref_motion.clear();
	param->ref_motion.push_back(ref_motion);
	param->id = id;
}


bool PrehensionParamTab::setTextFromPrehensionParam(const PrehensionParameterExt* param) {
	nameLine->setText(param->name);

	GRCposXSpin->setValue(param->GRCmax_pos.x());
	GRCposYSpin->setValue(param->GRCmax_pos.y());
	GRCposZSpin->setValue(param->GRCmax_pos.z());
	GRCrpyRSpin->setValue(rad2deg(param->GRCmax_rpy(0)));
	GRCrpyPSpin->setValue(rad2deg(param->GRCmax_rpy(1)));
	GRCrpyYSpin->setValue(rad2deg(param->GRCmax_rpy(2)));
	GRCedgeXSpin->setValue(param->GRCmax_edge.x());
	GRCedgeYSpin->setValue(param->GRCmax_edge.y());
	GRCedgeZSpin->setValue(param->GRCmax_edge.z());

	int LR_flag=0;
	if(PlanBase::instance()->targetArmFinger == PlanBase::instance()->armsList[1]) LR_flag = 1;

	for (size_t i = 0; (i <  param->fingers.size()) && (i < finger_components.size()); i++) {
		for (size_t j = 0; (j < param->angles[i].size() || j < param->fingers[i].contact.size()) && (j < finger_components[i].fingerAngleSpins.size()); j++) {
			if (j < param->angles[i].size()) {
				if(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->isRotationalJoint() )
					finger_components[i].fingerAngleSpins[j]->setValue(rad2deg(param->angles[i][j]));
				else
					finger_components[i].fingerAngleSpins[j]->setValue(param->angles[i][j]);
			}
			if (j < param->fingers[i].contact.size()) {
				finger_components[i].fingerContactChecks[j]->setChecked(param->fingers[i].contact[j]);
				int index = finger_components[i].fingerCompLinkCombo[j]->findText(QString::number(param->fingers[i].complink[j]));
				if (index != -1) {
					finger_components[i].fingerCompLinkCombo[j]->setCurrentIndex(index);
				}
				if(PlanBase::instance()->fingers(LR_flag,i)->fing_path->joint(j)->isRotationalJoint() )
					finger_components[i].fingerCloseSpins[j]->setValue(rad2deg(param->fingers[i].close[j]));
				else
					finger_components[i].fingerCloseSpins[j]->setValue(param->fingers[i].close[j]);
			}
		}
	}

	loadMinSpin->setValue(param->load_min);
	loadMaxSpin->setValue(param->load_max);

	usePalmCheck->setChecked(param->use_palmclose);

	palmCloseXSpin->setValue(param->use_palmclose ? param->palmclose_dir.x() : 0.0);
	palmCloseYSpin->setValue(param->use_palmclose ? param->palmclose_dir.y() : 0.0);
	palmCloseZSpin->setValue(param->use_palmclose ? param->palmclose_dir.z() : 0.0);

	
	ref_motion = (!param->ref_motion.empty()) ? param->ref_motion[0] : param->name;

	return true;
}


PrehensionParamDialog::PrehensionParamDialog(ArmFingers* arm) {
	target_arm = arm;

	tabWidget = new QTabWidget;
	tabWidget->setMovable(true);

	cnoid::PushButton* addButton = new cnoid::PushButton(tr("&Add"));
	cnoid::PushButton* saveButton = new cnoid::PushButton(tr("&Save All"));
	cnoid::PushButton* closeButton = new cnoid::PushButton(tr("&Close"));

	
	addButton->sigClicked().connect(boost::bind(&PrehensionParamDialog::addClicked, this));
	saveButton->sigClicked().connect(boost::bind(&PrehensionParamDialog::saveClicked, this));
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));

	// layout
	setWindowTitle(tr("Setting prehension parameters"));

	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addWidget(tabWidget);
	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(addButton);
	hbox_button->addWidget(saveButton);
	hbox_button->addWidget(closeButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

void PrehensionParamDialog::updateTabs() {
	if(tabWidget == NULL) {
		tabWidget = new QTabWidget;
	}
	for (int i = tabWidget->count(); i > 0; i--) {
		tabWidget->removeTab(i-1);
	}
	PrehensionParamHandler pphandler;
	PrehensionParamTab* target_tab;
	PlanBase* tc = PlanBase::instance();
	pphandler.loadParams();
	for (int i = 0; i < pphandler.getNumParams(); i++) {
		PrehensionParameterExt param = pphandler.getParam(i);
		if (tc->nFing() != param.fing_size()) {
			std::string warning_message = "Warning : the number of fingers in prehension yaml file is not the same as that of robot! [" + param.name +"]\n";
			cnoid::MessageView::instance()->put(warning_message);
		}
		for (int j = 0; (j < tc->nFing() && j < param.fing_size()); j++) {
			if (tc->fingers(j)->nJoints != param.joint_size(j)) {
				std::string warning_message = "Warning : the number of joints of Finger" + boost::lexical_cast<std::string>(j) + " in prehension yaml file is not the same as that of robot! [" + param.name +"]\n";
				cnoid::MessageView::instance()->put(warning_message);
			}
		}
		
		target_tab = new PrehensionParamTab(i);
		tabWidget->addTab(target_tab, tr(param.name.c_str()));
		target_tab->setTextFromPrehensionParam(&param);
	}
}

void PrehensionParamDialog::addClicked() {
	PrehensionParamTab* target_tab = new PrehensionParamTab;
	tabWidget->addTab(target_tab, tr("*"));
	tabWidget->setCurrentWidget(target_tab);
}

void PrehensionParamDialog::saveClicked() {
	if (PlanBase::instance()->targetArmFinger != target_arm) {
		cnoid::MessageView::instance()->put("Target arm is changed!\n");
		return;
	}

	QMessageBox msgBox;
	msgBox.setText("Are you sure you want to save all parameters?");
	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
  msgBox.setDefaultButton(QMessageBox::Yes);
  if (msgBox.exec() == QMessageBox::No) return;

	PrehensionParamHandler pphandler;
	for (int i = 0; i < tabWidget->count(); i++) {
		PrehensionParameterExt param;
		PrehensionParamTab* target_tab = dynamic_cast<PrehensionParamTab*>(tabWidget->widget(i));
		target_tab->makePrehensionParam(&param);
		tabWidget->setTabText(i, param.name.c_str());
		if (target_tab->isNew()) {
			target_tab->setRefmotion(param.name);
			param.ref_motion.clear();
			param.ref_motion.push_back(param.name);
		}
		pphandler.addParam(param);
	}
	pphandler.updateYamlFile();
	
	for (int i = 0; i < tabWidget->count(); i++) {
		PrehensionParamTab* target_tab = dynamic_cast<PrehensionParamTab*>(tabWidget->widget(i));
		target_tab->setID(i);
	}

	cnoid::MessageView::instance()->put("Parameters saved!\n");
}

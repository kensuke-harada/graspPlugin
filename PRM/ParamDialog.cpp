#include "ParamDialog.h"

#include <boost/bind.hpp>

#include <QSettings>

#include <cnoid/ExecutablePath>

using namespace grasp;

PRMParams::PRMParams() {
	ini_file_path = cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PRM/config.ini";
	param.algo = ALGO_SBL;
	param.eps = 0.05;
	param.iteration_num = 30000;
	param.delta = 0.01;
	param.smoothestep = 50;
	param.num_path = 10;
	param.nnmethod = NN_RADIUS;
	param.radius = 0.2;
}

void PRMParams::loadParams() {
	QSettings setting(ini_file_path.c_str(), QSettings::IniFormat);
	param.algo = static_cast<ALGORITHM>(setting.value("algorithm", ALGO_SBL).toInt());
	param.eps = setting.value("eps", 0.05).toDouble();
	param.iteration_num = setting.value("iteration", 30000).toDouble();
	param.delta = setting.value("delta", 0.01).toDouble();
	param.smoothestep = setting.value("smoothestep", 50).toInt();
	param.num_path = setting.value("num_path", 10).toInt();
	param.nnmethod = static_cast<NNMETHOD>(setting.value("nnmethod", NN_RADIUS).toInt());
	param.radius = setting.value("radius", 0.2).toDouble();
}

void PRMParams::saveParams() {
	QSettings setting(ini_file_path.c_str(), QSettings::IniFormat);
	setting.setValue("algorithm", param.algo);
	setting.setValue("eps", param.eps);
	setting.setValue("iteration", param.iteration_num);
	setting.setValue("delta", param.delta);
	setting.setValue("smoothestep", param.smoothestep);
	setting.setValue("num_path", param.num_path);
	setting.setValue("nnmethod", param.nnmethod);
	setting.setValue("radius", param.radius);
}

PRMParamDialog::PRMParamDialog() {
	setWindowTitle(tr("Setting parameters"));

	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addLayout(createLayout());
	vbox->addLayout(createButtons());
	setLayout(vbox);

	setInitvalues();
}

QGridLayout* PRMParamDialog::createLayout() {
	algoCombo = new cnoid::ComboBox();
	epsSpin = new cnoid::DoubleSpinBox();
	maxiterSpin = new cnoid::SpinBox();
	deltaSpin = new cnoid::DoubleSpinBox();
	smoothestepSpin = new cnoid::SpinBox();
	pathSpin = new cnoid::SpinBox();
	nnsearchCombo = new cnoid::ComboBox();
	radiusSpin = new cnoid::DoubleSpinBox();

	QStringList algolist;
	algolist << "SBL" << "RRT" << "RRT*" << "PRM";
	algoCombo->addItems(algolist);

	epsSpin->setAlignment(Qt::AlignCenter);
	epsSpin->setMinimum(0.001);
	epsSpin->setSingleStep(0.01);
	epsSpin->setDecimals(3);

	maxiterSpin->setAlignment(Qt::AlignCenter);
	maxiterSpin->setRange(1, 100000);

	deltaSpin->setAlignment(Qt::AlignCenter);
	deltaSpin->setMinimum(0.001);
	deltaSpin->setSingleStep(0.01);
	deltaSpin->setDecimals(3);
	
	smoothestepSpin->setAlignment(Qt::AlignCenter);
	smoothestepSpin->setRange(1, 1000);

	pathSpin->setAlignment(Qt::AlignCenter);
	pathSpin->setRange(1, 1000);

	nnsearchCombo->addItem("radius");
	nnsearchCombo->addItem("k-NN");

	radiusSpin->setAlignment(Qt::AlignCenter);
	radiusSpin->setMinimum(0.001);
	radiusSpin->setSingleStep(0.1);
	radiusSpin->setDecimals(2);

	algoCombo->sigCurrentIndexChanged().connect(boost::bind(&PRMParamDialog::algoChanged, this));
	nnsearchCombo->sigCurrentIndexChanged().connect(boost::bind(&PRMParamDialog::nnChanged, this));

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("algorithm")), 0, 0);
	grid->addWidget(algoCombo, 0, 1);
	grid->addWidget(new QLabel(tr("epsilon")), 1, 0);
	grid->addWidget(epsSpin, 1, 1);
	grid->addWidget(new QLabel(tr("max iteration (planner)")), 2, 0);
	grid->addWidget(maxiterSpin, 2, 1);
	grid->addWidget(new QLabel(tr("delta")), 3, 0);
	grid->addWidget(deltaSpin, 3, 1);
	grid->addWidget(new QLabel(tr("smoothe step")), 4, 0);
	grid->addWidget(smoothestepSpin, 4, 1);
	grid->addWidget(new QLabel(tr("number of paths (end condition)")), 5, 0);
	grid->addWidget(pathSpin, 5, 1);
	grid->addWidget(new QLabel(tr("NN search method")), 6, 0);
	grid->addWidget(nnsearchCombo, 6, 1);
	grid->addWidget(new QLabel(tr("radius")), 7, 0);
	grid->addWidget(radiusSpin, 7, 1);
	grid->setColumnMinimumWidth(1, 100);
	return grid;
}

QHBoxLayout* PRMParamDialog::createButtons() {
	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
	cnoid::PushButton* cancelButton = new cnoid::PushButton(tr("&Cancel"));

	okButton->sigClicked().connect(boost::bind(&PRMParamDialog::okClicked, this));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addStretch();
	hbox->addWidget(okButton);
	hbox->addWidget(cancelButton);
	return hbox;
}

void PRMParamDialog::setInitvalues() {
	PRMParams params;
	params.loadParams();
	algoCombo->setCurrentIndex(params.param.algo);
	epsSpin->setValue(params.param.eps);
	maxiterSpin->setValue(params.param.iteration_num);
	deltaSpin->setValue(params.param.delta);
	smoothestepSpin->setValue(params.param.smoothestep);
	pathSpin->setValue(params.param.num_path);
	nnsearchCombo->setCurrentIndex(params.param.nnmethod);
	radiusSpin->setValue(params.param.radius);
	algoChanged();
}

void PRMParamDialog::okClicked() {
	PRMParams params;
	params.param.algo = static_cast<PRMParams::ALGORITHM>(algoCombo->currentIndex());
	params.param.eps = epsSpin->value();
	params.param.iteration_num = maxiterSpin->value();
	params.param.delta = deltaSpin->value();
	params.param.smoothestep = smoothestepSpin->value();
	params.param.num_path = pathSpin->value();
	params.param.nnmethod = static_cast<PRMParams::NNMETHOD>(nnsearchCombo->currentIndex());
	params.param.radius = radiusSpin->value();
	params.saveParams();
}

void PRMParamDialog::algoChanged() {
	PRMParams::ALGORITHM algo = static_cast<PRMParams::ALGORITHM>(algoCombo->currentIndex());
	PRMParams::NNMETHOD method = static_cast<PRMParams::NNMETHOD>(nnsearchCombo->currentIndex());
	if ((algo == PRMParams::ALGO_SBL) || (algo == PRMParams::ALGO_RRT) || (algo == PRMParams::ALGO_PRM)) {
		smoothestepSpin->setEnabled(true);
		deltaSpin->setEnabled(algo == PRMParams::ALGO_SBL);
		pathSpin->setEnabled(false);
		nnsearchCombo->setEnabled(false);
		radiusSpin->setEnabled(false);
	} else if (algo == PRMParams::ALGO_RRTSTAR) {
		smoothestepSpin->setEnabled(false);
		deltaSpin->setEnabled(false);
		pathSpin->setEnabled(true);
		nnsearchCombo->setEnabled(true);
		radiusSpin->setEnabled(method == PRMParams::NN_RADIUS);
	}
}

void PRMParamDialog::nnChanged() {
	PRMParams::NNMETHOD method = static_cast<PRMParams::NNMETHOD>(nnsearchCombo->currentIndex());
	radiusSpin->setEnabled(method == PRMParams::NN_RADIUS);
}


#include "GenerateDescriptorDialog.h"

#include <QFileInfo>
#include <QSettings>

#include <boost/bind.hpp>

#include <pcl/pcl_config.h>

#include <cnoid/ExecutablePath>

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include "CVFHDescriptorManipulator.h"
#endif

GenDescriptorDialog::GenDescriptorDialog() : QDialog(cnoid::MainWindow::instance()) {
	readParams();

	setWindowTitle(tr("Generate descriptors"));

	QVBoxLayout* vbox = new QVBoxLayout();

	le_dir = new cnoid::LineEdit();
	le_dir->setText(QString(param.work_dir.c_str()));
	le_dir->setReadOnly(true);
	cnoid::PushButton* loadButton = new cnoid::PushButton(tr("select..."));

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("partial model dir")), 0, 0);
	grid->addWidget(le_dir, 0, 1);
	grid->addWidget(loadButton, 0, 2);
	grid->setColumnMinimumWidth(1, 300);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addLayout(grid);
	vbox->addLayout(hbox);

	cb_param = new cnoid::CheckBox();
	cb_param->setText(tr("parameters"));
	cb_param->setChecked(false);
	cb_param->setStyleSheet(
													 "QCheckBox::indicator:checked {image: url(extplugin/graspPlugin/PCL/icon/checked_arrow.png);}"
													 "QCheckBox::indicator:unchecked {image: url(extplugin/graspPlugin/PCL/icon/unchecked_arrow.png);}"
													 );

	QHBoxLayout* hbox_param = new QHBoxLayout();
	hbox_param->addWidget(cb_param);
	vbox->addLayout(hbox_param);

	ds_sampling = new cnoid::DoubleSpinBox();
	ds_sampling->setMinimum(0.0);
	ds_sampling->setMaximum(10.0);
	ds_sampling->setSingleStep(0.001);
	ds_sampling->setDecimals(4);
	ds_sampling->setValue(param.sampling_density);

	ds_normalradius = new cnoid::DoubleSpinBox();
	ds_normalradius->setMinimum(0.0);
	ds_normalradius->setMaximum(10.0);
	ds_normalradius->setSingleStep(0.001);
	ds_normalradius->setDecimals(4);
	ds_normalradius->setValue(param.normal_radius);

	QGridLayout* grid_param = new QGridLayout();
	grid_param->addWidget(new QLabel(tr("sampling density")), 0, 0);
	grid_param->addWidget(ds_sampling, 0, 1);
	grid_param->addWidget(new QLabel(tr("normal radius")), 1, 0);
	grid_param->addWidget(ds_normalradius, 1, 1);

	QHBoxLayout* hbox_paramsetting = new QHBoxLayout();
	hbox_paramsetting->addLayout(grid_param);
	hbox_paramsetting->addStretch();

	frame_param = new QFrame();
	frame_param->setLayout(hbox_paramsetting);
	frame_param->hide();
	frame_param->setFrameShape(QFrame::NoFrame);
	vbox->addWidget(frame_param);

	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
	okButton->setDefault(true);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(okButton);
	vbox->addLayout(hbox_button);

	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));

	cb_param->sigToggled().connect(boost::bind(&GenDescriptorDialog::cbParamToggled, this));
	loadButton->sigClicked().connect(boost::bind(&GenDescriptorDialog::selectDirClicked, this));
	okButton->sigClicked().connect(boost::bind(&GenDescriptorDialog::okClicked, this));

	setLayout(vbox);
}

void GenDescriptorDialog::selectDirClicked() {
	std::string default_path = param.work_dir;
	QString dirpath =  QFileDialog::getExistingDirectory(this, tr("Select partial model dir"), QString(default_path.c_str()), QFileDialog::ShowDirsOnly);
	if (!dirpath.isEmpty()) {
		le_dir->setText(dirpath);
	}
}

void GenDescriptorDialog::cbParamToggled() {
	frame_param->setVisible(cb_param->isChecked());
	this->adjustSize();
}

void GenDescriptorDialog::okClicked() {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
	CVFHDescriptorManipulator manip;
	NormalEstimator normal_est;
	normal_est.setRadius(ds_normalradius->value());
	manip.setNormalEstimator(&normal_est);
	if (manip.generateDescriptors(le_dir->text().toStdString(), ds_sampling->value())) {
		cnoid::MessageView::instance()->cout() << "Generating descriptors is finished" << std::endl;
	} else {
		cnoid::MessageView::instance()->cout() << "Generating descriptors is FAILED!!" << std::endl;
	}
#endif
}

void GenDescriptorDialog::readParams() {
	QString default_path = QString::fromStdString(cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PCL/");

	QSettings setting(default_path + "config.ini", QSettings::IniFormat);
	setting.beginGroup("GenerateDescriptors");
	param.sampling_density = setting.value("sampling_density", 0.005).toDouble();
	param.normal_radius = setting.value("radius_normal", 0.015).toDouble();
	QDir::setCurrent(default_path);
	param.work_dir = QFileInfo(setting.value("default_dir", default_path).toString()).absoluteFilePath().toStdString();
	QDir::setCurrent(QString::fromStdString(cnoid::executableTopDirectory()));
	setting.endGroup();
}


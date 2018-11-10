/**
 * @file   FittingDialog.cpp
 * @author Akira Ohchi
*/

#include "FittingDialog.h"

#include <string>
#include <vector>

#include <QString>
#include <QFileDialog>
#include <QFileInfo>

#include <boost/bind.hpp>

#include "Fitting.h"
#include "PrimitiveShapeParameter.h"

FittingDialog::FittingDialog()
	: QDialog(cnoid::MainWindow::instance()) {
	fitter = new Fitting();

	setWindowTitle(tr("Fitting primitive shapes"));

	le_file = new cnoid::LineEdit();

	cnoid::PushButton* selectButton = new cnoid::PushButton(tr("select..."));
	cnoid::PushButton* fittingButton = new cnoid::PushButton(tr("&Fitting"));
	cnoid::PushButton* loadButton = new cnoid::PushButton(tr("&Load"));
	cnoid::PushButton* saveButton = new cnoid::PushButton(tr("&Save"));
	cnoid::PushButton* closeButton = new cnoid::PushButton(tr("&Close"));

	le_file->setReadOnly(true);
	fittingButton->setDefault(true);

	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));

	selectButton->sigClicked().connect(boost::bind(&FittingDialog::selectFileClicked, this));
	fittingButton->sigClicked().connect(boost::bind(&FittingDialog::fittingClicked, this));
	loadButton->sigClicked().connect(boost::bind(&FittingDialog::loadClicked, this));
	saveButton->sigClicked().connect(boost::bind(&FittingDialog::saveClicked, this));

	QVBoxLayout* vbox = new QVBoxLayout();

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("Object YAML file")), 0, 0);
	grid->addWidget(le_file, 0, 1);
	grid->addWidget(selectButton, 0, 2);
	grid->setColumnMinimumWidth(1, 300);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addLayout(grid);
	vbox->addLayout(hbox);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(loadButton);
	hbox_button->addWidget(fittingButton);
	hbox_button->addWidget(saveButton);
	hbox_button->addWidget(closeButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

FittingDialog::~FittingDialog() {
	delete fitter;
}

void FittingDialog::setYamlPath(const std::string& path) {
	QFileInfo file_path(QString::fromStdString(path));
	if (file_path.suffix().compare("yaml", Qt::CaseInsensitive) == 0) {
		le_file->setText(path);
	} else {
		QFileInfo yaml_path(file_path.absoluteDir(), file_path.completeBaseName() + ".yaml");
		le_file->setText(yaml_path.absoluteFilePath().toStdString());
	}
}

void FittingDialog::selectFileClicked() {
	QString filepath =  QFileDialog::getSaveFileName(this, tr("Save file as"), le_file->text(), tr("YAML File (*.yaml)"));
	if (!filepath.isEmpty()) {
		le_file->setText(filepath);
	}
}

void FittingDialog::fittingClicked() {
	fitter->fit();
}

void FittingDialog::loadClicked() {
	PrimitiveShapeHandler psh;
	psh.loadYaml(le_file->text().toStdString());
}

void FittingDialog::saveClicked() {
	PrimitiveShapeHandler psh;
	std::vector<cnoid::Vector3> center;
	std::vector<cnoid::Vector3> dir;
	std::vector<double> radius;
	std::vector<double> len;
	fitter->getCylinders(center, dir, radius, len);
	for (int i = 0; i < center.size(); i++) {
		psh.addCylinder(center[i], dir[i], radius[i], len[i]);
	}
	std::vector<cnoid::Matrix3> R;
	std::vector<cnoid::Vector3> edge;
	fitter->getBoxes(R, center, edge);
	for (int i = 0; i < R.size(); i++) {
		psh.addBox(R[i], center[i], edge[i]);
	}
	psh.saveYaml(le_file->text().toStdString());
}


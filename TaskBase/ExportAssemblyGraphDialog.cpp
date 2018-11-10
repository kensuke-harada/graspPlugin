#include "ExportAssemblyGraphDialog.h"

#include <boost/bind.hpp>

#include <QtGui>

#include "Task.h"
#include "AssemblyGraphExporter.h"

using namespace grasp;

namespace {
	QString getPluginDirPath() {
		return QString::fromStdString(cnoid::executableTopDirectory() + "/ext/graspPlugin/TaskBase/");
	}
}

ExportAssemblyGraphDialog::ExportAssemblyGraphDialog() :
	QDialog(cnoid::MainWindow::instance()) {
	setWindowTitle("export graph");

	QVBoxLayout* vbox = new QVBoxLayout();

	vbox->addLayout(createLayout());
	vbox->addLayout(createButtons());
	setLayout(vbox);
}

QGridLayout* ExportAssemblyGraphDialog::createLayout() {
	le_graph_path = new cnoid::LineEdit();
	le_relation_path = new cnoid::LineEdit();
	cb_include_order = new cnoid::CheckBox();
	ds_time = new cnoid::DoubleSpinBox();
	cnoid::PushButton* loadButtonGraph = new cnoid::PushButton("select...");
	loadButtonGraph->sigClicked().connect(boost::bind(&ExportAssemblyGraphDialog::selectGraphClicked, this));
	cnoid::PushButton* loadButtonRelation = new cnoid::PushButton("select...");
	loadButtonRelation->sigClicked().connect(boost::bind(&ExportAssemblyGraphDialog::selectRelationClicked, this));
	
	le_graph_path->setReadOnly(true);
	le_relation_path->setReadOnly(true);

	cb_include_order->setText("include sequnetial order information");
	cb_include_order->setChecked(true);

	ds_time->setMinimum(0.01);
	ds_time->setMaximum(100);
	ds_time->setDecimals(2);
	ds_time->setSingleStep(0.1);
	ds_time->setMaximumWidth(100);
	ds_time->setValue(1.0);

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel("Assembly Graph YAML"), 0, 0);
	grid->addWidget(le_graph_path, 0, 1);
	grid->addWidget(loadButtonGraph, 0, 2);
	grid->addWidget(new QLabel("Relation YAML"), 1, 0);
	grid->addWidget(le_relation_path, 1, 1);
	grid->addWidget(loadButtonRelation, 1, 2);
	grid->addWidget(cb_include_order, 2, 0, 1, 3);
	grid->addWidget(new QLabel("Time for approach vector caluculation"), 3, 0);
	grid->addWidget(ds_time, 3, 1);
	grid->setColumnMinimumWidth(1, 400);
	return grid;
}

QHBoxLayout* ExportAssemblyGraphDialog::createButtons() {
	cnoid::PushButton* okButton = new cnoid::PushButton("&Export");
	cnoid::PushButton* cancelButton = new cnoid::PushButton("&Cancel");

	okButton->sigClicked().connect(boost::bind(&ExportAssemblyGraphDialog::okClicked, this));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addStretch();
	hbox->addWidget(okButton);
	hbox->addWidget(cancelButton);
	return hbox;
}

ExportAssemblyGraphDialog::~ExportAssemblyGraphDialog() {
}

void ExportAssemblyGraphDialog::setTaskItem(TaskItem* task) {
	task_ = task;
	QString plugin_dir = getPluginDirPath();
	le_graph_path->setText(plugin_dir + QString::fromStdString("assemblygraph_" + task->name() + ".yaml"));
	le_relation_path->setText(plugin_dir + QString::fromStdString("relation_" + task->name() + ".yaml"));
}

void ExportAssemblyGraphDialog::selectGraphClicked() {
	QString filepath = QFileDialog::getSaveFileName(this, "Export Assembly Graph as",
																									le_graph_path->text(), ("YAML file (*.yaml)"));
	if (!filepath.isEmpty()) {
		le_graph_path->setText(filepath);
	}
}

void ExportAssemblyGraphDialog::selectRelationClicked() {
	QString filepath = QFileDialog::getSaveFileName(this, "Export Object Realtions as",
																									le_relation_path->text(), ("YAML file (*.yaml)"));
	if (!filepath.isEmpty()) {
		le_relation_path->setText(filepath);
	}
}

void ExportAssemblyGraphDialog::okClicked() {
	AssemblyGraphExporter exporter;
	exporter.setTimeAppvecCalc(ds_time->value());
	bool ret = exporter.exportYaml(task_, le_graph_path->text().toStdString(),
																 le_relation_path->text().toStdString(), cb_include_order->isChecked());
}

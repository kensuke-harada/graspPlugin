// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include "GraspDataGenBar.h"
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/Archive>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>
#include <boost/format.hpp>
// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  

#include "GraspDataGenerator.h"
#include "WaistPositionSearcher.h"
#include "WaistPosSearchMotion.h"
#include "GraspPatternDialog.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

int GraspDataGenBar::count = 0;

SaveGraspPatternDialog::SaveGraspPatternDialog() : QDialog(cnoid::MainWindow::instance()) {
	setWindowTitle("SaveGraspPettern parameter");

	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	rb_cluster_bb = new cnoid::RadioButton();
	rb_cluster_bb->setText(tr("BoundingBox"));
	rb_cluster_bb->setChecked(true);
	rb_cluster_cylinder = new cnoid::RadioButton();
	rb_cluster_cylinder->setText(tr("Cylinder"));

	QVBoxLayout* vbox_type = new QVBoxLayout();
	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(rb_cluster_bb);
	hbox->addWidget(rb_cluster_cylinder);
	hbox->addStretch();
	vbox_type->addLayout(hbox);

	QGroupBox* gbox = new QGroupBox(tr("Clustering type"));
	gbox->setLayout(vbox_type);
	hbox = new QHBoxLayout();
	hbox->addWidget(gbox);
	vbox->addLayout(hbox);

	QVBoxLayout* vbox_param = new QVBoxLayout();

	rb_num_cluster = new cnoid::RadioButton();
	rb_num_cluster->setText(tr("Nubmer of clusters"));
	rb_volume_ratio = new cnoid::RadioButton();
	rb_volume_ratio->setText(tr("Overlap volume ratio"));
	rb_volume_ratio->setChecked(true);

	QButtonGroup* bgrp_terminatecond = new QButtonGroup();
	bgrp_terminatecond->setExclusive(true);
	bgrp_terminatecond->addButton(rb_num_cluster);
	bgrp_terminatecond->addButton(rb_volume_ratio);

	sb_num_cluster = new cnoid::SpinBox();
	sb_num_cluster->setAlignment(Qt::AlignCenter);
	sb_num_cluster->setMinimum(1);
	sb_num_cluster->setMaximum(200);
	sb_num_cluster->setValue(1);
	sb_num_cluster->setEnabled(false);

	ds_volume_ratio = new cnoid::DoubleSpinBox();
	ds_volume_ratio->setAlignment(Qt::AlignCenter);
	ds_volume_ratio->setMinimum(0.0);
	ds_volume_ratio->setMaximum(1.0);
	ds_volume_ratio->setSingleStep(0.1);
	ds_volume_ratio->setDecimals(2);
	ds_volume_ratio->setValue(0.2);

	ds_scale = new cnoid::DoubleSpinBox();
	ds_scale->setAlignment(Qt::AlignCenter);
	ds_scale->setMinimum(0.01);
	ds_scale->setMaximum(100);
	ds_scale->setSingleStep(0.1);
	ds_scale->setDecimals(2);
	ds_scale->setValue(1.00);	

	ds_dist_th = new cnoid::DoubleSpinBox();
	ds_dist_th->setAlignment(Qt::AlignCenter);
	ds_dist_th->setMinimum(0.0);
	ds_dist_th->setMaximum(1.0);
	ds_dist_th->setSingleStep(0.001);
	ds_dist_th->setDecimals(4);
	ds_dist_th->setValue(0.04);	
	ds_dist_th->setEnabled(false);

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(rb_num_cluster, 0, 0);
	grid->addWidget(sb_num_cluster, 0, 1);
	grid->addWidget(rb_volume_ratio, 1, 0);
	grid->addWidget(ds_volume_ratio, 1, 1);
	grid->addWidget(new QLabel(tr("Scale")), 2, 0);
	grid->addWidget(ds_scale, 2, 1);
	grid->addWidget(new QLabel(tr("DistanceThreshold")), 3, 0);
	grid->addWidget(ds_dist_th, 3, 1);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	hbox->addLayout(grid);
	hbox->addStretch();
	vbox_param->addLayout(hbox);

	gbox = new QGroupBox(tr("Clustering parameters"));
	gbox->setLayout(vbox_param);
	hbox = new QHBoxLayout();
	hbox->addWidget(gbox);
	vbox->addLayout(hbox);

	QVBoxLayout* vbox_grasp_type = new QVBoxLayout();

	rb_single = new cnoid::RadioButton();
	rb_single->setText(tr("grasp with single hand"));
	rb_single->setChecked(true);
	rb_liftup = new cnoid::RadioButton();
	rb_liftup->setText(tr("lift up with dual arm"));
	rb_fromsideOCP = new cnoid::RadioButton();
	rb_fromsideOCP->setText(tr("grasp with dual arm (rotate OCP)"));
	rb_fromsidestable = new cnoid::RadioButton();
	rb_fromsidestable->setText(tr("grasp with dual arm (placed in a stable)"));

	vbox_grasp_type->addWidget(rb_single);
	vbox_grasp_type->addWidget(rb_liftup);
	vbox_grasp_type->addWidget(rb_fromsideOCP);
	vbox_grasp_type->addWidget(rb_fromsidestable);

	gbox = new QGroupBox(tr("Grasp type"));
	gbox->setLayout(vbox_grasp_type);
	hbox = new QHBoxLayout();
	hbox->addWidget(gbox);
	vbox->addLayout(hbox);

	ds_grid_interval = new cnoid::DoubleSpinBox();
	ds_grid_interval->setAlignment(Qt::AlignCenter);
	ds_grid_interval->setMinimum(0.0001);
	ds_grid_interval->setSingleStep(0.01);
	ds_grid_interval->setDecimals(3);
	ds_grid_interval->setValue(0.010);

	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("Distance between search points[m]")));
	hbox->addWidget(ds_grid_interval);
	hbox->addStretch();
	vbox->addLayout(hbox);

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);

	vbox->addWidget(okButton);

	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	
	rb_volume_ratio->sigToggled().connect(boost::bind(&SaveGraspPatternDialog::rbVolumeRatioToggled,this));
	rb_cluster_cylinder->sigToggled().connect(boost::bind(&SaveGraspPatternDialog::rbClusterCylinderToggled,this));
	okButton->sigClicked().connect(boost::bind(&SaveGraspPatternDialog::okClicked, this));
	
	setLayout(vbox);
}

void SaveGraspPatternDialog::okClicked(){
	this->hide();
	GraspDataGenerator* gdg = GraspDataGenerator::instance();
	gdg->num_cluster =rb_volume_ratio->isChecked() ? 1 : sb_num_cluster->value();
	gdg->depth_grid_interval = ds_grid_interval->value();
	gdg->vol_ratio_threshold = rb_volume_ratio->isChecked() ? ds_volume_ratio->value() : 0.0;
	gdg->scale = ds_scale->value();
	gdg->dist_th = ds_dist_th->value();
	if(rb_single->isChecked()) gdg->grasp_type = GraspDataGenerator::T_SINGLE;
	if(rb_liftup->isChecked()) gdg->grasp_type = GraspDataGenerator::T_LIFTUP;
	if(rb_fromsideOCP->isChecked()) gdg->grasp_type = GraspDataGenerator::T_SIDEDUAL;
	gdg->rotation_type = rb_fromsidestable->isChecked() ? GraspDataGenerator::R_STABLE : GraspDataGenerator::R_OCP;
	gdg->clustering_type = (rb_cluster_cylinder->isChecked()) ? GraspDataGenerator::C_CYLINDER : GraspDataGenerator::C_BOUNDINGBOX;
	gdg->generateGraspPattern();

}

void SaveGraspPatternDialog::rbVolumeRatioToggled(){
	ds_volume_ratio->setEnabled(rb_volume_ratio->isChecked());
	sb_num_cluster->setEnabled(!rb_volume_ratio->isChecked());
}

void SaveGraspPatternDialog::rbClusterCylinderToggled(){
	ds_dist_th->setEnabled(rb_cluster_cylinder->isChecked());
	if(rb_cluster_cylinder->isChecked()) {
		rb_num_cluster->setChecked(true);
	}
	rb_volume_ratio->setEnabled(!(rb_cluster_cylinder->isChecked()));
	ds_scale->setEnabled(!(rb_cluster_cylinder->isChecked()));
}

ShowBoundingBoxDialog::ShowBoundingBoxDialog() : QDialog(cnoid::MainWindow::instance()) {
	setWindowTitle(tr("Show Bounding Box"));
	
	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	rb_show_bb = new cnoid::RadioButton();
	rb_show_bb->setText(tr("BoundingBox"));
	rb_show_bb->setChecked(true);
	rb_show_bbmesh = new cnoid::RadioButton();
	rb_show_bbmesh->setText(tr("Cluster mesh"));
	rb_show_cylinder = new cnoid::RadioButton();
	rb_show_cylinder->setText(tr("Cylinder"));
	cb_is_showonlytarget = new cnoid::CheckBox();
	cb_is_showonlytarget->setText(tr("only target cluseter"));

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(rb_show_bb, 0, 0);
	grid->addWidget(rb_show_bbmesh, 1, 0);
	grid->addWidget(rb_show_cylinder, 2, 0);
	grid->addWidget(cb_is_showonlytarget, 0, 1);

	QGroupBox* gbox = new QGroupBox(tr("Type"));
	gbox->setLayout(grid);
	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(gbox);
	vbox->addLayout(hbox);


	QVBoxLayout* vbox_param = new QVBoxLayout();

	rb_num_cluster = new cnoid::RadioButton();
	rb_num_cluster->setText(tr("Nubmer of clusters"));
	rb_volume_ratio = new cnoid::RadioButton();
	rb_volume_ratio->setText(tr("Overlap volume ratio"));
	rb_volume_ratio->setChecked(true);

	QButtonGroup* bgrp_terminatecond = new QButtonGroup();
	bgrp_terminatecond->setExclusive(true);
	bgrp_terminatecond->addButton(rb_num_cluster);
	bgrp_terminatecond->addButton(rb_volume_ratio);

	sb_num_cluster = new cnoid::SpinBox();
	sb_num_cluster->setAlignment(Qt::AlignCenter);
	sb_num_cluster->setMinimum(1);
	sb_num_cluster->setMaximum(200);
	sb_num_cluster->setValue(1);
	sb_num_cluster->setEnabled(false);

	ds_volume_ratio = new cnoid::DoubleSpinBox();
	ds_volume_ratio->setAlignment(Qt::AlignCenter);
	ds_volume_ratio->setMinimum(0.0);
	ds_volume_ratio->setMaximum(1.0);
	ds_volume_ratio->setSingleStep(0.1);
	ds_volume_ratio->setDecimals(2);
	ds_volume_ratio->setValue(0.2);

	ds_scale = new cnoid::DoubleSpinBox();
	ds_scale->setAlignment(Qt::AlignCenter);
	ds_scale->setMinimum(0.01);
	ds_scale->setMaximum(100);
	ds_scale->setSingleStep(0.1);
	ds_scale->setDecimals(2);
	ds_scale->setValue(1.00);	

	ds_dist_th = new cnoid::DoubleSpinBox();
	ds_dist_th->setAlignment(Qt::AlignCenter);
	ds_dist_th->setMinimum(0.0);
	ds_dist_th->setMaximum(1.0);
	ds_dist_th->setSingleStep(0.001);
	ds_dist_th->setDecimals(4);
	ds_dist_th->setValue(0.04);	
	ds_dist_th->setEnabled(false);

	grid = new QGridLayout();
	grid->addWidget(rb_num_cluster, 0, 0);
	grid->addWidget(sb_num_cluster, 0, 1);
	grid->addWidget(rb_volume_ratio, 1, 0);
	grid->addWidget(ds_volume_ratio, 1, 1);
	grid->addWidget(new QLabel(tr("Scale")), 2, 0);
	grid->addWidget(ds_scale, 2, 1);
	grid->addWidget(new QLabel(tr("DistanceThreshold")), 3, 0);
	grid->addWidget(ds_dist_th, 3, 1);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	hbox->addLayout(grid);
	hbox->addStretch();
	vbox_param->addLayout(hbox);

	gbox = new QGroupBox(tr("Parameters"));
	gbox->setLayout(vbox_param);
	hbox = new QHBoxLayout();
	hbox->addWidget(gbox);
	vbox->addLayout(hbox);

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);

	vbox->addWidget(okButton);

	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));

	rb_volume_ratio->sigToggled().connect(boost::bind(&ShowBoundingBoxDialog::rbVolumeRatioToggled,this));
	rb_show_cylinder->sigToggled().connect(boost::bind(&ShowBoundingBoxDialog::rbShowCylinderToggled,this));
	okButton->sigClicked().connect(boost::bind(&ShowBoundingBoxDialog::okClicked, this));
	
	setLayout(vbox);
}

void ShowBoundingBoxDialog::okClicked(){
	GraspDataGenerator* gdg = GraspDataGenerator::instance();
	gdg->num_cluster = rb_volume_ratio->isChecked() ? 1 : sb_num_cluster->value();
	gdg->vol_ratio_threshold = rb_volume_ratio->isChecked() ? ds_volume_ratio->value() : 0.0;
	gdg->is_show_all_cluster = !(cb_is_showonlytarget->isChecked());
	gdg->scale = ds_scale->value();
	MessageView::mainInstance()->cout() << "Number of Clusters:" << sb_num_cluster->value() << endl;
	gdg->dist_th = ds_dist_th->value();
	if (rb_show_bbmesh->isChecked()){
		gdg->showClusterMesh();
	} else if (rb_show_cylinder->isChecked()) {
		gdg->showCylinder();
	} else {
		gdg->showBoundingBox();
	}
}

void ShowBoundingBoxDialog::rbVolumeRatioToggled(){
	ds_volume_ratio->setEnabled(rb_volume_ratio->isChecked());
	sb_num_cluster->setEnabled(!rb_volume_ratio->isChecked());
}

void ShowBoundingBoxDialog::rbShowCylinderToggled(){
	ds_dist_th->setEnabled(rb_show_cylinder->isChecked());
	if(rb_show_cylinder->isChecked()) {
		rb_num_cluster->setChecked(true);
	}
	rb_volume_ratio->setEnabled(!(rb_show_cylinder->isChecked()));
	ds_scale->setEnabled(!(rb_show_cylinder->isChecked()));
}

ScalingHandDialog::ScalingHandDialog() : QDialog(cnoid::MainWindow::instance()) {
	
	setWindowTitle("Scaling hand");

	QButtonGroup* bgrp_scale = new QButtonGroup();
	bgrp_scale->setExclusive(true);
	
	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Scaling target:"));
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	rbxyz.setText("xyz-axis");
	rbxyz.setChecked(true);
	bgrp_scale->addButton(&rbxyz);
	hbox->addWidget(&rbxyz);
	hbox->addStretch();
	vbox->addLayout(hbox);


	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	rbx.setText("x-axis");
	bgrp_scale->addButton(&rbx);
	hbox->addWidget(&rbx);

	hbox->addSpacing(5);
	rby.setText("y-axis");
	bgrp_scale->addButton(&rby);
	hbox->addWidget(&rby);

	hbox->addSpacing(5);
	rbz.setText("z-axis");
	bgrp_scale->addButton(&rbz);
	hbox->addWidget(&rbz);
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	rbxyz.sigToggled().connect(boost::bind(&ScalingHandDialog::rbToggled,this));
	rbx.sigToggled().connect(boost::bind(&ScalingHandDialog::rbToggled,this));
	rby.sigToggled().connect(boost::bind(&ScalingHandDialog::rbToggled,this));
	rbz.sigToggled().connect(boost::bind(&ScalingHandDialog::rbToggled,this));
	
	hbox = new QHBoxLayout();
	lbAxis = new QLabel("Scale range(xyz):");
	hbox->addWidget(lbAxis);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	hbox->addWidget(new QLabel(" start "));
	scaleStart.setAlignment(Qt::AlignCenter);
	scaleStart.setMinimum(0.01);
	scaleStart.setMaximum(10.0);
	scaleStart.setSingleStep(0.1);
	scaleStart.setValue(1.00);
	hbox->addWidget(&scaleStart);
	hbox->addWidget(new QLabel(" end "));
	scaleEnd.setAlignment(Qt::AlignCenter);
	scaleEnd.setMinimum(0.01);
	scaleEnd.setMaximum(10.0);
	scaleEnd.setSingleStep(0.1);
	scaleEnd.setValue(1.00);
	hbox->addWidget(&scaleEnd);
	hbox->addWidget(new QLabel(" step "));
	scaleStep.setAlignment(Qt::AlignCenter);
	scaleStep.setMinimum(0.01);
	scaleStep.setMaximum(10.0);
	scaleStep.setSingleStep(0.5);
	scaleStep.setValue(0.5);
	hbox->addWidget(&scaleStep);
	hbox->addStretch();
	vbox->addLayout(hbox);


	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Scale:"));
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	hbox->addWidget(new QLabel(" x "));
	scaleX.setAlignment(Qt::AlignCenter);
	scaleX.setMinimum(0.01);
	scaleX.setMaximum(10.0);
	scaleX.setSingleStep(0.1);
	scaleX.setValue(1.00);
	scaleX.setEnabled(false);
	hbox->addWidget(&scaleX);
	hbox->addWidget(new QLabel(" y "));
	scaleY.setAlignment(Qt::AlignCenter);
	scaleY.setMinimum(0.01);
	scaleY.setMaximum(10.0);
	scaleY.setSingleStep(0.1);
	scaleY.setValue(1.00);
	scaleY.setEnabled(false);
	hbox->addWidget(&scaleY);
	hbox->addWidget(new QLabel(" z "));
	scaleZ.setAlignment(Qt::AlignCenter);
	scaleZ.setMinimum(0.01);
	scaleZ.setMaximum(10.0);
	scaleZ.setSingleStep(0.1);
	scaleZ.setValue(1.00);
	hbox->addWidget(&scaleZ);
	scaleZ.setEnabled(false);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("Clustering parameters:"));
	hbox->addStretch();
	vbox->addLayout(hbox);

	QButtonGroup* bgrp_cluster = new QButtonGroup();
	bgrp_cluster->setExclusive(true);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	rbNumber.setText("Nubmer of clusters");
	bgrp_cluster->addButton(&rbNumber);
	hbox->addWidget(&rbNumber);
	clusterNumber.setAlignment(Qt::AlignCenter);
	clusterNumber.setMinimum(1);
	clusterNumber.setValue(1);
	clusterNumber.setEnabled(false);
	hbox->addWidget(&clusterNumber);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	rbVolumeRatio.setText("Overlap volume ratio");
	rbVolumeRatio.setChecked(true);
	bgrp_cluster->addButton(&rbVolumeRatio);
	hbox->addWidget(&rbVolumeRatio);
	overlapVolumeRatio.setAlignment(Qt::AlignCenter);
	overlapVolumeRatio.setMinimum(0.0);
	overlapVolumeRatio.setMaximum(1.0);
	overlapVolumeRatio.setSingleStep(0.1);
	overlapVolumeRatio.setDecimals(2);
	overlapVolumeRatio.setValue(0.2);
	hbox->addWidget(&overlapVolumeRatio);
	hbox->addStretch();
	vbox->addLayout(hbox);
	rbVolumeRatio.sigToggled().connect(boost::bind(&ScalingHandDialog::rbVolumeRatioToggled,this));

	hbox = new QHBoxLayout();
	hbox->addSpacing(10);
	hbox->addWidget(new QLabel("Distance between search points[m]"));
	gridInterval.setAlignment(Qt::AlignCenter);
	gridInterval.setMinimum(0.0001);
	gridInterval.setSingleStep(0.01);
	gridInterval.setDecimals(3);
	gridInterval.setValue(0.010);
	hbox->addWidget(&gridInterval);
	hbox->addStretch();
	vbox->addLayout(hbox);

	cnoid::PushButton* okButton = new cnoid::PushButton("&OK");
	okButton->setDefault(true);
	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
	okButton->sigClicked().connect(boost::bind(&ScalingHandDialog::okClicked, this));
	
	vbox->addWidget(okButton);
}

void ScalingHandDialog::okClicked(){
	this->hide();
	double start_x,start_y,start_z,end;
	GraspDataGenerator::SearchMode mode;

	end = scaleEnd.value();
	start_x = scaleX.value();
	start_y = scaleY.value();
	start_z = scaleZ.value();

	if(rbx.isChecked()){
		start_x = scaleStart.value();
		mode = GraspDataGenerator::SCALE_X;
	}else if(rby.isChecked()){
		start_y = scaleStart.value();
		mode = GraspDataGenerator::SCALE_Y;
	}else if(rbz.isChecked()){
		start_z = scaleStart.value();
		mode = GraspDataGenerator::SCALE_Z;
	}else{
		start_x = start_y = start_z = scaleStart.value();
		mode = GraspDataGenerator::SCALE_XYZ;
	}

	GraspDataGenerator::instance()->num_cluster =rbVolumeRatio.isChecked() ? 1 : clusterNumber.value();
	GraspDataGenerator::instance()->depth_grid_interval = gridInterval.value();
	GraspDataGenerator::instance()->vol_ratio_threshold = rbVolumeRatio.isChecked() ? overlapVolumeRatio.value() : 0.0;
	GraspDataGenerator::instance()->scale = 1.0;
	GraspDataGenerator::instance()->searchHandScale(start_x,start_y,start_z,end,scaleStep.value(),mode);
}

void ScalingHandDialog::rbToggled(){
	scaleX.setEnabled(!rbx.isChecked());
	scaleY.setEnabled(!rby.isChecked());
	scaleZ.setEnabled(!rbz.isChecked());
	if(rbx.isChecked()) lbAxis->setText("Scale range(x):");
	if(rby.isChecked()) lbAxis->setText("Scale range(y):");
	if(rbz.isChecked()) lbAxis->setText("Scale range(z):");
	if(rbxyz.isChecked()){
		lbAxis->setText("Scale range(xyz):");
		scaleX.setEnabled(false);
		scaleY.setEnabled(false);
		scaleZ.setEnabled(false);
	}
}

void ScalingHandDialog::rbVolumeRatioToggled(){
	overlapVolumeRatio.setEnabled(rbVolumeRatio.isChecked());
	clusterNumber.setEnabled(!rbVolumeRatio.isChecked());
}

GraspDialog::GraspDialog() : QDialog(cnoid::MainWindow::instance()),
							 os(cnoid::MessageView::instance()->cout()) {
	pDialog = NULL;
	modify_flag_ = false;
	setWindowTitle("Grasp");
	
	QVBoxLayout* vbox = new QVBoxLayout();
	setLayout(vbox);
	
	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel("target number"));
	targetNumber.setAlignment(Qt::AlignCenter);
	targetNumber.setRange(1, 500);
	targetNumber.setValue(1);
	hbox->addWidget(&targetNumber);
	hbox->addStretch();
	vbox->addLayout(hbox);
	
	hbox = new QHBoxLayout();
	isShowContactCluster = new QCheckBox("show contact cluseter");
	hbox->addWidget(isShowContactCluster);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	isShowHandContactCluster = new QCheckBox("show hand contact cluseter");
	hbox->addWidget(isShowHandContactCluster);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	rb_move_obj = new cnoid::RadioButton();
	rb_move_obj->setText(tr("move object"));
	hbox->addWidget(rb_move_obj);
	rb_move_arm = new cnoid::RadioButton();
	rb_move_arm->setText(tr("move arm"));
	hbox->addWidget(rb_move_arm);
	rb_move_obj->setChecked(true);
	hbox->addStretch();
	vbox->addLayout(hbox);

	vbox->addStretch();

	hbox = new QHBoxLayout();
	cnoid::PushButton* okButton = new cnoid::PushButton("&Grasp");
	cnoid::PushButton* modifyButton = new cnoid::PushButton("&Modify");
	cnoid::PushButton* closeButton = new cnoid::PushButton("&Close");
	hbox->addWidget(okButton);
	hbox->addWidget(modifyButton);
	hbox->addWidget(closeButton);
	hbox->addStretch();
	okButton->setDefault(true);
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));
	okButton->sigClicked().connect(boost::bind(&GraspDialog::okClicked, this));
	targetNumber.sigValueChanged().connect(boost::bind(&GraspDialog::okClicked, this));
	modifyButton->sigClicked().connect(boost::bind(&GraspDialog::modifyClicked, this));
	
	vbox->addLayout(hbox);
}

GraspDialog::~GraspDialog() {
}

void GraspDialog::okClicked(){
	if(!(GraspDataGenerator::instance()->grasp(targetNumber.value(), rb_move_arm->isChecked()))) {
		os << "Grasping failure at " << targetNumber.value() << "-th grasping data" << std::endl;
		return;
	}
	os << "Grasping at " << targetNumber.value() << "-th grasping data" << std::endl;
	
	if(isShowContactCluster->isChecked() || isShowHandContactCluster->isChecked()){
		SoftFingerStabilityHandler::instance()->initialize();
		SoftFingerStabilityHandler::instance()->showContactCluster(targetNumber.value(),isShowContactCluster->isChecked(),isShowHandContactCluster->isChecked());
		SoftFingerStabilityHandler::instance()->displayClusterData();
	}
}

void GraspDialog::afterFinishModifyData(bool is_done) {
	this->show();
	modify_flag_ |= is_done;
}

void GraspDialog::reject() {
	finalize();
	QDialog::reject();
}

void GraspDialog::accept() {
	finalize();
	QDialog::accept();
}

void GraspDialog::modifyClicked() {
	if (!GraspDataUpdater::instance()->init()) {
		return;
	}
	if(!(GraspDataGenerator::instance()->grasp(targetNumber.value(), rb_move_arm->isChecked()))) {
		os << "Grasping failure at " << targetNumber.value() << "-th grasping data" << std::endl;
		return;
	}
	if (!pDialog) {
		pDialog = new ModifyPatternDialog();
		pDialog->sigFinished().connect(boost::bind(&GraspDialog::afterFinishModifyData, this, _1));
	}
	pDialog->setTargetID(targetNumber.value() - 1);
	pDialog->show();
	this->hide();
}

void GraspDialog::finalize() {
	if (modify_flag_ && pDialog) {
		os << "update grasp database" << std::endl;
		pDialog->updateDataBase();
	}
	modify_flag_ = false;
}

GraspDataGenBar* GraspDataGenBar::instance()
{
	static GraspDataGenBar* instance = new GraspDataGenBar();
	return instance;
}



GraspDataGenBar::GraspDataGenBar()
	: ToolBar("GraspDataGenBar"),
	  mes(*MessageView::mainInstance()),
   	os (MessageView::mainInstance()->cout() ),
		prehen_param_dialog(NULL),
		append_pattern_dialog(NULL)
{
	
	addSeparator();
	
	addLabel(("=DataGen="));

	addButton(("GenerateGraspPattern"), ("Generate grasp pattern"))->
		sigClicked().connect(bind(&GraspDataGenBar::onGenerateGraspPattern, this));

	addButton(("AppendGraspPattern"), ("Append grasp pattern manually"))->
		sigClicked().connect(bind(&GraspDataGenBar::onAddGraspPattern, this));

	addButton(("DisplayBoundingBox"), ("Display Bounding Box of each cluseter"))->
		sigClicked().connect(bind(&GraspDataGenBar::onDisplayBoundingBox, this));
#ifdef DEBUG_MODE
	addButton(("DisplayContactPoint"),("Display contact points"))->
		sigClicked().connect(bind(&GraspDataGenBar::onDisplayContactPoint, this));
	addButton(("DisplayApproachPoint"),("Display approach points"))->
		sigClicked().connect(bind(&GraspDataGenBar::onDisplayApproachPoint, this));
#endif	
	addButton(("SearchHandScale"), ("Search scaling hand"))->
		sigClicked().connect(bind(&GraspDataGenBar::onSearchHandScale,this));
#ifdef DEBUG_MODE
	addButton(("SeqGenGraspPattern"), ("Genrate grap pattern"))->
		sigClicked().connect(bind(&GraspDataGenBar::onSeqGenerateGraspPattern, this));
#endif
	addButton(("Grasp"),("Grasp"))->
		sigClicked().connect(bind(&QDialog::show, &grasp));

	addButton(("SearchWaistPos"),("Search waist position"))->
		sigClicked().connect(bind(&GraspDataGenBar::onSearchWaistPos,this));

	addButton(("DisplayWaistPos"),("Display waist positions"))->
		sigClicked().connect(bind(&GraspDataGenBar::onDisplayWaistPosResult,this));

	addButton(("WaistPosMotion"),("Generate motions"))->
		sigClicked().connect(bind(&GraspDataGenBar::onGenerateMotion,this));

	addButton(("SetCollectionBox"),("Set selected bodyitem as a collection box"))->
		sigClicked().connect(bind(&GraspDataGenBar::onSetCollectionBox,this));

	addButton(("SetCollectionObj"),("Set selected bodyitem as a collected object"))->
		sigClicked().connect(bind(&GraspDataGenBar::onSetCollectedObj,this));

	addButton(("ClearCollection"),("Clear collection box and collected objects"))->
		sigClicked().connect(bind(&GraspDataGenBar::onClearCollection,this));

	addButton(("Param"), ("Generate prm file"))->
		sigClicked().connect(bind(&GraspDataGenBar::onParam, this));

		addSeparator();
	// show_all_children();	/* modified by qtconv.rb 7th rule*/  

	ItemTreeView::mainInstance()->sigSelectionChanged().connect(
		bind(&GraspDataGenBar::onItemSelectionChanged, this, _1));
	count++;
}


GraspDataGenBar::~GraspDataGenBar()
{
	if (prehen_param_dialog != NULL) {
		delete prehen_param_dialog;
	}
	if (append_pattern_dialog != NULL) {
		delete append_pattern_dialog;
	}

	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	count--;
}


/**
   \todo ItemTreeView::sigSelectionChanged() should be emitted
   after the final selection state has been determined.
*/
bool GraspDataGenBar::makeSingleSelection(BodyItemPtr bodyItem)
{
	ItemTreeView* tree = ItemTreeView::mainInstance()->mainInstance();

	ItemList<BodyItem> prevSelected = selectedBodyItems_;

	for(size_t i=0; i < prevSelected.size(); ++i){
#ifdef CNOID_10_11_12_13
		BodyItem* item = prevSelected[i];
#else
		BodyItem* item = prevSelected.get(i);
#endif
		if(item != bodyItem && tree->isItemSelected(item)){
			tree->selectItem(item, false);
		}
	}

	bool isSelected = tree->isItemSelected(bodyItem);
	if(!isSelected){
		isSelected = tree->selectItem(bodyItem, true);
	}

	return isSelected;
}


void GraspDataGenBar::onItemSelectionChanged(const ItemList<BodyItem>& bodyItems)
{
	bool selectedBodyItemsChanged = false;
	if(count < 1) return;
	if(selectedBodyItems_ != bodyItems){
		selectedBodyItems_ = bodyItems;
		selectedBodyItemsChanged = true;
	}

	BodyItemPtr firstItem = bodyItems.toSingle();

	if(firstItem && firstItem != currentBodyItem_){
		currentBodyItem_ = firstItem;
		connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
		connectionOfCurrentBodyItemDetachedFromRoot = currentBodyItem_->sigDetachedFromRoot().connect(
			bind(&GraspDataGenBar::onBodyItemDetachedFromRoot, this));
		sigCurrentBodyItemChanged_(currentBodyItem_.get());
	}

	if(selectedBodyItemsChanged){
		sigBodyItemSelectionChanged_(selectedBodyItems_);
	}

	targetBodyItems.clear();
	if(selectedBodyItems_.empty()){
//		if(currentBodyItem_){
//			targetBodyItems.push_back(currentBodyItem_);
//		}
	} else {
		targetBodyItems = selectedBodyItems_;
	}
}


void GraspDataGenBar::onBodyItemDetachedFromRoot()
{
	currentBodyItem_ = 0;
	connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
	sigCurrentBodyItemChanged_(0);
}


void GraspDataGenBar::onGenerateGraspPattern(){
	os << "generate GraspPattern" << endl;

	PlanBase* tc = PlanBase::instance();
	bool init = tc->initial();
	if(!init){
		os << "Failed: Grasp Planning Initial" << endl;
		return;
	}

	SaveGraspPatternDialog* SGPDialog = new SaveGraspPatternDialog ();
	SGPDialog->exec();

	delete SGPDialog;
}

void GraspDataGenBar::onAddGraspPattern() {
	if (!GraspDataAppender::instance()->init()) {
		return;
	}
	ClusteringMethodSelectionDialog* dialog = new ClusteringMethodSelectionDialog();
	if (dialog->exec() == QDialog::Accepted) {
		if (append_pattern_dialog == NULL) {
			append_pattern_dialog = new AppendPatternDialog();
		}
		append_pattern_dialog->show();
	}
}

void GraspDataGenBar::onDisplayBoundingBox(){
	PlanBase* tc = PlanBase::instance();
	bool init = tc->initial();
	if(!init){
		os << "Failed: Grasp Planning Initial" << endl;
		return;
	}
	
	ShowBoundingBoxDialog* SBBDialog = new ShowBoundingBoxDialog ();
	SBBDialog->exec();

	delete SBBDialog;
}

void GraspDataGenBar::onDisplayContactPoint(){
	GraspDataGenerator::instance()->showContactPoint();
}

void GraspDataGenBar::onDisplayApproachPoint(){
	GraspDataGenerator::instance()->showApproachPoint();
}

void GraspDataGenBar::onSearchHandScale(){
	ScalingHandDialog* SHDialog = new ScalingHandDialog();
	SHDialog->exec();

	delete SHDialog;
}

void GraspDataGenBar::onSeqGenerateGraspPattern(){

	 QFileDialog dialog(MainWindow::instance());
	 dialog.setWindowTitle(tr("Select list of target hrp file path"));
	 dialog.setFileMode(QFileDialog::ExistingFiles);
   dialog.setViewMode(QFileDialog::List);
   dialog.setLabelText(QFileDialog::Accept, tr("Open"));
   dialog.setLabelText(QFileDialog::Reject, tr("Cancel"));

	 if(dialog.exec()){
		 GraspDataGenerator::instance()->seqGenerateGraspPattern(dialog.selectedFiles()[0].toStdString());
	 }
}

void GraspDataGenBar::onSearchWaistPos(){
	WaistPosSearchDialog* WPDialog = new WaistPosSearchDialog();
	WPDialog->exec();
}

void GraspDataGenBar::onDisplayWaistPosResult() {
	WaistPosSearchResultViewDialog* WPSDialog =  new WaistPosSearchResultViewDialog();
	WPSDialog->exec();
}

void GraspDataGenBar::onGenerateMotion() {
	WaistPosSearchMotion::instance()->addMotion();
}

void GraspDataGenBar::onSetCollectionBox() {
	if (targetBodyItems.size() == 1) {
		WaistPosSearchMotion::instance()->setCollectionBox(targetBodyItems[0]);
		os << PlanBase::instance()->targetObject->bodyItemObject->name() << " is collection box"<< endl;
	} else {
		os <<  "Please select one bodyitem" << endl;
	}
}

void GraspDataGenBar::onSetCollectedObj() {
	if (targetBodyItems.size() == 1) {
		WaistPosSearchMotion::instance()->setCollectedObj(targetBodyItems[0]);
		os << PlanBase::instance()->targetObject->bodyItemObject->name() << " is collected"<< endl;
	} else {
		os <<  "Please select one bodyitem" << endl;
	}
}

void GraspDataGenBar::onClearCollection() {
	WaistPosSearchMotion::instance()->clearCollection();
}


void GraspDataGenBar::onParam() {
	if (PlanBase::instance()->targetArmFinger == NULL) {
		os << "Please set target robot" << endl;
		return;
	}

	if (prehen_param_dialog != NULL) {
		delete prehen_param_dialog;
	}
	prehen_param_dialog = new PrehensionParamDialog(PlanBase::instance()->targetArmFinger);
	prehen_param_dialog->updateTabs();

	prehen_param_dialog->show();

}

bool GraspDataGenBar::storeState(Archive& archive)
{
	return true;
}


bool GraspDataGenBar::restoreState(const Archive& archive)
{
	return true;
}

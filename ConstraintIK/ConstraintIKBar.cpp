// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <iostream>
#include "ConstraintIKBar.h"
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include <boost/bind.hpp>

#include "ConstraintIK.h"
#include "../Grasp/DrawUtility.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

ConstraintIKBar* ConstraintIKBar::instance()
{
  static ConstraintIKBar* instance = new ConstraintIKBar();
  return instance;
}

ConstraintIKBar::ConstraintIKBar()
  : ToolBar("ConstraintIK"),
  mes(*MessageView::mainInstance()),
  os (MessageView::mainInstance()->cout() )
{

  addSeparator();

  addLabel(("=ConstraintIK="));

  addButton(("SaveObstacleShape"), ("Save obstacle shape information"))->
    sigClicked().connect(bind(&ConstraintIKBar::onSaveObstacle, this));

  addButton(("DisplayObstacleShape"), ("Display obstacle shapes"))->
    sigClicked().connect(bind(&ConstraintIKBar::onDisplayObstacle, this));

  addButton(("HideObstacleShape"), ("Hide obstacle shapes"))->
    sigClicked().connect(bind(&ConstraintIKBar::onHideObstacle, this));

  addSeparator();
}

ConstraintIKBar::~ConstraintIKBar()
{
  // do nothing
}

bool ConstraintIKBar::storeState(Archive& archive)
{
  return true;
}


bool ConstraintIKBar::restoreState(const Archive& archive)
{
  return true;
}

void ConstraintIKBar::onSaveObstacle()
{
  SaveObstacleShapeDialog* SOSDialog = new SaveObstacleShapeDialog();
  SOSDialog->show();
}

void ConstraintIKBar::onDisplayObstacle()
{
	ObstacleShapeDrawer osd;
	osd.show();
}

void ConstraintIKBar::onHideObstacle()
{
  DrawUtility* draw = DrawUtility::instance();
  draw->clear();
  // draw->ellipsoids.clear();
  // draw->displayEllipsoids();
}

SaveObstacleShapeDialog::SaveObstacleShapeDialog() : QDialog(cnoid::MainWindow::instance()) {
  setWindowTitle("Save obstacle shape information");

  QButtonGroup* bgrp_cluster = new QButtonGroup();
  bgrp_cluster->setExclusive(true);

  QVBoxLayout* vbox = new QVBoxLayout();
  setLayout(vbox);

  QHBoxLayout* hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel("Clustering:"));
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();
  rbNumber.setText("Nubmer of clusters  ");
  bgrp_cluster->addButton(&rbNumber);
  clusterNumber.setAlignment(Qt::AlignCenter);
  clusterNumber.setMinimum(1);
  clusterNumber.setMaximum(200);
  clusterNumber.setValue(1);
  clusterNumber.setEnabled(false);
  hbox->addStretch();
  hbox->addWidget(&rbNumber);
  hbox->addWidget(&clusterNumber);
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();

  rbVolumeRatio.setText("Overlap volume ratio");
  rbVolumeRatio.setChecked(true);
  bgrp_cluster->addButton(&rbVolumeRatio);
  overlapVolumeRatio.setAlignment(Qt::AlignCenter);
  overlapVolumeRatio.setMinimum(0.0);
  overlapVolumeRatio.setMaximum(1.0);
  overlapVolumeRatio.setSingleStep(0.1);
  overlapVolumeRatio.setDecimals(2);
  overlapVolumeRatio.setValue(0.2);
  hbox->addStretch();
  hbox->addWidget(&rbVolumeRatio);
  hbox->addWidget(&overlapVolumeRatio);
  vbox->addLayout(hbox);
  rbVolumeRatio.sigToggled().connect(boost::bind(&SaveObstacleShapeDialog::rbVolumeRatioToggled, this));

  hbox = new QHBoxLayout();
  modelScale.setAlignment(Qt::AlignCenter);
  modelScale.setMinimum(0.01);
  modelScale.setMaximum(100);
  modelScale.setSingleStep(0.1);
  modelScale.setDecimals(2);
  modelScale.setValue(1.00);
  hbox->addStretch();
  hbox->addWidget(new QLabel("Scale"));
  hbox->addWidget(&modelScale);
  vbox->addLayout(hbox);

  QButtonGroup* bgrp_type = new QButtonGroup();
  bgrp_type->setExclusive(true);

  hbox = new QHBoxLayout();
  hbox->addWidget(new QLabel("Approximation type:"));
  vbox->addLayout(hbox);

  hbox = new QHBoxLayout();

  rbBbox.setText("boundingbox");
  rbBbox.setChecked(true);
  bgrp_type->addButton(&rbBbox);
  rbEllipsoid.setText("ellipsoid");
  bgrp_type->addButton(&rbEllipsoid);
  hbox->addStretch();
  hbox->addWidget(&rbBbox);
  hbox->addWidget(&rbEllipsoid);
  vbox->addLayout(hbox);

  cnoid::PushButton* showButton = new cnoid::PushButton("&show");
  cnoid::PushButton* writeButton = new cnoid::PushButton("&write");
  cnoid::PushButton* closeButton = new cnoid::PushButton("&close");
  showButton->setDefault(true);
  connect(writeButton, SIGNAL(clicked()), this, SLOT(accept()));
  connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));
  showButton->sigClicked().connect(boost::bind(&SaveObstacleShapeDialog::showClicked, this));
  writeButton->sigClicked().connect(boost::bind(&SaveObstacleShapeDialog::writeClicked, this));

  hbox = new QHBoxLayout();
  hbox->addWidget(showButton);
  hbox->addWidget(writeButton);
  hbox->addWidget(closeButton);
  vbox->addLayout(hbox);
}

void SaveObstacleShapeDialog::showClicked() {
  ObstacleShapeHandler* osh = ObstacleShapeHandler::instance();

  int num_cluster = rbVolumeRatio.isChecked() ? 1 : clusterNumber.value();
  double ratio_threshold = rbVolumeRatio.isChecked() ? overlapVolumeRatio.value() : 0.0;

  osh->setNumCluster(num_cluster);
  osh->setVolRatioTh(ratio_threshold);
  osh->setScale(modelScale.value());
  osh->showObstacleShapes(rbBbox.isChecked(), false);
}

void SaveObstacleShapeDialog::writeClicked() {
  ObstacleShapeHandler* osh = ObstacleShapeHandler::instance();

  int num_cluster = rbVolumeRatio.isChecked() ? 1 : clusterNumber.value();
  double ratio_threshold = rbVolumeRatio.isChecked() ? overlapVolumeRatio.value() : 0.0;

  osh->setNumCluster(num_cluster);
  osh->setVolRatioTh(ratio_threshold);
  osh->setScale(modelScale.value());
  osh->showObstacleShapes(rbBbox.isChecked(), true);
}

void SaveObstacleShapeDialog::rbVolumeRatioToggled() {
  overlapVolumeRatio.setEnabled(rbVolumeRatio.isChecked());
  clusterNumber.setEnabled(!rbVolumeRatio.isChecked());
}

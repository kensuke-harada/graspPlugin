#include "GraspPatternDialog.h"

#include <string>

#include <boost/bind.hpp>

#include "GraspDataGenerator.h"

#include <QtGui>

using namespace grasp;

ClusteringMethodSelectionDialog::ClusteringMethodSelectionDialog() {
	setWindowTitle(tr("clustering parameter setting"));

	rb_cluster_bb = new cnoid::RadioButton();
	rb_cluster_cylinder = new cnoid::RadioButton();
	rb_num_cluster = new cnoid::RadioButton();
	rb_volume_ratio = new cnoid::RadioButton();
	sb_num_cluster = new cnoid::SpinBox();
	ds_volume_ratio = new cnoid::DoubleSpinBox();
	ds_dist_th = new cnoid::DoubleSpinBox();
	ds_scale = new cnoid::DoubleSpinBox();

	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
	cnoid::PushButton* closeButton = new cnoid::PushButton(tr("&Close"));

	// Setting
	rb_cluster_bb->setText(tr("BoundingBox"));
	rb_cluster_cylinder->setText(tr("Cylinder"));
	rb_cluster_bb->setChecked(true);
	rb_num_cluster->setText(tr("Number of clusters"));
	rb_volume_ratio->setText(tr("Overlap volume ratio"));
	rb_volume_ratio->setChecked(true);
	sb_num_cluster->setAlignment(Qt::AlignCenter);
	sb_num_cluster->setMinimum(1);
	sb_num_cluster->setMaximum(200);
	sb_num_cluster->setValue(1);
	sb_num_cluster->setEnabled(false);
	ds_volume_ratio->setAlignment(Qt::AlignCenter);
	ds_volume_ratio->setMinimum(0.0);
	ds_volume_ratio->setMaximum(1.0);
	ds_volume_ratio->setSingleStep(0.1);
	ds_volume_ratio->setDecimals(2);
	ds_volume_ratio->setValue(0.2);
	ds_dist_th->setAlignment(Qt::AlignCenter);
	ds_dist_th->setMinimum(0.0);
	ds_dist_th->setMaximum(1.0);
	ds_dist_th->setSingleStep(0.001);
	ds_dist_th->setDecimals(4);
	ds_dist_th->setValue(0.04);
	ds_dist_th->setEnabled(false);
	ds_scale->setAlignment(Qt::AlignCenter);
	ds_scale->setMinimum(0.01);
	ds_scale->setMaximum(100);
	ds_scale->setSingleStep(0.1);
	ds_scale->setDecimals(2);
	ds_scale->setValue(1.00);

	okButton->setDefault(true);

	rb_volume_ratio->sigToggled().connect(boost::bind(&ClusteringMethodSelectionDialog::rbVolumeRatioToggled, this));
	rb_cluster_cylinder->sigToggled().connect(boost::bind(&ClusteringMethodSelectionDialog::rbClusterCylinderToggled, this));
	okButton->sigClicked().connect(boost::bind(&ClusteringMethodSelectionDialog::okClicked, this));
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));

	// layout
	QVBoxLayout* vbox = new QVBoxLayout();

	QVBoxLayout* vbox_type = new QVBoxLayout();
	QHBoxLayout* hbox_type = new QHBoxLayout();
	hbox_type->addWidget(rb_cluster_bb);
	hbox_type->addWidget(rb_cluster_cylinder);
	hbox_type->addStretch();
	vbox_type->addLayout(hbox_type);

	QGroupBox* gbox_type = new QGroupBox(tr("Clustering type"));
	gbox_type->setLayout(vbox_type);
	vbox->addWidget(gbox_type);

	QGridLayout* grid_param = new QGridLayout();
	grid_param->addWidget(rb_num_cluster, 0, 0);
	grid_param->addWidget(sb_num_cluster, 0, 1);
	grid_param->addWidget(rb_volume_ratio, 1, 0);
	grid_param->addWidget(ds_volume_ratio, 1, 1);
	grid_param->addWidget(new QLabel(tr("Scale")), 2, 0);
	grid_param->addWidget(ds_scale, 2, 1);
	grid_param->addWidget(new QLabel(tr("DistanceThreshold")), 3, 0);
	grid_param->addWidget(ds_dist_th, 3, 1);

	QVBoxLayout* vbox_param = new QVBoxLayout();
	QHBoxLayout* hbox_param = new QHBoxLayout();
	hbox_param->addSpacing(10);
	hbox_param->addLayout(grid_param);
	hbox_param->addStretch();
	vbox_param->addLayout(hbox_param);

	QGroupBox* gbox_param = new QGroupBox(tr("Clustering parameters"));
	gbox_param->setLayout(vbox_param);
	vbox->addWidget(gbox_param);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(okButton);
	hbox_button->addWidget(closeButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

void ClusteringMethodSelectionDialog::okClicked() {
	GraspDataAppender* gda = GraspDataAppender::instance();
	int num_cluster = rb_volume_ratio->isChecked() ? 1 : sb_num_cluster->value();
	double vol_ratio_th = rb_volume_ratio->isChecked() ? ds_volume_ratio->value() : 0.0;
	double dist_th = ds_dist_th->value();
	double scale = ds_scale->value();

	if (rb_cluster_bb->isChecked()) {
		gda->calcBoundingBoxCluster(num_cluster, vol_ratio_th, scale);
	}
	if (rb_cluster_cylinder->isChecked()) {
		gda->calcBoundingBoxCylinder(num_cluster, dist_th);
	}
	gda->moveInit();
	accept();
}

void ClusteringMethodSelectionDialog::rbVolumeRatioToggled() {
	ds_volume_ratio->setEnabled(rb_volume_ratio->isChecked());
	sb_num_cluster->setEnabled(!rb_volume_ratio->isChecked());
}

void ClusteringMethodSelectionDialog::rbClusterCylinderToggled() {
	ds_dist_th->setEnabled(rb_cluster_cylinder->isChecked());
	if(rb_cluster_cylinder->isChecked()) {
		rb_num_cluster->setChecked(true);
	}
	rb_volume_ratio->setEnabled(!(rb_cluster_cylinder->isChecked()));
	ds_scale->setEnabled(!(rb_cluster_cylinder->isChecked()));
}

AppendPatternDialog::AppendPatternDialog() {
	setWindowTitle(tr("append grasp pattern"));

	ds_x = new cnoid::DoubleSpinBox();
	ds_y = new cnoid::DoubleSpinBox();
	ds_z = new cnoid::DoubleSpinBox();
	ds_r = new cnoid::DoubleSpinBox();
	ds_p = new cnoid::DoubleSpinBox();
	ds_w = new cnoid::DoubleSpinBox();
	le_value = new cnoid::LineEdit();

	cnoid::PushButton* appendButton = new cnoid::PushButton(tr("&Append"));
	cnoid::PushButton* nextfaceButton = new cnoid::PushButton(tr("Next&face"));
	cnoid::PushButton* nextBBButton = new cnoid::PushButton(tr("Next&BB"));
	cnoid::PushButton* closeButton = new cnoid::PushButton(tr("&Close"));
	cnoid::PushButton* xPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* xMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* yPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* yMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* zPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* zMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* rPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* rMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* pPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* pMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* wPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* wMinusButton = new cnoid::PushButton(tr("-"));

	// Setting
	ds_x->setAlignment(Qt::AlignCenter);
	ds_x->setMinimum(0.0);
	ds_x->setMaximum(1.0);
	ds_x->setSingleStep(0.001);
	ds_x->setDecimals(4);
	ds_x->setValue(0.01);
	ds_y->setAlignment(Qt::AlignCenter);
	ds_y->setMinimum(0.0);
	ds_y->setMaximum(1.0);
	ds_y->setSingleStep(0.001);
	ds_y->setDecimals(4);
	ds_y->setValue(0.01);
	ds_z->setAlignment(Qt::AlignCenter);
	ds_z->setMinimum(0.0);
	ds_z->setMaximum(1.0);
	ds_z->setSingleStep(0.001);
	ds_z->setDecimals(4);
	ds_z->setValue(0.01);
	ds_r->setAlignment(Qt::AlignCenter);
	ds_r->setMinimum(0.0);
	ds_r->setMaximum(360);
	ds_r->setSingleStep(1);
	ds_r->setDecimals(2);
	ds_r->setValue(1);
	ds_p->setAlignment(Qt::AlignCenter);
	ds_p->setMinimum(0.0);
	ds_p->setMaximum(360);
	ds_p->setSingleStep(1);
	ds_p->setDecimals(2);
	ds_p->setValue(1);
	ds_w->setAlignment(Qt::AlignCenter);
	ds_w->setMinimum(0.0);
	ds_w->setMaximum(360);
	ds_w->setSingleStep(1);
	ds_w->setDecimals(2);
	ds_w->setValue(1);
	le_value->setMaximumWidth(100);
	le_value->setReadOnly(true);

	nextfaceButton->sigClicked().connect(boost::bind(&AppendPatternDialog::nextFaceClicked, this));
	nextBBButton->sigClicked().connect(boost::bind(&AppendPatternDialog::nextBBClicked, this));
	appendButton->sigClicked().connect(boost::bind(&AppendPatternDialog::appendClicked, this));
	xPlusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 1, 0, 0, 0, 0, 0));
	xMinusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, -1, 0, 0, 0, 0, 0));
	yPlusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 1, 0, 0, 0, 0));
	yMinusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, -1, 0, 0, 0, 0));
	zPlusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 1, 0, 0, 0));
	zMinusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, -1, 0, 0, 0));
	rPlusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 0, 1, 0, 0));
	rMinusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 0, -1, 0, 0));
	pPlusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 0, 0, 1, 0));
	pMinusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 0, 0, -1, 0));
	wPlusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 0, 0, 0, 1));
	wMinusButton->sigClicked().connect(boost::bind(&AppendPatternDialog::moveClicked, this, 0, 0, 0, 0, 0, -1));	
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));

	// layout
	QVBoxLayout* vbox = new QVBoxLayout();

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("x")), 0, 0);
	grid->addWidget(xPlusButton, 0, 1);
	grid->addWidget(xMinusButton, 0, 2);
	grid->addWidget(ds_x, 0, 3);
	grid->addWidget(new QLabel(tr("y")), 1, 0);
	grid->addWidget(yPlusButton, 1, 1);
	grid->addWidget(yMinusButton, 1, 2);
	grid->addWidget(ds_y, 1, 3);
	grid->addWidget(new QLabel(tr("z")), 2, 0);
	grid->addWidget(zPlusButton, 2, 1);
	grid->addWidget(zMinusButton, 2, 2);
	grid->addWidget(ds_z, 2, 3);
	grid->addWidget(new QLabel(tr("roll")), 3, 0);
	grid->addWidget(rPlusButton, 3, 1);
	grid->addWidget(rMinusButton, 3, 2);
	grid->addWidget(ds_r, 3, 3);
	grid->addWidget(new QLabel(tr("pitch")), 4, 0);
	grid->addWidget(pPlusButton, 4, 1);
	grid->addWidget(pMinusButton, 4, 2);
	grid->addWidget(ds_p, 4, 3);
	grid->addWidget(new QLabel(tr("yaw")), 5, 0);
	grid->addWidget(wPlusButton, 5, 1);
	grid->addWidget(wMinusButton, 5, 2);
	grid->addWidget(ds_w, 5, 3);
	grid->addWidget(new QLabel(tr("stability")), 6, 0);
	grid->addWidget(le_value, 6, 1, 1, 2);
	vbox->addLayout(grid);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(appendButton);
	hbox_button->addWidget(nextfaceButton);
	hbox_button->addWidget(nextBBButton);
	hbox_button->addWidget(closeButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

void AppendPatternDialog::showEvent(QShowEvent *event) {
	double q = GraspDataAppender::instance()->getCurQ();
	le_value->setText(QString::number(q));
}

void AppendPatternDialog::appendClicked() {
	if (GraspDataAppender::instance()->getCurQ() < 0) {
		GraspDataAppender::instance()->append();
		return;
	}

	std::string msg_str = "";
	bool need_confirm = false;
	if (GraspDataAppender::instance()->getState() == GraspDataAppender::S_NOTCONTACT) {
		msg_str += "NOT grasp with all figners.\n";
		need_confirm = true;
	} else if (GraspDataAppender::instance()->getState() == GraspDataAppender::S_NOTCONTACT) {
		msg_str += "NOT grasp with finger tips.\n";
		need_confirm = true;
	} 
	if (GraspDataAppender::instance()->isCollide()) {
		msg_str += "Collision occurs.\n";
		need_confirm = true;
	}

	if (need_confirm) {
		QMessageBox msgBox;
		msg_str += "Are you sure you want to append this grasp pose?";
		msgBox.setText(QString::fromStdString(msg_str));
		msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
		msgBox.setDefaultButton(QMessageBox::No);
		if (msgBox.exec() != QMessageBox::Yes) return;
	}
	GraspDataAppender::instance()->append();
}

void AppendPatternDialog::nextFaceClicked() {
	GraspDataAppender::instance()->nextFace();
	double q = GraspDataAppender::instance()->getCurQ();
	le_value->setText(QString::number(q));
}

void AppendPatternDialog::nextBBClicked() {
	GraspDataAppender::instance()->nextBB();
	double q = GraspDataAppender::instance()->getCurQ();
	le_value->setText(QString::number(q));
}

void AppendPatternDialog::moveClicked(int x, int y, int z, int r, int p, int w) {
	Vector3 pos = Vector3::Zero();
	Vector3 rpy = Vector3::Zero();
	if (x > 0) {
		pos[0] = ds_x->value();
	} else if (x < 0) {
		pos[0] = -ds_x->value();
	}

	if (y > 0) {
		pos[1] = ds_y->value();
	} else if (y < 0) {
		pos[1] = -ds_y->value();
	}

	if (z > 0) {
		pos[2] = ds_z->value();
	} else if (z < 0) {
		pos[2] = -ds_z->value();
	}

	if (r > 0) {
		rpy[0] = (ds_r->value() / 180.0) * M_PI;
	} else if (r < 0) {
		rpy[0] = -(ds_r->value() / 180.0) * M_PI;
	}
	
	if (p > 0) {
		rpy[1] = (ds_p->value() / 180.0) * M_PI;
	} else if (p < 0) {
		rpy[1] = -(ds_p->value() / 180.0) * M_PI;
	}

	if (w > 0) {
		rpy[2] = (ds_w->value() / 180.0) * M_PI;
	} else if (w < 0) {
		rpy[2] = -(ds_w->value() / 180.0) * M_PI;
	}

	GraspDataAppender::instance()->move(pos, rpy);
	double q = GraspDataAppender::instance()->getCurQ();
	le_value->setText(QString::number(q));
}

ModifyPatternDialog::ModifyPatternDialog() :
	target_id_(-1) {
	setWindowTitle(tr("grasp pattern modification"));

	ds_x = new cnoid::DoubleSpinBox();
	ds_y = new cnoid::DoubleSpinBox();
	ds_z = new cnoid::DoubleSpinBox();
	ds_r = new cnoid::DoubleSpinBox();
	ds_p = new cnoid::DoubleSpinBox();
	ds_w = new cnoid::DoubleSpinBox();
	le_value = new cnoid::LineEdit();

	cnoid::PushButton* updateButton = new cnoid::PushButton(tr("&Update"));
	cnoid::PushButton* closeButton = new cnoid::PushButton(tr("&Close"));
	cnoid::PushButton* xPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* xMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* yPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* yMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* zPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* zMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* rPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* rMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* pPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* pMinusButton = new cnoid::PushButton(tr("-"));
	cnoid::PushButton* wPlusButton = new cnoid::PushButton(tr("+"));
	cnoid::PushButton* wMinusButton = new cnoid::PushButton(tr("-"));

	// Setting
	ds_x->setAlignment(Qt::AlignCenter);
	ds_x->setMinimum(0.0);
	ds_x->setMaximum(1.0);
	ds_x->setSingleStep(0.001);
	ds_x->setDecimals(4);
	ds_x->setValue(0.01);
	ds_y->setAlignment(Qt::AlignCenter);
	ds_y->setMinimum(0.0);
	ds_y->setMaximum(1.0);
	ds_y->setSingleStep(0.001);
	ds_y->setDecimals(4);
	ds_y->setValue(0.01);
	ds_z->setAlignment(Qt::AlignCenter);
	ds_z->setMinimum(0.0);
	ds_z->setMaximum(1.0);
	ds_z->setSingleStep(0.001);
	ds_z->setDecimals(4);
	ds_z->setValue(0.01);
	ds_r->setAlignment(Qt::AlignCenter);
	ds_r->setMinimum(0.0);
	ds_r->setMaximum(360);
	ds_r->setSingleStep(1);
	ds_r->setDecimals(2);
	ds_r->setValue(1);
	ds_p->setAlignment(Qt::AlignCenter);
	ds_p->setMinimum(0.0);
	ds_p->setMaximum(360);
	ds_p->setSingleStep(1);
	ds_p->setDecimals(2);
	ds_p->setValue(1);
	ds_w->setAlignment(Qt::AlignCenter);
	ds_w->setMinimum(0.0);
	ds_w->setMaximum(360);
	ds_w->setSingleStep(1);
	ds_w->setDecimals(2);
	ds_w->setValue(1);
	le_value->setMaximumWidth(100);
	le_value->setReadOnly(true);

	updateButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::acceptClicked, this));
	xPlusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 1, 0, 0, 0, 0, 0));
	xMinusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, -1, 0, 0, 0, 0, 0));
	yPlusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 1, 0, 0, 0, 0));
	yMinusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, -1, 0, 0, 0, 0));
	zPlusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 1, 0, 0, 0));
	zMinusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, -1, 0, 0, 0));
	rPlusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 0, 1, 0, 0));
	rMinusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 0, -1, 0, 0));
	pPlusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 0, 0, 1, 0));
	pMinusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 0, 0, -1, 0));
	wPlusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 0, 0, 0, 1));
	wMinusButton->sigClicked().connect(boost::bind(&ModifyPatternDialog::moveClicked, this, 0, 0, 0, 0, 0, -1));
	connect(closeButton, SIGNAL(clicked()), this, SLOT(reject()));
	
	// layout
	QVBoxLayout* vbox = new QVBoxLayout();

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("x")), 0, 0);
	grid->addWidget(xPlusButton, 0, 1);
	grid->addWidget(xMinusButton, 0, 2);
	grid->addWidget(ds_x, 0, 3);
	grid->addWidget(new QLabel(tr("y")), 1, 0);
	grid->addWidget(yPlusButton, 1, 1);
	grid->addWidget(yMinusButton, 1, 2);
	grid->addWidget(ds_y, 1, 3);
	grid->addWidget(new QLabel(tr("z")), 2, 0);
	grid->addWidget(zPlusButton, 2, 1);
	grid->addWidget(zMinusButton, 2, 2);
	grid->addWidget(ds_z, 2, 3);
	grid->addWidget(new QLabel(tr("roll")), 3, 0);
	grid->addWidget(rPlusButton, 3, 1);
	grid->addWidget(rMinusButton, 3, 2);
	grid->addWidget(ds_r, 3, 3);
	grid->addWidget(new QLabel(tr("pitch")), 4, 0);
	grid->addWidget(pPlusButton, 4, 1);
	grid->addWidget(pMinusButton, 4, 2);
	grid->addWidget(ds_p, 4, 3);
	grid->addWidget(new QLabel(tr("yaw")), 5, 0);
	grid->addWidget(wPlusButton, 5, 1);
	grid->addWidget(wMinusButton, 5, 2);
	grid->addWidget(ds_w, 5, 3);
	grid->addWidget(new QLabel(tr("stability")), 6, 0);
	grid->addWidget(le_value, 6, 1, 1, 2);
	vbox->addLayout(grid);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(updateButton);;
	hbox_button->addWidget(closeButton);
	vbox->addLayout(hbox_button);

	setLayout(vbox);
}

void ModifyPatternDialog::setTargetID(int i) {
	target_id_ = i;
	GraspDataUpdater::instance()->moveInit(i);
}

void ModifyPatternDialog::updateDataBase() {
	GraspDataUpdater::instance()->updateDataBase();
}

void ModifyPatternDialog::showEvent(QShowEvent *event) {
	double q = GraspDataUpdater::instance()->getCurQ();
	le_value->setText(QString::number(q));
}

void ModifyPatternDialog::acceptClicked() {
	std::string msg_str = "";
	bool need_confirm = false;
	if (GraspDataUpdater::instance()->getState() == GraspDataAppender::S_NOTCONTACT) {
		msg_str += "NOT grasp with all figners.\n";
		need_confirm = true;
	} else if (GraspDataUpdater::instance()->getState() == GraspDataAppender::S_NOTCONTACT) {
		msg_str += "NOT grasp with finger tips.\n";
		need_confirm = true;
	}
	if (GraspDataUpdater::instance()->isCollide()) {
		msg_str += "Collision occurs.\n";
		need_confirm = true;
	}

	QMessageBox msgBox;
	msg_str += "Are you sure you want to update?";
	msgBox.setText(QString::fromStdString(msg_str));
	msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
	msgBox.setDefaultButton(QMessageBox::No);
	if (msgBox.exec() != QMessageBox::Yes) return;

	GraspDataUpdater::instance()->update();
	accept();
}

void ModifyPatternDialog::moveClicked(int x, int y, int z, int r, int p, int w) {
	Vector3 pos = Vector3::Zero();
	Vector3 rpy = Vector3::Zero();
	if (x > 0) {
		pos[0] = ds_x->value();
	} else if (x < 0) {
		pos[0] = -ds_x->value();
	}

	if (y > 0) {
		pos[1] = ds_y->value();
	} else if (y < 0) {
		pos[1] = -ds_y->value();
	}

	if (z > 0) {
		pos[2] = ds_z->value();
	} else if (z < 0) {
		pos[2] = -ds_z->value();
	}

	if (r > 0) {
		rpy[0] = (ds_r->value() / 180.0) * M_PI;
	} else if (r < 0) {
		rpy[0] = -(ds_r->value() / 180.0) * M_PI;
	}

	if (p > 0) {
		rpy[1] = (ds_p->value() / 180.0) * M_PI;
	} else if (p < 0) {
		rpy[1] = -(ds_p->value() / 180.0) * M_PI;
	}

	if (w > 0) {
		rpy[2] = (ds_w->value() / 180.0) * M_PI;
	} else if (w < 0) {
		rpy[2] = -(ds_p->value() / 180.0) * M_PI;
	}

	GraspDataUpdater::instance()->move(pos, rpy);
	double q = GraspDataUpdater::instance()->getCurQ();
	le_value->setText(QString::number(q));
}

#include <vector>

#include <boost/bind.hpp>

#include <pcl/pcl_config.h>

#include "PoseEstimateDialog.h"
#include "ObjectPoseEstimator.h"
#include "ObjectPoseEstimatorInterface.h"
#include "PointCloudHolder.h"
#include "PointCloudDrawer.h"

#include "../Grasp/PlanBase.h"

#define DEBUG_MODE
#ifdef DEBUG_MODE
#include <time.h>
#endif

using namespace std;

PoseEstimateDialog::PoseEstimateDialog() : QDialog(cnoid::MainWindow::instance()) {
	setWindowTitle(tr("Pose Estimation"));

	param = new ObjectPoseEstimateParams;
	readParams();

	QVBoxLayout* vbox = new QVBoxLayout();

	// Initial alignment method
	rb_use_cvfh = new cnoid::RadioButton();
	rb_use_cvfh->setText(tr("CVFH"));
	rb_use_cvfh->setChecked(param->do_recog_cvfh);
#if PCL_VERSION_COMPARE(<, 1, 7, 0)
	rb_use_cvfh->setEnabled(false);
#endif

	rb_use_sacia = new cnoid::RadioButton();
	rb_use_sacia->setText(tr("SAC-IA"));
	rb_use_sacia->setChecked(!param->do_recog_cvfh);

	QHBoxLayout* hbox_method = new QHBoxLayout();
	hbox_method->addWidget(rb_use_cvfh);
	hbox_method->addWidget(rb_use_sacia);
	hbox_method->addStretch();
	QGroupBox* gbox_method = new QGroupBox(tr("Initial alignment method"));
	gbox_method->setLayout(hbox_method);
	vbox->addWidget(gbox_method);

	// PointCloud
	cb_merge_cloud = new cnoid::CheckBox();
	cb_merge_cloud->setText(tr("merge with previous captured pointcloud"));
	cb_merge_cloud->setChecked(false);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(cb_merge_cloud);
	hbox->addStretch();

	QGroupBox* gbox_capture = new QGroupBox(tr("Capture"));
	gbox_capture->setLayout(hbox);
	vbox->addWidget(gbox_capture);

#ifdef EXPERIMENT_DATA_OUTPUT
	cb_data_output = new cnoid::CheckBox();
	cb_data_output->setText(tr("output data"));
	cb_data_output->setChecked(true);

	QHBoxLayout* hbox_dataoutput = new QHBoxLayout();
	hbox_dataoutput->addWidget(cb_data_output);
	vbox->addLayout(hbox_dataoutput);
#endif

	tab_param = new QTabWidget;
	tab_param->addTab(new RegionTab(param, this), tr("Region"));
	tab_param->addTab(new PCDTab(param, this), tr("PCD/Model"));
	tab_param->addTab(new ShowTab(param, this), tr("Show"));
	tab_param->addTab(new ParamTab(param, this), tr("Parameter"));
	cvfh_tab_idx = tab_param->addTab(new CVFHTab(param, this), tr("CVFH"));
	sacia_tab_idx = tab_param->addTab(new SaciaTab(param, this), tr("SAC-IA"));
	// tab_param->addTab(new CameraTab(param, this), tr("Camera"));
	tab_param->addTab(new CalibrationTab(param, this), tr("calibration"));

	vbox->addWidget(tab_param);

	cnoid::PushButton* okButton = new cnoid::PushButton(tr("&OK"));
	okButton->setDefault(true);

	QHBoxLayout* hbox_button = new QHBoxLayout();
	hbox_button->addStretch();
	hbox_button->addWidget(okButton);
	vbox->addLayout(hbox_button);

	connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));

	rb_use_sacia->sigToggled().connect(boost::bind(&PoseEstimateDialog::rbIAMethodToggled, this));
	okButton->sigClicked().connect(boost::bind(&PoseEstimateDialog::okClicked, this));
	rbIAMethodToggled();

	setLayout(vbox);
}

PoseEstimateDialog::~PoseEstimateDialog() {
	if (param != NULL) delete param;
}

void PoseEstimateDialog::rbIAMethodToggled() {
	tab_param->setTabEnabled(sacia_tab_idx, rb_use_sacia->isChecked());
	tab_param->setTabEnabled(cvfh_tab_idx, rb_use_cvfh->isChecked());
	this->adjustSize();
}

void PoseEstimateDialog::okClicked() {
	for (int i = 0; i < tab_param->count(); i++) {
		dynamic_cast<AbstractParamTab*>(tab_param->widget(i))->getParameters(param);
	}
	param->do_recog_cvfh = rb_use_cvfh->isChecked();
	param->is_mergecloud = cb_merge_cloud->isChecked();

#ifdef EXPERIMENT_DATA_OUTPUT
	if (cb_data_output->isChecked()) {
		const boost::filesystem::path path("./data/");
		boost::filesystem::directory_iterator end;

		int id = 0;
		for (boost::filesystem::directory_iterator p(path); p != end; ++p) {
			if (boost::filesystem::is_directory(*p)) {
				continue;
			}

			std::vector<std::string> splited;
			std::string filename = p->path().stem().string();
			boost::algorithm::split(splited, filename, boost::is_any_of("_"));
			if (splited.size() != 2) continue;

			int x = boost::lexical_cast<int>(splited[0]);
			if (x >= id) id = x + 1;
		}

		std::string id_str = (boost::format("%04d") % id).str();
		grasp::ObjPoseEstimateSol::datafile_id = id_str;
		param->is_save_file = true;
		param->save_file_path = "./data/" + id_str + "_scene.pcd";
		grasp::PlanBase* pb = grasp::PlanBase::instance();
		// pb->datafile_id = id_str;
		std::string palm_file = "./data/" + id_str + "_palm.txt";
		std::ofstream palm_out(palm_file.c_str());
		for (int i = 0; i < 3; i++) {
			palm_out << pb->palm(param->handcamera_arm_id)->p()(i);
			if (i != 2) palm_out << " ";
		}
		palm_out << std::endl;
		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 3; j++) {
				palm_out << pb->palm(param->handcamera_arm_id)->R()(i, j);
				if (!(i == 2 && j == 2))
					palm_out << " ";
			}
		}
		palm_out.close();

		std::string region_file = "./data/" + id_str + "_region.txt";
		std::ofstream region_out(region_file.c_str());
		region_out << param->xmin << " " << param->xmax << " ";
		region_out << param->ymin << " " << param->ymax << " ";
		region_out << param->zmin << " " << param->zmax;
		region_out.close();
	}
#endif
	PoseEstimator pe;
	pe.poseEstimate(*param);
}

void PoseEstimateDialog::readParams() {
	PoseEstimator::readParams(param);
}

RegionTab::RegionTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	// Region setting
	ds_xmax = new cnoid::DoubleSpinBox();
	ds_xmax->setMinimum(-10.0);
	ds_xmax->setMaximum(10.0);
	ds_xmax->setSingleStep(0.1);
	ds_xmax->setDecimals(4);
	ds_xmax->setValue(param->xmax);
	ds_xmax->setEnabled(param->is_do_narrow);

	ds_xmin = new cnoid::DoubleSpinBox();
	ds_xmin->setMinimum(-10.0);
	ds_xmin->setMaximum(10.0);
	ds_xmin->setSingleStep(0.1);
	ds_xmin->setDecimals(4);
	ds_xmin->setValue(param->xmin);
	ds_xmin->setEnabled(param->is_do_narrow);

	ds_ymax = new cnoid::DoubleSpinBox();
	ds_ymax->setMinimum(-10.0);
	ds_ymax->setMaximum(10.0);
	ds_ymax->setSingleStep(0.1);
	ds_ymax->setDecimals(4);
	ds_ymax->setValue(param->ymax);
	ds_ymax->setEnabled(param->is_do_narrow);

	ds_ymin = new cnoid::DoubleSpinBox();
	ds_ymin->setMinimum(-10.0);
	ds_ymin->setMaximum(10.0);
	ds_ymin->setSingleStep(0.1);
	ds_ymin->setDecimals(4);
	ds_ymin->setValue(param->ymin);
	ds_ymin->setEnabled(param->is_do_narrow);

	ds_zmax = new cnoid::DoubleSpinBox();
	ds_zmax->setMinimum(-10.0);
	ds_zmax->setMaximum(10.0);
	ds_zmax->setSingleStep(0.1);
	ds_zmax->setDecimals(4);
	ds_zmax->setValue(param->zmax);
	ds_zmax->setEnabled(param->is_do_narrow);

	ds_zmin = new cnoid::DoubleSpinBox();
	ds_zmin->setMinimum(-10.0);
	ds_zmin->setMaximum(10.0);
	ds_zmin->setSingleStep(0.1);
	ds_zmin->setDecimals(4);
	ds_zmin->setValue(param->zmin);
	ds_zmin->setEnabled(param->is_do_narrow);

	QVBoxLayout* vbox = new QVBoxLayout();
	cb_is_donarrow = new cnoid::CheckBox();
	cb_is_donarrow->setText(tr("set target region"));
	cb_is_donarrow->setChecked(param->is_do_narrow);
	vbox->addWidget(cb_is_donarrow);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("x:")));
	hbox->addWidget(ds_xmin);
	hbox->addWidget(new QLabel(tr(" - ")));
	hbox->addWidget(ds_xmax);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("y:")));
	hbox->addWidget(ds_ymin);
	hbox->addWidget(new QLabel(tr(" - ")));
	hbox->addWidget(ds_ymax);
	hbox->addStretch();
	vbox->addLayout(hbox);

	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("z:")));
	hbox->addWidget(ds_zmin);
	hbox->addWidget(new QLabel(tr(" - ")));
	hbox->addWidget(ds_zmax);
	hbox->addStretch();
	vbox->addLayout(hbox);
	vbox->addStretch();

	setLayout(vbox);

	cb_is_donarrow->sigToggled().connect(boost::bind(&RegionTab::cbDoNarrowToggled, this));
}

void RegionTab::getParameters(PoseEstimateDialog::Params* param) const {
	param->is_do_narrow = cb_is_donarrow->isChecked();
	param->xmax					= ds_xmax->value();
	param->xmin					= ds_xmin->value();
	param->ymax					= ds_ymax->value();
	param->ymin					= ds_ymin->value();
	param->zmax					= ds_zmax->value();
	param->zmin					= ds_zmin->value();
}

void RegionTab::cbDoNarrowToggled() {
	ds_xmax->setEnabled(cb_is_donarrow->isChecked());
	ds_xmin->setEnabled(cb_is_donarrow->isChecked());
	ds_ymax->setEnabled(cb_is_donarrow->isChecked());
	ds_ymin->setEnabled(cb_is_donarrow->isChecked());
	ds_zmax->setEnabled(cb_is_donarrow->isChecked());
	ds_zmin->setEnabled(cb_is_donarrow->isChecked());
}

PCDTab::PCDTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	param_ = param;
	cb_is_loadfile = new cnoid::CheckBox();
	cb_is_loadfile->setChecked(param->is_load_file);
	cb_is_savefile = new cnoid::CheckBox();
	cb_is_savefile->setChecked(param->is_save_file);
	cb_is_loadobj = new cnoid::CheckBox();
	cb_is_loadobj->setChecked(param->is_load_obj);
	le_loadfile = new cnoid::LineEdit();
	le_loadfile->setText(param->load_file_path);
	le_loadfile->setReadOnly(true);
	le_savefile = new cnoid::LineEdit();
	le_savefile->setText(param->save_file_path);
	le_savefile->setReadOnly(true);
	le_objfile = new cnoid::LineEdit();
	le_objfile->setText(param->obj_file_path);
	le_objfile->setReadOnly(true);
	cnoid::PushButton* loadButton = new cnoid::PushButton(tr("select..."));
	cnoid::PushButton* saveButton = new cnoid::PushButton(tr("select..."));
	cnoid::PushButton* objButton = new cnoid::PushButton(tr("select..."));

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(cb_is_loadfile, 0, 0);
	grid->addWidget(cb_is_savefile, 1, 0);
	grid->addWidget(cb_is_loadobj, 2, 0);
	grid->addWidget(new QLabel(tr("load scene from")), 0, 1);
	grid->addWidget(new QLabel(tr("save scene to")), 1, 1);
	grid->addWidget(new QLabel(tr("obj file path")), 2, 1);
	grid->addWidget(le_loadfile, 0, 2);
	grid->addWidget(le_savefile, 1, 2);
	grid->addWidget(le_objfile, 2, 2);
	grid->addWidget(loadButton, 0, 3);
	grid->addWidget(saveButton, 1, 3);
	grid->addWidget(objButton, 2, 3);
	grid->setColumnMinimumWidth(2, 200);

	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addLayout(grid);
	vbox->addStretch();
	setLayout(vbox);

	loadButton->sigClicked().connect(boost::bind(&PCDTab::loadFileClicked, this));
	saveButton->sigClicked().connect(boost::bind(&PCDTab::saveFileClicked, this));
	objButton->sigClicked().connect(boost::bind(&PCDTab::objectFileClicked, this));
}

void PCDTab::getParameters(PoseEstimateDialog::Params* param) const {
	param->is_load_file   = cb_is_loadfile->isChecked();
	param->is_save_file   = cb_is_savefile->isChecked();
	param->is_load_obj    = cb_is_loadobj->isChecked();
	param->load_file_path = le_loadfile->text().toStdString();
	param->save_file_path = le_savefile->text().toStdString();
	param->obj_file_path	= le_objfile->text().toStdString();
}

void PCDTab::loadFileClicked() {
	string default_path = param_->load_file_path;
	QString filepath =  QFileDialog::getOpenFileName(this, tr("Select a pcd file"), QString(default_path.c_str()), tr("PCD File (*.pcd)"));
	if (!filepath.isEmpty()) {
		le_loadfile->setText(filepath);
	}
}

void PCDTab::saveFileClicked() {
	string default_path = param_->save_file_path;
	QString filepath =  QFileDialog::getSaveFileName(this, tr("Save file as"), QString(default_path.c_str()), tr("PCD File (*.pcd)"));
	if (!filepath.isEmpty()) {
		le_savefile->setText(filepath);
	}
}

void PCDTab::objectFileClicked() {
	string default_path = param_->obj_file_path;
	QString filepath =  QFileDialog::getOpenFileName(this, tr("Select a vrml/yaml file"), QString(default_path.c_str()), tr("VRML/YAML File (*.wrl *.yaml)"));
	if (!filepath.isEmpty()) {
		le_objfile->setText(filepath);
	}
}

ShowTab::ShowTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	rb_show_scene = new cnoid::RadioButton();
	rb_show_scene->setText(tr("scene"));
	rb_show_scene->setChecked(param->is_show_scene);
	rb_show_segment = new cnoid::RadioButton();
	rb_show_segment->setText(tr("segmented scene"));
	rb_show_segment->setChecked(param->is_show_segment);
	rb_show_moved = new cnoid::RadioButton();
	rb_show_moved->setText(tr("clipped scene and moved object"));
	rb_show_moved->setChecked(param->is_show_moved);
	cb_show_is_color = new cnoid::CheckBox();
	cb_show_is_color->setText(tr("color"));
	cb_show_is_color->setChecked(param->is_color);

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(rb_show_scene, 0, 0);
	grid->addWidget(rb_show_segment, 1, 0);
	grid->addWidget(rb_show_moved, 2, 0);
	grid->addWidget(cb_show_is_color, 0, 1);

	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addLayout(grid);
	vbox->addStretch();
	setLayout(vbox);
}

void ShowTab::getParameters(PoseEstimateDialog::Params* param) const {
	param->is_color					= cb_show_is_color->isChecked();
	param->is_show_segment	= rb_show_segment->isChecked();
	param->is_show_moved		= rb_show_moved->isChecked();
}

ParamTab::ParamTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	QVBoxLayout* vbox_param = new QVBoxLayout();
	// Sampling
	ds_sampling = new cnoid::DoubleSpinBox();
	ds_sampling->setAlignment(Qt::AlignCenter);
	ds_sampling->setMinimum(0.0001);
	ds_sampling->setSingleStep(0.001);
	ds_sampling->setDecimals(4);
	ds_sampling->setValue(param->sampling_density);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("sampling density[m]")));
	hbox->addWidget(ds_sampling);
	hbox->addStretch();

	QGroupBox* gbox = new QGroupBox(tr("Sampling"));
	gbox->setLayout(hbox);
	vbox_param->addWidget(gbox);

	// Segmentation
	ds_planeremove_dist = new cnoid::DoubleSpinBox();
	ds_planeremove_dist->setSingleStep(0.001);
	ds_planeremove_dist->setDecimals(4);
	ds_planeremove_dist->setValue(param->plane_remove_dist);

	sb_planeremove_num = new cnoid::SpinBox();
	sb_planeremove_num->setMinimum(0);
	sb_planeremove_num->setMaximum(1000);
	sb_planeremove_num->setValue(param->num_plane_remove);

	sb_candidate_num = new cnoid::SpinBox();
	sb_candidate_num->setMinimum(1);
	sb_candidate_num->setMaximum(1000);
	sb_candidate_num->setValue(param->num_candidate);

	ds_segment_tolerance = new cnoid::DoubleSpinBox();
	ds_segment_tolerance->setSingleStep(0.001);
	ds_segment_tolerance->setDecimals(4);
	ds_segment_tolerance->setValue(param->segment_tolerance);

	ds_segment_vtolerance = new cnoid::DoubleSpinBox();
	ds_segment_vtolerance->setSingleStep(0.001);
	ds_segment_vtolerance->setDecimals(4);
	ds_segment_vtolerance->setValue(param->segment_vtolerance);

	ds_segment_resegradius = new cnoid::DoubleSpinBox();
	ds_segment_resegradius->setSingleStep(0.001);
	ds_segment_resegradius->setDecimals(4);
	ds_segment_resegradius->setValue(param->segment_resegment_radius);

	ds_segment_boundradius = new cnoid::DoubleSpinBox();
	ds_segment_boundradius->setSingleStep(0.001);
	ds_segment_boundradius->setDecimals(4);
	ds_segment_boundradius->setValue(param->segment_boundary_radius);

	ds_segment_lccpseed = new cnoid::DoubleSpinBox();
	ds_segment_lccpseed->setSingleStep(0.001);
	ds_segment_lccpseed->setDecimals(4);
	ds_segment_lccpseed->setValue(param->segment_sv_seed_resolution);

	sb_segment_lccpminsize = new cnoid::SpinBox();
	sb_segment_lccpminsize->setMinimum(1);
	sb_segment_lccpminsize->setMaximum(1000);
	sb_segment_lccpminsize->setValue(param->segment_lccp_minsize);

	ds_segment_lccpconvextol = new cnoid::DoubleSpinBox();
	ds_segment_lccpconvextol->setSingleStep(0.1);
	ds_segment_lccpconvextol->setDecimals(2);
	ds_segment_lccpconvextol->setValue(param->segment_lccp_concavitytolerance);

	rb_algo_euclidean = new cnoid::RadioButton();
	rb_algo_euclidean->setText(tr("Euclidean clustering"));
	rb_algo_euclidean->setChecked(!param->use_lccp_segmentation);

	rb_algo_lccp = new cnoid::RadioButton();
	rb_algo_lccp->setText(tr("LCCP segmentation"));
	rb_algo_lccp->setChecked(param->use_lccp_segmentation);

	QButtonGroup* bgrp_algo = new QButtonGroup(this);
	bgrp_algo->setExclusive(true);
	bgrp_algo->addButton(rb_algo_euclidean);
	bgrp_algo->addButton(rb_algo_lccp);


	euclidean_seg_widget = new QWidget();
	QGridLayout* euclidean_seg_grid = new QGridLayout();
	euclidean_seg_grid->addWidget(new QLabel(tr("horizonal tolerance[m]")), 0, 0);
	euclidean_seg_grid->addWidget(ds_segment_tolerance, 0, 1);
	euclidean_seg_grid->addWidget(new QLabel(tr("vertical tolerance[m]")), 1, 0);
	euclidean_seg_grid->addWidget(ds_segment_vtolerance, 1, 1);
	euclidean_seg_grid->addWidget(new QLabel(tr("radius(resegment)[m]")), 2, 0);
	euclidean_seg_grid->addWidget(ds_segment_resegradius, 2, 1);
	euclidean_seg_grid->addWidget(new QLabel(tr("radius(boundary estimation)[m]")), 3, 0);
	euclidean_seg_grid->addWidget(ds_segment_boundradius, 3, 1);
	euclidean_seg_widget->setLayout(euclidean_seg_grid);

	lccp_seg_widget = new QWidget();
	QGridLayout* lccp_seg_grid = new QGridLayout();
	lccp_seg_grid->addWidget(new QLabel(tr("seed resolution[m]")), 0, 0);
	lccp_seg_grid->addWidget(ds_segment_lccpseed, 0, 1);
	lccp_seg_grid->addWidget(new QLabel(tr("minimum point size")), 1, 0);
	lccp_seg_grid->addWidget(sb_segment_lccpminsize, 1, 1);
	lccp_seg_grid->addWidget(new QLabel(tr("concavity tolerance[deg]")), 2, 0);
	lccp_seg_grid->addWidget(ds_segment_lccpconvextol, 2, 1);
	lccp_seg_widget->setLayout(lccp_seg_grid);

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("dist threshold(plane remove)[m]")), 0, 0);
	grid->addWidget(ds_planeremove_dist, 0, 1);
	grid->addWidget(new QLabel(tr("number of planes(plane remove)")), 1, 0);
	grid->addWidget(sb_planeremove_num, 1, 1);
	grid->addWidget(new QLabel(tr("number of candidate clusters")), 2, 0);
	grid->addWidget(sb_candidate_num, 2, 1);
	grid->addWidget(new QLabel(tr("segmentation algorithm:")), 3, 0);
	grid->addWidget(rb_algo_euclidean, 4, 0);
	grid->addWidget(rb_algo_lccp, 5, 0);
	grid->addWidget(euclidean_seg_widget, 6, 0, 1, 2);
	grid->addWidget(lccp_seg_widget, 7, 0, 1, 2);
	grid->setColumnMinimumWidth(0, 250);

	hbox = new QHBoxLayout();
	hbox->addLayout(grid);
	hbox->addStretch();
	gbox = new QGroupBox(tr("Segmentation"));
	gbox->setLayout(hbox);
	vbox_param->addWidget(gbox);

	// ICP
	sb_iteration = new cnoid::SpinBox();
	sb_iteration->setAlignment(Qt::AlignCenter);
	sb_iteration->setMinimum(1);
	sb_iteration->setMaximum(10000);
	sb_iteration->setValue(param->iteration);

	hbox = new QHBoxLayout();
	hbox->addWidget(new QLabel(tr("iteration")));
	hbox->addWidget(sb_iteration);
	hbox->addStretch();

	gbox = new QGroupBox(tr("ICP"));
	gbox->setLayout(hbox);
	vbox_param->addWidget(gbox);
	vbox_param->addStretch();

	setLayout(vbox_param);

	rbAlgoToggled();
	rb_algo_euclidean->sigToggled().connect(boost::bind(&ParamTab::rbAlgoToggled, this));
}

void ParamTab::getParameters(PoseEstimateDialog::Params* param) const {
	param->sampling_density  = ds_sampling->value();
	param->iteration				 = sb_iteration->value();
	param->plane_remove_dist = ds_planeremove_dist->value();
	param->num_plane_remove  = sb_planeremove_num->value();
	param->num_candidate		 = sb_candidate_num->value();
	param->segment_tolerance  = ds_segment_tolerance->value();
	param->segment_vtolerance = ds_segment_vtolerance->value();
	param->segment_resegment_radius = ds_segment_resegradius->value();
	param->segment_boundary_radius = ds_segment_boundradius->value();
	param->segment_sv_seed_resolution = ds_segment_lccpseed->value();
	param->segment_lccp_minsize = sb_segment_lccpminsize->value();
	param->segment_lccp_concavitytolerance = ds_segment_lccpconvextol->value();
	param->use_lccp_segmentation = rb_algo_lccp->isChecked();
}

void ParamTab::rbAlgoToggled() {
	euclidean_seg_widget->setVisible(rb_algo_euclidean->isChecked());
	lccp_seg_widget->setVisible(rb_algo_lccp->isChecked());
}

SaciaTab::SaciaTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	// SAC-IA
	rb_ksearch = new cnoid::RadioButton();
	rb_ksearch->setText(tr("K-nearest"));
	rb_ksearch->setChecked(param->use_ksearch);
	rb_rsearch = new cnoid::RadioButton();
	rb_rsearch->setText(tr("Radius"));
	rb_rsearch->setChecked(!param->use_ksearch);

	QButtonGroup* bgrp_search = new QButtonGroup(this);
	bgrp_search->setExclusive(true);
	bgrp_search->addButton(rb_ksearch);
	bgrp_search->addButton(rb_rsearch);

	ds_normal_radius = new cnoid::DoubleSpinBox();
	ds_normal_radius->setAlignment(Qt::AlignCenter);
	ds_normal_radius->setMinimum(0.0001);
	ds_normal_radius->setSingleStep(0.001);
	ds_normal_radius->setDecimals(4);
	ds_normal_radius->setValue(param->normal_radius);
	ds_normal_radius->setEnabled(!param->use_ksearch);

	ds_feature_radius = new cnoid::DoubleSpinBox();
	ds_feature_radius->setAlignment(Qt::AlignCenter);
	ds_feature_radius->setMinimum(0.0001);
	ds_feature_radius->setSingleStep(0.001);
	ds_feature_radius->setDecimals(4);
	ds_feature_radius->setValue(param->feature_radius);
	ds_feature_radius->setEnabled(!param->use_ksearch);

	sb_normal_k = new cnoid::SpinBox();
	sb_normal_k->setAlignment(Qt::AlignCenter);
	sb_normal_k->setMinimum(1);
	sb_normal_k->setValue(param->normal_k);
	sb_normal_k->setEnabled(param->use_ksearch);

	sb_feature_k = new cnoid::SpinBox();
	sb_feature_k->setAlignment(Qt::AlignCenter);
	sb_feature_k->setMinimum(param->feature_k);
	sb_feature_k->setValue(10);
	sb_feature_k->setEnabled(param->use_ksearch);

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(rb_rsearch, 0, 0);
	grid->addWidget(new QLabel(tr("normal")), 0, 1);
	grid->addWidget(ds_normal_radius, 0, 2);
	grid->addWidget(new QLabel(tr("feature")), 0, 3);
	grid->addWidget(ds_feature_radius, 0, 4);
	grid->addWidget(rb_ksearch, 1, 0);
	grid->addWidget(new QLabel(tr("normal")), 1, 1);
	grid->addWidget(sb_normal_k, 1, 2);
	grid->addWidget(new QLabel(tr("feature")), 1, 3);
	grid->addWidget(sb_feature_k, 1, 4);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addLayout(grid);
	hbox->addStretch();
	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addWidget(new QLabel(tr("NN search method")));
	vbox->addLayout(hbox);
	vbox->addStretch();
	setLayout(vbox);

	rb_rsearch->sigToggled().connect(boost::bind(&SaciaTab::rbSearchToggled, this));
}

void SaciaTab::getParameters(PoseEstimateDialog::Params* param) const {
	param->normal_radius		= ds_normal_radius->value();
	param->feature_radius		= ds_feature_radius->value();
	param->normal_k					= sb_normal_k->value();
	param->feature_k				= sb_feature_k->value();
	param->use_ksearch			= rb_ksearch->isChecked();
}

void SaciaTab::rbSearchToggled() {
	ds_normal_radius->setEnabled(rb_rsearch->isChecked());
	ds_feature_radius->setEnabled(rb_rsearch->isChecked());
	sb_normal_k->setEnabled(rb_ksearch->isChecked());
	sb_feature_k->setEnabled(rb_ksearch->isChecked());
}

CVFHTab::CVFHTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	// CVFH
	sb_NN = new cnoid::SpinBox();
	sb_NN->setMinimum(1);
	sb_NN->setValue(param->num_neighbor);

	// ds_viewx = new cnoid::DoubleSpinBox();
	// ds_viewx->setAlignment(Qt::AlignCenter);
	// ds_viewx->setMinimum(-10);
	// ds_viewx->setSingleStep(10);
	// ds_viewx->setDecimals(4);
	// ds_viewx->setValue(param->view_point.x());

	// ds_viewy = new cnoid::DoubleSpinBox();
	// ds_viewy->setAlignment(Qt::AlignCenter);
	// ds_viewy->setMinimum(-10);
	// ds_viewy->setSingleStep(10);
	// ds_viewy->setDecimals(4);
	// ds_viewy->setValue(param->view_point.y());

	// ds_viewz = new cnoid::DoubleSpinBox();
	// ds_viewz->setAlignment(Qt::AlignCenter);
	// ds_viewz->setMinimum(-10);
	// ds_viewz->setSingleStep(10);
	// ds_viewz->setDecimals(4);
	// ds_viewz->setValue(param->view_point.z());

	cb_use_bbsimilarity = new cnoid::CheckBox();
	cb_use_bbsimilarity->setText(tr("use boudningbox similarity"));
	cb_use_bbsimilarity->setChecked(param->use_bbsimilarity);

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("Candidate poses")), 0, 0);
	grid->addWidget(sb_NN, 0, 1, 1, 2);
	// grid->addWidget(new QLabel(tr("View point")), 1, 0);
	// grid->addWidget(new QLabel(tr("x")), 1, 1);
	// grid->addWidget(ds_viewx, 1, 2);
	// grid->addWidget(new QLabel(tr("y")), 1, 3);
	// grid->addWidget(ds_viewy, 1, 4);
	// grid->addWidget(new QLabel(tr("z")), 1, 5);
	// grid->addWidget(ds_viewz, 1, 6);
	grid->addWidget(cb_use_bbsimilarity, 2, 0);

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addLayout(grid);
	hbox->addStretch();
	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addLayout(hbox);
	vbox->addStretch();
	setLayout(vbox);
}

void CVFHTab::getParameters(PoseEstimateDialog::Params* param) const {
	// param->view_point				= cnoid::Vector3(ds_viewx->value(), ds_viewy->value(), ds_viewz->value());
	param->use_bbsimilarity = cb_use_bbsimilarity->isChecked();
	param->num_neighbor			= sb_NN->value();
}

// CameraTab::CameraTab(PoseEstimateDialog::Params* param, QWidget* parent) :
// 	AbstractParamTab(parent) {
// 	cb_handcamera = new cnoid::CheckBox();
// 	cb_handcamera->setText(tr("hand camera"));
// 	cb_handcamera->setChecked(param->is_handcamera);

// 	sb_handcamera_arm_id = new cnoid::SpinBox();
// 	sb_handcamera_arm_id->setMinimum(0);
// 	sb_handcamera_arm_id->setValue(param->handcamera_arm_id);

// 	QHBoxLayout* hbox = new QHBoxLayout();
// 	hbox->addWidget(cb_handcamera);
// 	hbox->addStretch();
// 	QHBoxLayout* hbox_armid = new QHBoxLayout();
// 	hbox_armid->addWidget(new QLabel(tr("arm id")));
// 	hbox_armid->addWidget(sb_handcamera_arm_id);
// 	hbox_armid->addStretch();

// 	QVBoxLayout* vbox = new QVBoxLayout();
// 	vbox->addLayout(hbox);
// 	vbox->addLayout(hbox_armid);
// 	vbox->addStretch();
// 	setLayout(vbox);
// }

// void CameraTab::getParameters(PoseEstimateDialog::Params* param) const {
// 	 param->is_handcamera  = cb_handcamera->isChecked();
// 	 param->handcamera_arm_id = sb_handcamera_arm_id->value();
// }

CalibrationTab::CalibrationTab(PoseEstimateDialog::Params* param, QWidget* parent) :
	AbstractParamTab(parent) {
	le_file = new cnoid::LineEdit();
	le_file->setText(param->save_file_path);
	le_file->setReadOnly(true);
	cnoid::PushButton* selectButton = new cnoid::PushButton(tr("select..."));

	QGridLayout* grid = new QGridLayout();
	grid->addWidget(new QLabel(tr("file path")), 0, 0);
	grid->addWidget(le_file, 0, 1);
	grid->addWidget(selectButton, 0, 2);
	grid->setColumnMinimumWidth(1, 200);

	QVBoxLayout* vbox = new QVBoxLayout();
	vbox->addLayout(grid);

	cb_applytransmat = new cnoid::CheckBox();
	cb_applytransmat->setText(tr("apply trans mat"));
	cb_applytransmat->setChecked(true);
	vbox->addWidget(cb_applytransmat);


	cnoid::PushButton* saveButton = new cnoid::PushButton(tr("capture and save"));
	cnoid::PushButton* loadButton = new cnoid::PushButton(tr("load"));
	cnoid::PushButton* showButton = new cnoid::PushButton(tr("show palm pos"));

	QHBoxLayout* hbox = new QHBoxLayout();
	hbox->addStretch();
	hbox->addWidget(saveButton);
	hbox->addWidget(loadButton);
	hbox->addWidget(showButton);

	vbox->addLayout(hbox);
	vbox->addStretch();
	setLayout(vbox);

	connect(showButton, SIGNAL(clicked()), this->parentWidget(), SLOT(accept()));
	connect(saveButton, SIGNAL(clicked()), this->parentWidget(), SLOT(accept()));
	connect(loadButton, SIGNAL(clicked()), this->parentWidget(), SLOT(accept()));

	selectButton->sigClicked().connect(boost::bind(&CalibrationTab::selectFileClicked, this));
	saveButton->sigClicked().connect(boost::bind(&CalibrationTab::saveFileClicked, this));
	loadButton->sigClicked().connect(boost::bind(&CalibrationTab::loadFileClicked, this));
	showButton->sigClicked().connect(boost::bind(&CalibrationTab::showPalmPosClicked, this));
}

void CalibrationTab::getParameters(PoseEstimateDialog::Params* param) const {
}

void CalibrationTab::selectFileClicked() {
	QString filepath =  QFileDialog::getOpenFileName(this, tr("Select a pcd file"), le_file->text(), tr("PCD File (*.pcd)"));
	if (!filepath.isEmpty()) {
		le_file->setText(filepath);
	}
}

void CalibrationTab::loadFileClicked() {
	PointCloudHolder* ph = PointCloudHolder::instance();
	ph->capture(true, false, le_file->text().toStdString(), "",
							false, cb_applytransmat->isChecked());

	PointCloudDraw* draw = new PointCloudDraw();
	draw->clear();
	draw->addPointCloud(ph->getCloudPtr());
	draw->draw();
	delete draw;
}

void CalibrationTab::saveFileClicked() {
	PointCloudHolder* ph = PointCloudHolder::instance();
	ph->capture(false, true, "", le_file->text().toStdString(),
							false, cb_applytransmat->isChecked());

	PointCloudDraw* draw = new PointCloudDraw();
	draw->clear();
	draw->addPointCloud(ph->getCloudPtr());
	draw->draw();
	delete draw;
}

void CalibrationTab::showPalmPosClicked() {
	grasp::PlanBase* pb = grasp::PlanBase::instance();
	cnoid::MessageView::instance()->cout() << "palm_p:" << std::endl <<  pb->palm()->p().transpose() << std::endl;
	cnoid::MessageView::instance()->cout() << "palm_R:" << std::endl << pb->palm()->R() << std::endl;
}


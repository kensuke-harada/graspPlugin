#ifndef _PCL_POSEESTIMATEDIALOG_H_
#define _PCL_POSEESTIMATEDIALOG_H_

#include <string>
#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/Button>
#include <cnoid/LineEdit>
#include <QDialog>
#include <QLayout>
#include <QLabel>
#include <QFileDialog>
#include <QGroupBox>
#include <QSettings>
#include <cnoid/SpinBox>

//#define EXPERIMENT_DATA_OUTPUT
#ifdef EXPERIMENT_DATA_OUTPUT
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#endif

struct ObjectPoseEstimateParams;

class PoseEstimateDialog
: public QDialog {
 public:
	typedef ObjectPoseEstimateParams Params;

	PoseEstimateDialog();
	virtual ~PoseEstimateDialog();

 private:
	cnoid::RadioButton*   rb_use_sacia;
	cnoid::RadioButton*   rb_use_cvfh;
	cnoid::CheckBox*      cb_merge_cloud;
#ifdef EXPERIMENT_DATA_OUTPUT
	cnoid::CheckBox*      cb_data_output;
#endif

	QTabWidget* tab_param;
	int cvfh_tab_idx;
	int sacia_tab_idx;

	void rbIAMethodToggled();
	void okClicked();

	void readParams();

	Params* param;
};

class AbstractParamTab
: public QWidget {
 public:
	explicit AbstractParamTab(QWidget* parent) : QWidget(parent) {;}
	virtual ~AbstractParamTab() {;}

	virtual void getParameters(PoseEstimateDialog::Params* param) const  = 0;
};

class RegionTab
: public AbstractParamTab {
 public:
	explicit RegionTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* param) const;

 private:
	cnoid::DoubleSpinBox* ds_xmax;
	cnoid::DoubleSpinBox* ds_xmin;
	cnoid::DoubleSpinBox* ds_ymax;
	cnoid::DoubleSpinBox* ds_ymin;
	cnoid::DoubleSpinBox* ds_zmax;
	cnoid::DoubleSpinBox* ds_zmin;
	cnoid::CheckBox*      cb_is_donarrow;

	void cbDoNarrowToggled();
};

class PCDTab
: public AbstractParamTab {
 public:
	explicit PCDTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* param) const;

 private:
	PoseEstimateDialog::Params* param_;

	cnoid::CheckBox*      cb_is_loadfile;
	cnoid::CheckBox*      cb_is_savefile;
	cnoid::CheckBox*      cb_is_loadobj;
	cnoid::LineEdit*      le_loadfile;
	cnoid::LineEdit*      le_savefile;
	cnoid::LineEdit*      le_objfile;

	void loadFileClicked();
	void saveFileClicked();
	void objectFileClicked();
};

class ShowTab
: public AbstractParamTab {
 public:
	explicit ShowTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* param) const;

 private:
	cnoid::RadioButton*   rb_show_scene;
	cnoid::RadioButton*   rb_show_segment;
	cnoid::RadioButton*   rb_show_moved;
	cnoid::CheckBox*      cb_show_is_color;
};

class ParamTab
: public AbstractParamTab {
 public:
	explicit ParamTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* param) const;

 private:
	cnoid::DoubleSpinBox* ds_sampling;
	cnoid::SpinBox*       sb_iteration;
	cnoid::DoubleSpinBox* ds_planeremove_dist;
	cnoid::SpinBox*       sb_planeremove_num;
	cnoid::RadioButton*   rb_algo_euclidean;
	cnoid::RadioButton*   rb_algo_lccp;
	QWidget*              euclidean_seg_widget;
	cnoid::DoubleSpinBox* ds_segment_tolerance;
	cnoid::DoubleSpinBox* ds_segment_vtolerance;
	cnoid::DoubleSpinBox* ds_segment_resegradius;
	cnoid::DoubleSpinBox* ds_segment_boundradius;
	QWidget*              lccp_seg_widget;
	cnoid::DoubleSpinBox* ds_segment_lccpseed;
	cnoid::SpinBox*       sb_segment_lccpminsize;
	cnoid::DoubleSpinBox* ds_segment_lccpconvextol;
	cnoid::SpinBox*       sb_candidate_num;

	void rbAlgoToggled();
};

class SaciaTab
: public AbstractParamTab {
 public:
	explicit SaciaTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* param) const;

 private:
	cnoid::RadioButton*   rb_ksearch;
	cnoid::RadioButton*   rb_rsearch;
	cnoid::DoubleSpinBox* ds_normal_radius;
	cnoid::DoubleSpinBox* ds_feature_radius;
	cnoid::SpinBox*       sb_normal_k;
	cnoid::SpinBox*       sb_feature_k;

	void rbSearchToggled();
};

class CVFHTab
: public AbstractParamTab {
 public:
	explicit CVFHTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* param) const;

 private:
	/* cnoid::DoubleSpinBox* ds_viewx; */
	/* cnoid::DoubleSpinBox* ds_viewy; */
	/* cnoid::DoubleSpinBox* ds_viewz; */
	cnoid::SpinBox*       sb_NN;
	cnoid::CheckBox*      cb_use_bbsimilarity;
};

/* class CameraTab */
/* : public AbstractParamTab { */
/*  public: */
/* 	explicit CameraTab(PoseEstimateDialog::Params* param, QWidget* parent = 0); */

/* 	void getParameters(PoseEstimateDialog::Params* param) const; */

/*  private: */
/* 	cnoid::CheckBox*      cb_handcamera; */
/* 	cnoid::SpinBox*       sb_handcamera_arm_id; */
/* }; */

class CalibrationTab
: public AbstractParamTab {
 public:
	explicit CalibrationTab(PoseEstimateDialog::Params* param, QWidget* parent = 0);

	void getParameters(PoseEstimateDialog::Params* parm) const;

 private:
	cnoid::CheckBox*      cb_applytransmat;
	cnoid::LineEdit*      le_file;

	void selectFileClicked();
	void loadFileClicked();
	void saveFileClicked();
	void showPalmPosClicked();
};

#endif /* _PCL_POSEESTIMATEDIALOG_H_ */


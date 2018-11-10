#ifndef _PCL_GENERATEDESCRIPTORDIALOG_H_
#define _PCL_GENERATEDESCRIPTORDIALOG_H_

#include <string>
#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/Button>
#include <cnoid/LineEdit>
#include <cnoid/SpinBox>
#include <QDialog>
#include <QLayout>
#include <QFileDialog>
#include <QGroupBox>
#include <QLabel>

class GenDescriptorDialog
	: public QDialog {
public:
	struct Params {
		std::string work_dir;
		double			sampling_density;
		double			normal_radius;
	};

	GenDescriptorDialog();
private:
	cnoid::LineEdit*      le_dir;
	cnoid::CheckBox*			cb_param;
	cnoid::DoubleSpinBox* ds_sampling;
	cnoid::DoubleSpinBox* ds_normalradius;
	QFrame*								frame_param;

	void selectDirClicked();
	void cbParamToggled();
	void okClicked();

	void readParams();

	Params param;
};

#endif

#ifndef _PRM_PARAMDIALOG_H_
#define _PRM_PARAMDIALOG_H_

#include <string>
#include <QtGui>
#include <QGridLayout>
#include <QLabel>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/ComboBox>
#include <cnoid/SpinBox>

namespace grasp {
	class PRMParams {
	public:
		enum ALGORITHM {ALGO_SBL, ALGO_RRT, ALGO_RRTSTAR, ALGO_PRM};
		enum NNMETHOD {NN_RADIUS, NN_KNN};
		struct Params {
			ALGORITHM algo;
			double eps;
			int iteration_num;
			double delta;
			int smoothestep;
			NNMETHOD nnmethod;
			double radius;
			int num_path;
		};

		PRMParams();

		void loadParams();
		void saveParams();

		Params param;
	private:
		std::string ini_file_path;
	};

	class PRMParamDialog
	: public cnoid::Dialog {
	public:
		PRMParamDialog();
	private:
		QGridLayout* createLayout();
		QHBoxLayout* createButtons();
		void setInitvalues();

		void okClicked();
		void algoChanged();
		void nnChanged();

		cnoid::ComboBox* algoCombo;
		cnoid::DoubleSpinBox* epsSpin;
		cnoid::SpinBox* maxiterSpin;
		cnoid::DoubleSpinBox* deltaSpin;
		cnoid::SpinBox* smoothestepSpin;
		cnoid::SpinBox* pathSpin;
		cnoid::ComboBox* nnsearchCombo;
		cnoid::DoubleSpinBox* radiusSpin;
	};
}

#endif

















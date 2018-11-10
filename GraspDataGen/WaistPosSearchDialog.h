#ifndef _GRASPDATAGEN_WAISTPOSSEARCHDIALOG_H_
#define _GRASPDATAGEN_WAISTPOSSEARCHDIALOG_H_

#include <vector>

#include <QTableWidget>
#include <QHeaderView>
#include <QLabel>
#include <QGroupBox>

#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/ComboBox>
#include <cnoid/SpinBox>

#include "BoxInfo.h"

//#define DEBUG_MODE

namespace grasp {
	class WaistPosSearchDialog
		: public cnoid::Dialog {
	public:
		WaistPosSearchDialog();

		void okClicked();
#ifdef DEBUG_MODE
		void debugClicked();
#endif
	private:
		cnoid::CheckBox* useCameraCheck;
		std::vector<cnoid::ComboBox*> object_items;
		cnoid::CheckBox* useRightArmCheck;
		cnoid::CheckBox* useLeftArmCheck;
		cnoid::DoubleSpinBox* gridIntervalSpin;
		cnoid::CheckBox* displayGraspablePointCheck;
#ifdef DEBUG_MODE
		cnoid::SpinBox* cornerIDSpin;
#endif

		BoxInfoArray boxinfos;

		void readBoxInfos();
	};
}

#endif

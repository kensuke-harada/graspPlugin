#ifndef _GRASPDATAGEN_WAISTPOSSEARCHRESULTVIEWDIALOG_H_
#define _GRASPDATAGEN_WAISTPOSSEARCHRESULTVIEWDIALOG_H_

#include <vector>

#include <QLabel>

#include <cnoid/Dialog>
#include <cnoid/Button>

namespace grasp {
	class WaistPosSearchResultViewDialog
		: public cnoid::Dialog {
	public:
		WaistPosSearchResultViewDialog();

		void okClicked();
	private:
		std::vector<cnoid::CheckBox*> displayBoxRightCheck, displayBoxLeftCheck;
		cnoid::CheckBox* displayRouteCheck;
	};
}

#endif

#ifndef _GRASPDATAGEN_PARAMDIALOG_H_
#define _GRASPDATAGEN_PARAMDIALOG_H_

#include <vector>
#include <string>

#include <QLabel>
#include <QMessageBox>

#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/ComboBox>
#include <cnoid/LineEdit>
#include <cnoid/SpinBox>

#ifndef M_PI
#define M_PI 3.1415923
#endif

namespace grasp {
	class PrehensionParameterExt;

	class PrehensionParamTab
		: public QWidget {
	public:
		explicit PrehensionParamTab(int id_ = -1);

		int getID() const {return id;}
		void setID(int id_) {id = id_;}
		bool isNew() const {return (id == -1);}
		void setRefmotion(std::string name) {ref_motion = name;}

		void makePrehensionParam(PrehensionParameterExt* param);
		bool setTextFromPrehensionParam(const PrehensionParameterExt* param);
	private:
		struct FingerComponent {
			std::vector<cnoid::DoubleSpinBox*> fingerAngleSpins;
			std::vector<cnoid::CheckBox*>      fingerContactChecks;
			std::vector<cnoid::ComboBox*>      fingerCompLinkCombo;
			std::vector<cnoid::DoubleSpinBox*> fingerCloseSpins;
		};

		cnoid::LineEdit      *nameLine;
		cnoid::DoubleSpinBox *GRCposXSpin, *GRCposYSpin, *GRCposZSpin;
		cnoid::DoubleSpinBox *GRCrpyRSpin, *GRCrpyPSpin, *GRCrpyYSpin;
		cnoid::DoubleSpinBox *GRCedgeXSpin, *GRCedgeYSpin, *GRCedgeZSpin;
		cnoid::DoubleSpinBox *loadMinSpin, *loadMaxSpin;
		cnoid::CheckBox      *usePalmCheck;
		cnoid::DoubleSpinBox *palmCloseXSpin, *palmCloseYSpin, *palmCloseZSpin;
		std::vector<FingerComponent> finger_components;

		int id;
		std::string ref_motion;

		void usePalmcloseToggled();
		void displayClicked();
		void graspClicked();
		void removeClicked();

		double rad2deg(double angle) const {return (angle / M_PI) * 180;}
		double deg2rad(double angle) const {return (angle / 180.0) * M_PI;}
	};

	class ArmFingers;
	
	class PrehensionParamDialog
		: public cnoid::Dialog {
	public:
		explicit PrehensionParamDialog(ArmFingers* arm);

		void updateTabs();
	private:
		void addClicked();
		void saveClicked();

		QTabWidget *tabWidget;

		ArmFingers* target_arm;
	};
}


#endif /* _GRASPDATAGEN_PARAMDIALOG_H_ */

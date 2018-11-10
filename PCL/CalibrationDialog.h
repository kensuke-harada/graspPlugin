#ifndef _PCL_CALIBRATIONDIALOG_H_
#define _PCL_CALIBRATIONDIALOG_H_

#include <string>

#include <QtGui>
#include <QVBoxLayout>
#include <QLabel>
#include <QHBoxLayout>

#include <cnoid/Dialog>
#include <cnoid/Button>

class Calibration;

class CalibrationDialog :
public cnoid::Dialog {
 public:
	CalibrationDialog();
	~CalibrationDialog();

	void setMatFilePath(const std::string& path);

 private:
	Calibration* calib_;
	std::string output_path_;

	QLabel*            msgLabel;

	cnoid::PushButton* doneButton;
	cnoid::PushButton* skipButton;

	cnoid::PushButton* acceptButton;
	cnoid::PushButton* retryButton;
	cnoid::PushButton* rejectButton;

	void acceptClicked();
	void rejectClicked();
	void retryClicked();
	void doneClicked();
	void skipClicked();

	void adjustDialogMode();
	void decisionDialogMode();

	void goNextPose();
};

#endif /* _PCL_CALIBRATIONDIALOG_H_ */

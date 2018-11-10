/**
 * @file   FittingDialog.h
 * @author Akira Ohchi
*/

#ifndef _PCL_FITTINGDIALOG_H_
#define _PCL_FITTINGDIALOG_H_

#include <string>

#include <QDialog>
#include <QLayout>
#include <QLabel>

#include <cnoid/MainWindow>
#include <cnoid/Button>
#include <cnoid/LineEdit>

class Fitting;

class FittingDialog
: public QDialog {
 public:
	FittingDialog();
	virtual ~FittingDialog();

	void setYamlPath(const std::string& path);
 private:
	cnoid::LineEdit* le_file;

	Fitting* fitter;

	void selectFileClicked();
	void fittingClicked();
	void loadClicked();
	void saveClicked();
	void closeClicked();
};

#endif /* _PCL_FITTINGDIALOG_H_ */


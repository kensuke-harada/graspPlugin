/**
 * @file   LearningDialog.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACE_LEARNINGDIALOG_H_
#define _PICKANDPLACE_LEARNINGDIALOG_H_

#include <QDialog>
#include <QBoxLayout>

#include <cnoid/MainWindow>
#include <cnoid/Button>

class LearningDialog :
public QDialog {
 public:
	LearningDialog();
	virtual ~LearningDialog();

 private:
	void successClicked();
	void failClicked();

	void addData(bool is_succeed);
};

#endif /* _PICKANDPLACE_LEARNINGDIALOG_H_ */


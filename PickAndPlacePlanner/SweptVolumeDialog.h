/**
 * @file   SweptVolumeDialog.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACE_SWEPTVOLUMEDIALOG_H_
#define _PICKANDPLACE_SWEPTVOLUMEDIALOG_H_

#include <QDialog>
#include <QLabel>

#include <cnoid/MainWindow>
#include <cnoid/Button>
#include <cnoid/SpinBox>

class SweptVolumeDialog :
public QDialog {
 public:
	SweptVolumeDialog();
	virtual ~SweptVolumeDialog();

 private:
	cnoid::SpinBox* sp_target_sol_id;
	
	void okClicked();
	void clearClicked();
};

#endif /* _PICKANDPLACE_SWEPTVOLUMEDIALOG_H_ */


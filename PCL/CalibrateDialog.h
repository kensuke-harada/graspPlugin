/*
 * CalibrateDialog.h
 *
 *  Created on: 2011/12/01
 *      Author: asahi
 */

#ifndef CALIBRATEDIALOG_H_
#define CALIBRATEDIALOG_H_

#include <cnoid/Dialog>
#include <cnoid/LineEdit>
//#include <exportdecl.h>

class QLineEdit;

namespace cnoid {

    class
    CalibrateDialog : public Dialog
    {
    public:
    	CalibrateDialog();
    	virtual ~CalibrateDialog();

    	float getXMin() { return xmin->text().toFloat(); }
    	float getXMax() { return xmax->text().toFloat(); }
    	float getYMin() { return ymin->text().toFloat(); }
    	float getYMax() { return ymax->text().toFloat(); }
    	float getZMin() { return zmin->text().toFloat(); }
    	float getZMax() { return zmax->text().toFloat(); }

    	void setXMin(float val) { xmin->setText(QString::number(val)); }
    	void setXMax(float val) { xmax->setText(QString::number(val)); }
    	void setYMin(float val) { ymin->setText(QString::number(val)); }
    	void setYMax(float val) { ymax->setText(QString::number(val)); }
    	void setZMin(float val) { zmin->setText(QString::number(val)); }
    	void setZMax(float val) { zmax->setText(QString::number(val)); }

    private:
    	QLineEdit * xmin;
    	QLineEdit * xmax;
    	QLineEdit * ymin;
    	QLineEdit * ymax;
    	QLineEdit * zmin;
    	QLineEdit * zmax;
    };
}

#endif /* CALIBRATEDIALOG_H_ */

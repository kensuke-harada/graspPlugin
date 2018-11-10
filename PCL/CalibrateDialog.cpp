/*
 * CalibrateDialog.cpp
 *
 *  Created on: 2011/12/01
 *      Author: asahi
 */

#include <QtGui>
#include <QGridLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <QDoubleValidator>
#include <cnoid/MainWindow>

#include "CalibrateDialog.h"
#include "gettext.h"

namespace cnoid {

CalibrateDialog::CalibrateDialog()
: Dialog(MainWindow::instance())
{
	QGridLayout * layout = new QGridLayout;

	xmin = new QLineEdit;
	layout->addWidget(xmin, 0, 0);
	QLabel * xlabel = new QLabel(_(" < x < "));
	layout->addWidget(xlabel, 0, 1);
	xmax = new QLineEdit;
	layout->addWidget(xmax, 0, 2);

	ymin = new QLineEdit;
	layout->addWidget(ymin, 1, 0);
	QLabel * ylabel = new QLabel(_(" < y < "));
	layout->addWidget(ylabel, 1, 1);
	ymax = new QLineEdit;
	layout->addWidget(ymax, 1, 2);

	zmin = new QLineEdit;
	layout->addWidget(zmin, 2, 0);
	QLabel * zlabel = new QLabel(_(" < z < "));
	layout->addWidget(zlabel, 2, 1);
	zmax = new QLineEdit;
	layout->addWidget(zmax, 2, 2);

    QDialogButtonBox * buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok
                                         | QDialogButtonBox::Cancel);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
	layout->addWidget(buttonBox, 3, 0);

	setLayout(layout);

	const float vmin = -10.0, vmax = 10.0, vdecimals = 3;
	QDoubleValidator* val = new QDoubleValidator(vmin, vmax, vdecimals, this);
	val->setNotation(QDoubleValidator::StandardNotation);
	xmin->setValidator(val);
	xmax->setValidator(val);
	ymin->setValidator(val);
	ymax->setValidator(val);
	zmin->setValidator(val);
	zmax->setValidator(val);
}

CalibrateDialog::~CalibrateDialog() {
}

}

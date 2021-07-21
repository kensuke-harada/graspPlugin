#ifndef _PICKINGTASKPLANNER_CONFIRMDIALOG_H_
#define _PICKINGTASKPLANNER_CONFIRMDIALOG_H_

#include <QtGui>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QBoxLayout>
#include <QLabel>
#endif

#include <cnoid/Dialog>
#include <cnoid/Button>

namespace grasp {
	class ConfirmDialog:
		public QDialog {
	public:
		ConfirmDialog();
		~ConfirmDialog();

		int getState() const;
	protected:
		void closeEvent(QCloseEvent* ce);
	private:
		void okClicked();
		void cancelClicked();

		int state;
	};
}

#endif /* _PICKINGTASKPLANNER_CONFIRMDIALOG_H_ */

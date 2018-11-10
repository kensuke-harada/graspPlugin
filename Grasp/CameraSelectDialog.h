#ifndef _GRASP_CAMERASELECTDIALOG_H_
#define _GRASP_CAMERASELECTDIALOG_H_

#include <QLayout>
#include <QLabel>

#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/ComboBox>

namespace grasp {
	class CameraSelectDialog :
		public cnoid::Dialog {
	public:
		CameraSelectDialog();
		~CameraSelectDialog();

	private:
		cnoid::ComboBox cameras;

		void okClicked();
	};
}  // namespace grasp

#endif /* _GRASP_CAMERASELECTDIALOG_H_ */

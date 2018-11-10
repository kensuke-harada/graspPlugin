#ifndef _GRASP_HANDSELECTDIALOG_H_
#define _GRASP_HANDSELECTDIALOG_H_

#include <QLayout>
#include <QLabel>

#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/ComboBox>

namespace grasp {
	class HandSelectDialog :
		public cnoid::Dialog {
	public:
		HandSelectDialog();
		~HandSelectDialog();

		bool hasAttachHandList() const;
		bool hasDetachHandList() const;
		bool setAttachMode();
		bool setDetachMode();

	private:
		cnoid::ComboBox armNumber;
		cnoid::ComboBox hands;

		bool attach_mode_;

		void okClicked();
		void armChanged();
	};
}  // namespace grasp

#endif /* _GRASP_HANDSELECTDIALOG_H_ */

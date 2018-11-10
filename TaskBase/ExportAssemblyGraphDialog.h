#ifndef _TASKBASE_EXPORTASSEMBLYGRAPHDIALOG_H_
#define _TASKBASE_EXPORTASSEMBLYGRAPHDIALOG_H_

#include <QLabel>
#include <QBoxLayout>
#include <QLayout>
#include <QGridLayout>
#include <QFileDialog>

#include <cnoid/MainWindow>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/LineEdit>
#include <cnoid/SpinBox>
#include <cnoid/ExecutablePath>

namespace grasp {
	class TaskItem;

	class ExportAssemblyGraphDialog :
		public QDialog {
	public:
		ExportAssemblyGraphDialog();
		~ExportAssemblyGraphDialog();

		void setTaskItem(TaskItem* task);

	private:
		TaskItem* task_;
 
		QGridLayout* createLayout();
		QHBoxLayout* createButtons();
		
		cnoid::LineEdit* le_graph_path;
		cnoid::LineEdit* le_relation_path;
		cnoid::CheckBox* cb_include_order;
		cnoid::DoubleSpinBox* ds_time;

		void selectGraphClicked();
		void selectRelationClicked();
		void okClicked();
	};
}


#endif /* _TASKBASE_EXPORTASSEMBLYGRAPHDIALOG_H_ */

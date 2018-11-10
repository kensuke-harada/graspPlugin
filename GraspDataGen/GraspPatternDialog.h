#ifndef _GRASPDATAGEN_GRASPPATTERNDIALOG_H_
#define _GRASPDATAGEN_GRASPPATTERNDIALOG_H_

#include <vector>
#include <string>

#include <QGroupBox>
#include <QMessageBox>

#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/LineEdit>

namespace grasp {
	class ClusteringMethodSelectionDialog
		: public cnoid::Dialog {
	public:
		ClusteringMethodSelectionDialog();
	private:
		cnoid::RadioButton   *rb_cluster_bb, *rb_cluster_cylinder;
		cnoid::RadioButton   *rb_num_cluster, *rb_volume_ratio;
		cnoid::SpinBox       *sb_num_cluster;
		cnoid::DoubleSpinBox *ds_volume_ratio, *ds_dist_th, *ds_scale;

		void okClicked();
		void rbVolumeRatioToggled();
		void rbClusterCylinderToggled();
	};

	class AppendPatternDialog
		: public cnoid::Dialog {
	public:
		AppendPatternDialog();
	protected:
		void showEvent(QShowEvent *event);
	private:
		cnoid::DoubleSpinBox *ds_x, *ds_y, *ds_z, *ds_r, *ds_p, *ds_w;
		cnoid::LineEdit *le_value;

		void appendClicked();
		void nextFaceClicked();
		void nextBBClicked();
		void moveClicked(int x, int y, int z, int r, int p, int w);
	};

	class ModifyPatternDialog
		: public cnoid::Dialog {
	public:
		ModifyPatternDialog();

		void setTargetID(int i);

		void updateDataBase();

	protected:
		void showEvent(QShowEvent *event);

	private:
		cnoid::DoubleSpinBox *ds_x, *ds_y, *ds_z, *ds_r, *ds_p, *ds_w;
		cnoid::LineEdit *le_value;
		int target_id_;

		void acceptClicked();
		void moveClicked(int x, int y, int z, int r, int p, int w);
	};
}

#endif /* _GRASPDATAGEN_GRASPPATTERNDAILOG_H_ */

/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_ROBOTICS_GRASP_DATA_GEN_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GRASP_DATA_GEN_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/ 

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

#include "exportdef.h"

#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/Button>
#include <iostream>

#include <QDialog>
#include <QCheckBox>
#include <QLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QGroupBox>

#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif

#include "ParamDialog.h"
#include "GraspPatternDialog.h"
#include "WaistPosSearchDialog.h"
#include "WaistPosSearchResultViewDialog.h"

namespace cnoid {

    class MessageView;
	
}

namespace grasp {

	class SaveGraspPatternDialog : public QDialog
	{
	public:
		SaveGraspPatternDialog();
	private:
		cnoid::RadioButton*    rb_cluster_bb;
		cnoid::RadioButton*    rb_cluster_cylinder;
		cnoid::RadioButton*    rb_num_cluster;
		cnoid::RadioButton*    rb_volume_ratio;
		cnoid::SpinBox*        sb_num_cluster;
		cnoid::DoubleSpinBox*  ds_volume_ratio;
		cnoid::DoubleSpinBox*  ds_dist_th;
		cnoid::DoubleSpinBox*  ds_scale;
		cnoid::DoubleSpinBox*  ds_grid_interval;
		cnoid::RadioButton*    rb_single;
		cnoid::RadioButton*    rb_liftup;
		cnoid::RadioButton*    rb_fromsideOCP;
		cnoid::RadioButton*    rb_fromsidestable;
		
		void okClicked();
		void rbVolumeRatioToggled();
		void rbClusterCylinderToggled();
	};

	class ShowBoundingBoxDialog : public QDialog
	{
	public:
		ShowBoundingBoxDialog();
	private:
		cnoid::RadioButton*   rb_show_bb;
		cnoid::RadioButton*   rb_show_bbmesh;
		cnoid::RadioButton*   rb_show_cylinder;
		cnoid::CheckBox*      cb_is_showonlytarget;
		cnoid::RadioButton*   rb_num_cluster;
		cnoid::RadioButton*   rb_volume_ratio;
		cnoid::SpinBox*				sb_num_cluster;
		cnoid::DoubleSpinBox* ds_volume_ratio;
		cnoid::DoubleSpinBox* ds_scale;
		cnoid::DoubleSpinBox* ds_dist_th;

		void okClicked();
		void rbVolumeRatioToggled();
		void rbShowCylinderToggled();
	};

	class ScalingHandDialog : public QDialog
	{
	public:
		cnoid::RadioButton rbx,rby,rbz,rbxyz;
		cnoid::DoubleSpinBox scaleX;
		cnoid::DoubleSpinBox scaleY;
		cnoid::DoubleSpinBox scaleZ;
		cnoid::DoubleSpinBox scaleStart;
		cnoid::DoubleSpinBox scaleEnd;
		cnoid::DoubleSpinBox scaleStep;
		QLabel* lbAxis;

		cnoid::SpinBox clusterNumber;
		cnoid::DoubleSpinBox overlapVolumeRatio;
		cnoid::DoubleSpinBox gridInterval;
		cnoid::RadioButton rbNumber,rbVolumeRatio;

		ScalingHandDialog();
		void okClicked();
		void rbToggled();
		void rbVolumeRatioToggled();
	};

	class ModifyPatternDialog;

	class GraspDialog : public QDialog
	{
	public:
		GraspDialog();
		~GraspDialog();

		void afterFinishModifyData(bool is_done);

		virtual void reject();
		virtual void accept();

	private:
		std::ostream& os;

		cnoid::SpinBox targetNumber;
		QCheckBox* isShowContactCluster;
		QCheckBox* isShowHandContactCluster;
		cnoid::RadioButton*   rb_move_obj;
		cnoid::RadioButton*   rb_move_arm;

		ModifyPatternDialog* pDialog;
		bool modify_flag_;

		void okClicked();
		void modifyClicked();

		void finalize();
	};

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API GraspDataGenBar : public cnoid::ToolBar, public boost::signals::trackable
#else
        class EXCADE_API GraspDataGenBar : public cnoid::ToolBar
#endif
        {
          public:

            static GraspDataGenBar* instance();

            virtual ~GraspDataGenBar();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
#else
            cnoid::Signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
#endif
                return sigBodyItemSelectionChanged_;
            }

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            cnoid::SignalProxy< boost::signal<void(cnoid::BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
#else
            cnoid::SignalProxy<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged() {
#endif
                return sigCurrentBodyItemChanged_;
            }

            const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems() {
                return selectedBodyItems_;
            }

            cnoid::BodyItem* currentBodyItem() {
                return currentBodyItem_.get();
            }

            bool makeSingleSelection(cnoid::BodyItemPtr bodyItem);

          protected:

            virtual bool storeState(cnoid::Archive& archive);
            virtual bool restoreState(const cnoid::Archive& archive);

          private:

            GraspDataGenBar();

            cnoid::MessageView& mes;
            std::ostream& os;

            cnoid::BodyItemPtr currentBodyItem_;
            cnoid::ItemList<cnoid::BodyItem> selectedBodyItems_;
            cnoid::ItemList<cnoid::BodyItem> targetBodyItems;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#else
            cnoid::Connection connectionOfCurrentBodyItemDetachedFromRoot;
            cnoid::Signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            cnoid::Signal<void(cnoid::BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif
            

            void onItemSelectionChanged(const cnoid::ItemList<cnoid::BodyItem>& bodyItems);
            void onBodyItemDetachedFromRoot();

						void onGenerateGraspPattern();
						void onAddGraspPattern();
						void onDisplayBoundingBox();
						void onDisplayContactPoint();
						void onDisplayApproachPoint();
						void onSearchHandScale();
						void onSeqGenerateGraspPattern();
						void onSearchWaistPos();
						void onDisplayWaistPosResult();
						void onGenerateMotion();
						void onSetCollectionBox();
						void onSetCollectedObj();
						void onClearCollection();
						void onParam();

						GraspDialog grasp;
						PrehensionParamDialog* prehen_param_dialog;
						AppendPatternDialog* append_pattern_dialog;

		static int count;
        };
}

#endif

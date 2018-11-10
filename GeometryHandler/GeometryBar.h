/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_ROBOTICS_GEOMETRY_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GEOMETRY_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/
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
#include <QCheckBox>
#include <QPushButton>

#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

namespace cnoid {

    class MessageView;

}

namespace grasp {
	class ClusteringDialog : public QDialog
	{
		public:
		cnoid::SpinBox targetNumber;

		ClusteringDialog() ;
		void okClicked();
	};
	class SaveClusteringDialog : public QDialog
	{
		public:
		cnoid::SpinBox targetNumber;

		SaveClusteringDialog() ;
		void okClicked();
	};


#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	class EXCADE_API GeometryBar : public cnoid::ToolBar, public boost::signals::trackable
#else
	class EXCADE_API GeometryBar : public cnoid::ToolBar
#endif
	{
		public:
			friend class ClusteringDialog;
			friend class SaveClusteringDialog;

		static GeometryBar* instance();

		virtual ~GeometryBar();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		boost::signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
			return sigBodyItemSelectionChanged_;
		}
		cnoid::SignalProxy< boost::signal<void(cnoid::BodyItem* currentBodyItem)> > sigCurrentBodyItemChanged() {
#else
		cnoid::Signal<void(const cnoid::ItemList<cnoid::BodyItem>& selectedBodyItems)>& sigBodyItemSelectionChanged() {
			return sigBodyItemSelectionChanged_;
		}
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

		GeometryBar();

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
		void onClusteringButtonClicked();
		//tajima
		void onCalcClusteringButtonClicked();

		void onStepButtonClicked();
		void onLoadClustersButtonClicked();
		void onObjectButtonClicked();
		void onRobotButtonClicked();
		void onEnvironButtonClicked();
		void onRemoveEnvButtonClicked();
		void onGraspButtonClicked();
		void onPlaceButtonClicked();
		void onPickAndPlaceButtonClicked();
		void onStopButtonClicked();
		void onTrajectoryPlanButtonClicked();
		void onArmButtonClicked();
		void onCloseButtonClicked();
		void onSelectGraspPattern();
		ClusteringDialog clustering;
		SaveClusteringDialog saveClustering;

		static int count;
	};
}

#endif

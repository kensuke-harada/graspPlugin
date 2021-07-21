/**
 * @file   PickingTaskPlannerBar.h
 * @author Akira Ohchi
*/

#ifndef _PICKINGTASKPLANNER_PICKINGTASKPLANNERBAR_H_
#define _PICKINGTASKPLANNER_PICKINGTASKPLANNERBAR_H_

#include <iostream>

#include <cnoid/BodyItem>
#include <cnoid/ItemList>
#include <cnoid/ItemTreeView>
#include <cnoid/ToolBar>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#endif

namespace cnoid {
	class MessageView;
}

namespace grasp {
	class PickingTaskPlannerBar :
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		public cnoid::ToolBar, public boost::signals::trackable {
#else
		public cnoid::ToolBar {
#endif
	public:
		static  PickingTaskPlannerBar* instance();
		virtual ~PickingTaskPlannerBar();

	private:
		PickingTaskPlannerBar();

		cnoid::MessageView& mes;
		std::ostream&				os;

		void onStartPickingClicked();
		void onStartPickingNoRejectionClicked();
		void onStartPickingMultiPCClicked();
		void onInitClicked();
		void onShowClusterClicked();
		void onShowPrevClicked();
		void onTriggerStartClicked();
		void onShowPrevGridClicked();
		void onShowCurrGridClicked();
		void onShowMergePointsClicked();
	};
}

#endif /* _PICKINGTASKPLANNER_PICKINGTASKPLANNERBAR_H_ */

/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _TASK_BASE_BAR_H_INCLUDED
#define _TASK_BASE_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/ 
 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

//#include "exportdef.h"

//#include "TaskBasePlanner.h"

#include "exportdef.h"

#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif

#include "Action.h"
#include "Task.h"

using namespace cnoid;

namespace cnoid {

    class MessageView;
	
}
    
namespace grasp {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	class EXCADE_API TaskBaseBar : public cnoid::ToolBar, public boost::signals::trackable
#else
	class EXCADE_API TaskBaseBar : public cnoid::ToolBar
#endif
        {
		public:

			static TaskBaseBar* instance();
			virtual ~TaskBaseBar();


		protected:
			virtual bool storeState(Archive& archive);
			virtual bool restoreState(const Archive& archive);
		
		private:

			TaskBaseBar();
			MessageView& mes;
			std::ostream& os;

			BodyItemPtr currentBodyItem_;
			ItemList<BodyItem> selectedBodyItems_;
			ItemList<BodyItem> targetBodyItems;

			static int count;
			ActionItemPtr action;
            TaskItemPtr task;
			int actionCount;
		
            void onNewTaskItemButtonClicked();
			void onNewActionItemButtonClicked();
			void onSaveActionItemButtonClicked();
			void onSetMainActionItemButtonClicked();
			void onSetSubActionItemButtonClicked();
			void onSetActionStateButtonClicked();

			void onTaskBasePlanButtonClicked();

			void onPlanButtonClicked();

			void onExportAssemblyGraphClicked();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
			boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
			boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
#else
			cnoid::Connection connectionOfCurrentBodyItemDetachedFromRoot;
			Signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
			Signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif

            void onItemSelectionChanged(const ItemList<Item> &items);
			void onBodyItemDetachedFromRoot();
	};
}

#endif

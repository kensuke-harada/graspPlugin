/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/ 
 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

//#include "exportdef.h"

#include "TrajectoryPlanner.h"

#include "exportdef.h"

#ifndef  CNOID_10_11_12_13
	#include <cnoid/ItemList>
#endif


using namespace cnoid;

namespace cnoid {

    class MessageView;
	
}
    
    namespace grasp {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API TrajectoryBar : public cnoid::ToolBar, public boost::signals::trackable
#else
        class EXCADE_API TrajectoryBar : public cnoid::ToolBar
#endif
        {
          public:

            static TrajectoryBar* instance();

            virtual ~TrajectoryBar();


          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            TrajectoryBar();

            MessageView& mes;
		std::ostream& os;
	//	  GraspController* gc;
			TrajectoryPlanner* trajectoryPlanner_;

//			BodyItemPtr currentBodyItem_;
  //          ItemList<BodyItem> selectedBodyItems_;
    //        ItemList<BodyItem> targetBodyItems;

     //       boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#else
            Signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
            Signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;
#endif

            void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
            void onBodyItemDetachedFromRoot();
	    void onTrajectoryPlanButtonClicked();
	    void onResetButtonClicked();
	    void onSetStartMotionStateButtonClicked();
	    void onSetEndMotionStateButtonClicked();
	    void onParameterButtonClicked();
        };
    }

#endif

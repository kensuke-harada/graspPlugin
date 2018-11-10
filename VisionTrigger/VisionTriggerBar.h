/**
   c) Tokuo Tsuji (Kyushu univ./AIST)
*/

#ifndef EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"


#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#endif


//#include <cnoid/Grasp/TrajectoryPlanner.h>

using namespace cnoid;
    
    namespace grasp {

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API VisionTriggerBar : public ToolBar, public boost::signals::trackable
#else
        class EXCADE_API VisionTriggerBar : public ToolBar
#endif
        {
          public:

            static VisionTriggerBar* instance();

            virtual ~VisionTriggerBar();

          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            VisionTriggerBar();

            MessageView& mes;
			std::ostream& os;

//	    BodyItemPtr currentBodyItem_;
//            ItemList<BodyItem> targetBodyItems;

//            boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            
//            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
//            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

//            void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
//            void onBodyItemDetachedFromRoot();
            void onStartButtonClicked();
            void onStopButtonClicked();

						void onExtStartButtonClicked();
        };
    }

#endif

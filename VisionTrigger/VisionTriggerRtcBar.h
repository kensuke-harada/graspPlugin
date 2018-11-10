/**
   c) Tokuo Tsuji (Kyushu univ./AIST)
*/

#ifndef EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED
#define EXCADE_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/Robotics/BodyItem.h>
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/SignalProxy>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

//#include <cnoid/Grasp/TrajectoryPlanner.h>

using namespace cnoid;

namespace grasp {


        class EXCADE_API VisionTriggerRtcBar : public ToolBar, public boost::signals::trackable
        {
          public:

            static VisionTriggerRtcBar* instance();

            virtual ~VisionTriggerRtcBar();

          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            VisionTriggerRtcBar();

            MessageView& mes;
	    std::ostream& os;

//	    BodyItemPtr currentBodyItem_;
//            ItemList<BodyItem> targetBodyItems;

            boost::signals::connection connectionOfCurrentBodyItemDetachedFromRoot;
            
//            boost::signal<void(const ItemList<BodyItem>& selectedBodyItems)> sigBodyItemSelectionChanged_;
//            boost::signal<void(BodyItem* currentBodyItem)> sigCurrentBodyItemChanged_;

//            void onItemSelectionChanged(const ItemList<BodyItem>& bodyItems);
            void onBodyItemDetachedFromRoot();
            void onStartButtonClicked();
            void onStopButtonClicked();
        };
    }

#endif

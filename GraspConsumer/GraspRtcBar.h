/**
   @author Shin'ichiro Nakaoka
*/

#ifndef cnoid_ROBOTICS_GRASP_BAR_H_INCLUDED
#define cnoid_ROBOTICS_GRASP_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#else
#include <cnoid/Signal>
#endif

//#include <cnoidPlugins/Grasp/TrajectoryPlanner.h>

using namespace cnoid;


//    class MessageView;
    
    namespace grasp {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API GraspRtcBar : public cnoid::ToolBar, public boost::signals::trackable
#else
        class EXCADE_API GraspRtcBar : public cnoid::ToolBar
#endif
        {
          public:

            static GraspRtcBar* instance();

            virtual ~GraspRtcBar();

          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            GraspRtcBar();

            MessageView& mes;
			std::ostream& os;

            void onStartButtonClicked();
            void onStartButtonClicked2();
            void onStopButtonClicked();
        };
    }

#endif

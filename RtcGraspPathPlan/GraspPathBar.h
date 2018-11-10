/**
   @author Shin'ichiro Nakaoka
*/

#ifndef _GRASP_PATH_BAR_H_INCLUDED
#define _GRASP_PATH_BAR_H_INCLUDED

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ToolBar>	/* modified by qtconv.rb 0th rule*/   
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  
#include "../Grasp/exportdef.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/SignalProxy>
#endif

//#include <cnoidPlugins/Grasp/TrajectoryPlanner.h>

using namespace cnoid;


//    class MessageView;
    
   namespace grasp {

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
        class EXCADE_API GraspPathBar : public cnoid::ToolBar, public boost::signals::trackable
#else
        class EXCADE_API GraspPathBar : public cnoid::ToolBar
#endif
        {
          public:

            static GraspPathBar* instance();

            virtual ~GraspPathBar();

          protected:

            virtual bool storeState(Archive& archive);
            virtual bool restoreState(const Archive& archive);

          private:

            GraspPathBar();

            MessageView& mes;
            std::ostream& os;

            void onStartButtonClicked();
            void onStartButtonClicked2();
            void onStopButtonClicked();
            void onCreateRecordButtonClicked();
            void onDeleteRecordButtonClicked();
            void onAppearButtonClicked();
            void onDisappearButtonClicked();
        };
   }

#endif

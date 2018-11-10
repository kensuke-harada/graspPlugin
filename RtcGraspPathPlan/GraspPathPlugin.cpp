
/*! @file
  @author Shin'ichiro Nakaoka
*/

// #include <glibmm/i18n.h>	/* modified by qtconv.rb 5th rule*/  
#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ItemManager>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyMotionItem>	/* modified by qtconv.rb 0th rule*/  

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>
//#include "GraspConsumer.h"

#include "GraspPathBar.h"
#include "GraspPathController.h"


using namespace cnoid;
using namespace cnoid;
using namespace grasp;


namespace {
    class GraspPathPlugin : public Plugin
    {
    public:
        
        GraspPathPlugin() : Plugin("RtcGraspPath") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
            depend("Trajectory");
#else
            require("Trajectory");
#endif
        }

        bool initialize() {
	
			GraspPathController::instance()->RtcStart ();
			addToolBar(grasp::GraspPathBar::instance());
            
            return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(GraspPathPlugin);

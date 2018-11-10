/**
   c) Tokuo Tsuji (Kyushu univ./AIST)
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

#include "VisionTriggerBar.h"
#include "VisionTriggerRtcController.h"


using namespace cnoid;
using namespace grasp;




namespace {
    class VisionTriggerPlugin : public Plugin
    {
    public:
        
        VisionTriggerPlugin() : Plugin("VisionTrigger") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Grasp");
#else
		require("Grasp");
#endif
        }

        bool initialize() {
	
			VisionTriggerRtcController::RtcStart ();
			addToolBar(grasp::VisionTriggerBar::instance());
            
            return true;
        }
    };
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(VisionTriggerPlugin);

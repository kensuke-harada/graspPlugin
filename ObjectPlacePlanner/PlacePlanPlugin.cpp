/*!
  @file
  @author Kensuke Harada
*/

#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <iostream>

#include "PlacePlanner.h"
#include "PlaceBar.h"

using namespace std;
using namespace cnoid;

namespace {

    class PlacePlanPlugin : public Plugin
    {
    public:
	PlacePlanPlugin() : Plugin("ObjectPlacePlanner")
	{

        }

        virtual ~PlacePlanPlugin()
        {

        }

        virtual bool initialize() {
    		addToolBar(PlaceBar::instance());
    		return true;
        }

        virtual bool finalize() {
		return true;
        }

    private:
        //AudioItemManager* audioItemManager;
    };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(PlacePlanPlugin);

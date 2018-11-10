#include <iostream>
#include <cnoid/Plugin>

#include "PCLBar.h"

using namespace cnoid;
using namespace std;

class GraspPCLPlugin : public cnoid::Plugin
{
public:

    GraspPCLPlugin() : Plugin("GraspPCL") {
    	require("Grasp");
    	require("GeometryHandler");
    }

    virtual bool initialize() {
    	addToolBar(PCLBar::instance());
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(GraspPCLPlugin);

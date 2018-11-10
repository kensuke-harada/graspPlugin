/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include "SoftFingerStabilityBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

class SoftFingerStabilityPlugin : public cnoid::Plugin
{
private:
public:
	
	SoftFingerStabilityPlugin() : Plugin("SoftFingerStability") { 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Grasp");
		depend("GeometryHandler");
#else
		require("Grasp");
		require("GeometryHandler");
#endif
	}
	
	virtual bool initialize() {
	
		addToolBar(SoftFingerStabilityBar::instance());

		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(SoftFingerStabilityPlugin);

/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <cnoid/Plugin>	/* modified by qtconv.rb 0th rule*/  
#include "GraspDataGenBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;


class GraspDataGenPlugin : public cnoid::Plugin
{
private:
public:
	
	GraspDataGenPlugin() : Plugin("GraspDataGen") { 
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		depend("Grasp");
		depend("GeometryHandler");
		depend("SoftFingerStability");
		depend("ConstraintIK");
#else
		require("Grasp");
		require("GeometryHandler");
		require("SoftFingerStability");
		require("ConstraintIK");
		require("GraspPCL");
		require("PoseSeq");
#endif
	}
	
	virtual bool initialize() {
	
		addToolBar(GraspDataGenBar::instance());

		return true;
	}
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(GraspDataGenPlugin);

/**
c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <cnoid/Plugin> /* modified by qtconv.rb 0th rule*/
#include "ConstraintIKBar.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

class ConstraintIKPlugin : public cnoid::Plugin {
public:
  ConstraintIKPlugin() : Plugin("ConstraintIK") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    depend("Grasp");
    depend("GeometryHandler");
#else
    require("Grasp");
    require("GeometryHandler");
#endif
  }

  virtual bool initialize() {
    addToolBar(ConstraintIKBar::instance());
    return true;
  }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(ConstraintIKPlugin);

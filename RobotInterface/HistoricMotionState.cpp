#include "HistoricMotionState.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

HistoricMotionState::HistoricMotionState()
{
    this->historicMS.clear();
}

HistoricMotionState *HistoricMotionState::instance()
{
    static HistoricMotionState* instance = new HistoricMotionState();
    return instance;
}

HistoricMotionState::~HistoricMotionState()
{

}



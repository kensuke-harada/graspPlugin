/**
  *@ Weiwei Wan
*/

#ifndef HISTORICMOTIONSTATE_H
#define HISTORICMOTIONSTATE_H

#include <vector>
#include "../Grasp/PlanBase.h"

class HistoricMotionState
{
  public:
    static HistoricMotionState* instance();

    ~HistoricMotionState();

    // save all historical motion states to historicMS
		std::vector<grasp::MotionState> historicMS;

  private:
    HistoricMotionState();
};

#endif // HISTORICMOTIONSTATE_H



/**
 * @file   PickingTaskResultDrawer.h
 * @author Akira Ohchi
 */

#ifndef _PICKINGTASKPLANNER_PICKINGTASKRESULTDRAWER_H_
#define _PICKINGTASKPLANNER_PICKINGTASKRESULTDRAWER_H

#include "exportdef.h"

namespace grasp {
	class EXCADE_API PickingTaskResultDrawer {
	public:
		PickingTaskResultDrawer();
		virtual ~PickingTaskResultDrawer();

		static void showPrevGrid();
		static void showCurrGrid();
		static void showMergePoints();
		static void clearAll();
	};
}

#endif /* _PICKINGTASKPLANNER_PICKINGTASKRESULTDRAWER_H_ */

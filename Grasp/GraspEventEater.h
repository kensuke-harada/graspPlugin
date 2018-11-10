#ifndef _GRASP_GRASPEVENTEATER_H_
#define _GRASP_GRASPEVENTEATER_H_

#include <cnoid/Link>

#include "EventEater.h"

namespace grasp {
	class GraspEventEater :
		public EventEater {
	public:
		GraspEventEater();
		virtual ~GraspEventEater();

    virtual bool onButtonPressEvent(const cnoid::SceneWidgetEvent& event);
	};
}

#endif

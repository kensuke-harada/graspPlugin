#ifndef _GEOMETRY_GEOMETRYEVENTEATER_H_
#define _GEOMETRY_GEOMETRYEVENTEATER_H_

#include <map>

#include <cnoid/BodyItem>

#include "../Grasp/EventEater.h"

namespace grasp {
	class GeometryExecutor;

	class GeometryEventEater :
		public EventEater {
	public:
		GeometryEventEater();
		virtual ~GeometryEventEater();

    virtual bool onButtonPressEvent(const cnoid::SceneWidgetEvent& event);
		virtual void onContextMenuRequest(const cnoid::SceneWidgetEvent& event, cnoid::MenuManager& menuManager);
	private:
		std::map<cnoid::BodyItem*, GeometryExecutor*> exe_;

		GeometryExecutor* findOrCreateExecutor(const cnoid::BodyItemPtr item);
	};
}

#endif

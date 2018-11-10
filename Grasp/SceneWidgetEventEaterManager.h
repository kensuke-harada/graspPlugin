#ifndef _GRASP_SCENEWIDGETEVENTEATERMANGER_H_
#define _GRASP_SCENEWIDGETEVENTEATERMANGER_H_

#include <vector>

#include <cnoid/SceneWidgetEditable>
/* #include <cnoid/SceneDrawables>*/
#include <cnoid/SceneView>

#include "exportdef.h"

namespace grasp {
	class EventEater;
	class EXCADE_API SceneWidgetEventEaterManager :
		public cnoid::SceneWidgetEditable {
	public:
		static SceneWidgetEventEaterManager* instance();

		void initialize();
		void registrate();

		bool addEater(EventEater* eater);
		bool removeEater(EventEater* eater);

		virtual void onSceneModeChanged(const cnoid::SceneWidgetEvent& event);
    virtual bool onButtonPressEvent(const cnoid::SceneWidgetEvent& event);
    virtual bool onButtonReleaseEvent(const cnoid::SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const cnoid::SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const cnoid::SceneWidgetEvent& event, cnoid::MenuManager& menuManager);

	private:
		SceneWidgetEventEaterManager();
		~SceneWidgetEventEaterManager();

		std::vector<EventEater*> eaters_;
		typedef std::vector<EventEater*>::iterator EaterIte;
		typedef std::vector<EventEater*>::reverse_iterator EaterRIte;
	};
}

#endif

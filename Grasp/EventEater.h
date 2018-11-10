#ifndef _GRASP_EVENTEATER_H_
#define _GRASP_EVENTEATER_H_

#include <QMouseEvent>
#include <cnoid/SceneWidgetEditable>
#include <cnoid/SceneBody>

#include "exportdef.h"
#include <iostream>

namespace grasp {
	class EXCADE_API EventEater {
		public:
		EventEater() {;}
		virtual~ EventEater() {;}

		virtual void onSceneModeChanged(const cnoid::SceneWidgetEvent& event) {
		}

    virtual bool onButtonPressEvent(const cnoid::SceneWidgetEvent& event) {
			return false;
		}

		virtual bool onButtonReleaseEvent(const cnoid::SceneWidgetEvent& event) {
			return false;
		}

    virtual bool onPointerMoveEvent(const cnoid::SceneWidgetEvent& event) {
			return false;
		}

    virtual void onContextMenuRequest(const cnoid::SceneWidgetEvent& event, cnoid::MenuManager& menuManager) {
		}

	protected:
		bool getBodyLink(const cnoid::SceneWidgetEvent& event, cnoid::SceneLink** link, cnoid::SceneBody** body) {
			bool is_selected = false;
			for (size_t i = 0; i < event.nodePath().size(); i++) {
				cnoid::SceneLink* scene_link =  dynamic_cast<cnoid::SceneLink*>(event.nodePath()[i]);
				if (scene_link) {
					*link = scene_link;
					is_selected = true;
				}
				cnoid::SceneBody* scene_body =  dynamic_cast<cnoid::SceneBody*>(event.nodePath()[i]);
				if (scene_body) {
					*body = scene_body;
					is_selected = true;
				}
			}
			return is_selected;
		}
	};
}

#endif

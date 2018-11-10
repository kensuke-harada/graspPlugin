#include "SceneWidgetEventEaterManager.h"

#include <boost/bind.hpp>

#include "EventEater.h"

using namespace grasp;

SceneWidgetEventEaterManager* SceneWidgetEventEaterManager::instance() {
	static SceneWidgetEventEaterManager* instance = new SceneWidgetEventEaterManager();
	return instance;
}

SceneWidgetEventEaterManager::SceneWidgetEventEaterManager() {
}

SceneWidgetEventEaterManager::~SceneWidgetEventEaterManager() {
}

void SceneWidgetEventEaterManager::initialize() {
	cnoid::SceneView::instance()->sceneWidget()->sigStateChanged().connect(boost::bind(&SceneWidgetEventEaterManager::registrate, this));
}

void SceneWidgetEventEaterManager::registrate() {
	cnoid::SceneWidget* scene = cnoid::SceneView::instance()->sceneWidget();
	if (scene->isEditMode()) {
		if (scene->activeEventFilter()) return;
		scene->installEventFilter(this);
	} else {
		scene->removeEventFilter(this);
	}
}

bool SceneWidgetEventEaterManager::addEater(EventEater* eater) {
	for (size_t i = 0; i < eaters_.size(); i++) {
		if (eaters_[i] == eater) return false;
	}
	eaters_.push_back(eater);

	return true;
}

bool SceneWidgetEventEaterManager::removeEater(EventEater* eater) {
	for (EaterIte it = eaters_.begin(); it != eaters_.end(); ++it) {
		if (*it == eater) {
			eaters_.erase(it);
			return true;
		}
	}
	return true;
}

void SceneWidgetEventEaterManager::onSceneModeChanged(const cnoid::SceneWidgetEvent& event) {
	for (EaterRIte ite = eaters_.rbegin(); ite != eaters_.rend(); ++ite) {
		(*ite)->onSceneModeChanged(event);
	}
}

bool SceneWidgetEventEaterManager::onButtonPressEvent(const cnoid::SceneWidgetEvent& event) {
	for (EaterRIte ite = eaters_.rbegin(); ite != eaters_.rend(); ++ite) {
		if((*ite)->onButtonPressEvent(event)) break;
	}
	return false;
}

bool SceneWidgetEventEaterManager::onButtonReleaseEvent(const cnoid::SceneWidgetEvent& event) {
	for (EaterRIte ite = eaters_.rbegin(); ite != eaters_.rend(); ++ite) {
		if((*ite)->onButtonReleaseEvent(event)) break;
	}
	return false;
}

bool SceneWidgetEventEaterManager::onPointerMoveEvent(const cnoid::SceneWidgetEvent& event) {
	for (EaterRIte ite = eaters_.rbegin(); ite != eaters_.rend(); ++ite) {
		if((*ite)->onPointerMoveEvent(event)) break;
	}
	return false;
}

void SceneWidgetEventEaterManager::onContextMenuRequest(const cnoid::SceneWidgetEvent& event, cnoid::MenuManager& menuManager) {
	for (EaterRIte ite = eaters_.rbegin(); ite != eaters_.rend(); ++ite) {
		(*ite)->onContextMenuRequest(event, menuManager);
	}
}

#ifndef _SCENECAPTURE_H_
#define _SCENECAPTURE_H_

#include <string>

#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/SceneRenderer>
#include <cnoid/MessageView>

#include "exportdef.h"

namespace grasp {
	void EXCADE_API captureScene(const std::string& filepath) {
		cnoid::MessageView::instance()->flush();
		cnoid::SceneView::instance()->sceneWidget()->saveImage(filepath);
	}
}

#endif

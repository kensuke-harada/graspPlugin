// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*! @file
  @author Tokuo Tsuji
  @author Kensuke Harada
*/

#include <cnoid/Plugin>
#include <cnoid/MessageView>

#include <iostream>

#include "GeometryHandle.h"
#include "GeometryBar.h"

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include "../Grasp/SceneWidgetEventEaterManager.h"
#include "GeometryEventEater.h"
#endif

#ifdef CNOID_ENABLE_OSG 
#include "GeometryHandleSceneBody.h"
cnoid::OSGSceneBody* createGeometryHandle(cnoid::BodyItem* item) {
	return new grasp::GeometryHandleSceneBody(item);;
}
#endif


class GeometryHandlerPlugin : public cnoid::Plugin {
private:

public:

//	static GeometryHandleSceneBody* GeometryHandleSceneBody;



	GeometryHandlerPlugin() : Plugin("GeometryHandler") {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
 		depend("Grasp");
#else
		require("Grasp");
#endif
	}

	~GeometryHandlerPlugin() { }



	virtual bool initialize() {
#ifdef CNOID_ENABLE_OSG 
        cnoid::OSGSceneBodyManager* manager = cnoid::OSGSceneBodyManager::instance();
		manage(manager->addSceneBodyFactory(createGeometryHandle));
#endif
		addToolBar(grasp::GeometryBar::instance());
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
		grasp::SceneWidgetEventEaterManager::instance()->addEater(new grasp::GeometryEventEater());
#endif
		return true;
	}

};



CNOID_IMPLEMENT_PLUGIN_ENTRY(GeometryHandlerPlugin);

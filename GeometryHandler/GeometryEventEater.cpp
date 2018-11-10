#include "GeometryEventEater.h"

#include <cnoid/SceneDrawables>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>

#include <boost/bind.hpp>

#include "FindParallelPlane.h"
#include "GeometryExecutor.h"

using namespace grasp;

namespace {
	bool findPointedBodyItem(cnoid::SceneBody* body, cnoid::BodyItemPtr& pointedItem) {
		cnoid::ItemList<cnoid::BodyItem> bodyitemlist;
		bodyitemlist.extractChildItems(cnoid::ItemTreeView::instance()->rootItem());
		for (size_t i = 0; i < bodyitemlist.size(); i++) {
			if (bodyitemlist[i]->body() == body->body()) {
				pointedItem = bodyitemlist[i];
				return true;
			}
		}
		return false;
	}
}

GeometryEventEater::GeometryEventEater() {
}

GeometryEventEater::~GeometryEventEater() {
	std::map<cnoid::BodyItem*, GeometryExecutor*>::iterator it;
	for (it = exe_.begin(); it != exe_.end(); ++it) {
		if ((*it).second != NULL) delete (*it).second;
	}
	exe_.clear();
}

bool GeometryEventEater::onButtonPressEvent(const cnoid::SceneWidgetEvent& event) {
	if (event.button() == Qt::LeftButton && event.modifiers() & Qt::ControlModifier) {
		cnoid::SceneLink* link = NULL;
		cnoid::SceneBody* body = NULL;
		if (getBodyLink(event, &link, &body)) {
			cnoid::Vector3 pressPos = event.point();
			cnoid::BodyItemPtr pointedItem;
			if (findPointedBodyItem(body, pointedItem)) {
				FindParallelPlane::instance()->findTargetTriangle(link->link(), pressPos, pointedItem);
			}
		}
	}
	return true;
}

void GeometryEventEater::onContextMenuRequest(const cnoid::SceneWidgetEvent& event, cnoid::MenuManager& menuManager) {
	cnoid::SceneLink* link = NULL;
	cnoid::SceneBody* body = NULL;
	if (getBodyLink(event, &link, &body)) {
		cnoid::BodyItemPtr pointedItem;
		if (!findPointedBodyItem(body, pointedItem)) return;

		GeometryExecutor* exe = findOrCreateExecutor(pointedItem);

		menuManager.setPath("Geometry");
		menuManager.addItem("LimitedClustering")->sigTriggered().connect(boost::bind(&GeometryExecutor::limitedCluster, exe));
		menuManager.addItem("CreateBoundaryData")->sigTriggered().connect(boost::bind(&GeometryExecutor::createBoundaryData, exe));
		menuManager.addItem("BinaryClustering")->sigTriggered().connect(boost::bind(&GeometryExecutor::clusterBinary, exe));
		menuManager.addItem("CreateDepartData")->sigTriggered().connect(boost::bind(&GeometryExecutor::createDepartData, exe));
		menuManager.addItem("StepClustering")->sigTriggered().connect(boost::bind(&GeometryExecutor::clusterStep, exe));
		menuManager.addItem("Reset")->sigTriggered().connect(boost::bind(&GeometryExecutor::reset, exe));
		menuManager.addItem("Quadrical Clustering")->sigTriggered().connect(boost::bind(&GeometryExecutor::matchCluster, exe));
		menuManager.addItem("Plane Clustering(old)")->sigTriggered().connect(boost::bind(&GeometryExecutor::clusterOld, exe));
		menuManager.addItem("Find Parallel Plane Pair")->sigTriggered().connect(boost::bind(&GeometryExecutor::findPair, exe));
		menuManager.addItem("Output Parallel Plane Pair")->sigTriggered().connect(boost::bind(&GeometryExecutor::resultOutput, exe));
		menuManager.addItem("Increase Seed Size")->sigTriggered().connect(boost::bind(&GeometryExecutor::incrementSeedSize, exe));
		menuManager.addItem("Decrease Seed Size")->sigTriggered().connect(boost::bind(&GeometryExecutor::decrementSeedSize, exe));
		menuManager.addItem("Set Intention Parameter")->sigTriggered().connect(boost::bind(&GeometryExecutor::clusterOutput, exe));
		menuManager.addItem("Plane Clustering (new)")->sigTriggered().connect(boost::bind(&GeometryExecutor::clusterObjEnvContact, exe));
		menuManager.addItem("PlaneClustering for Assembly")->sigTriggered().connect(boost::bind(&GeometryExecutor::clusterForAssembly, exe));
		menuManager.addItem("Show Contact Locus")->sigTriggered().connect(boost::bind(&GeometryExecutor::assemblyShowContactPointLocus, exe));
		menuManager.addItem("Generate Assembly Graph")->sigTriggered().connect(boost::bind(&GeometryExecutor::assemblyGenerateGraph, exe));
		menuManager.setPath("/");
		menuManager.addSeparator();
	}
}

GeometryExecutor* GeometryEventEater::findOrCreateExecutor(const cnoid::BodyItemPtr item) {
	if (exe_.count(item.get()) == 0) exe_[item.get()] = new GeometryExecutor(item);
	return exe_[item.get()];
}

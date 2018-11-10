#include "SgVisitShapeGet.h"

using namespace grasp;

struct NodeTypeRegistration {
	NodeTypeRegistration() {
		cnoid::SgNode::registerType<SgClusterRenderer, cnoid::SgNode>();

		cnoid::SceneRenderer::addExtension(
		 [](cnoid::SceneRenderer* renderer){
#ifdef CNOID_GE_16
			 auto functions = renderer->renderingFunctions();
			 functions->setFunction<SgClusterRenderer>(
#else
			 auto& functions = renderer->renderingFunctions();
			 functions.setFunction<SgClusterRenderer>(
#endif
				 [=](cnoid::SgNode* node){
					 static_cast<SgClusterRenderer*>(node)->renderCluster(renderer);
									});
					});
	}
} registration;

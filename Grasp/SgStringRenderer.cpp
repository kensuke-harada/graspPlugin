#include "SgStringRenderer.h"

using namespace grasp;

struct NodeTypeRegistration {
	NodeTypeRegistration() {
		cnoid::SgNode::registerType<SgStringRenderer, cnoid::SgNode>();

		cnoid::SceneRenderer::addExtension(
		 [](cnoid::SceneRenderer* renderer){
#ifdef CNOID_GE_16
			 auto functions = renderer->renderingFunctions();
			 functions->setFunction<SgStringRenderer>(
#else
			 auto& functions = renderer->renderingFunctions();
			 functions.setFunction<SgStringRenderer>(
#endif
			 	 [=](cnoid::SgNode* node){
					 static_cast<SgStringRenderer*>(node)->render(renderer);
									});
					});
	}
} registration;

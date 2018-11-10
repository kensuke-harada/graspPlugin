#include "GraspPoints.h"

#include <cnoid/EditableSceneBody>

using namespace grasp;

GraspPointsItem::GraspPointsItem() :
	is_activate_(false),
	is_show_(false),
	n_points_(0),
	orig_shape_(NULL) {
	point_size_ = 0.005;
	point_color_ = cnoid::Vector3(1, 0, 0);
	setAttribute(cnoid::Item::TEMPORAL);
}

GraspPointsItem::~GraspPointsItem() {
}

void GraspPointsItem::initializeClass(cnoid::ExtensionManager* ext) {
	static bool initialized = false;

	if (!initialized) {
		cnoid::ItemManager& im = ext->itemManager();
		im.registerClass<GraspPointsItem>("GraspPointsItem");
		initialized = true;
	}
}

void GraspPointsItem::setActionItem(const ActionItemPtr& action_item) {
	if (is_activate_) {
		std::cerr << "Implemetaion error: GraspPointsItem::setActionItem is called more than once!" << std::endl;
			return;
	}
	action_item_ = action_item;
	connection_ref_changed_ =
		action_item->sigRefPoseChanged().connect(boost::bind(&GraspPointsItem::onRefPoseChanged, this));
	connection_actionitem_disconnected_ =
		action_item->sigDisconnectedFromRoot().connect(boost::bind(&GraspPointsItem::onActionItemDisconnected, this));

	addGraspPointsNode();
	cur_ite_ = action_item_->actionSeq().begin();

	activate();
}

cnoid::BodyItemPtr GraspPointsItem::getTargetBodyItem() const {
	return action_item_->getSubItem();
}

void GraspPointsItem::activate() {
	if (is_activate_) return;

	int checkID = 0;
	connection_check_toggled_ = cnoid::ItemTreeView::instance()->
		sigCheckToggled(this, checkID).connect(boost::bind(&GraspPointsItem::onCheckToggled, this, _1));
	connection_time_changed_ = cnoid::TimeBar::instance()->
		sigTimeChanged().connect(boost::bind(&GraspPointsItem::onTimeChanged, this, _1));
	Item::sigDisconnectedFromRoot().connect(boost::bind(&GraspPointsItem::onDetachFromRoot, this));

	updatePoints();
	onTimeChanged(cnoid::TimeBar::instance()->time());

	is_activate_ = true;
}

void GraspPointsItem::onCheckToggled(bool check) {
	is_show_ = check;
	if (check) {
		onTimeChanged(cnoid::TimeBar::instance()->time());
	} else {
		hidePoints();
	}
}

bool GraspPointsItem::onTimeChanged(double time) {
	if (is_show_) {
		if (seek(time)) {
			showPoints();
		} else {
			hidePoints();
		}
	}

	return true;
}

void GraspPointsItem::onDetachFromRoot() {
	hidePoints();
	connection_time_changed_.disconnect();
	connection_check_toggled_.disconnect();
	connection_ref_changed_.disconnect();
	connection_actionitem_disconnected_.disconnect();
}

void GraspPointsItem::onRefPoseChanged() {
	cur_ite_ = action_item_->actionSeq().begin();
}

void GraspPointsItem::onActionItemDisconnected() {
	detachFromParentItem();
}

bool GraspPointsItem::seek(double time) {
	const ActionSequence& action_seq = action_item_->actionSeq();
	if (action_seq.empty()) return false;

	double time_offset = action_item_->getStartTime();
	double time_inaction = time - time_offset;

	if ((action_seq.begin())->time() > time_inaction) return false;
	if ((action_seq.back()).time() < time_inaction) return false;

	double cur_time = cur_ite_->time();
	if (time_inaction == cur_time) {
		if (cur_ite_ == action_seq.begin()) {
			updatePoints();
		}
	} else if (time_inaction < cur_time) {
		ActionSeqConstIte target_ite = cur_ite_;
		target_ite--;
		while (target_ite != action_seq.begin()) {
			if (target_ite->time() < time_inaction) {
				break;
			}
			target_ite--;
		}
		cur_ite_ = target_ite;
		updatePoints();
	} else {
		ActionSeqConstIte target_ite = cur_ite_;
		target_ite++;
		while (target_ite != action_seq.end()) {
			if (target_ite->time() > time_inaction) {
				break;
			}
			target_ite++;
		}
		target_ite--;
		if (target_ite != cur_ite_) {
			cur_ite_ = target_ite;
			updatePoints();
		}
	}
	return true;
}

void GraspPointsItem::updatePoints() {
	if (cur_ite_ == action_item_->actionSeq().end()) return;
	const std::vector<cnoid::Vector3>& gp = (*cur_ite_).subPrehension().grasp_points;
	for (size_t i = 0; i < gp.size(); i++) {
		grasp_point_transforms_[i]->setTranslation(gp[i]);
	}
}

void GraspPointsItem::showPoints() {
	if (cur_ite_ == action_item_->actionSeq().end()) return;
	const std::vector<cnoid::Vector3>& gp = (*cur_ite_).subPrehension().grasp_points;
	for (int i = 0; i < n_points_; i++) {
		bool has_point = (i < gp.size());
		grasp_point_switches_[i]->setTurnedOn(has_point);
		grasp_point_switches_[i]->notifyUpdate();
	}
#ifndef CNOID_GE_16
	dynamic_cast<cnoid::GLSceneRenderer&>(cnoid::SceneView::instance()->sceneWidget()->renderer()).requestToClearCache();
#endif
}

void GraspPointsItem::hidePoints() {
	for (int i = 0; i < n_points_; i++) {
		grasp_point_switches_[i]->setTurnedOn(false);
		grasp_point_switches_[i]->notifyUpdate();
	}
#ifndef CNOID_GE_16
	dynamic_cast<cnoid::GLSceneRenderer&>(cnoid::SceneView::instance()->sceneWidget()->renderer()).requestToClearCache();
#endif
}

void GraspPointsItem::addGraspPointsNode() {
	// obtain max number of points in target action sequence
	const ActionSequence& action_seq = action_item_->actionSeq();
	for (ActionSeqConstIte ite = action_seq.begin(); ite != action_seq.end(); ++ite) {
		int n = (*ite).subPrehension().grasp_points.size();
		if (n > n_points_) {
			n_points_ = n;
		}
	}

	// clone visual shape of target link
	cnoid::Link* root_link = getTargetBodyItem()->body()->rootLink();
	cnoid::SgCloneMap cloneMap;
	orig_shape_ = root_link->collisionShape()->cloneNode(cloneMap);

	// add point shape nodes to the target link
	for (int i = 0; i < n_points_; i++) {
		addSphere();
	}
	root_link->setCollisionShape(orig_shape_);
	// if (getTargetBodyItem()->existingSceneBody()) {
	// 	cloneMap.clear();
	// 	cloneMap.setNonNodeCloning(true);
	// 	getTargetBodyItem()->existingSceneBody()->cloneShapes(cloneMap);
	// 	getTargetBodyItem()->existingSceneBody()->updateModel();
	// }
}

void GraspPointsItem::addSphere() {
	cnoid::MeshGenerator generator;
	cnoid::SgMeshPtr mesh = generator.generateSphere(point_size_);

	cnoid::SgShapePtr shape = new cnoid::SgShape();
	shape->setMesh(mesh);

	cnoid::SgMaterialPtr material = shape->getOrCreateMaterial();
	material->setTransparency(0.4);
	material->setDiffuseColor(point_color_);
	material->setEmissiveColor(point_color_);

	cnoid::SgPosTransformPtr pos = new cnoid::SgPosTransform();
	pos->setTranslation(cnoid::Vector3::Zero());
	pos->setRotation(cnoid::Matrix3::Identity());
	pos->addChild(shape);

	SgGraspPointPtr sw = new SgGraspPoint();
	sw->addChild(pos);

	grasp_point_transforms_.push_back(pos);
	grasp_point_switches_.push_back(sw);

	dynamic_cast<cnoid::SgGroup*>(getTargetBodyItem()->body()->rootLink()->visualShape())
		->addChild(sw, true);
}

#ifdef CNOID_GE_16
struct NodeTypeRegistration {
	NodeTypeRegistration() {
		cnoid::SgNode::registerType<SgGraspPoint, cnoid::SgGroup>();

		cnoid::SceneRenderer::addExtension
			([](cnoid::SceneRenderer* renderer) {
#ifdef CNOID_GE_16
			 	auto functions = renderer->renderingFunctions();
			 	functions->setFunction<SgGraspPoint>
#else
			 	auto& functions = renderer->renderingFunctions();
			 	functions.setFunction<SgGraspPoint>
#endif
					 ([=](cnoid::SgNode* node) {
						 SgGraspPoint* gp = static_cast<SgGraspPoint*>(node);
						 if (gp->isTurnedOn()) {
								 for (cnoid::SgGroup::const_iterator p = gp->begin(); p != gp->end(); ++p) {
									 renderer->renderNode(*p);
								 }
							 }
					 });
			 });
	}
} registeration;
#endif

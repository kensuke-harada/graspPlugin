#include "WaistPosSearchMotion.h"

#include <string>

#include <boost/lexical_cast.hpp>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>

#include <../src/PoseSeqPlugin/PoseSeqItem.h>

using namespace std;
using namespace grasp;

WaistPosSearchMotion::WaistPosSearchMotion() 
	 : 	os (cnoid::MessageView::mainInstance()->cout()), collection_box(NULL) {
		 init();
}

WaistPosSearchMotion* WaistPosSearchMotion::instance(WaistPosSearchMotion *wpsm) {
	static WaistPosSearchMotion* instance = (wpsm) ? wpsm : new WaistPosSearchMotion();
	if(wpsm) instance = wpsm;
	return instance;
}


void WaistPosSearchMotion::init() {
	tc = PlanBase::instance();

	res = WaistPosSearchResult::instance();

	cur_pos_id = 0;

	BoxInfoArray boxes = res->getBoxes();

	for (size_t i = 0; i < boxes.size(); i++) {
		done_grasp[boxes[i]] = false;
	}
}

void WaistPosSearchMotion::addMotion() {
	if (cur_pos_id == res->route_coords.size() + 1) {
		init();
		return;
	}

	if (cur_pos_id == 0) {
		addMoveMotion();
		return;
	}

	Eigen::Vector2i coord = res->route_coords[cur_pos_id-1];
	BoxInfoArray boxes = res->region.getBoxes(coord.x(), coord.y());

	for (size_t i = 0; i < boxes.size(); i++) {
		if (!done_grasp[boxes[i]]) {
			done_grasp[boxes[i]] = true;
			return;
		}
	}
	addMoveMotion();
}

void WaistPosSearchMotion::addMoveMotion() {
	cnoid::Vector3 start_p;
	cnoid::Vector3 dest_p;
	if (cur_pos_id == 0) {
		start_p = res->start;
		Eigen::Vector2i coord = res->route_coords[cur_pos_id];
		dest_p = res->region.getCoord(coord.x(), coord.y());
	} else if (cur_pos_id == res->route_coords.size()) {
		Eigen::Vector2i coord = res->route_coords[cur_pos_id-1];
		start_p = res->region.getCoord(coord.x(), coord.y());
		dest_p = res->start;
	} else {
		Eigen::Vector2i coord1 = res->route_coords[cur_pos_id-1];
		Eigen::Vector2i coord2 = res->route_coords[cur_pos_id];
		start_p = res->region.getCoord(coord1.x(), coord1.y());
		dest_p = res->region.getCoord(coord2.x(), coord2.y());
	}
	start_p(2) = tc->bodyItemRobot()->body()->rootLink()->p()(2);
	dest_p(2) = tc->bodyItemRobot()->body()->rootLink()->p()(2);
	cnoid::Vector3 diff = dest_p - start_p;
	
	cnoid::PoseSeqItemPtr poseSeqItemRobot = new cnoid::PoseSeqItem();
	std::string name = "GraspPoseSeqItem"  + boost::lexical_cast<std::string>(cur_pos_id);
	poseSeqItemRobot->setName(name);
	tc->bodyItemRobot()->addSubItem(poseSeqItemRobot);

	cnoid::PosePtr poseRobotStart = new cnoid::Pose(tc->bodyItemRobot()->body()->numJoints());
	poseRobotStart->setBaseLink(0, start_p, tc->body()->rootLink()->R());
	for(int i=0;i<tc->bodyItemRobot()->body()->numJoints();i++){
    poseRobotStart->setJointPosition(tc->bodyItemRobot()->body()->joint(i)->jointId(), tc->bodyItemRobot()->body()->joint(i)->q());
	}
	poseSeqItemRobot->poseSeq()->insert(poseSeqItemRobot->poseSeq()->end(), 0 , poseRobotStart);
	
	cnoid::PosePtr poseRobotEnd = new cnoid::Pose(tc->bodyItemRobot()->body()->numJoints());
	poseRobotEnd->setBaseLink(0, dest_p, tc->body()->rootLink()->R());
	for(int i=0;i<tc->bodyItemRobot()->body()->numJoints();i++){
    poseRobotEnd->setJointPosition(tc->bodyItemRobot()->body()->joint(i)->jointId(), tc->bodyItemRobot()->body()->joint(i)->q());
	}
	poseSeqItemRobot->poseSeq()->insert(poseSeqItemRobot->poseSeq()->end(), 1.0 , poseRobotEnd);

	poseSeqItemRobot->updateInterpolation();
	poseSeqItemRobot->updateTrajectory();
	cnoid::ItemTreeView::mainInstance()->selectItem(poseSeqItemRobot->bodyMotionItem());

	if (collection_box != NULL) {
		cnoid::PoseSeqItemPtr poseSeqItemBox = new cnoid::PoseSeqItem();
		std::string name = "GraspPoseSeqItem"  + boost::lexical_cast<std::string>(cur_pos_id);
		poseSeqItemBox->setName(name);
		collection_box->addSubItem(poseSeqItemBox);

		cnoid::PosePtr poseBoxStart = new cnoid::Pose(1);
		poseBoxStart->setBaseLink(0, collection_box->body()->rootLink()->p(), collection_box->body()->rootLink()->R());
		
		poseSeqItemBox->poseSeq()->insert(poseSeqItemBox->poseSeq()->end(), 0 , poseBoxStart);
	
		cnoid::PosePtr poseBoxEnd = new cnoid::Pose(1);
		poseBoxEnd->setBaseLink(0, collection_box->body()->rootLink()->p() + diff, collection_box->body()->rootLink()->R());
		
		poseSeqItemBox->poseSeq()->insert(poseSeqItemBox->poseSeq()->end(), 1.0 , poseBoxEnd);

		poseSeqItemBox->updateInterpolation();
		poseSeqItemBox->updateTrajectory();
		cnoid::ItemTreeView::mainInstance()->selectItem(poseSeqItemBox->bodyMotionItem());
	}

	for (size_t i = 0; i < collected_objs.size(); i++) {
		cnoid::PoseSeqItemPtr poseSeqItemObj = new cnoid::PoseSeqItem();
		std::string name = "GraspPoseSeqItem"  + boost::lexical_cast<std::string>(cur_pos_id);
		poseSeqItemObj->setName(name);
		collected_objs[i]->addSubItem(poseSeqItemObj);

		cnoid::PosePtr poseObjStart = new cnoid::Pose(1);
		poseObjStart->setBaseLink(0, collected_objs[i]->body()->rootLink()->p(), collected_objs[i]->body()->rootLink()->R());
		
		poseSeqItemObj->poseSeq()->insert(poseSeqItemObj->poseSeq()->end(), 0 , poseObjStart);
	
		cnoid::PosePtr poseObjEnd = new cnoid::Pose(1);
		poseObjEnd->setBaseLink(0, collected_objs[i]->body()->rootLink()->p() + diff, collected_objs[i]->body()->rootLink()->R());
		
		poseSeqItemObj->poseSeq()->insert(poseSeqItemObj->poseSeq()->end(), 1.0 , poseObjEnd);

		poseSeqItemObj->updateInterpolation();
		poseSeqItemObj->updateTrajectory();
		cnoid::ItemTreeView::mainInstance()->selectItem(poseSeqItemObj->bodyMotionItem());
	}

	cur_pos_id++;
}

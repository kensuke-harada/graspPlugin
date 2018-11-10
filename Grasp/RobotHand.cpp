#include "RobotHand.h"

#include <boost/filesystem.hpp>

#include "ObjectManager.h"

using namespace grasp;

RobotHand::RobotHand(cnoid::BodyItemPtr bodyitem) {
	makeHand(bodyitem);
	body_item_ = bodyitem;
	type_ = TYPE_HAND;
	bb_safety_size_ = cnoid::Vector3(0.03, 0.03, 0.03);
	calcSafeBoundingBox(bb_safety_size_);
}

ObjectBasePtr RobotHand::clone(ObjectManager* owner) const {
	cnoid::BodyItemPtr new_body = new cnoid::BodyItem(*(bodyItem()));
	RobotHandPtr ret = RobotHandPtr(new RobotHand(new_body));
	cloneProc(ret.get());
	ret->id = id;
	ret->bodyItem_ = ret->bodyItem();
	return ret;
}

RobotHand::~RobotHand() {
	removePrehension();
	for (int i = 0; i < nFing; i++) {
		if (fingers[i] != NULL) delete fingers[i];
	}
	if (fingers != NULL) delete [] fingers;
	if (handJoint != NULL) delete handJoint;
}

void RobotHand::makeHand(cnoid::BodyItemPtr bodyitem) {
	bodyItem_ = bodyitem;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	boost::filesystem::path robotfullpath(bodyitem->lastAccessedFilePath());
#else
	boost::filesystem::path robotfullpath(bodyitem->filePath());
#endif

	std::string bodyItemPath = boost::filesystem::path(robotfullpath.branch_path()).string();

	const cnoid::Mapping& gSettings = *(bodyitem->body()->info()->findMapping("graspPluginHandSetting"));

	palm = bodyitem->body()->link(gSettings["palm"].toString());

	const cnoid::Listing& tips = *gSettings["fingerEnds"].toListing();

	nFing = tips.size();
	fingers = new FingerPtr[nFing];

	for (int i = 0; i < tips.size(); i++) {
		fingers[i] = NULL;
	}

	for (int i = 0; i < tips.size(); i++) {
		if (!fingers[i]) fingers[i] = new Finger(bodyitem->body(), palm, bodyitem->body()->link(tips[i].toString()) );
		fingers[i]->number = i;
	}

	dataFilePath = bodyItemPath + "/data/";
	if (gSettings.find("dataFilePath")->isString()) {
		dataFilePath = bodyItemPath + "/" + gSettings["dataFilePath"].toString() +"/";
	}

	const cnoid::Listing& fingeropenpose_list = *gSettings.findListing("fingerOpenPose");
	if (fingeropenpose_list.isValid()) {
		int j = 0;
		int k = 0;
		for (int i = 0; i < fingeropenpose_list.size(); i++) {
			if (i >= k+fingers[j]->fing_path->numJoints()) {
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPose.push_back(fingeropenpose_list[i].toDouble());
		}
	}

	const cnoid::Listing& fingeropenoffset_list = *gSettings.findListing("fingerOpenPoseOffset");
	if (fingeropenoffset_list.isValid()) {
		int j = 0;
		int k = 0;
		for (int i = 0; i < fingeropenoffset_list.size(); i++) {
			if (i >= k+fingers[j]->fing_path->numJoints()) {
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerOpenPoseOffset.push_back(fingeropenoffset_list[i].toDouble());
		}
	}

	const cnoid::Listing& fingercloseoffset_list = *gSettings.findListing("fingerCloseOffset");
	if (fingercloseoffset_list.isValid()) {
		int j = 0;
		int k = 0;
		for (int i = 0; i < fingercloseoffset_list.size(); i++) {
			if (i >= k+fingers[j]->fing_path->numJoints()) {
				k += fingers[j]->fing_path->numJoints();
				j++;
			}
			fingers[j]->fingerCloseOffset.push_back(fingercloseoffset_list[i].toDouble());
		}
	}

	const cnoid::Listing& fingeroffset_list = *gSettings.findListing("fingerOffset");
	if (fingeroffset_list.isValid()) {
		for (int j = 0; j < nFing; j++)
			fingers[j]->offset = fingeroffset_list[0].toDouble();
	}

	const cnoid::Listing& interlink_list = *gSettings.findListing("interlink");
	if (interlink_list.isValid()) {
		for (int i = 0; i < interlink_list.size(); i++) {
			const cnoid::Listing& ilist = *interlink_list[i].toListing();
			cnoid::Link* master = bodyitem->body()->link(ilist[0].toString());
			double baseratio = ilist[1].toDouble();
			for (int j = 1; j < ilist.size()/2; j++) {
				InterLink temp;
				temp.master = master;
				temp.slave = bodyitem->body()->link(ilist[2*j].toString());
				temp.ratio = ilist[2*j+1].toDouble()/baseratio;
				interLinkList.push_back(temp);
			}
		}
	}

	const cnoid::Listing& selfcontact_list = *gSettings.findListing("selfContactPair");
	if (selfcontact_list.isValid()) {
		if (selfcontact_list[0].isString()) {
			for (int i = 0; i < selfcontact_list.size()/2; i++) {
				contactLinks.insert(make_pair(selfcontact_list[i*2].toString(), selfcontact_list[i*2+1].toString()));
			}
		}
		if (selfcontact_list[0].isListing()) {
			for (int i = 0; i < selfcontact_list.size(); i++) {
				const cnoid::Listing& plist = *selfcontact_list[i].toListing();
				for (int j = 0; j < plist.size(); j++) {
					for (int k = j+1; k < plist.size(); k++) {
						contactLinks.insert(make_pair(plist[j].toString(), plist[k].toString()));
					}
				}
			}
		}
	}

	if (!gSettings.read("mu", mu)) mu = 0.5;
	if (!gSettings.read("fmax", fmax)) fmax = 10;
	if (!gSettings.read("hmax", hmax)) hmax = 0.005;
	if (!gSettings.read("handName", handName)) handName = "";

	prehensionFilePathList.clear();
	const cnoid::Listing& prehension_list = *gSettings.findListing("prehensionList");
	if (prehension_list.isValid() && !prehension_list.empty()) {
		for (int i = 0; i < prehension_list.size() ; i++) {
			std::string path = dataFilePath + prehension_list[i].toString() + ".yaml";
			prehensionFilePathList.push_back(path);
			if (boost::filesystem::exists(boost::filesystem::path(path.c_str()))) {
				PrehensionLoader::load(path, prehensionList);
				for (size_t i = 0; i < prehensionList.size(); i++) {
					bodyitem->addSubItem(prehensionList[i]);
				}
			}
		}
	}

	handJoint = new cnoid::LinkTraverse(palm);
	nHandLink = handJoint->numLinks();
}

void RobotHand::removePrehension() {
	cnoid::ItemList<Prehension> prehensionitemlist;
	prehensionitemlist.extractChildItems(bodyItem_);
	for (size_t i = 0; i < prehensionitemlist.size(); i++) {
		prehensionitemlist[i]->detachFromParentItem();
	}
}

Arm* const RobotHand::arm() const {
	return connected_arm_;
}

void RobotHand::setArm(Arm* arm) {
	connected_arm_ = arm;
}

void RobotHand::initialCollisionSelf() {
	typedef std::multimap<std::string, std::string>::iterator multimapIte;

	self_pairs_.clear();
	for (int i = 0; i < bodyItem()->body()->numLinks(); i++) {
		for (int j = i + 1; j < bodyItem()->body()->numLinks(); j++) {
			bool pass = false;
			std::pair<multimapIte, multimapIte> ppp;
			ppp = contactLinks.equal_range(bodyItem()->body()->link(i)->name());
			for (multimapIte it = ppp.first; it != ppp.second; ++it) {
				if (it->second == bodyItem()->body()->link(j)->name()) pass = true;
			}
			ppp = contactLinks.equal_range(bodyItem()->body()->link(j)->name());
			for (multimapIte it = ppp.first; it != ppp.second; ++it) {
				if (it->second == bodyItem()->body()->link(i)->name()) pass = true;
			}
			if (pass) continue;
#ifdef  CNOID_10_11_12_13
			cnoid::ColdetLinkPairPtr temp = new cnoid::ColdetLinkPair(bodyItem()->body()->link(i), bodyItem()->body()->link(j));
#else
			cnoid::ColdetLinkPairPtr temp = boost::make_shared<cnoid::ColdetLinkPair>(bodyItem()->body(), bodyItem()->body()->link(i), bodyItem()->body(), bodyItem()->body()->link(j));
#endif
			temp->updatePositions();
			int t1, t2;
			double p1[3], p2[3];
			double distance = temp->computeDistance(t1, p1, t2, p2);
			if (distance > 1.0e-04)	self_pairs_.push_back(temp);
#ifdef DEBUG_MODE
			else os <<"collide on initial condition at RobotHand self pair"  << distance << " " << temp->model(0)->name() << " " << temp->model(1)->name()  << endl;
#endif
		}
	}
}

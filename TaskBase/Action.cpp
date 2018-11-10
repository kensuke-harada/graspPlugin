// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "Action.h"

#include <cnoid/BodyMotion>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLWriter>

#include "../../../src/PoseSeqPlugin/PoseSeq.h"
#include <cnoid/EigenArchive>
#include "../../../src/Util/ValueTree.h"
#include "../Grasp/VectorMath.h"
#include "../../../src/Base/TimeBar.h"

#include "GraspPoints.h"

#ifndef WIN32
#include <dirent.h>
#endif

//#define USE_EXACT_TIME

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;


ActionItem::ActionItem()
    : mes(*cnoid::MessageView::mainInstance()),
      os (cnoid::MessageView::mainInstance()->cout())
{
    mainItem=NULL;
    subItem=NULL;
    poseSeq=NULL;
    refPoseSeq=NULL;
    timeOffset = -1;
    t = -1;
    dTime=1.0;
}

void ActionItem::initializeClass(cnoid::ExtensionManager *ext){

    static bool initialized = false;

    if(!initialized){
        cnoid::ItemManager& im = ext->itemManager();
        im.registerClass<ActionItem>(N_("ActionItem"));
#ifdef CNOID_GE_16
		im.addLoaderAndSaver<ActionItem>(_("Action File"), "Action-YAML", "yaml;yml", std::bind(ActionItem::loadFromFile, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), std::bind(ActionItem::saveToFile, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
#else
        im.addLoaderAndSaver<ActionItem>(_("Action File"), "Action-YAML", "yaml;yml", boost::bind(ActionItem::loadFromFile, _1, _2, _3), boost::bind(ActionItem::saveToFile, _1, _2, _3));
#endif
        initialized = true;
    }
}

bool ActionItem::loadFromFile(ActionItem *item, const string &filename, ostream &os)
{
    // 名前の設定
    item->setName(filesystem::basename(filesystem::path(filename)));

    cnoid::YAMLReader reader;
    const cnoid::MappingPtr archive = reader.loadDocument(filename)->toMapping();

    // 情報の読み込み
    std::string mainItemName, subItemName;
    archive->read("mainItem", mainItemName);
    archive->read("subItem", subItemName);
    archive->read("durationTime", item->t);

    // ActionItemの再構成
    // item treeからbodyitemの読み出し
    cnoid::ItemList<cnoid::BodyItem> bodyitemlist;
    bodyitemlist.extractChildItems(cnoid::ItemTreeView::mainInstance()->rootItem());

    // bodyitemの中からmainItem, subItemと同じ名前のものを探索
    for (size_t j = 0; j < bodyitemlist.size(); j++) {
        cnoid::BodyItemPtr bodyitem = bodyitemlist[j];
        if (bodyitem->name() == mainItemName)
        {
            item->mainItem = bodyitem;
        }
        else if (bodyitem->name() == subItemName){
            item->subItem = bodyitem;
        }
        if(item->mainItem != NULL & item->subItem != NULL){
            break;
        }
    }
    if(item->mainItem == NULL){
        os << "no object for MainItem in the ItemTree (in yaml file, MainItem name is <" << mainItemName << ">" << endl;
        return false;
    }
    if(item->subItem == NULL){
        os << "no object for SubItem in the ItemTree (in yaml file, SubItem name is <" << subItemName << ">" << endl;
        return false;
    }

    //action**.yamlからrefPoseSeqを取得
    const cnoid::Listing& refs = *archive->findListing("sequence");
    if(!refs.isValid()){
        return false;
    }
    item->refPoseSeq = new cnoid::PoseSeqItem();
    cnoid::PoseSeq::iterator currentRefPose = item->refPoseSeq->poseSeq()->begin();

    item->poseSeq = new cnoid::PoseSeqItem();
    cnoid::PoseSeq::iterator currentPose = item->poseSeq->poseSeq()->begin();

    //開始時間の取得
    if(item->timeOffset == -1){
        // 設定されていなければChoreonoid上の時間を取得する
        item->timeOffset = cnoid::TimeBar::instance()->time();
    }

    // 開始時間のmainItemの位置・姿勢を取得
    cnoid::PoseSeqItemPtr lastPoseSeq = NULL;
    bool included = false;
    for(cnoid::Item* child = item->mainItem->childItem(); child; child = child->nextItem()){
        cnoid::PoseSeqItem* mainPoseSeq = dynamic_cast<cnoid::PoseSeqItem*>(child);
        if(mainPoseSeq){
				if (mainPoseSeq->poseSeq()->empty()) continue;
            double beginingtime = mainPoseSeq->poseSeq()->beginningTime();
            double endingtime = mainPoseSeq->poseSeq()->endingTime();
            // 開始時間が含まれている場合
            if (item->timeOffset >= beginingtime & item->timeOffset <= endingtime){
                included = true;
                lastPoseSeq = mainPoseSeq;
                break;
            }
            else if (item->timeOffset > endingtime){
                lastPoseSeq = mainPoseSeq;
            }
        }
    }
    Vector3 wPm;
    Matrix3 wRm;


    if (lastPoseSeq == NULL){
        wPm = item->mainItem->body()->link(0)->p();
        wRm = item->mainItem->body()->link(0)->R();
    }
    else{
        if(included){
            cnoid::PoseSeq::iterator itr = lastPoseSeq->poseSeq()->seek(lastPoseSeq->poseSeq()->begin(), item->timeOffset, false);
            cnoid::PoseRef& poseRef = *itr;
            if (item->timeOffset != poseRef.time()){
                --itr;
                poseRef = *itr;
                os << "TODO: item->timeOffset != poseRef.time()" << endl;
                os << "item->timeOffset = " << item->timeOffset << ", poseRef.time() = " << poseRef.time() << endl;
            }
            cnoid::Pose* tmpPose = static_cast<cnoid::Pose*>(poseRef.poseUnit().get());
            wPm = tmpPose->baseLinkInfo()->p;
            wRm = tmpPose->baseLinkInfo()->R;
        }
        else{
            const cnoid::PoseRef& poseRef = lastPoseSeq->poseSeq()->back();
            cnoid::Pose* tmpPose = static_cast<cnoid::Pose*>(poseRef.poseUnit().get());
            wPm = tmpPose->baseLinkInfo()->p;
            wRm = tmpPose->baseLinkInfo()->R;
        }
    }

	ActionSequenceLoader seqloader;
	if (!seqloader.load(refs, item->action_seq_)) {
			return false;
	}
	item->format_flag_[POSE_LISTING] = seqloader.isPoseListingFormat();

	GraspPointsItemPtr gpitem = new GraspPointsItem();
	item->subItem->addChildItem(gpitem);
	gpitem->setName("grasppoints_" + item->name());
	gpitem->setActionItem(item);
	ItemTreeView::instance()->checkItem(gpitem);

	for (ActionSeqIte ite = item->action_seq_.begin(); ite != item->action_seq_.end(); ++ite) {
			const ActionContent& action = (*ite);
			double time = action.time();
			cnoid::PosePtr pose = new cnoid::Pose(1);

			const cnoid::Vector3& p = action.ref_pose().p;
			const cnoid::Matrix3& R = action.ref_pose().R;
			pose->setBaseLink(0,p,R);

			currentRefPose = item->refPoseSeq->poseSeq()->insert(currentRefPose, time, pose);
			ite->refPoseIte() = currentRefPose;

			// 絶対値のsubItemのposeを取得
			Vector3 wPsub;
			Matrix3 wRsub;

//        Vector3 wPm = item->mainItem->body()->link(0)->p();
//        Matrix3 wRm = item->mainItem->body()->link(0)->R();

			wPsub = wPm + (wRm * p);
			Matrix3 mRs = R;
			wRsub = wRm * mRs;

			cnoid::PosePtr wPose = new cnoid::Pose(1);
			wPose->setBaseLink(0,wPsub,wRsub);
			time += item->timeOffset;

			currentPose = item->poseSeq->poseSeq()->insert(currentPose,time,wPose);
			item->ite_map_.insert(std::map<cnoid::PoseUnit*, ActionSeqIte>::value_type((*currentPose).poseUnit().get(), ite));
	}

    item->poseSeq->setName(item->name());

    // 同じ名前のPoseSeqデータがSubItemのChildItemにあればデータを入れ替える
    bool poseSeqExist = false;
    for(cnoid::Item* child = item->subItem->childItem(); child; child = child->nextItem())
    {
        cnoid::PoseSeqItem* iTreePoseSeq = dynamic_cast<cnoid::PoseSeqItem*>(child);
        if (iTreePoseSeq && iTreePoseSeq->name() == item->poseSeq->name())
        {
            // child->detachFromParentItem();
            // itemTreeのposeseqの中身を全て消去
            for (cnoid::PoseSeq::iterator it=iTreePoseSeq->poseSeq()->begin(); it!=iTreePoseSeq->poseSeq()->end(); )
            {
                it = iTreePoseSeq->poseSeq()->erase(it);
            }
            // item->poseSeq()とiTreePoseSeq->poseSeq()の中身を入れ替える
            cnoid::PoseSeq::iterator curr = iTreePoseSeq->poseSeq()->begin();
            for(cnoid::PoseSeq::iterator it=item->poseSeq->poseSeq()->begin(); it!=item->poseSeq->poseSeq()->end(); it++)
            {
                curr = iTreePoseSeq->poseSeq()->insert(curr, it->time(), it->poseUnit());
            }
            item->poseSeq = iTreePoseSeq;
            poseSeqExist = true;
        }
    }
    if (!poseSeqExist){
        item->subItem->addChildItem(item->poseSeq);
    }

	item->connectPoseSeq();

    return true;
}

bool ActionItem::saveToFile(ActionItem *item, const string &filename, ostream &os)
{
    cnoid::YAMLWriter writer(filename);
    writer.setKeyOrderPreservationMode(true);

    cnoid::MappingPtr archive = new cnoid::Mapping();
    archive->setDoubleFormat("%.9g");
    item->saveToFileSub(*archive);

    writer.putComment("Assembly action data format version 0.1\n");
    writer.putNode(archive);
    return true;
}

bool ActionItem::saveToFileSub(cnoid::Mapping &archive)
{
    if(mainItem != NULL & subItem != NULL & poseSeq !=NULL){
        archive.write("mainItem", mainItem->name());
        archive.write("subItem", subItem->name());
        archive.write("durationTime", t);

        cnoid::Listing& refsNode = *archive.createListing("sequence");

		bool do_export_prehension = (refPoseSeq->poseSeq()->size() == action_seq_.size());
		ActionSeqConstIte action_ite = action_seq_.begin();

        for(cnoid::PoseSeq::iterator it = refPoseSeq->poseSeq()->begin(); it != refPoseSeq->poseSeq()->end(); it++){
            cnoid::MappingPtr refNode = refsNode.newMapping();

            const cnoid::PoseRef& poseRef = *it;
            refNode->write("time", poseRef.time());

			if (format_flag_[POSE_LISTING]) {
					cnoid::Pose* tmpPose = static_cast<cnoid::Pose*>(poseRef.poseUnit().get());
					cnoid::Vector3& p = tmpPose->baseLinkInfo()->p;
					cnoid::Matrix3& R = tmpPose->baseLinkInfo()->R;
					cnoid::ListingPtr childNode = refNode->createFlowStyleListing("pose");
					childNode->setDoubleFormat("%.9g");
					for (int i = 0; i < 3; i++) {
							childNode->append(p(i));
					}
					for (int i = 0; i < 3; i++) {
							for (int j = 0; j < 3; j++) {
									childNode->append(R(i, j));
							}
					}
			} else {
					cnoid::MappingPtr childNode = refNode->createMapping("pose");

					cnoid::Pose* tmpPose = static_cast<cnoid::Pose*>(poseRef.poseUnit().get());
					cnoid::write(*childNode, "translation",  tmpPose->baseLinkInfo()->p);
					cnoid::write(*childNode, "rotation", tmpPose->baseLinkInfo()->R );
			}

			if (do_export_prehension) {
					const ActionContent& action = *action_ite;
					const ActionPrehension& prehen = action.subPrehension();
					if (prehen.type != ActionPrehension::NONE) {
							cnoid::MappingPtr m_subobject = refNode->createMapping("subObject");
							cnoid::MappingPtr m_prehen = m_subobject->createMapping("prehension");
							std::string type_str;
							switch (prehen.type) {
							case (ActionPrehension::FREE) :
									type_str = "free"; break;
							case (ActionPrehension::PREHENSION) :
									type_str = "prehension"; break;
							case (ActionPrehension::FIXED) :
									type_str = "fixed"; break;
							case (ActionPrehension::ENVIRONMENT) :
									type_str = "environment"; break;
							default:
									;
							}
							m_prehen->write("type", type_str);

							if (prehen.has_hand_frame) {
									if (format_flag_[POSE_LISTING]) {
											const cnoid::Vector3& p = prehen.hand_frame.p;
											const cnoid::Matrix3& R = prehen.hand_frame.R;
											cnoid::ListingPtr childNode = m_prehen->createFlowStyleListing("handFrame");
											childNode->setDoubleFormat("%.9g");
											for (int i = 0; i < 3; i++) {
													childNode->append(p(i));
											}
											for (int i = 0; i < 3; i++) {
													for (int j = 0; j < 3; j++) {
															childNode->append(R(i, j));
													}
											}
									} else {
											cnoid::MappingPtr childNode = m_prehen->createMapping("handFrame");
											cnoid::write(*childNode, "translation",  prehen.hand_frame.p);
											cnoid::write(*childNode, "rotation", prehen.hand_frame.R);
									}
							}
							if (!prehen.grasp_points.empty()) {
									cnoid::ListingPtr grasppoint_list = m_prehen->createListing("graspPoints");
									for (size_t i = 0; i < prehen.grasp_points.size(); i++) {
											cnoid::ListingPtr grasppoint = new Listing();
											grasppoint->setFlowStyle();
											for (int j = 0; j < 3; j++) {
													grasppoint->append(prehen.grasp_points[i](j));
											}
											grasppoint_list->append(grasppoint);
									}
							}
					}
					action_ite++;
			}
        }
        return true;
    }
    return false;
}

bool ActionItem::store(cnoid::Archive &archive)
{
    if(overwrite(true)){
        archive.writeRelocatablePath("filename", filePath());
        archive.write("format", fileFormat());

        archive.write("startTime", timeOffset);
        archive.write("durationTime", t);
        return true;
    }
    return false;
}

bool ActionItem::restore(const cnoid::Archive &archive)
{
    std::string filename, formatId;
    cnoid::Item* parent = archive.currentParentItem();

    //cnoid::Item* child = parent->childItem();
    //ActionItem* item = dynamic_cast<ActionItem*>(child);
    bool itemCheck = false;
    for(cnoid::Item* wChild = parent->childItem(); wChild; wChild = wChild->nextItem()){
        ActionItem* aItem = dynamic_cast<ActionItem*>(wChild);
        if (aItem){
            if(aItem->name() == this->name()){
                itemCheck = true;
            }
        }
    }
    if(itemCheck == true){
        os << "ActionItem instance with the same name already exits." << endl;
        return false;
    }

    if(archive.readRelocatablePath("filename", filename) && archive.read("format", formatId)){
        archive.read("startTime", timeOffset);
        archive.read("durationTime", t);

        if(load(filename, formatId)){
            return true;
        }
    }
    else{
        os << "cannot open " << filename << " by format " << formatId << endl;
    }
    return false;
}

void ActionItem::doPutProperties(cnoid::PutPropertyFunction& putProperty) {
		putProperty.min(0.0)(("Start time"), getStartTime(),
					boost::bind(&ActionItem::onStartTimeChanged, this, _1), true);
		putProperty(("Duration time"), getDurationTime());
}

void ActionItem::setMainItem(BodyItemPtr body)
{
    mainItem = body;
}

BodyItemPtr ActionItem::getMainItem()
{
    return mainItem;
}

void ActionItem::setSubItem(BodyItemPtr body)
{
    subItem = body;
}

BodyItemPtr ActionItem::getSubItem()
{
    return subItem;
}

void ActionItem::setStartTime(double st)
{
    timeOffset = st;
}

double ActionItem::getStartTime()
{
    return timeOffset;
}

bool ActionItem::setNewActionState()
{
    if(mainItem == NULL){
        os << "Set MainItem" << endl;
        return false;
    }
    if(subItem == NULL){
        os << "Set SubItem" << endl;
        return false;
    }

    // 時間の更新
    if (t < 0){
        t = 0;
    }
    else{
        t += dTime;
    }

    // PoseSeqItemの作成
    if(poseSeq==NULL){
        poseSeq = new PoseSeqItem();
        poseSeq->setName(name());
        subItem->addChildItem(poseSeq);
		connectPoseSeq();
    }
    // ref PoseSeqItemの作成
    if(refPoseSeq==NULL){
        refPoseSeq = new PoseSeqItem();
        refPoseSeq->setName(name());
    }

    // 現状のPoseをPoseSeqに追加
    PosePtr pose = new Pose(1);
    pose->setBaseLink(0, subItem->body()->link(0)->p(), subItem->body()->link(0)->R());
    poseSeq->poseSeq()->insert(poseSeq->poseSeq()->end(), t+timeOffset, pose);

    // // 現状の相対位置姿勢をrefPoseSeqに追加
    // PosePtr refPose = new Pose(1);

    // // calcurate reference pose
    // Vector3 mainPsub;
    // Matrix3 mainRsub;

    // Matrix3 wRm = mainItem->body()->link(0)->R();
    // Matrix3 wRs = subItem->body()->link(0)->R();
    // Matrix3 mRw = trans(wRm);

    // mainPsub = mRw * (subItem->body()->link(0)->p() - mainItem->body()->link(0)->p());
    // mainRsub = mRw * wRs;
    // refPose->setBaseLink(0, mainPsub, mainRsub);

    // // insert reference pose to refPoseSeq
    // refPoseSeq->poseSeq()->insert(refPoseSeq->poseSeq()->end(), t, refPose);

    return true;
}

double ActionItem::getDurationTime()
{
    return t < 0 ? 0 : t;
}

cnoid::PoseSeqItemPtr& ActionItem::absPoseSeqItem() {
		return poseSeq;
}

const cnoid::PoseSeqItemPtr& ActionItem::absPoseSeqItem() const {
		return poseSeq;
}

const cnoid::PoseSeqItemPtr& ActionItem::refPoseSeqItem() const {
		return refPoseSeq;
}

ActionSequence& ActionItem::actionSeq() {
		return action_seq_;
}

const ActionSequence& ActionItem::actionSeq() const {
		return action_seq_;
}

cnoid::SignalProxy<void()> ActionItem::sigRefPoseChanged() {
		return sigRefPoseChanged_;
}

void ActionItem::connectPoseSeq() {
		con_poseseq_ = poseSeq->poseSeq()->
				connectSignalSet(boost::bind(&ActionItem::onPoseInserted, this, _1, _2),
								 boost::bind(&ActionItem::onPoseRemoving, this, _1, _2),
								 boost::bind(&ActionItem::onPoseModified, this, _1)
								 );
}

void ActionItem::disconnectPoseSeq() {
		con_poseseq_.disconnect();
}

void ActionItem::onPoseInserted(cnoid::PoseSeq::iterator it, bool isMoving) {
		// get next iterator of PoseSeq, ActionSequece and refPoseSeq
		cnoid::PoseSeq::iterator next_ite = it;
		next_ite++;
		ActionSeqIte next_action_ite = (next_ite == poseSeq->poseSeq()->end()) ?
				action_seq_.end() : ite_map_[(*next_ite).poseUnit().get()];
		cnoid::PoseSeq::iterator next_ref_ite = (next_action_ite == action_seq_.end()) ?
				refPoseSeq->poseSeq()->end() : next_action_ite->refPoseIte();

		// make new action and refPose
		ActionContent action;

		if (isMoving) {
				action = tmp_action_;
		} else {
				// copy previous action content if it exists,
				// Otherwise copy next action content.
				if (next_action_ite == action_seq_.begin()) {
						if (next_action_ite != action_seq_.end()) {
								action = (*next_action_ite);
						}
				} else {
						ActionSeqIte prev_action_ite = next_action_ite;
						prev_action_ite--;
						action = (*prev_action_ite);
				}
		}

		action.time() = (*it).time() - timeOffset;
		cnoid::Link* main_link = mainItem->body()->link(0);
		action.ref_pose().p = main_link->R().transpose() * ((*it).get<cnoid::Pose>()->baseLinkInfo()->p - main_link->p());
		action.ref_pose().R = main_link->R().transpose() * (*it).get<cnoid::Pose>()->baseLinkInfo()->R;
		cnoid::PosePtr pose = new Pose(1);
		pose->setBaseLink(0, action.ref_pose().p, action.ref_pose().R);

		// insert new action and refpose to sequence
		cnoid::PoseSeq::iterator ref_ite = refPoseSeq->poseSeq()->insert(next_ref_ite, action.time(), pose);
		action.refPoseIte() = ref_ite;
		ActionSeqIte action_ite = action_seq_.insert(next_action_ite, action);
		ite_map_.insert(std::map<cnoid::PoseUnit*, ActionSeqIte>::value_type((*it).poseUnit().get(), action_ite));

		// update duration time
		t = poseSeq->poseSeq()->endingTime() - poseSeq->poseSeq()->beginningTime();

		// emit ref pose change signal
		sigRefPoseChanged_();
}

void ActionItem::onPoseRemoving(cnoid::PoseSeq::iterator it, bool isMoving) {
		// error check
		if (ite_map_.count((*it).poseUnit().get()) == 0) {
				std::cerr << "ERROR: pose_seq and ref_pose_seq are not syncronized!" << std::endl;
				return;
		}

		// get corresponding actioncontent. remove it
		ActionSeqIte ite = ite_map_[(*it).poseUnit().get()];
		if (isMoving) {
				tmp_action_ = (*ite);
		}
		refPoseSeq->poseSeq()->erase(ite->refPoseIte());
		action_seq_.erase(ite);
		ite_map_.erase((*it).poseUnit().get());

		// update duration time
		t = poseSeq->poseSeq()->endingTime() - poseSeq->poseSeq()->beginningTime();

		// emit ref pose change signal
		sigRefPoseChanged_();
}

void ActionItem::onPoseModified(cnoid::PoseSeq::iterator it) {
		if ((*it).get<cnoid::Pose>()->baseLinkInfo() == NULL) {
				// ik check button is toggled
				if ((*it).get<cnoid::Pose>()->ikLinkInfo(0) != NULL) {
						(*it).get<cnoid::Pose>()->setBaseLink(0);
				} else {
						return;
				}
		}
		// error check
		if (ite_map_.count((*it).poseUnit().get()) == 0) {
				std::cerr << "ERROR: pose_seq and ref_pose_seq are not syncronized!" << std::endl;
				return;
		}
		// get corresponding actioncontent. set time and ref pose
		ActionContent& action = *(ite_map_[(*it).poseUnit().get()]);
		action.time() = (*it).time() - timeOffset;
		cnoid::Link* main_link = mainItem->body()->link(0);
		action.ref_pose().p = main_link->R().transpose() * ((*it).get<cnoid::Pose>()->baseLinkInfo()->p - main_link->p());
		action.ref_pose().R = main_link->R().transpose() * (*it).get<cnoid::Pose>()->baseLinkInfo()->R;

		// set corresponding refPose
		action.refPoseIte()->get<cnoid::Pose>()->setBaseLink(0, action.ref_pose().p, action.ref_pose().R);
		if (action.time() != action.refPoseIte()->time()) {
				refPoseSeq->poseSeq()->changeTime(action.refPoseIte(), action.time());
		}

		// update duration time
		t = poseSeq->poseSeq()->endingTime() - poseSeq->poseSeq()->beginningTime();

		// emit ref pose change signal
		sigRefPoseChanged_();
}

void ActionItem::onStartTimeChanged(double time) {
		if (timeOffset == time) return;
		double dtime = time - timeOffset;
		timeOffset = time;
		con_poseseq_.block();
		shiftTimePoseSeq(dtime);
		con_poseseq_.unblock();
		sigRefPoseChanged_();
}

void ActionItem::shiftTimePoseSeq(double dtime) {
		const cnoid::PoseSeqPtr& poseseq = poseSeq->poseSeq();
		if (dtime > 0) {
				cnoid::PoseSeq::iterator it = poseseq->end();
				if (it == poseseq->begin()) return;
				it--;
				for (; it != poseseq->begin(); --it) {
						poseseq->changeTime(it, (*it).time() + dtime);
				}
				poseseq->changeTime(poseseq->begin(), (*(poseseq->begin())).time() + dtime);
		} else {
				for (cnoid::PoseSeq::iterator it = poseseq->begin();
					 it != poseseq->end(); ++it) {
						poseseq->changeTime(it, (*it).time() + dtime);
				}
		}
}

ActionSequenceLoader::ActionSequenceLoader() :
		is_pose_listing_(false) {
}

ActionSequenceLoader::~ActionSequenceLoader() {
}

bool ActionSequenceLoader::load(const std::string& yaml_file, ActionSequence& seq, std::string& sub_item_name, std::string& main_item_name) {
    cnoid::YAMLReader reader;
	try {
			const cnoid::MappingPtr archive = reader.loadDocument(yaml_file)->toMapping();
			archive->read("subItem", sub_item_name);
			archive->read("mainItem", main_item_name);
			const cnoid::Listing& refs = *archive->findListing("sequence");
			if(!refs.isValid()){
					return false;
			}
			return load(refs, seq);
	} catch(...) {
			return false;
	}
}

bool ActionSequenceLoader::load(const cnoid::Listing& seq_arc, ActionSequence& seq) {
		seq.clear();

		for (int i = 0; i < seq_arc.size(); i++) {
				const cnoid::Mapping& content = *seq_arc[i].toMapping();
				seq.push_back(ActionContent());
				ActionContent& action = seq.back();
				action.time() = content["time"].toDouble();
				bool ret = readPose(content["pose"], action.ref_pose());
				if (!ret) return false;
				cnoid::Mapping* subobject = content.findMapping("subObject");
				if (!subobject->isValid()) {
						std::cout << "Could not find subObject in Action YAML" << std::endl;
						continue;
				}
				cnoid::Mapping* prehen = subobject->findMapping("prehension");
				if (!prehen->isValid()) {
						std::cout << "Could not find preheinsion in Action YAML" << std::endl;
						continue;
				}
				readPrehension(*prehen, action.subPrehension());
		}
		return true;
}

bool ActionSequenceLoader::isPoseListingFormat() const {
		return is_pose_listing_;
}

bool ActionSequenceLoader::readPose(const cnoid::ValueNode& node, ActionPose& pose) {
		if (node.isMapping()) {
				const cnoid::Mapping& m_node = *node.toMapping();
				read(m_node, "translation", pose.p);
				read(m_node, "rotation", pose.R);
				is_pose_listing_ = false;
		} else if(node.isListing()) {
				const cnoid::Listing& m_list = *node.toListing();
				Eigen::Matrix<double, 4, 3> pose_mat;
				read(m_list, pose_mat);
				pose.p = pose_mat.row(0).transpose();
				pose.R = pose_mat.bottomRows(3);
				is_pose_listing_ = true;
		} else {
				return false;
		}

		return true;
}

bool ActionSequenceLoader::readGraspPoints(const cnoid::Mapping& m_prehen, ActionPrehension& prehension) const {
		cnoid::Listing* grasp_points = m_prehen.findListing("graspPoints");
		if (!grasp_points->isValid()) {
				return true;
		}
		prehension.grasp_points.resize(grasp_points->size());
		for (int i = 0; i < grasp_points->size(); i++) {
				if (!grasp_points->at(i)->isListing()) return false;
				cnoid::Listing* point = grasp_points->at(i)->toListing();
				read(*point, prehension.grasp_points[i]);
		}
		return true;
}

bool ActionSequenceLoader::readPrehension(const cnoid::Mapping& m_prehen, ActionPrehension& prehension) {
		std::string type;
		type = m_prehen["type"].toString();
		if (type == "prehension") {
				prehension.type = ActionPrehension::PREHENSION;
		} else if (type == "fixed") {
				prehension.type = ActionPrehension::FIXED;
		} else if (type == "environment") {
				prehension.type = ActionPrehension::ENVIRONMENT;
		} else if (type == "free") {
				prehension.type = ActionPrehension::FREE;
		} else {
				prehension.type = ActionPrehension::NONE;
		}
		bool has_hand_frame = readPose(m_prehen["handFrame"], prehension.hand_frame);
		prehension.has_hand_frame = has_hand_frame;
		readGraspPoints(m_prehen, prehension);
		return true;
}

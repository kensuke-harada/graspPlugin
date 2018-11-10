/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _Action_H
#define _Action_H

#include <iostream>
#include <bitset>
#include <map>
#include <boost/bind.hpp>

#include <cnoid/Signal>
#include <cnoid/Archive>
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/RootItem>

#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <../src/PoseSeqPlugin/PoseSeqItem.h>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include "ActionSequence.h"

#include "exportdef.h"
#include "gettext.h"


namespace grasp{

class ActionItem;
typedef cnoid::ref_ptr<ActionItem> ActionItemPtr;
	
class EXCADE_API ActionItem :  public cnoid::Item
{

    public :
        ActionItem();

        static void initializeClass(cnoid::ExtensionManager* ext);
//        static bool load(ActionItem *item,  const std::string& filename);
        static bool loadFromFile(ActionItem* item, const std::string& filename, std::ostream& os);
        static bool saveToFile(ActionItem *item, const std::string &filename, std::ostream &os);

        virtual bool store(cnoid::Archive& archive);
        virtual bool restore(const cnoid::Archive& archive);

				virtual void doPutProperties(cnoid::PutPropertyFunction& putProperty);

        void setMainItem(cnoid::BodyItemPtr body);
        cnoid::BodyItemPtr getMainItem();
        void setSubItem(cnoid::BodyItemPtr body);
        cnoid::BodyItemPtr getSubItem();
        void setStartTime(double st);
        double getStartTime();
        double getDurationTime();

        bool setNewActionState();

				cnoid::PoseSeqItemPtr& absPoseSeqItem();
				const cnoid::PoseSeqItemPtr& absPoseSeqItem() const;
				const cnoid::PoseSeqItemPtr& refPoseSeqItem() const;
				ActionSequence& actionSeq() ;
				const ActionSequence& actionSeq() const;

				enum ActionFileFromatFlag {
					POSE_LISTING,

					NumFormatFlag,
				};

				cnoid::SignalProxy<void()> sigRefPoseChanged();

protected:
        bool saveToFileSub(cnoid::Mapping &archive);
        cnoid::BodyItemPtr mainItem;
        cnoid::BodyItemPtr subItem;

        cnoid::PoseSeqItemPtr poseSeq;
        cnoid::PoseSeqItemPtr refPoseSeq;
//        double startTime, endTime;
        double timeOffset;
        double t;
        double dTime;

				ActionSequence action_seq_;
				ActionContent tmp_action_;
				std::bitset<NumFormatFlag> format_flag_;

				std::map<cnoid::PoseUnit*, ActionSeqIte> ite_map_;

				cnoid::ConnectionSet con_poseseq_;
				void connectPoseSeq();
				void disconnectPoseSeq();

				void onPoseInserted(cnoid::PoseSeq::iterator it, bool isMoving);
				void onPoseRemoving(cnoid::PoseSeq::iterator it, bool isMoving);
				void onPoseModified(cnoid::PoseSeq::iterator it);
				void onStartTimeChanged(double time);

				void shiftTimePoseSeq(double dtime);
private:
        cnoid::MessageView& mes;
        std::ostream& os;

				cnoid::Signal<void()> sigRefPoseChanged_;
};

 class EXCADE_API ActionSequenceLoader {
 public:
	 ActionSequenceLoader();
	 virtual ~ActionSequenceLoader();

	 bool load(const std::string& yaml_file, ActionSequence& seq, std::string& sub_item_name, std::string& main_item_name);
	 bool load(const cnoid::Listing& seq_arc, ActionSequence& seq);
	 bool isPoseListingFormat() const;

 private:
	 bool readPose(const cnoid::ValueNode& node, ActionPose& pose);
	 bool readGraspPoints(const cnoid::Mapping& m_prehen, ActionPrehension& prehension) const;
	 bool readPrehension(const cnoid::Mapping& m_prehen, ActionPrehension& prehension);

	 bool is_pose_listing_;
 };
}
#endif

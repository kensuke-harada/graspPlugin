#ifndef _TASKBASE_GRASPPOINTS_H_
#define _TASKBASE_GRASPPOINTS_H_

#include <iostream>
#include <vector>

#include <boost/bind.hpp>

#include <cnoid/EigenTypes>
#include <cnoid/BodyItem>
#include <cnoid/Item>
#include <cnoid/ItemManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneShape>
#include <cnoid/GLSceneRenderer>
#include <cnoid/TimeBar>
#include <cnoid/SceneView>
#include <cnoid/SceneRenderer>

#include "Action.h"

namespace grasp {
	class GraspPointsItem;
	typedef cnoid::ref_ptr<GraspPointsItem> GraspPointsItemPtr;

#ifdef CNOID_GE_16
	class SgGraspPoint :
		public cnoid::SgGroup {
	public:
	SgGraspPoint() :
		cnoid::SgGroup(findPolymorphicId<SgGraspPoint>()) {
			isTurnedOn_ = false;
		}

		void turnOn() {isTurnedOn_ = true;}
		void turnOff() {isTurnedOn_ = false;}
		void setTurnedOn(bool on) {isTurnedOn_ = on;}
		bool isTurnedOn() const {return isTurnedOn_;}

	private:
		bool isTurnedOn_;
	};
	typedef cnoid::ref_ptr<SgGraspPoint> SgGraspPointPtr;
#else
	typedef cnoid::SgSwitch SgGraspPoint;
	typedef cnoid::SgSwitchPtr SgGraspPointPtr;
#endif
	
	class GraspPointsItem :
		public cnoid::Item {
	public:
		GraspPointsItem();
		~GraspPointsItem();
		static void initializeClass(cnoid::ExtensionManager* ext);
		virtual bool store(cnoid::Archive& archive){;}
		virtual bool restore(const cnoid::Archive& archive){;}

		void setActionItem(const ActionItemPtr& action_item);

		cnoid::BodyItemPtr getTargetBodyItem() const;

	protected:
		ActionItemPtr action_item_;
		std::vector<SgGraspPointPtr> grasp_point_switches_;
		std::vector<cnoid::SgPosTransformPtr> grasp_point_transforms_;
		ActionSeqConstIte cur_ite_;
		int n_points_;
		bool is_show_;
		bool is_activate_;

		cnoid::SgNode* orig_shape_;

		cnoid::Connection connection_time_changed_;
		cnoid::Connection connection_check_toggled_;
		cnoid::Connection connection_ref_changed_;
		cnoid::Connection connection_actionitem_disconnected_;

		double point_size_;
		cnoid::Vector3 point_color_;

		void activate();

		void onCheckToggled(bool check);
		bool onTimeChanged(double time);
		void onDetachFromRoot();
		void onRefPoseChanged();
		void onActionItemDisconnected();

		bool seek(double time);
		void updatePoints();
		void showPoints();
		void hidePoints();

		void addGraspPointsNode();
		void addSphere();
	};
}

#endif /* _TASKBASE_GRASPPOINTS_H_ */

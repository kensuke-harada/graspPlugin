#ifndef _GRASPADAATAGENERATOR_WAISTPOSSEARCHMOTION_H_
#define _GRASPADAATAGENERATOR_WAISTPOSSEARCHMOTION_H_

#include <vector>
#include <map>

#include "../Grasp/PlanBase.h"

#include "BoxInfo.h"
#include "WaistPositionSearcher.h"


namespace grasp {
	class WaistPosSearchMotion {
	public:
		WaistPosSearchMotion();
	static WaistPosSearchMotion* instance(WaistPosSearchMotion *wpsm=NULL);

		void init();
		void addMotion();
		void setCollectionBox(cnoid::BodyItemPtr box) {collection_box = box;}
		void setCollectedObj(cnoid::BodyItemPtr obj) {collected_objs.push_back(obj);}
		void clearCollection() {collection_box = NULL; collected_objs.clear();}
	private:
		int cur_pos_id;
		std::map<BoxInfoPtr, bool> done_grasp;
		WaistPosSearchResult* res;
		PlanBase* tc;
		std::ostream& os;
		cnoid::BodyItemPtr collection_box;
		std::vector<cnoid::BodyItemPtr> collected_objs;

		void addMoveMotion();
	};
}

#endif

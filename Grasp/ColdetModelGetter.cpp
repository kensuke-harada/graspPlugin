#include "ColdetModelGetter.h"

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
namespace grasp {
	namespace ColdetModelGetter {
		const cnoid::ColdetModelPtr& get(cnoid::Link* link) {
			return link->coldetModel();
		}
	}
}
#else

#include "ColdetConverter.h"

namespace {
	typedef std::pair<cnoid::Link*, cnoid::ColdetModelPtr> LinkModelPair;
	typedef std::list<LinkModelPair> LinkModelList;
	static LinkModelList link_model_list = LinkModelList();

	void updateProc() {
		cnoid::ItemList<cnoid::BodyItem> bodyitem_list;
		bodyitem_list.extractChildItems(cnoid::ItemTreeView::instance()->rootItem());

		LinkModelList::iterator it;
		for (it = link_model_list.begin(); it != link_model_list.end();) {
			bool has_link = false;;
			for (size_t i = 0; i < bodyitem_list.size(); i++) {
				for (size_t j = 0; j < bodyitem_list[i]->body()->numLinks(); j++) {
					if ((*it).first == bodyitem_list[i]->body()->link(j)) {
						has_link = true;
						break;
					}
				}
				if (has_link) break;
			}
			if (!has_link) {
				it = link_model_list.erase(it);
			} else {
				++it;
			}
		}
	}

	void update(cnoid::Item* item) {
		if (dynamic_cast<cnoid::BodyItem*>(item) == NULL) return;
		updateProc();
	}

	void init() {
		cnoid::ItemTreeView::instance()->rootItem()->sigSubTreeRemoved().connect(boost::bind(&update, _1));
	}
}

namespace grasp {
	namespace ColdetModelGetter {
		const cnoid::ColdetModelPtr& get(cnoid::Link* link) {
			static bool initialized = false;
			if (!initialized) {
				init();
				initialized = true;
			}

			LinkModelList::iterator it;
			for (it = link_model_list.begin(); it != link_model_list.end(); ++it) {
				if ((*it).first == link) {
					return (*it).second;
				}
			}

			cnoid::ColdetModelPtr model = ColdetConverter::ConvertFrom(link->collisionShape());
			link_model_list.push_front(LinkModelPair(link, model));
			return link_model_list.front().second;
		}

		void update() {
			updateProc();
		}
	}
}
#endif

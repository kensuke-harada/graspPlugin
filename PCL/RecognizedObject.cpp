#include "RecognizedObject.h"

void RecognizedObjectManager::getOrCreateRecognizedObjectBodyItems(int n, const cnoid::BodyItemPtr& target,
																																	 std::vector<cnoid::BodyItemPtr>& items) {
	items.clear();

	int count = 0;

	std::string name = "RecognizedObjects";
	cnoid::FolderItem* ro_item = cnoid::RootItem::instance()->findItem<cnoid::FolderItem>(name);
	if (ro_item == NULL) {
		ro_item = new cnoid::FolderItem();
		ro_item->setName(name);
		cnoid::RootItem::instance()->addChildItem(ro_item);
	}

	cnoid::ItemList<cnoid::BodyItem> bodyitems;
	bodyitems.extractChildItems(ro_item);
	for (size_t i = 0; i < bodyitems.size(); i++) {
		if (target->body()->modelName() != bodyitems[i]->body()->modelName()) {
			cnoid::ItemTreeView::instance()->checkItem(bodyitems[i], false);
			continue;
		}
		if (count < n) {
			count++;
			bodyitems[i]->setName(target->name());
			items.push_back(bodyitems[i]);
			cnoid::ItemTreeView::instance()->checkItem(bodyitems[i], true);
		} else {
			bodyitems[i]->setName(target->name() + "invalid");
			cnoid::ItemTreeView::instance()->checkItem(bodyitems[i], false);
		}
	}

	for (int i = count; i < n; i++) {
		cnoid::BodyItemPtr bodyitem = cnoid::BodyItemPtr(dynamic_cast<cnoid::BodyItem*>(target->duplicateAll()));
		ro_item->addChildItem(bodyitem);
		cnoid::ItemTreeView::instance()->checkItem(bodyitem, true);
		items.push_back(bodyitem);
	}
}

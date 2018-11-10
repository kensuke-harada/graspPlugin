#ifndef _PCL_POINTSETITEMDRAWER_H_
#define _PCL_POINTSETITEMDRAWER_H_

#include <string>
#include <vector>

#include <cnoid/EigenTypes>
#include <cnoid/Item>
#include <cnoid/ItemTreeView>
#include <cnoid/PointSetItem>
#include <cnoid/RootItem>

#include "PointCloudTypes.h"

#include "exportdef.h"

class EXCADE_API PointSetItemDrawer {
 public:
	static PointSetItemDrawer* instance();
	virtual ~PointSetItemDrawer();

	cnoid::PointSetItemPtr addPointSet(const PointCloudTConstPtr& cloud, const std::string& item_name = "",
																		 const cnoid::Vector3& color = cnoid::Vector3(0, 0, 0), bool is_show = true,
																		 cnoid::Item* parent_item = NULL);
	cnoid::PointSetItemPtr addPointSet(const ColorPointCloudConstPtr& cloud, const std::string& item_name = "",
																		 bool is_show = true, cnoid::Item* parent_item = NULL);
	void clear();

	static void cloud2PointItem(const PointCloudTConstPtr& cloud, cnoid::PointSetItemPtr& pointsetitem,
															const cnoid::Vector3& color = cnoid::Vector3(0, 0, 0));
	static void cloud2PointItem(const ColorPointCloudConstPtr& cloud, cnoid::PointSetItemPtr& pointsetitem);
	static void pointItem2Cloud(const cnoid::PointSetItemPtr& pointsetitem, PointCloudTPtr& cloud,
															bool transformed = false);
	static void pointItem2Cloud(const cnoid::PointSetItemPtr& pointsetitem, ColorPointCloudPtr& cloud,
															bool transformed = false);

 private:
	PointSetItemDrawer();

	std::vector<cnoid::PointSetItemPtr> items_;
};

#endif /* _PCL_POINTSETITEMDRAWER_H_ */

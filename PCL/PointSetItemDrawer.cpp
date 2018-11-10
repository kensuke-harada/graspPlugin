#include "PointSetItemDrawer.h"

PointSetItemDrawer::PointSetItemDrawer() {
}

PointSetItemDrawer::~PointSetItemDrawer() {
}

PointSetItemDrawer* PointSetItemDrawer::instance() {
	static PointSetItemDrawer* instance = new PointSetItemDrawer();
	return instance;
}

cnoid::PointSetItemPtr PointSetItemDrawer::addPointSet(const PointCloudTConstPtr& cloud, const std::string& item_name,
																											 const cnoid::Vector3& color, bool is_show,
																											 cnoid::Item* parent_item) {
	// create point set item
	cnoid::PointSetItemPtr ps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	ps_item->setName(item_name);
	items_.push_back(ps_item);

	// convert point cloud to point set item
	cloud2PointItem(cloud, ps_item, color);

	// add point set item to parent item
	if (parent_item == NULL) {
		cnoid::Item* ro_item = cnoid::RootItem::instance();
		ro_item->addChildItem(ps_item);
	} else {
		parent_item->addChildItem(ps_item);
	}

	if (is_show) {
		// check point set item
		cnoid::ItemTreeView::instance()->checkItem(ps_item);
		ps_item->notifyUpdate();
	}

	return ps_item;
}

cnoid::PointSetItemPtr PointSetItemDrawer::addPointSet(const ColorPointCloudConstPtr& cloud, const std::string& item_name,
																											 bool is_show, cnoid::Item* parent_item) {
		// create point set item
	cnoid::PointSetItemPtr ps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	ps_item->setName(item_name);
	items_.push_back(ps_item);

	// convert point cloud to point set item
	cloud2PointItem(cloud, ps_item);

	// add point set item to parent item
	if (parent_item == NULL) {
		cnoid::Item* ro_item = cnoid::RootItem::instance();
		ro_item->addChildItem(ps_item);
	} else {
		parent_item->addChildItem(ps_item);
	}

	if (is_show) {
		// check point set item
		cnoid::ItemTreeView::instance()->checkItem(ps_item);
		ps_item->notifyUpdate();
	}

	return ps_item;
}

void PointSetItemDrawer::clear() {
	for (size_t i = 0; i < items_.size(); i++) {
		items_[i]->detachFromParentItem();
	}
	items_.clear();
}

void PointSetItemDrawer::cloud2PointItem(const PointCloudTConstPtr& cloud, cnoid::PointSetItemPtr& pointsetitem,
																				 const cnoid::Vector3& color) {
	int num_points = cloud->points.size();

	cnoid::SgVertexArrayPtr vertices = new cnoid::SgVertexArray();
	vertices->reserve(num_points);

	cnoid::SgColorArrayPtr colors = new cnoid::SgColorArray();
	colors->reserve(num_points);

	cnoid::Vector3f colorf = color.cast<float>();

	for (int i = 0; i < num_points; i++) {
		vertices->push_back(cnoid::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
		colors->push_back(colorf);
	}
	pointsetitem->pointSet()->setVertices(vertices);
	pointsetitem->pointSet()->setColors(colors);
}

void PointSetItemDrawer::cloud2PointItem(const ColorPointCloudConstPtr& cloud, cnoid::PointSetItemPtr& pointsetitem) {
	int num_points = cloud->points.size();

	cnoid::SgVertexArrayPtr vertices = new cnoid::SgVertexArray();
	vertices->reserve(num_points);

	cnoid::SgColorArrayPtr colors = new cnoid::SgColorArray();
	colors->reserve(num_points);

	const double coeff = 1.0 / 255;
	for (int i = 0; i < num_points; i++) {
		vertices->push_back(cnoid::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
		colors->push_back(coeff * cnoid::Vector3f(cloud->points[i].r, cloud->points[i].g, cloud->points[i].b));
	}
	pointsetitem->pointSet()->setVertices(vertices);
	pointsetitem->pointSet()->setColors(colors);
}

void PointSetItemDrawer::pointItem2Cloud(const cnoid::PointSetItemPtr& pointsetitem, PointCloudTPtr& cloud,
																				 bool transformed) {
	cloud->clear();
	cnoid::SgPointSet* point_set = (transformed) ? pointsetitem->getTransformedPointSet() : pointsetitem->pointSet();
	if (!point_set->hasVertices()) return;
	cnoid::SgVertexArray* vertices = point_set->vertices();
	int num_point = vertices->size();
	cloud->resize(num_point);
	for (int i = 0; i < num_point; i++) {
		cloud->points[i].x = vertices->at(i).x();
		cloud->points[i].y = vertices->at(i).y();
		cloud->points[i].z = vertices->at(i).z();
	}
}

void PointSetItemDrawer::pointItem2Cloud(const cnoid::PointSetItemPtr& pointsetitem, ColorPointCloudPtr& cloud,
																				 bool transformed) {
	cloud->clear();
	cnoid::SgPointSet* point_set = (transformed) ? pointsetitem->getTransformedPointSet() : pointsetitem->pointSet();
	if (!point_set->hasVertices()) return;
	if (!point_set->hasColors()) return;
	cnoid::SgVertexArray* vertices = point_set->vertices();
	cnoid::SgColorArray* colors = point_set->colors();
	int num_point = vertices->size();
	cloud->resize(num_point);
	for (int i = 0; i < num_point; i++) {
		cloud->points[i].x = vertices->at(i).x();
		cloud->points[i].y = vertices->at(i).y();
		cloud->points[i].z = vertices->at(i).z();
		cloud->points[i].r = 255 * colors->at(i)(0);
		cloud->points[i].g = 255 * colors->at(i)(1);
		cloud->points[i].b = 255 * colors->at(i)(2);
	}
}

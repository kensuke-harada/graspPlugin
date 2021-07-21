/**
 * @file   PickingTaskResultDrawer.cpp
 * @author Akira Ohchi
 */

#include "PickingTaskResultDrawer.h"

#include <cnoid/MessageView>
#include <cnoid/RootItem>
#include <cnoid/ItemTreeView>

#include "../PCL/ObjectPoseEstimatorInterface.h"
#include "../ViewPlanner/Box.h"
#include "../ViewPlanner/OccupancyGridMap.h"
#include "../ViewPlanner/OccupancyGridDrawer.h"
#include "PickingTaskPlanner.h"

using namespace grasp;

PickingTaskResultDrawer::PickingTaskResultDrawer() {
}

PickingTaskResultDrawer::~PickingTaskResultDrawer() {
}

void PickingTaskResultDrawer::showPrevGrid() {
	BoxInfo* box = BoxHolder::instance()->getTargetBox();

	if (box == NULL) {
		cnoid::MessageView::instance()->cout() << "Please set box" << std::endl;
		return;
	}

	std::vector<cnoid::Vector3> prev_cloud;
	PoseEstimator::getPrevSampledCloud(prev_cloud);


	BoxOccupancyGrid grid(box);
	BoxOccupancyGridHandler ogh(&grid);
	for (size_t i = 0; i < prev_cloud.size(); i++) {
		cnoid::Vector3 p = box->R().transpose() * (prev_cloud[i] - box->center());
		if (ogh.isInsideBox(p)) {
			GridCoord coord = ogh.getCoord(p);
			grid.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::OCCUPIED;
		}
	}
	BoxOccupancyGridDrawer draw;
	draw.clear();
	draw.draw(&grid, box);
}

void PickingTaskResultDrawer::showCurrGrid() {
	BoxInfo* box = BoxHolder::instance()->getTargetBox();
	if (box == NULL) {
		cnoid::MessageView::instance()->cout() << "Please set box" << std::endl;
		return;
	}

	std::vector<cnoid::Vector3> curr_cloud;
	PoseEstimator::getCurrSampledCloud(curr_cloud);


	BoxOccupancyGrid grid(box);
	BoxOccupancyGridHandler ogh(&grid);
	for (size_t i = 0; i < curr_cloud.size(); i++) {
		cnoid::Vector3 p = box->R().transpose() * (curr_cloud[i] - box->center());
		if (ogh.isInsideBox(p)) {
			GridCoord coord = ogh.getCoord(p);
			grid.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::OCCUPIED;
		}
	}

	BoxOccupancyGrid grid_prev(box);
	BoxOccupancyGridHandler ogh_prev(&grid_prev);
	std::vector<cnoid::Vector3> prev_cloud;
	PoseEstimator::getPrevClusterCloud(prev_cloud);
	for (size_t i = 0; i < prev_cloud.size(); i++) {
		cnoid::Vector3 p = box->R().transpose() * (prev_cloud[i] - box->center());
		if (ogh_prev.isInsideBox(p)) {
			GridCoord coord = ogh_prev.getCoord(p);
			grid_prev.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::OCCUPIED;
		}
	}

	BoxOccupancyGridDrawer draw;
	draw.clear();
	draw.drawDiff(&grid_prev, &grid, box);
}

void PickingTaskResultDrawer::showMergePoints() {
	BoxInfo* box = BoxHolder::instance()->getTargetBox();
	if (box == NULL) {
		cnoid::MessageView::instance()->cout() << "Please set box" << std::endl;
		return;
	}

	BoxOccupancyGrid grid_prev(box);
	BoxOccupancyGridHandler ogh_prev(&grid_prev);
	std::vector<cnoid::Vector3> prev_cloud;
	PoseEstimator::getPrevClusterCloud(prev_cloud);
	for (size_t i = 0; i < prev_cloud.size(); i++) {
		cnoid::Vector3 p = box->R().transpose() * (prev_cloud[i] - box->center());
		if (ogh_prev.isInsideBox(p)) {
			GridCoord coord = ogh_prev.getCoord(p);
			grid_prev.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::OCCUPIED;
		}
	}

	std::vector<cnoid::Vector3> curr_cloud;
	PoseEstimator::getCurrSampledCloud(curr_cloud);

	std::vector<std::vector<cnoid::Vector3> > points(2);

	for (size_t i = 0; i < curr_cloud.size(); i++) {
		cnoid::Vector3 p = box->R().transpose() * (curr_cloud[i] - box->center());
		if (ogh_prev.isInsideBox(p)) {
			GridCoord coord = ogh_prev.getCoord(p);
			if (grid_prev.data[coord.x()][coord.y()][coord.z()] == BoxOccupancyGrid::OCCUPIED) {
				points[0].push_back(curr_cloud[i]);
			} else {
				points[1].push_back(curr_cloud[i]);
			}
		}
	}

	std::vector<cnoid::Vector3> color;
	color.push_back(cnoid::Vector3(0, 0.8, 0));
	color.push_back(cnoid::Vector3(1, 0, 0));

	PoseEstimator::drawPointClouds(points, color);

	// hide objects located outside the box
	cnoid::	ItemPtr ro_item = cnoid::RootItem::mainInstance()->findItem<cnoid::Item>("RecognizedObjects");
	if (ro_item == NULL) {
		return;
	}
	cnoid::Item* target_item = ro_item->childItem();
	while (target_item != NULL) {
		if (cnoid::ItemTreeView::mainInstance()->isItemChecked(target_item)) {
			cnoid::BodyItem* bodyitem = dynamic_cast<cnoid::BodyItem*>(target_item);
			if (bodyitem != NULL) {
				if (!PickingTaskPlanner::isSolInsideBox(bodyitem->body()->rootLink()->p(), bodyitem->body()->rootLink()->R())) {
					cnoid::ItemTreeView::mainInstance()->checkItem(target_item, false);
				}
			}
		}
		target_item = target_item->nextItem();
	}
}

void PickingTaskResultDrawer::clearAll() {
	BoxOccupancyGridDrawer draw;
	draw.clear();
}

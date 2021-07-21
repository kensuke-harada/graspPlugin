/**
 * @file   ResultDataManipulator.h
 * @author Akira Ohchi
 */

#ifndef _PICKINGTASKPLANNER_RESULTDATAMANIPULATOR_H_
#define _PICKINGTASKPLANNER_RESULTDATAMANIPULATOR_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <QString>
#include <QStringList>
#include <QMessageBox>
#include <QFile>
#include <QDir>

#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/FolderItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/ItemTreeView>
#ifdef CNOID_GE_15
#include <cnoid/PointSetItem>
#include <cnoid/PointSetUtil>
#include <cnoid/SceneItem>

#include <cnoid/MeshGenerator>
#endif
#include <../src/PoseSeqPlugin/PoseSeqItem.h>

#include "exportdef.h"

class PoseEstimator;

namespace grasp {
	class ResultDataExporter {
	public:
		static ResultDataExporter* instance();
		virtual ~ResultDataExporter();

		void exportStart();
		void exportEnd(bool is_plan_succeed);
		void setViewCount(int count);
		void setInNextPlan();
		void exportViewPlannerPoseSeqItem(cnoid::PoseSeqItem* go, cnoid::PoseSeqItem* back);
		void exportPickingSeqItem(std::vector<cnoid::PoseSeqItem*>& robot_seqs, std::vector<cnoid::PoseSeqItem*>& obj_seqs);
		void exportPointClouds(PoseEstimator* estimator);
		void exportObjectEstimationSols() const;
		void exportBox() const;
		static void exportPoseSeqItem(cnoid::PoseSeqItem* item, const std::string& path);
		static void exportBodyMotionItem(cnoid::BodyMotionItem* motion, const std::string& path);

	private:
		ResultDataExporter();

		int curr_idx_;
		QDir data_dir_;
		QDir curr_dir_;
		QDir next_dir_;
		int view_count_;
		bool in_next_plan_;
	};

	class EXCADE_API ResultDataImporter {
	public:
		ResultDataImporter();
		virtual ~ResultDataImporter();

		bool importLastView(int i);
		bool import(int i, int view_count);
		int getMaxViewCount(int i);

	private:
		QDir data_dir_;

		void importBox(int i, int view_count);
		void importViewPlanTrajectories(int i, int view_count);
		void importPickingTrajectories(int i);
		void importObjectRecogResult(int i, int view_count);
		void importPointClouds(int i, int view_count);
		void makeVoxelGrid(cnoid::PointSetItem* sampled,
											 const std::vector<cnoid::PointSetItem*>& prev_full,
											 const std::vector<cnoid::PointSetItem*>& prev_partial,
											 cnoid::Item* parent_item);
	};
}

#endif /* _PICKINGTASKPLANNER_RESULTDATAMANIPULATOR_H_ */

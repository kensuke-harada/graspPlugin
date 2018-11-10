/**
 * @file   LearningDataManipulator.h
 * @author Akira Ohchi
 */

#ifndef _PICKANDPLACEPLANNER_LEARNINGDATAMANIPULATOR_H_
#define _PICKANDPLACEPLANNER_LEARNINGDATAMANIPULATOR_H_

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <cnoid/Body>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/PointSetItem>

#include "../Grasp/ObjectPoseEstimationSolution.h"

#include "exportdef.h"

class AngleFileManipulator {
 public:
	static void write(const std::string& filepath, const std::vector<double>& q);
	static void read(const std::string& filepath, cnoid::BodyPtr& body);
};

class ObjectPosFileManipulator {
 public:
	static void write(const std::string& filepath, cnoid::Link* object_link);
	static void read(const std::string& filepath, cnoid::Link* object_link);
};

class PointCloudFileManipulator {
 public:
	static void write(const std::string& point_filepath, const std::string& idx_filepath, const grasp::ObjPoseEstimateSol& sol);
	static void read(const std::string& point_filepath, const std::string& idx_filepath, grasp::ObjPoseEstimateSol& sol);
};

class EXCADE_API GraspReconstractor {
 public:
	static GraspReconstractor* instance();

	bool reconstract(int id) const;

	void computeInsidePoints(int id, bool output_points, bool show_sv);

	void showPointCloud();

 private:
	GraspReconstractor();

	bool checkFiles(const std::string& id_str) const;

	void loadObjectPose(const std::string& id_str) const;
	void loadRobotPose(const std::string& id_str) const;
	void loadPointCloud(const std::string& id_str) const;

	void initialSetting();

	const std::string data_dir_;
	const std::string angle_suffix_;
	const std::string obj_suffix_;
	const std::string idx_suffix_;
	const std::string pcd_suffix_;

	bool done_init_;

	cnoid::Matrix3 GRCR_;

	cnoid::PointSetItemPtr point_cloud_;
};

#endif

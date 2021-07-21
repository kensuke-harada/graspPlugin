#include "ResultDataManipulator.h"

#include "../Grasp/PlanBase.h"
#include "../Grasp/ObjectPoseEstimationSolution.h"
#include "../ViewPlanner/Box.h"
#include "../ViewPlanner/OccupancyGridMap.h"
#include "../PCL/ObjectPoseEstimatorInterface.h"
#include "../PCL/RecognizedObject.h"

using namespace grasp;

namespace {
	void getOrCreateDir(const QString& path, QDir& dir) {
		if (!QDir().exists(path)) {
			QDir().mkpath(path);
		}
		dir = path;
	}

	void maxDirIdx(const QDir& target_dir, int& max_idx) {
		QStringList dir_list = target_dir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot, QDir::Name);
		if (dir_list.empty()) {
			max_idx = 0;
		} else {
			max_idx = dir_list.back().toInt();
		}
	}

	cnoid::PointSetItem* loadPointSetItem(const QString& pcd_path, cnoid::Item* parent, const std::string name) {
		if (!QFile::exists(pcd_path)) return NULL;

		cnoid::PointSetItemPtr ps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
		ps_item->setName(name);
		loadPCD(ps_item->pointSet(), pcd_path.toStdString());
		parent->addChildItem(ps_item);
		ps_item->notifyUpdate();
		return ps_item;
	}

	void changePointSetItemColor(cnoid::PointSetItem* point_item, const cnoid::Vector3& color) {
		cnoid::SgPointSet* point_set = point_item->pointSet();
		int n_point = point_set->vertices()->size();
		cnoid::SgColorArray* colors = point_set->getOrCreateColors();
		colors->clear();
		colors->reserve(n_point);

		cnoid::Vector3f colorf = color.cast<float>();

		for (int i = 0;  i < n_point; i++) {
			colors->push_back(colorf);
		}
	}
}

ResultDataExporter::ResultDataExporter() {
	QString data_dir_path = "./data";
	getOrCreateDir(data_dir_path, data_dir_);

	maxDirIdx(data_dir_, curr_idx_);

	view_count_ = 0;
	in_next_plan_ = false;
}

ResultDataExporter::~ResultDataExporter() {
}

ResultDataExporter* ResultDataExporter::instance() {
	static ResultDataExporter* instance = new ResultDataExporter();
	return instance;
}

void ResultDataExporter::exportStart() {
	view_count_ = 0;
	in_next_plan_ = false;
	curr_idx_++;
	QString target_dir_path = data_dir_.path() + "/" + QString("%1").arg(curr_idx_, 3, 10, QChar('0'));
	QString next_dir_path = data_dir_.path() + "/" + QString("%1").arg((curr_idx_ + 1), 3, 10, QChar('0'));
	getOrCreateDir(target_dir_path, curr_dir_);
	getOrCreateDir(next_dir_path, next_dir_);
}

void ResultDataExporter::exportEnd(bool is_plan_succeed) {
	int result_val;
	std::string result_message;

	if (is_plan_succeed) {
		QMessageBox::StandardButton reply;
		reply = QMessageBox::question(NULL, "Picking result", "Is picking successful?",
																	 QMessageBox::Yes|QMessageBox::No);
		if (reply == QMessageBox::Yes) {
			result_val = 1;
			result_message = "success";
		} else {
			result_val = 0;
			result_message = "fail";
		}
	} else {
		result_val = -1;
		result_message = "planning failed";
	}

	std::ofstream ofs;
	ofs.open(std::string(curr_dir_.path().toStdString() + "/result.txt").c_str());
	ofs << result_val << std::endl << result_message;
	ofs.close();
}

void ResultDataExporter::setViewCount(int count) {
	view_count_ = count;
}

void ResultDataExporter::setInNextPlan() {
	in_next_plan_ = true;
}

void ResultDataExporter::exportViewPlannerPoseSeqItem(cnoid::PoseSeqItem* go,
																											cnoid::PoseSeqItem* back) {
	std::string go_path = QString("%1/view_%2_go.pseq").arg(curr_dir_.path()).arg(view_count_).toStdString();
	std::string back_path = QString("%1/view_%2_back.pseq").arg(curr_dir_.path()).arg(view_count_).toStdString();
	exportPoseSeqItem(go, go_path);
	exportPoseSeqItem(back, back_path);
}

void ResultDataExporter::exportPickingSeqItem(std::vector<cnoid::PoseSeqItem*>& robot_seqs, std::vector<cnoid::PoseSeqItem*>& obj_seqs) {
	const QString& target_dir = (in_next_plan_ ? next_dir_.path() : curr_dir_.path());
	for (size_t i = 0; i < robot_seqs.size(); i++) {
		std::string robot_path = QString("%1/picking_%2_robot.pseq").arg(target_dir).arg(i).toStdString();
		std::string obj_path = QString("%1/picking_%2_obj.yaml").arg(target_dir).arg(i).toStdString();
		exportPoseSeqItem(robot_seqs[i], robot_path);
		exportBodyMotionItem(obj_seqs[i]->bodyMotionItem(), obj_path);
	}
}

void ResultDataExporter::exportObjectEstimationSols() const {
	const QString& target_dir = (in_next_plan_ ? next_dir_.path() : curr_dir_.path());
	std::string obj_recog_path = QString("%1/obj_recog_%2.txt").arg(target_dir).arg(view_count_).toStdString();
	grasp::ObjPoseEstimateSolHolder* opes = grasp::ObjPoseEstimateSolHolder::instance();
	std::ofstream ofs;
	ofs.open(obj_recog_path.c_str());
	for (int i = 0; i < opes->size(); i++) {
		const ObjPoseEstimateSol& target = opes->at(i);
		for (int j = 0; j < 3; j++) {
			ofs << target.p(j) << " ";
		}
		for (int m = 0; m < 3; m++) {
			for (int n = 0; n < 3; n++) {
				ofs << target.R(m, n) << " ";
			}
		}
		ofs << target.is_feasible << std::endl;
	}
	ofs.close();
}

void ResultDataExporter::exportPointClouds(PoseEstimator* estimator) {
#ifdef CNOID_GE_15
	const QString& target_dir = (in_next_plan_ ? next_dir_.path() : curr_dir_.path());
	std::string pc_path = QString("%1/point_cloud_%2.pcd").arg(target_dir).arg(view_count_).toStdString();
	cnoid::PointSetItemPtr ps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	estimator->getCurrPointCloudItem(ps_item);
	savePCD(ps_item->pointSet(), pc_path);

	std::string spc_path = QString("%1/sampled_cloud_%2.pcd").arg(target_dir).arg(view_count_).toStdString();
	cnoid::PointSetItemPtr sps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	estimator->getSampledCurrPointCloudItem(sps_item);
	savePCD(sps_item->pointSet(), spc_path);

	std::string cpc_path = QString("%1/clusterd_cloud_%2.pcd").arg(target_dir).arg(view_count_).toStdString();
	cnoid::PointSetItemPtr cps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	estimator->getClusteredCloudItem(cps_item);
	savePCD(cps_item->pointSet(), cpc_path);

	std::vector<cnoid::PointSetItemPtr> cluster_items;
	std::vector<int> flags;
	estimator->getClusterPointCloudItems(cluster_items, flags);
	for (size_t i = 0; i < cluster_items.size(); i++) {
		QString flag_str;
		if (flags[i] == 2) {
			flag_str = "full";
		} else if (flags[i] == 1) {
			flag_str = "partial";
		} else {
			flag_str = "notuse";
		}
		std::string cluster_path = QString("%1/cluster_%2_%3_%4.pcd").arg(target_dir).arg(view_count_).arg(i).arg(flag_str).toStdString();
		// cnoid::RootItem::instance()->addChildItem(cluster_items[i]);
		savePCD(cluster_items[i]->pointSet(), cluster_path);
	}
#endif
}

void ResultDataExporter::exportBox() const {
	const QString& target_dir = (in_next_plan_ ? next_dir_.path() : curr_dir_.path());
	std::string box_path = QString("%1/box_%2.txt").arg(target_dir).arg(view_count_).toStdString();
	const cnoid::BodyPtr body = BoxHolder::instance()->getTargetBox()->body();
	std::ofstream ofs;
	ofs.open(box_path.c_str());
	for (int i = 0; i < 3; i++) {
		ofs << body->rootLink()->p()(i) << " ";
	}
	for (int m = 0; m < 3; m++) {
		for (int n = 0; n < 3; n++) {
			ofs << body->rootLink()->R()(m, n) << " ";
		}
	}
	ofs.close();
}

void ResultDataExporter::exportPoseSeqItem(cnoid::PoseSeqItem* item, const std::string& path) {
	cnoid::PoseSeqPtr poseseq = item->poseSeq();
	cnoid::BodyItem* body_item = dynamic_cast<cnoid::BodyItem*>(item->parentItem());
	if (body_item == NULL) {
		std::cerr << "err" << std::endl;
	}
	poseseq->save(path, body_item->body());
}

void ResultDataExporter::exportBodyMotionItem(cnoid::BodyMotionItem* item, const std::string& path) {
	cnoid::BodyMotionPtr motion = item->motion();
	motion->saveAsStandardYAMLformat(path);
}

ResultDataImporter::ResultDataImporter() {
	QString data_dir_path = "./data";
	getOrCreateDir(data_dir_path, data_dir_);
}

ResultDataImporter::~ResultDataImporter() {
}

bool ResultDataImporter::importLastView(int i) {
	int max_view = getMaxViewCount(i);
	bool ret = import(i, max_view);
	importPickingTrajectories(i);
	return ret;
}

bool ResultDataImporter::import(int i, int view_count) {
	QString target_dir_path = data_dir_.path() + "/" + QString("%1").arg(i, 3, 10, QChar('0'));
	if (!QDir().exists(target_dir_path)) {
		return false;
	}

	importBox(i, view_count);
	importViewPlanTrajectories(i, view_count);
	importPointClouds(i, view_count);
	importObjectRecogResult(i, view_count);

	PlanBase::instance()->flush();
	return true;
}

int ResultDataImporter::getMaxViewCount(int i) {
	for (int j = 1; ; j++) {
		QString target_file_path = data_dir_.path() + "/" + QString("%1/box_%2.txt").arg(i, 3, 10, QChar('0')).arg(j);
		if (!QFile::exists(target_file_path)) {
			return j - 1;
		}
	}
}

void ResultDataImporter::importBox(int i, int view_count) {
	QString box_path = data_dir_.path() + "/" + QString("%1/box_%2.txt").arg(i, 3, 10, QChar('0')).arg(view_count);
	std::ifstream ifs;
	ifs.open(box_path.toStdString().c_str());
	cnoid::Vector3 p;
	for (int j = 0; j < 3; j++) {
		ifs >> p(j);
	}
	cnoid::Matrix3 R;
	for (int m = 0; m < 3; m++) {
		for (int n = 0; n < 3; n++) {
			ifs >> R(m, n);
		}
	}

	cnoid::BodyPtr body = BoxHolder::instance()->getTargetBox()->body();
	body->rootLink()->p() = p;
	body->rootLink()->R() = R;

	ifs.close();
}

void ResultDataImporter::importViewPlanTrajectories(int i, int view_count) {
	QString go_path = data_dir_.path() + "/" + QString("%1/view_%2_go.pseq").arg(i, 3, 10, QChar('0')).arg(view_count);
	QString back_path = data_dir_.path() + "/" + QString("%1/view_%2_back.pseq").arg(i, 3, 10, QChar('0')).arg(view_count);
	if (!QFile::exists(go_path)) {
		return;
	}

	cnoid::BodyItemPtr robot = PlanBase::instance()->bodyItemRobot();

	cnoid::PoseSeqItemPtr go_pose_item = cnoid::PoseSeqItemPtr(new cnoid::PoseSeqItem());
	robot->addChildItem(go_pose_item);
	go_pose_item->poseSeq()->load(go_path.toStdString(), robot->body());
	go_pose_item->setName(QString("view_go_%1_%2").arg(i).arg(view_count).toStdString());

	cnoid::PoseSeqItemPtr back_pose_item = cnoid::PoseSeqItemPtr(new cnoid::PoseSeqItem());
	robot->addChildItem(back_pose_item);
	back_pose_item->poseSeq()->load(back_path.toStdString(), robot->body());
	back_pose_item->setName(QString("view_back_%1_%2").arg(i).arg(view_count).toStdString());
}

void ResultDataImporter::importPickingTrajectories(int i) {
	PlanBase* pb = PlanBase::instance();
	cnoid::BodyItemPtr robot = pb->bodyItemRobot();
	cnoid::BodyItemPtr obj = pb->targetObject->bodyItemObject;

	for (int j = 0; ; j++) {
		QString robot_path = data_dir_.path() + "/" + QString("%1/picking_%2_robot.pseq").arg(i, 3, 10, QChar('0')).arg(j);
		QString obj_path = data_dir_.path() + "/" + QString("%1/picking_%2_obj.yaml").arg(i, 3, 10, QChar('0')).arg(j);
		if (!QFile::exists(robot_path)) {
			break;
		}
		cnoid::PoseSeqItemPtr robot_pose_item = cnoid::PoseSeqItemPtr(new cnoid::PoseSeqItem());
		robot->addChildItem(robot_pose_item);
		robot_pose_item->poseSeq()->load(robot_path.toStdString(), robot->body());
		robot_pose_item->setName(QString("picking_%1_%2").arg(i).arg(j).toStdString());

		cnoid::PoseSeqItemPtr obj_pose_item = cnoid::PoseSeqItemPtr(new cnoid::PoseSeqItem());
		obj->addChildItem(obj_pose_item);
		obj_pose_item->bodyMotionItem()->motion()->loadStandardYAMLformat(obj_path.toStdString());
		obj_pose_item->setName(QString("picking_%1_%2").arg(i).arg(j).toStdString());
	}
}

void ResultDataImporter::importObjectRecogResult(int i, int view_count) {
	QString sol_path = data_dir_.path() + "/" + QString("%1/obj_recog_%2.txt").arg(i, 3, 10, QChar('0')).arg(view_count);

	std::ifstream ifs;
	ifs.open(sol_path.toStdString().c_str());

	std::vector<cnoid::Vector3> p_list;
	std::vector<cnoid::Matrix3> R_list;
	std::vector<bool> feasible_list;

	std::string line;
	while (getline(ifs, line)) {
		if (line.empty()) continue;
		std::stringstream buf(line);

		cnoid::Vector3 p;
		for (int j = 0; j < 3; j++) {
			buf >> p(j);
		}
		p_list.push_back(p);

		cnoid::Matrix3 R;
		for (int m = 0; m < 3; m++) {
			for (int n = 0; n < 3; n++) {
				buf >> R(m, n);
			}
		}
		R_list.push_back(R);

		int feasible;
		buf >> feasible;
		feasible_list.push_back((feasible == 1));
	}

	std::vector<cnoid::BodyItemPtr> items;
	int n = p_list.size();
	cnoid::BodyItemPtr target = grasp::PlanBase::instance()->targetObject->bodyItemObject;
	RecognizedObjectManager::getOrCreateRecognizedObjectBodyItems(n, target, items);

	for (int i = 0; i < n; i++) {
		items[i]->body()->rootLink()->p() = p_list[i];
		items[i]->body()->rootLink()->R() = R_list[i];
		cnoid::ItemTreeView::instance()->checkItem(items[i], feasible_list[i]);
		items[i]->notifyKinematicStateChange();
	}

	ifs.close();
}

void ResultDataImporter::importPointClouds(int i, int view_count) {
	QDir target_dir = data_dir_.path() + "/" + QString("%1").arg(i, 3, 10, QChar('0'));
	cnoid::FolderItemPtr pc_root_item = cnoid::FolderItemPtr(new cnoid::FolderItem());
	pc_root_item->setName(QString("PointCloud_%1_%2").arg(i).arg(view_count).toStdString());
	cnoid::RootItem::instance()->addChildItem(pc_root_item);

	QString pcd_path = target_dir.path() + "/"  + QString("point_cloud_%1.pcd").arg(view_count);
	loadPointSetItem(pcd_path, pc_root_item, "PointCloud");

	QString sampled_pcd_path = target_dir.path() + "/"  + QString("sampled_cloud_%1.pcd").arg(view_count);
	cnoid::PointSetItem* sampled_ps_item = loadPointSetItem(sampled_pcd_path, pc_root_item, "SampledPointCloud");

	QString clustered_pcd_path = target_dir.path() + "/"  + QString("clusterd_cloud_%1.pcd").arg(view_count);
	loadPointSetItem(clustered_pcd_path, pc_root_item, "ClusteredPointCloud");

	std::vector<cnoid::PointSetItem*> prev_full_ps_items;
	std::vector<cnoid::PointSetItem*> prev_partial_ps_items;
	QStringList filter;
	filter << "cluster_*";
	QStringList cluster_files = target_dir.entryList(filter, QDir::Files, QDir::Name);
	for (int i = 0; i < cluster_files.size(); i++) {
		QStringList split_list = cluster_files[i].split('_');
		if (split_list.size() != 4) continue;
		if (split_list[1].toInt() != view_count) continue;

		QString suffix;
		std::vector<cnoid::PointSetItem*>* items_ptr = NULL;
		cnoid::Vector3 color(1.0, 0, 0);
		if (split_list[3] == "notuse.pcd") {
			suffix = "notuse";
			color = cnoid::Vector3(1.0, 0, 0);
		} else if (split_list[3] == "partial.pcd") {
			suffix = "partial";
			color = cnoid::Vector3(0, 1.0, 0);
			items_ptr = &prev_partial_ps_items;
		} else if (split_list[3] == "full.pcd") {
			suffix = "full";
			color = cnoid::Vector3(0, 0, 1.0);
			items_ptr = &prev_full_ps_items;
		} else {
			continue;
		}
		cnoid::PointSetItem* cluster_item = loadPointSetItem(target_dir.path() + "/" + cluster_files[i], pc_root_item, QString("Cluster%1%2").arg(split_list[2]).arg(suffix).toStdString());

		changePointSetItemColor(cluster_item, color);

		if (items_ptr != NULL) {
			items_ptr->push_back(cluster_item);
		}
	}
	makeVoxelGrid(sampled_ps_item, prev_full_ps_items, prev_partial_ps_items, pc_root_item);
}

namespace {
	void addBox(const cnoid::SgGroupPtr& parent, const cnoid::Vector3& p, const cnoid::Matrix3& R,
							const cnoid::Vector3& l, const cnoid::Vector3& c) {
		cnoid::MeshGenerator generator;
		cnoid::SgMeshPtr mesh = generator.generateBox(l);

		cnoid::SgShapePtr shape = new cnoid::SgShape();
		shape->setMesh(mesh);

		cnoid::SgMaterialPtr material = shape->getOrCreateMaterial();
		material->setTransparency(0.4);
		material->setDiffuseColor(c);
		material->setEmissiveColor(c);

		cnoid::SgPosTransform* pos = new cnoid::SgPosTransform();
		pos->setTranslation(p);
		pos->setRotation(R);
		pos->addChild(shape);

		parent->addChild(pos, false);
	}
}

void ResultDataImporter::makeVoxelGrid(cnoid::PointSetItem* sampled,
																			 const std::vector<cnoid::PointSetItem*>& prev_full,
																			 const std::vector<cnoid::PointSetItem*>& prev_partial,
																			 cnoid::Item* parent_item) {
	BoxInfo* box = BoxHolder::instance()->getTargetBox();
	cnoid::Matrix3 box_R = box->R();

	BoxOccupancyGrid grid(box);
	BoxOccupancyGridHandler ogh(&grid);

	cnoid::SgPointSet* point_set = sampled->pointSet();
	cnoid::SgVertexArray* vertices = point_set->vertices();
	int num_point = vertices->size();

	for (int i = 0; i < num_point; i++) {
		cnoid::Vector3 p = box->R().transpose() * (vertices->at(i).cast<double>() - box->center());
		if (ogh.isInsideBox(p)) {
			GridCoord coord = ogh.getCoord(p);
			grid.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::OCCUPIED;
		}
	}

	for (size_t j = 0; j < prev_partial.size(); j++) {
		cnoid::SgPointSet* point_set = prev_partial[j]->pointSet();
		cnoid::SgVertexArray* vertices = point_set->vertices();
		int num_point = vertices->size();

		for (int i = 0; i < num_point; i++) {
			cnoid::Vector3 p = box->R().transpose() * (vertices->at(i).cast<double>() - box->center());
			if (ogh.isInsideBox(p)) {
				GridCoord coord = ogh.getCoord(p);
				grid.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::FREE;
			}
		}
	}

	for (size_t j = 0; j < prev_full.size(); j++) {
		cnoid::SgPointSet* point_set = prev_full[j]->pointSet();
		cnoid::SgVertexArray* vertices = point_set->vertices();
		int num_point = vertices->size();

		for (int i = 0; i < num_point; i++) {
			cnoid::Vector3 p = box->R().transpose() * (vertices->at(i).cast<double>() - box->center());
			if (ogh.isInsideBox(p)) {
				GridCoord coord = ogh.getCoord(p);
				grid.data[coord.x()][coord.y()][coord.z()] = BoxOccupancyGrid::UNKNOWN;
			}
		}
	}

	cnoid::SceneItemPtr voxel_item = cnoid::SceneItemPtr(new cnoid::SceneItem());
	voxel_item->setName("voxelgrid");
	parent_item->addChildItem(voxel_item);

	cnoid::SgGroupPtr node = voxel_item->topNode();

	for (int x = 0; x < grid.size.x(); x++) {
		for (int y = 0; y < grid.size.y(); y++) {
			for (int z = 0; z < grid.size.z(); z++) {
				cnoid::Vector3 color;
				if (grid.data[x][y][z] == BoxOccupancyGrid::FREE) color = cnoid::Vector3(0, 0, 1);
				if (grid.data[x][y][z] == BoxOccupancyGrid::UNKNOWN) color = cnoid::Vector3(0, 1, 0);
				if (grid.data[x][y][z] == BoxOccupancyGrid::OCCUPIED) color = cnoid::Vector3(1, 0, 0);
				// if (grid->data[x][y][z] == BoxOccupancyGrid::UNVISITED) color = cnoid::Vector3(0, 0, 1);
				if (grid.data[x][y][z] == BoxOccupancyGrid::UNVISITED) continue;
				cnoid::Vector3 center = box->center() + box_R * ogh.getCenter(x, y, z);
				cnoid::Vector3 len = grid.resolution;
				addBox(node, center, box_R, len, color);
			}
		}
	}


	cnoid::PointSetItemPtr merge_ps_item = cnoid::PointSetItemPtr(new cnoid::PointSetItem());
	merge_ps_item->setName("merge_point");
	parent_item->addChildItem(merge_ps_item);

	cnoid::SgVertexArrayPtr merge_vertices = new cnoid::SgVertexArray();
	merge_vertices->reserve(num_point);

	cnoid::SgColorArrayPtr colors = new cnoid::SgColorArray();
	colors->reserve(num_point);

	for (int i = 0; i < num_point; i++) {
		cnoid::Vector3 p = box->R().transpose() * (vertices->at(i).cast<double>() - box->center());
		if (ogh.isInsideBox(p)) {
			GridCoord coord = ogh.getCoord(p);
			cnoid::Vector3 color;
			if (grid.data[coord.x()][coord.y()][coord.z()] == BoxOccupancyGrid::FREE) color = cnoid::Vector3(0, 0, 1);
			if (grid.data[coord.x()][coord.y()][coord.z()] == BoxOccupancyGrid::UNKNOWN) color = cnoid::Vector3(0, 1, 0);
			if (grid.data[coord.x()][coord.y()][coord.z()] == BoxOccupancyGrid::OCCUPIED) color = cnoid::Vector3(1, 0, 0);
			merge_vertices->push_back(vertices->at(i));
			colors->push_back(color.cast<float>());
		}
	}

	merge_ps_item->pointSet()->setVertices(merge_vertices);
	merge_ps_item->pointSet()->setColors(colors);
}

#include "PrehensionParamHandler.h"

#include <cnoid/YAMLWriter>
#include <cnoid/ItemTreeView>

#include "../Grasp/PlanBase.h"
#include "../Grasp/DrawUtility.h"

#include "GraspDataGenerator.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

PrehensionParameterExt PrehensionParameterExt::createPrehensionPrameterExt(const PrehensionPtr prehen, int id) {
	PrehensionParameterExt param;
	prehen->convertToPrehensionParameter(&param);

	param.angles.clear();
	for (size_t i = 0; i < param.fingers.size(); i++) {
		param.angles.push_back(vector<double>(param.fingers[i].contact.size(), 0.0));
	}
	param.id = id;
	return param;
}

bool PrehensionParamHandler::loadParams() {
	PlanBase* tc = PlanBase::instance();
	if (tc->targetArmFinger == NULL) return false;

	params.clear();
	for (size_t i = 0; i < tc->targetArmFinger->prehensionList.size(); i++) {
		params.push_back(PrehensionParameterExt::createPrehensionPrameterExt(tc->targetArmFinger->prehensionList[i], i));
	}

	parsePosfiles();

	return true;
}

void PrehensionParamHandler::updateYamlFile() {
	PlanBase* tc = PlanBase::instance();
	if (tc->targetArmFinger == NULL) return;

	std::vector<PrehensionPtr> tmp_prehension_list;

	for (size_t i = 0; i < tc->targetArmFinger->prehensionList.size(); i++) {
		tc->targetArmFinger->prehensionList[i]->detachFromParentItem();
		tmp_prehension_list.push_back(tc->targetArmFinger->prehensionList[i]);
	}

	tc->targetArmFinger->prehensionList.clear();

	for (size_t i = 0; i < params.size(); i++) {
		PrehensionPtr prehen;
		if (params[i].id == -1) {
			prehen = new Prehension;
		} else {
			prehen = tmp_prehension_list[params[i].id];
		}
		prehen->setInfo(params[i]);
		prehen->setNameDefault();
		tc->targetArmFinger->prehensionList.push_back(prehen);
		tc->targetArmFinger->addSubItem(prehen);
	}

	writePosfiles();

	Mapping* root = new Mapping();
	Listing* prehen_list = root->createListing("Prehension");
	for (size_t i = 0; i < tc->targetArmFinger->prehensionList.size(); i++) {
		prehen_list->append(tc->targetArmFinger->prehensionList[i]->info);
		ItemTreeView::mainInstance()->checkItem(tc->targetArmFinger->prehensionList[i], true);
	}

	YAMLWriter writer(tc->targetArmFinger->prehensionFilePathList[0]);
	writer.putNode(root);
}

void PrehensionParamHandler::display(const PrehensionParameterExt& params) {
	setFingerAngle(params);
	displayGRC(params);
}

void PrehensionParamHandler::grasp(const PrehensionParameterExt& params) {
	GraspControllerWithInterface* gc = GraspControllerWithInterface::instance();
	PlanBase* tc = PlanBase::instance();
	gc->tc = tc;
	gc->settingPrehensionParam(params);

	gc->doGrasping(params.angles);
}

void PrehensionParamHandler::setFingerAngle(const PrehensionParameterExt& params) {
	PlanBase* tc = PlanBase::instance();

	for (int i = 0; i < params.fing_size(); i++) {
		for (int j = 0; j < params.joint_size(i); j++) {
			tc->fingers(i)->joint(j)->q() = params.angles[i][j];
		}
	}

	tc->calcForwardKinematics();
	tc->flush();
}

void PrehensionParamHandler::displayGRC(const PrehensionParameterExt& params) {
	PlanBase* tc = PlanBase::instance();
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	Vector3 grc_pos = tc->palm()->p() + tc->palm()->R() * params.GRCmax_pos;
	Matrix3 grc_R = tc->palm()->R() * rotFromRpy(params.GRCmax_rpy);
	Vector3 grc_edge = params.GRCmax_edge;
	Vector3 grc_color(0, 1.0, 0);
	draw->boxes.push_back(Boxes(grc_pos, grc_R, grc_edge, grc_color, 0.8));

	double app_len = 0.03;
	double app_radius = 0.003;
	Vector3 app_dir = grc_R * Vector3(0, 1, 0);
	Vector3 app_pos = grc_pos + ((grc_edge.y() / 2.0) + (app_len / 2.0)) * app_dir;
	Vector3 app_color(1.0, 0, 0);
	draw->cylinders.push_back(Cylinders(app_pos, app_dir, app_radius, app_len, app_color, 1.0));

	double app_tip_radius = 0.01;
	double app_tip_len = 0.015;
	Vector3 app_tip_pos = app_pos + (app_len / 2.0) * app_dir;
	Vector3 z_vec(0, 0, 1);
	Matrix3 app_tip_R = rotFromTwoVecs(z_vec, app_dir);

	draw->cones.push_back(Cones(app_tip_pos, app_tip_R, app_tip_radius, app_tip_len, app_color, 1.0));
	draw->displayShapes();
}

void PrehensionParamHandler::parsePosfiles() {
	PlanBase* tc = PlanBase::instance();

	for (size_t i = 0; i < params.size(); i++) {
		if (params[i].ref_motion.empty()) continue;
		string file_path = tc->dataFilePath() + params[i].ref_motion[0] + ".pos";
		ifstream fin(file_path.c_str());
		if (!fin) {
			continue;
		}

		if (!fin.eof()) {
			double val;
			fin >> val;
			for (int j = 0; (j < tc->nFing() && j < params[i].angles.size()); j++) {
				for (int k = 0; (k < tc->fingers(j)->nJoints && k < params[i].angles[j].size()); k++) {
					fin >> val;
					params[i].angles[j][k] = val;
				}
			}
		}
	}
}

void PrehensionParamHandler::writePosfiles() {
	PlanBase* tc = PlanBase::instance();

	for (size_t i = 0; i < params.size(); i++) {
		if (params[i].ref_motion.empty()) continue;
		string file_path = tc->dataFilePath() + params[i].ref_motion[0] + ".pos";
		ofstream fout(file_path.c_str());
		fout.setf(ios::fixed);
		if (!fout) {
			continue;
		}

		fout << "0";

		for (int j = 0; j <  params[i].fing_size(); j++) {
			for (int k = 0; k < params[i].joint_size(j); k++) {
				fout << " " << params[i].angles[j][k];
			}
		}

		fout << endl;
	}
}

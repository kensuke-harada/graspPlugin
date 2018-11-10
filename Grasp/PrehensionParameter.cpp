#include "PrehensionParameter.h"

#include <iostream>
using namespace std;
using namespace cnoid;
using namespace grasp;

void Prehension::initializeClass(ExtensionManager* ext) {
	static bool initialized = false;

	if (!initialized) {
		ItemManager& im = ext->itemManager();
		im.registerClass<Prehension>("Prehension");
		initialized = true;
	}
}

void Prehension::setNameDefault() {
	string name;
	info->read("name", name);
	setName(name);
}

void Prehension::convertToPrehensionParameter(PrehensionParameter* param) const {
	info->read("name", param->name);

	const Mapping& grcmin = *(info->findMapping("GRCMIN"));
	if (grcmin.isValid()) {
		const Listing& pos = *grcmin.findListing("Position");
		if (pos.isValid() && pos.size() == 3) {
			param->GRCmin_pos = Vector3(pos[0].toDouble(), pos[1].toDouble(), pos[2].toDouble());
		}

		const Listing& rpy = *grcmin.findListing("Rpy");
		if (rpy.isValid() && rpy.size() == 3) {
			param->GRCmin_rpy = Vector3(rpy[0].toDouble(), rpy[1].toDouble(), rpy[2].toDouble());
		}

		const Listing& edge = *grcmin.findListing("Edges");
		if (edge.isValid() && edge.size() == 3) {
			param->GRCmin_edge = Vector3(edge[0].toDouble(), edge[1].toDouble(), edge[2].toDouble());
		}
	}

	const Mapping& grcdes = *(info->findMapping("GRCDES"));
	if (grcdes.isValid()) {
		const Listing& pos = *grcdes.findListing("Position");
		if (pos.isValid() && pos.size() == 3) {
			param->GRCdes_pos = Vector3(pos[0].toDouble(), pos[1].toDouble(), pos[2].toDouble());
		}

		const Listing& rpy = *grcdes.findListing("Rpy");
		if (rpy.isValid() && rpy.size() == 3) {
			param->GRCdes_rpy = Vector3(rpy[0].toDouble(), rpy[1].toDouble(), rpy[2].toDouble());
		}

		const Listing& edge = *grcdes.findListing("Edges");
		if (edge.isValid() && edge.size() == 3) {
			param->GRCdes_edge = Vector3(edge[0].toDouble(), edge[1].toDouble(), edge[2].toDouble());
		}
	}

	const Mapping& grcmax = *(info->findMapping("GRCMAX"));
	if (grcmax.isValid()) {
		const Listing& pos = *grcmax.findListing("Position");
		if (pos.isValid() && pos.size() == 3) {
			param->GRCmax_pos = Vector3(pos[0].toDouble(), pos[1].toDouble(), pos[2].toDouble());
		}

		const Listing& rpy = *grcmax.findListing("Rpy");
		if (rpy.isValid() && rpy.size() == 3) {
			param->GRCmax_rpy = Vector3(rpy[0].toDouble(), rpy[1].toDouble(), rpy[2].toDouble());
		}

		const Listing& edge = *grcmax.findListing("Edges");
		if (edge.isValid() && edge.size() == 3) {
			param->GRCmax_edge = Vector3(edge[0].toDouble(), edge[1].toDouble(), edge[2].toDouble());
		}
	}

	const Listing& load = *(info->findListing("Load"));
	if (load.isValid() && load.size() == 2) {
		param->load_min = load[0].toDouble();
		param->load_max = load[1].toDouble();
	}

	ValueNode* ref = info->find("Reference_Motion");
	param->ref_motion.clear();
	if (ref->isString()) {param->ref_motion.push_back(ref->toString());}
	if (ref->isListing()) {
		Listing* refList = ref->toListing();
		for (int i = 0; i < refList->size(); i++) {
			param->ref_motion.push_back(refList->at(i)->toString());
		}
	}

	param->use_palmclose = false;
	const Listing& palmdir = *(info->findListing("palmCloseDir"));
	if (palmdir.isValid() && palmdir.size() == 3) {
		param->use_palmclose = true;
		param->palmclose_dir = Vector3(palmdir[0].toDouble(), palmdir[1].toDouble(), palmdir[2].toDouble());
	}

	const Listing& finger = *(info->findListing("FingerSetting"));
	if (finger.isValid()) {
		for (int i = 0; i < finger.size(); i++) {
			const Mapping& fing = *finger[i].toMapping();
			if (!fing.isValid() || fing.empty()) {
				continue;
			}

			PrehensionParameter::FingerParameter fing_param;

			const Listing& contact = *fing.findListing("Contact");
			if (contact.isValid()) {
				for (int j = 0; j < contact.size(); j++) {
					fing_param.contact.push_back((contact[j].toInt() == 0) ? false : true);
				}
			}

			const Listing& comp = *fing.findListing("Comp");
			if (comp.isValid()) {
				for (int j = 0; j < comp.size(); j++) {
					fing_param.complink.push_back(comp[j].toInt());
				}
			}

			const Listing& close = *fing.findListing("Close");
			if (close.isValid()) {
				for (int j = 0; j < close.size(); j++) {
					fing_param.close.push_back(close[j].toDouble());
				}
			}

			param->fingers.push_back(fing_param);
		}
	}
}

void Prehension::setInfo(const PrehensionParameter& param) {
	if (!info) {
		info = new Mapping();
	}
	info->write("name", param.name);

	Mapping* grcmin_map = info->createMapping("GRCMIN");
	Listing* grcmin_pos_list = grcmin_map->createFlowStyleListing("Position");
	grcmin_pos_list->clear();
	grcmin_pos_list->append(param.GRCmin_pos(0));
	grcmin_pos_list->append(param.GRCmin_pos(1));
	grcmin_pos_list->append(param.GRCmin_pos(2));

	Listing* grcmin_rpy_list = grcmin_map->createFlowStyleListing("Rpy");
	grcmin_rpy_list->clear();
	grcmin_rpy_list->append(param.GRCmin_rpy(0));
	grcmin_rpy_list->append(param.GRCmin_rpy(1));
	grcmin_rpy_list->append(param.GRCmin_rpy(2));

	Listing* grcmin_edge_list = grcmin_map->createFlowStyleListing("Edges");
	grcmin_edge_list->clear();
	grcmin_edge_list->append(param.GRCmin_edge(0));
	grcmin_edge_list->append(param.GRCmin_edge(1));
	grcmin_edge_list->append(param.GRCmin_edge(2));

	Mapping* grcdes_map = info->createMapping("GRCDES");
	Listing* grcdes_pos_list = grcdes_map->createFlowStyleListing("Position");
	grcdes_pos_list->clear();
	grcdes_pos_list->append(param.GRCdes_pos(0));
	grcdes_pos_list->append(param.GRCdes_pos(1));
	grcdes_pos_list->append(param.GRCdes_pos(2));

	Listing* grcdes_rpy_list = grcdes_map->createFlowStyleListing("Rpy");
	grcdes_rpy_list->clear();
	grcdes_rpy_list->append(param.GRCdes_rpy(0));
	grcdes_rpy_list->append(param.GRCdes_rpy(1));
	grcdes_rpy_list->append(param.GRCdes_rpy(2));

	Listing* grcdes_edge_list = grcdes_map->createFlowStyleListing("Edges");
	grcdes_edge_list->clear();
	grcdes_edge_list->append(param.GRCdes_edge(0));
	grcdes_edge_list->append(param.GRCdes_edge(1));
	grcdes_edge_list->append(param.GRCdes_edge(2));

	Mapping* grcmax_map = info->createMapping("GRCMAX");
	Listing* grcmax_pos_list = grcmax_map->createFlowStyleListing("Position");
	grcmax_pos_list->clear();
	grcmax_pos_list->append(param.GRCmax_pos(0));
	grcmax_pos_list->append(param.GRCmax_pos(1));
	grcmax_pos_list->append(param.GRCmax_pos(2));

	Listing* grcmax_rpy_list = grcmax_map->createFlowStyleListing("Rpy");
	grcmax_rpy_list->clear();
	grcmax_rpy_list->append(param.GRCmax_rpy(0));
	grcmax_rpy_list->append(param.GRCmax_rpy(1));
	grcmax_rpy_list->append(param.GRCmax_rpy(2));

	Listing* grcmax_edge_list = grcmax_map->createFlowStyleListing("Edges");
	grcmax_edge_list->clear();
	grcmax_edge_list->append(param.GRCmax_edge(0));
	grcmax_edge_list->append(param.GRCmax_edge(1));
	grcmax_edge_list->append(param.GRCmax_edge(2));

	Listing* load_list = info->createFlowStyleListing("Load");
	load_list->clear();
	load_list->append(param.load_min);
	load_list->append(param.load_max);

	if (!param.ref_motion.empty()) {
		info->remove("Reference_Motion");
		info->write("Reference_Motion", param.ref_motion[0]);
	}

	info->remove("palmCloseDir");
	if (param.use_palmclose) {
		Listing* closedir_list = info->createFlowStyleListing("palmCloseDir");
		closedir_list->append(param.palmclose_dir(0));
		closedir_list->append(param.palmclose_dir(1));
		closedir_list->append(param.palmclose_dir(2));
	}

	info->remove("FingerSetting");
	if (!param.fingers.empty()) {
		Listing* finger_list = info->createListing("FingerSetting");
		for (size_t i = 0; i < param.fingers.size(); i++) {
			Mapping* finger_map = finger_list->newMapping();
			Listing* contact_list = finger_map->createFlowStyleListing("Contact");
			Listing* comp_list = finger_map->createFlowStyleListing("Comp");
			Listing* close_list = finger_map->createFlowStyleListing("Close");
			for (size_t j = 0; j < param.fingers[i].contact.size(); j++) {
				contact_list->append(param.fingers[i].contact[j] ? "1" : "0");
				comp_list->append(param.fingers[i].complink[j]);
				close_list->append(param.fingers[i].close[j]);
			}
		}
	}
}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
ItemPtr Prehension::doDuplicate() const {
	return new Prehension(*this);
}
#endif

void Prehension::doPutProperties(PutPropertyFunction& putProperty) {
}


bool Prehension::store(Archive& archive) {
	return true;
}


bool Prehension::restore(const Archive& archive) {
	return true;
}

void PrehensionLoader::load(const std::string& filename, std::vector<PrehensionPtr>& prehension_list) {
	YAMLReader parser;
	Mapping* root = parser.loadDocument(filename)->toMapping();
	ValueNode* prehen_node = root->find("Prehension");
	if (prehen_node->isListing()) {
		Listing* list = prehen_node->toListing();
		for (int i = 0; i < list->size(); i++) {
			PrehensionPtr prehen = new Prehension;
			prehen->info = list->at(i)->toMapping();
			prehen->setNameDefault();
			prehension_list.push_back(prehen);
		}
	} else if (prehen_node->isMapping()) {
		PrehensionPtr prehen = new Prehension;
		prehen->info = prehen_node->toMapping();
		prehen->setNameDefault();
		prehension_list.push_back(prehen);
	}
}

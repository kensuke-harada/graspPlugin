#include "AssemblyGraphExporter.h"

#include <cnoid/EigenArchive>

#include "Task.h"

#include <iostream>

using namespace grasp;

AssemblyGraphExporter::AssemblyGraphExporter() :
	time_appvec_calc_(1.0) {
}

AssemblyGraphExporter::~AssemblyGraphExporter() {
	clear();
}

void AssemblyGraphExporter::setTimeAppvecCalc(double time) {
	time_appvec_calc_ = time;
}

void AssemblyGraphExporter::clear() {
	parts_.clear();
	for (size_t i = 0; i < nodes_.size(); i++) {
		delete nodes_[i];
	}
	nodes_.clear();
	current_node_.clear();
	rels_.clear();
}

bool AssemblyGraphExporter::exportYaml(TaskItem* task, const std::string& graph_path, const std::string& relation_path, bool is_include_order) {
	clear();
	bool ret = true;

	for (TaskItem::actionSeqConstIterator it = task->cbegin(); it != task->cend(); ++it) {
		ActionItemPtr action = (*it).action;

		cnoid::BodyItemPtr base = action->getMainItem();
		Node* base_node = findCurrentNode(base);

		cnoid::BodyItemPtr target = action->getSubItem();
		Node* target_node = findCurrentNode(target);

		Node* node = new Node();
		node->id = nodes_.size();
		nodes_.push_back(node);
		node->label = action->name();
		node->is_root = false;
		node->is_leaf = false;
		node->child1 = base_node;
		node->child2 = target_node;
		node->parts.insert(node->parts.end(), node->child1->parts.begin(), node->child1->parts.end());
		node->parts.insert(node->parts.end(), node->child2->parts.begin(), node->child2->parts.end());
		for (size_t i = 0; i < node->parts.size(); i++) {
			current_node_[node->parts[i]] = node;
		}
		if (is_include_order) {
			setOrderConstraint(node);
		}

		rels_.push_back(Relation());
		Relation& rel = rels_.back();
		rel.base_part_id = findPartID(base);
		rel.target_part_id = findPartID(target);
		const ActionSequence& action_seq = action->actionSeq();
		rel.rel_p = action_seq.back().ref_pose().p;
		rel.rel_R = action_seq.back().ref_pose().R;

		double end_time;
		double start_time;
		double pre_time;
		cnoid::Vector3 sum_vec = cnoid::Vector3::Zero();
		for (ActionSeqConstRIte rit = action_seq.rbegin(); rit != action_seq.rend(); ++rit) {
			double cur_time = (*rit).time();
			if (rit == action_seq.rbegin()) {
				end_time = cur_time;
				start_time = end_time - time_appvec_calc_;
				pre_time = cur_time;
				continue;
			}
			double duration = pre_time - ((cur_time < start_time) ? start_time : cur_time);
			sum_vec += duration * ((*rit).ref_pose().p - rel.rel_p);
			pre_time = cur_time;
			if (cur_time < start_time) break;
		}
		if (!action_seq.empty()) {
			rel.app_vec = sum_vec.normalized();
		}
	}
	nodes_.back()->is_root = true;

	ret &= writeGraphYAML(graph_path);
	ret &= writeRelationYAML(relation_path);
	return ret;
}

bool AssemblyGraphExporter::writeGraphYAML(const std::string& graph_path) {
	cnoid::YAMLWriter writer(graph_path);

	cnoid::MappingPtr root = new cnoid::Mapping();

	// write node
	cnoid::ListingPtr nodes = root->createListing("nodes");
	for (size_t i = 0; i < nodes_.size(); i++) {
		cnoid::MappingPtr node_map = new cnoid::Mapping();
		node_map->write("id", static_cast<int>(i));
		node_map->write("label", nodes_[i]->label, cnoid::DOUBLE_QUOTED);
		if (nodes_[i]->is_leaf) {
			node_map->write("partID", nodes_[i]->part_id);
		}
		if (nodes_[i]->is_root) {
			node_map->write("root", "true");
		}
		nodes->append(node_map);
	}

	// write edge
	int eid = 0;
	cnoid::ListingPtr edges = root->createListing("hyperedges");
	for (size_t i = 0; i < nodes_.size(); i++) {
		if (nodes_[i]->is_leaf) continue;
		cnoid::MappingPtr edge_map = new cnoid::Mapping();
		edge_map->write("id", eid++);
		edge_map->write("source", nodes_[i]->id);
		cnoid::ListingPtr targets = edge_map->createListing("targets");
		cnoid::MappingPtr target1 = new cnoid::Mapping();
		target1->write("target", nodes_[i]->child1->id);
		targets->append(target1);
		cnoid::MappingPtr target2 = new cnoid::Mapping();
		target2->write("target", nodes_[i]->child2->id);
		targets->append(target2);
		edges->append(edge_map);
	}

	// write constraint edges
	cnoid::ListingPtr constraint_edges = root->createListing("constraintedges");
	bool has_constraint_edge = false;
	eid = 0;
	for (size_t i = 0; i < nodes_.size(); i++) {
		for (size_t j = 0; j < nodes_[i]->order_constraint.size(); j++) {
			has_constraint_edge = true;
			cnoid::MappingPtr edge_map = new cnoid::Mapping();
			edge_map->write("id", eid++);
			edge_map->write("source", nodes_[i]->id);
			edge_map->write("target", nodes_[i]->order_constraint[j]->id);
			constraint_edges->append(edge_map);
		}
	}
	if (constraint_edges->empty()) {
		root->remove("constraintedges");
	}
	
	writer.putNode(root);
	
	return true;
}

bool AssemblyGraphExporter::writeRelationYAML(const std::string& relation_path) {
	cnoid::YAMLWriter writer(relation_path);

	cnoid::MappingPtr root = new cnoid::Mapping();

	// write parts
	cnoid::ListingPtr parts = root->createListing("parts");
	for (size_t i = 0; i < parts_.size(); i++) {
		cnoid::MappingPtr part_map = new cnoid::Mapping();
		part_map->write("id", static_cast<int>(i));
		part_map->write("object_name", parts_[i]->name(), cnoid::DOUBLE_QUOTED);
		parts->append(part_map);
	}

	// write realtions
	cnoid::ListingPtr rels = root->createListing("relations");
	for (size_t i = 0; i < rels_.size(); i++) {
		cnoid::MappingPtr rel_map = new cnoid::Mapping();
		cnoid::ListingPtr id_pair = rel_map->createFlowStyleListing("id_pair");
		id_pair->append(rels_[i].base_part_id);
		id_pair->append(rels_[i].target_part_id);
		write(*rel_map, "R", rels_[i].rel_R);
		write(*rel_map, "p", rels_[i].rel_p);
		write(*rel_map, "app_vec", rels_[i].app_vec);
		cnoid::ListingPtr base_id = rel_map->createFlowStyleListing("base_id");
		base_id->append(rels_[i].base_part_id);
		rels->append(rel_map);
	}

	writer.putNode(root);

	return true;
}

AssemblyGraphExporter::Node* AssemblyGraphExporter::findCurrentNode(const cnoid::BodyItemPtr& body) {
	int part_id = findPartID(body);
	if (part_id < 0) {
		// add leaf node
		part_id = parts_.size();
		parts_.push_back(body);
		Node* node = new Node();
		node->id = nodes_.size();
		nodes_.push_back(node);
		node->label = body->name();
		node->part_id = part_id;
		node->parts.push_back(part_id);
		node->is_root = false;
		node->is_leaf = true;
		node->child1 = NULL;
		node->child2 = NULL;
		current_node_.push_back(node);
		return node;
	}
	return current_node_[part_id];
}

void AssemblyGraphExporter::setOrderConstraint(Node* target_node) {
	std::vector<bool> is_constraint(nodes_.size(), true);

	// A Constraint edge is connected to the node which is not descendant node of target node

	// reomve descendant nodes from candidates
	std::queue<Node*> q_node;
	q_node.push(target_node);
	while(!q_node.empty()) {
		Node* node = q_node.front();
		is_constraint[node->id] = false;
		if (node->child1 != NULL) q_node.push(node->child1);
		if (node->child2 != NULL) q_node.push(node->child2);
		q_node.pop();
	}

	// node which have parent node is remove from candidates
	for (size_t i = 0; i < is_constraint.size(); i++) {
		if (!is_constraint[i]) continue;
		if (nodes_[i]->child1 != NULL) q_node.push(nodes_[i]->child1);
		if (nodes_[i]->child2 != NULL) q_node.push(nodes_[i]->child2);
		while(!q_node.empty()) {
			Node* node = q_node.front();
			is_constraint[node->id] = false;
			if (node->child1 != NULL) q_node.push(node->child1);
			if (node->child2 != NULL) q_node.push(node->child2);
			q_node.pop();
		}
	}

	for (size_t i = 0; i < is_constraint.size(); i++) {
		if (!is_constraint[i]) continue;
		target_node->order_constraint.push_back(nodes_[i]);
	}
}

int AssemblyGraphExporter::findPartID(const cnoid::BodyItemPtr& body) {
	int i = 0;
	for (; i < parts_.size(); i++) {
		if (parts_[i] == body) {
			return i;
		}
	}
	return -1;
}

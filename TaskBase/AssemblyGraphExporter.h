#ifndef _TASKBASE_ASSEMBLYGRAPHEXPORTER_H_
#define _TASKBASE_ASSEMBLYGRAPHEXPORTER_H_

#include <string>
#include <vector>
#include <queue>

#include <cnoid/BodyItem>
#include <cnoid/YAMLWriter>

namespace grasp {
	class TaskItem;

	class AssemblyGraphExporter {
	public:
		AssemblyGraphExporter();
		~AssemblyGraphExporter();

		void setTimeAppvecCalc(double time);
		bool exportYaml(TaskItem* task, const std::string& graph_path, const std::string& relation_path, bool is_include_order);

	private:
		class Node {
		public:
			int id;
			std::string label;
			int part_id;
			std::vector<int> parts;
			bool is_root;
			bool is_leaf;
			Node* child1;
			Node* child2;
			std::vector<Node*> order_constraint;
		};

		class Relation {
		public:
			int base_part_id;
			int target_part_id;
			cnoid::Vector3 rel_p;
			cnoid::Matrix3 rel_R;
			cnoid::Vector3 app_vec;
		};

		std::vector<cnoid::BodyItemPtr> parts_;
		std::vector<Node*> nodes_;
		std::vector<Node*> current_node_; // index denotes part id
		std::vector<Relation> rels_;
		double time_appvec_calc_;

		void clear();
		Node* findCurrentNode(const cnoid::BodyItemPtr& body);
		void setOrderConstraint(Node* target_node);
		int findPartID(const cnoid::BodyItemPtr& body);

		bool writeGraphYAML(const std::string& graph_path);
		bool writeRelationYAML(const std::string& relation_path);
	};
}

#endif

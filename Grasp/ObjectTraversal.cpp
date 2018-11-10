#include "ObjectTraversal.h"

using namespace grasp;

namespace {
	template<class OBJECT, class CONNECTION, class VISITOR>
	void BFTimpl(OBJECT start_object, VISITOR& visitor) {
		std::map<ObjectDescriptor, bool> visited;

		std::queue<OBJECT> que;
		visited[start_object->descriptor()] = true;
		que.push(start_object);

		while (!que.empty()) {
			OBJECT target = que.front();
			que.pop();
			visitor.examineObject(target);

			for (size_t i = 0; i < target->connections().size(); i++) {
				CONNECTION connection = target->connections()[i];
				if (!visitor.isTargetConnection(connection)) continue;
				OBJECT child = connection->slaveObject();
				if (!visitor.isTargetObject(child)) continue;
				if (visited.count(child->descriptor()) > 0) continue;
				visitor.examineConnection(connection);
				visited[child->descriptor()] = true;
				que.push(child);
			}

			visitor.finishObject(target);
			if (visitor.finishFlag()) return;
		}
	}

	template<class OBJECT, class CONNECTION, class VISITOR>
	void DFTimplProc(OBJECT cur_object, VISITOR& visitor, std::map<ObjectDescriptor, bool>& visited) {
		visited[cur_object->descriptor()] = true;
		visitor.examineObject(cur_object);

		for (size_t i = 0; i < cur_object->connections().size(); i++) {
			if (visitor.finishFlag()) break;
			CONNECTION connection = cur_object->connections()[i];
			if (!visitor.isTargetConnection(connection)) continue;
			OBJECT child = connection->slaveObject();
			if (!visitor.isTargetObject(child)) continue;
			if (visited.count(child->descriptor()) > 0) continue;
			visitor.examineConnection(connection);
			DFTimplProc<OBJECT, CONNECTION, VISITOR>(child, visitor, visited);
		}

		visitor.finishObject(cur_object);
	}

	template<class OBJECT, class CONNECTION, class VISITOR>
	void DFTimpl(OBJECT start_object, VISITOR& visitor) {
		std::map<ObjectDescriptor, bool> visited;
		DFTimplProc<OBJECT, CONNECTION, VISITOR>(start_object, visitor, visited);
	}
}

void grasp::objectBreadthFirstTraversal(ObjectBase* start_object, ObjectVisitor& visitor) {
	BFTimpl<ObjectBase*, Connection*, ObjectVisitor>(start_object, visitor);
}

void grasp::objectBreadthFirstTraversal(const ObjectBase* start_object, ObjectConstVisitor& visitor) {
	BFTimpl<const ObjectBase*, const Connection*, ObjectConstVisitor>(start_object, visitor);
}

void grasp::objectDepthFirstTraversal(ObjectBase* start_object, ObjectVisitor& visitor) {
	DFTimpl<ObjectBase*, Connection*, ObjectVisitor>(start_object, visitor);
}

void grasp::objectDepthFirstTraversal(const ObjectBase* start_object, ObjectConstVisitor& visitor) {
	DFTimpl<const ObjectBase*, const Connection*, ObjectConstVisitor>(start_object, visitor);
}

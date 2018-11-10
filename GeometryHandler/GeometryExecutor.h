#ifndef _GEOMETRYHANDLER_GEOMETRYEXECUTOR_H_
#define _GEOMETRYHANDLER_GEOMETRYEXECUTOR_H_

#include <vector>
#include <map>
#include <iostream>

#include <cnoid/BodyItem>
#include <cnoid/MessageView>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneView>
#include <cnoid/SceneRenderer>

namespace grasp {
	class ShapeExtractor;
	class ObjectShape;
	class VertexLink;

	class GeometryExecutor {
	public:
		explicit GeometryExecutor(const cnoid::BodyItemPtr& item);
		virtual ~GeometryExecutor();

		void limitedCluster(); // X
		void createBoundaryData(); // B
		void clusterBinary(); // C
		void createDepartData(); // V
		void clusterStep(); // S
		void reset(); //R
		void matchCluster(); // M
		void clusterOld(); // D
		void findPair(); // F
		void resultOutput(); //G
		void incrementSeedSize(); // H
		void decrementSeedSize(); // I
		void clusterOutput(); // J
		void clusterObjEnvContact(); // K
		void clusterForAssembly(); // L
		void assemblyShowContactPointLocus(); // N
		void assemblyGenerateGraph(); // O

		bool loadObjectShape();
		bool displayObjectShape(ObjectShape* object);
		bool displayClusterColor();
		bool displayClusterColor2(ObjectShape* object);

		void displayPointSet(std::vector<std::vector<VertexLink*> >& pSet);
	private:
		std::ostream& os;
		int intention;
		int methodType;
		std::vector<int> idCluster;
		bool show_all_clusters;
		ObjectShape* object;
		cnoid::BodyItemPtr pointedItem;
	};
}


#endif


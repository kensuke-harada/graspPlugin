#ifndef GEOMETRY_HANDLE_SCENE_BODY_H
#define GEOMETRY_HANDLE_SCENE_BODY_H

//#include <cnoid/SceneBody>
#ifdef  CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>
#endif	// 20140218 t.mizuno
#include <cnoid/BodyItem>
//#include <cnoid/SceneBodyManager>
#ifdef  CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBodyManager>
#endif // 20140218 t.mizuno
#include <osg/Geometry>
#include "GeometryHandle.h"
#include "ObjectShape.h"
#include "../Grasp/PlanBase.h"

namespace grasp{

	class GeodeFinder : public osg::NodeVisitor {
	public:

		// Constructor - sets the traversal mode to TRAVERSE_ALL_CHILDREN
		// and Visitor type to NODE_VISITOR
		GeodeFinder();

		// The 'apply' method for 'node' type instances.
		// See if a className() call to searchNode returns "Geode."
		// If so, add this node to our list.
		void apply(osg::Node &searchNode);

		// Return a pointer to the first node in the list
		// with a matching name
		osg::Geode* getFirst();

		// return a the list of nodes we found
		std::vector<osg::Geode*> getNodeList();

	private:
		// List of nodes with names that match the searchForName string
		std::vector<osg::Geode*> foundGeodes;
	};


//	class GeometryHandleSceneBody : public cnoid::SceneBody {
	class GeometryHandleSceneBody : public cnoid::OSGSceneBody {	// 20140218 t.mizuno
	public:
		GeometryHandleSceneBody(cnoid::BodyItemPtr bodyItem);

		//bool printVertexTriangleData(const SceneViewEvent& event);
		bool displayClusterColor();
		bool displayClusterColor2(ObjectShape* object);
		void displayPointSet(std::vector<std::vector<VertexLink*> >& pSet);

		bool loadObjectShape();
		bool displayObjectShape(ObjectShape* object);
		//hrp::Link* targetLink;
		//osg::ref_ptr<osgManipulator::Projector> projector;
		//osgManipulator::PointerInfo pointerInfo;
		//hrp::Vector3 pressPos;
		//osg::ref_ptr<osg::Node>  shapeNode;


		GeodeFinder geodeFinder;
		//std::vector<osg::Vec3Array*> verticesList;
		ObjectShape* object;

	protected:

		//virtual ~GeometryHandleSceneBody() {  }

		//virtual void onAttachedToScene();
		//virtual void onDetachedFromScene();

		virtual bool onKeyPressEvent(const cnoid::OSGSceneViewEvent& event); // 20140218 t.mizuno
		virtual bool onButtonPressEvent( const cnoid::OSGSceneViewEvent& event ); // 20140218 t.mizuno

		//virtual bool onKeyReleaseEvent(const SceneViewEvent& event);
		//virtual bool onButtonPressEvent(const SceneViewEvent& event);
		//virtual bool onButtonReleaseEvent(const SceneViewEvent& event);
		//virtual bool onDoubleClickEvent(const SceneViewEvent& event);
		//virtual bool onPointerMoveEvent(const SceneViewEvent& event);
		//virtual void onPointerLeaveEvent(const SceneViewEvent& event);
		//virtual void onContextMenuRequest(const SceneViewEvent& event, MenuManager& menuManager);
		//virtual void onSceneModeChanged();
		//virtual bool onUndoRequest();
		//virtual bool onRedoRequest();


	private:
		cnoid::MessageView& mes;
		std::ostream& os;

		bool isDragging;
		std::vector<int> idCluster;
		bool show_all_clusters;
		cnoid::BodyItemPtr pointedItem;

		osg::ref_ptr<osg::Geode> pclNode;
		osg::Vec3Array* curPoint;
		osg::Vec4Array* curColor;
		osg::Geometry* geom;
		cnoid::OSGSceneView * viewer;	// 20140218 t.mizuno
	};
	typedef osg::ref_ptr<GeometryHandleSceneBody> GeometryHandleSceneBodyPtr;

}


#endif

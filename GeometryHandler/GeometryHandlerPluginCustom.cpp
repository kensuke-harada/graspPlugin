// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*! @file
  @author Tokuo Tsuji
  @author Kensuke Harada
*/

#include <time.h>

#include <cnoid/Plugin>
#include <cnoid/MessageView>

#include <osgManipulator/Projector>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Depth>

#include <osgDB/WriteFile>
#include <osgDB/Archive>

#include <osgGA/GUIEventAdapter>
using namespace osgGA;

//#include <cnoid/SceneBody>
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>
#endif
#include <cnoid/BodyItem>

//#include <cnoid/SceneBodyManager>
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBodyManager>
#endif
#include <iostream>
#include "GeometryHandle.h"
#include "GeometryBar.h"

#include "MatchClusters.h"

#include "FindParallelPlane.h"
#include "AssemblyPlan.h"
#include "ObjEnvContact.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

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

GeodeFinder::GeodeFinder ()
	: NodeVisitor (osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

void GeodeFinder::apply (osg::Node &searchNode) {
	if (! strcmp (searchNode.className(), "Geode")) {
		foundGeodes.push_back ((osg::Geode*) &searchNode);
	}
	traverse (searchNode);
}

osg::Geode* GeodeFinder::getFirst () {
	if (foundGeodes.size() > 0)
		return foundGeodes.at(0);
	else
		return NULL;
}

std::vector<osg::Geode*> GeodeFinder::getNodeList() {
	return foundGeodes;
}

class GeometryHandleSceneBody : public cnoid::OSGSceneBody {
public:
	GeometryHandleSceneBody(cnoid::BodyItemPtr bodyItem) : OSGSceneBody(bodyItem), mes(*MessageView::mainInstance()), os (MessageView::mainInstance()->cout()) {	// 20140220 t.mizuno
		pointedItem = bodyItem;
		isDragging=false;
		object = NULL;
	}

protected:

	virtual ~GeometryHandleSceneBody() {  };

	//virtual void onAttachedToScene();
	//virtual void onDetachedFromScene();

	virtual bool onKeyPressEvent(const OSGSceneViewEvent& event);
	virtual bool onButtonPressEvent( const OSGSceneViewEvent& event );

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
	MessageView& mes;
	std::ostream& os;

	//bool printVertexTriangleData(const SceneViewEvent& event);
	bool displayClusterColor();
	bool displayClusterColor2(ObjectShape* object);
	void displayPointSet(vector<vector<VertexLink*> >& pSet);

	bool loadObjectShape(const SceneViewEvent& event);
	bool displayObjectShape(ObjectShape* object);
	//hrp::Link* targetLink;
	//osg::ref_ptr<osgManipulator::Projector> projector;
	//osgManipulator::PointerInfo pointerInfo;
	//hrp::Vector3 pressPos;
	//osg::ref_ptr<osg::Node>  shapeNode;


	GeodeFinder geodeFinder;

	//std::vector<osg::Vec3Array*> verticesList;

	ObjectShape* object;

	bool isDragging;
	vector<int> idCluster;
	bool show_all_clusters;
	cnoid::BodyItemPtr pointedItem;

	osg::ref_ptr<osg::Geode> pclNode;
	osg::Vec3Array* curPoint;
	osg::Vec4Array* curColor;
	osg::Geometry* geom;
	cnoid::OSGSceneView * viewer;	// 20140220 t.mizuno
};
typedef osg::ref_ptr<GeometryHandleSceneBody> GeometryHandleSceneBodyPtr;


bool GeometryHandleSceneBody::loadObjectShape(const OSGSceneViewEvent& event){

	if(object) delete object;

	osg::Node* shapeNode = getPointedShapeNode();
	//GeodeFinder geodeFinder;
	shapeNode->accept (geodeFinder);
	std::vector<osg::Geode*> gList = geodeFinder.getNodeList() ;

#if 0 //Used for Choreonoid 1.0 and 1.1
	vector<double> vertex;
	vector<int> crd;
	if(gList.size()){
		if( gList[0]->getNumDrawables() ){
			osg::Geometry* geometry=gList[0]->getDrawable(0)->asGeometry();
			if(geometry){
				osg::Vec3Array& vertices = *(osg::Vec3Array*)geometry->getVertexArray();
				for(int k=0;k<vertices.size();k++){
					vertex.push_back( (vertices)[k].x() );
					vertex.push_back( (vertices)[k].y() );
					vertex.push_back( (vertices)[k].z() );
				}

				osg::IntArray& vertexIndices = *((osg::IntArray*)geometry->getVertexIndices()) ;
				osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));
				int index = 0;
				for(int k=0;k<lengths.size();k++){
					for(int l=0; l<lengths[k];l++){
						crd.push_back( vertexIndices[index++] );
					}
					crd.push_back( -1 );
				}
			}
		}
	}
#else

	ColdetModelPtr c = pointedItem->body()->link(0)->coldetModel();

	vector<double> vertex;
	float tx, ty, tz;
	for(int k=0;k<c->getNumVertices();k++){
			c->getVertex(k, tx, ty, tz);
			vertex.push_back( tx );
			vertex.push_back( ty );
			vertex.push_back( tz );
	}
	vector<int> crd;
	int t1, t2, t3;
	for(int k=0;k<c->getNumTriangles();k++){
			c->getTriangle(k, t1, t2, t3);
			crd.push_back( t1 );
			crd.push_back( t2 );
			crd.push_back( t3 );
			crd.push_back( -1 );
	}

#endif

	object = new ObjectShape (vertex,crd);
	object->makeNbr();

	object->name = pointedItem->name();
}

bool GeometryHandleSceneBody::displayClusterColor(){

	std::vector<osg::Geode*> gList = geodeFinder.getNodeList() ;
	if(!gList.size()){
		os << "no geometry data  or  not initialized" << endl;
	}


	if(gList.size()){

		osg::StateSet* stateSet =gList[0]->getStateSet();

		if(0){
			// Enable blending, select transparent bin.
			stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
			stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

			// Enable depth test so that an opaque polygon will occlude a transparent one behind it.
			stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

			// Conversely, disable writing to depth buffer so that
			// a transparent polygon will allow polygons behind it to shine thru.
			// OSG renders transparent polygons after opaque ones.
			osg::Depth* depth = new osg::Depth;
			depth->setWriteMask(false);
			stateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);

			// Disable conflicting modes.
			stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		}
		osg::Material* material = (osg::Material *) gList[0]->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
		material->setColorMode(osg::Material::OFF);
		material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.1, 1));
		material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
		material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
		 material->setEmission(osg::Material::FRONT_AND_BACK,osg::Vec4(0, 0, 0, 1));	
		material->setColorMode(osg::Material::EMISSION); 
		material->setTransparency(osg::Material::FRONT_AND_BACK, 0.1);
		gList[0]->getStateSet()->setAttribute(material, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
		if( gList[0]->getNumDrawables() ){
			osg::Geometry* geometry=gList[0]->getDrawable(0)->asGeometry();

			if(geometry){
				osg::Vec3Array* vertices = new osg::Vec3Array;
				for(int i=0;i<object->nVerticies;i++){
					vertices->push_back( osg::Vec3( object->quadVerticies[i].pos[0],object->quadVerticies[i].pos[1],object->quadVerticies[i].pos[2]) );
				}
				geometry->setVertexArray( vertices );
				
				
				osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));
				osg::Vec4Array* colors = new osg::Vec4Array;
				for(int i=0;i < object->nClusterNodes;i++){
					float *c = object->clusterNodes[i].color;
					colors->push_back(osg::Vec4(c[0],c[1],c[2],1));
				}
				colors->push_back(osg::Vec4(1.0, 0, 0, 1)); //dummy color for debugging clustering
				colors->push_back(osg::Vec4(0, 1, 1, 1));    //dummy color for debugging clustering
				//int index = 0;
				osg::UIntArray* indices = new osg::UIntArray;
				for(int k=0;k<object->nTriangles;k++){
					indices->push_back(object->triangles[k].idCluster);
				}
				geometry->setColorIndices(indices);
				geometry->setColorArray(colors);
				geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
			}
		}
	}
	requestRedraw();
}


bool GeometryHandleSceneBody::displayObjectShape(ObjectShape* object){

	osg::Node* shapeNode = getPointedShapeNode();

	osg::Geode* geode = new osg::Geode();
	osg::Geometry* geometry = new osg::Geometry();

	geode->addDrawable(geometry);
	shapeNode->asGroup()->addChild(geode);
//	this->addChild(geode);

	osg::Vec3Array* vertices = new osg::Vec3Array;
	for(int i=0;i<object->nVerticies;i++){
		vertices->push_back( osg::Vec3( object->verticies[i].pos[0],object->verticies[i].pos[1],object->verticies[i].pos[2]) );
	}
	geometry->setVertexArray( vertices );

	osg::DrawElementsUInt* triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	for(int i=0;i<object->nTriangles;i++){
		for(int j=0;j<3;j++){
			triangles->push_back(object->triangles[i].ver[j]->id );
		}
	}
	geometry->addPrimitiveSet(triangles);

	if(object->clusterNodes){
		geometry->getOrCreateStateSet();
		osg::Material* material = new osg::Material;
		material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.3, 0.3, 0.3, 1));
		material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
		material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
		material->setColorMode(osg::Material::OFF);
		material->setColorMode(osg::Material::EMISSION);
		geometry->getStateSet()->setAttribute(material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

		if(geometry){
			osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));
			osg::Vec4Array* colors = new osg::Vec4Array;
			for(int i=0;i < object->nClusterNodes;i++){
				float *c = object->clusterNodes[i].color;
				colors->push_back(osg::Vec4(c[0],c[1],c[2],c[3]));
			}
			colors->push_back(osg::Vec4(1.0, 0, 0, 1)); //dummy color for debugging clustering
			colors->push_back(osg::Vec4(0, 1, 1, 1));    //dummy color for debugging clustering
			//int index = 0;
			osg::UIntArray* indices = new osg::UIntArray;
			for(int k=0;k<object->nTriangles;k++){
				indices->push_back(object->triangles[k].idCluster);
			}
			geometry->setColorIndices(indices);
			geometry->setColorArray(colors);
			geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
		}
	}
	requestRedraw();
}


int argument(const vector<int>& a, int n){

	for(unsigned int i=0; i<a.size(); i++)
		if(a[i] == n)
			return i;

	return -1;
}

bool GeometryHandleSceneBody::displayClusterColor2(ObjectShape* object){

	displayClusterColor();

	osg::Node* shapeNode = getPointedShapeNode();

	osg::Geode* geode = new osg::Geode();
	osg::Geometry* geometry = new osg::Geometry();

	geode->addDrawable(geometry);
	shapeNode->asGroup()->addChild(geode);
//	this->addChild(geode);

	osg::Vec3Array* vertices = new osg::Vec3Array;
	for(int i=0;i<object->nVerticies;i++){
		vertices->push_back( osg::Vec3( object->verticies[i].pos[0],object->verticies[i].pos[1],object->verticies[i].pos[2]) );
	}
	geometry->setVertexArray( vertices );

	osg::DrawElementsUInt* triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
	for(int i=0;i<object->nTriangles;i++){
		for(int j=0;j<3;j++){
			triangles->push_back(object->triangles[i].ver[j]->id );
		}
	}
	geometry->addPrimitiveSet(triangles);

	geometry->getOrCreateStateSet();
	osg::Material* material = new osg::Material;
	material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.3, 0.3, 0.3, 1));
	material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
	material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.1, 0.1, 0.1, 1));
	material->setColorMode(osg::Material::OFF);
	material->setColorMode(osg::Material::EMISSION);
	geometry->getStateSet()->setAttribute(material, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

	if(geometry){
			osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));
			osg::Vec4Array* colors = new osg::Vec4Array;

			for(unsigned int i=0;i < object->parentList.size();i++){
				float *c = object->clusters[object->parentList[i]].color;

				if(show_all_clusters && object->clusters[object->parentList[i]].closed)
					colors->push_back(osg::Vec4(c[0],c[1],c[2],c[3]));
				else if (!show_all_clusters && included(object->parentList[i], idCluster))
					colors->push_back(osg::Vec4(0.0, 0.0, 1.0, 1.0));
				else
					colors->push_back(osg::Vec4(0.5, 1.0, 0.5, 1.0));
			}
			colors->push_back(osg::Vec4(1.0, 0, 0, 1)); //dummy color for debugging clustering
			colors->push_back(osg::Vec4(0, 1, 1, 1));    //dummy color for debugging clustering
			//int index = 0;
			osg::UIntArray* indices = new osg::UIntArray;
			for(int k=0;k<object->nTriangles;k++){
				indices->push_back(grasp::argument(object->parentList, object->triangles[k].idCluster));
			}
			geometry->setColorIndices(indices);
			geometry->setColorArray(colors);
			geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
	}
	requestRedraw();

}

void GeometryHandleSceneBody::displayPointSet(vector<vector<VertexLink*> >& pSet){

	cnoid::SceneObjectPtr obj = new cnoid::SceneObject();

	pclNode = new osg::Geode;
	pclNode->setDataVariance(osg::Object::DYNAMIC); // STATIC or DYNAMIC

	osg::StateSet* state = pclNode->getOrCreateStateSet();
	state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	bool prevNumDrawables = pclNode->getNumDrawables();
	if(prevNumDrawables > 0)
		pclNode->removeDrawables(0, prevNumDrawables);

	curPoint = new osg::Vec3Array;
	curColor = new osg::Vec4Array;
	geom = new osg::Geometry;

	for(size_t i=0; i<pSet.size(); i++){

		osg::Vec4 color( drand48(), drand48(), drand48(), 1.0);

		for (size_t j=0; j<pSet[i].size(); j++){
			Vector3 pos0 = pointedItem->body()->link(0)->p + pointedItem->body()->link(0)->attitude()*pSet[i][j]->pos;
			curPoint->push_back(osg::Vec3(pos0(0), pos0(1), pos0(2) ));
			curColor->push_back(color);
			Vector3 pos1 = pointedItem->body()->link(0)->p + pointedItem->body()->link(0)->attitude()*pSet[i][(j+1)%pSet[i].size()]->pos;
			curPoint->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2) ));
			curColor->push_back(color);
		}
	}

	geom->setVertexArray(curPoint);
	geom->setColorArray(curColor);
	geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

	pclNode->addDrawable(geom);

	viewer = cnoid::SceneView::mainInstance();
	obj->addChild(pclNode);
	viewer->addSceneObject(obj);

	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, curPoint->size()));
	viewer->requestRedraw();
}

bool GeometryHandleSceneBody::onKeyPressEvent(const OSGSceneViewEvent& event) {

	clock_t start,end;
	bool handled = true;

	int key = event.key();
	key = toupper(key);

	FindParallelPlane *fp = FindParallelPlane::instance();
	ObjEnvContact *oe = ObjEnvContact::instance();
	static int intention = 0;
	static int methodType = 0;
	int cs;
	vector<double> area;
	vector<int> o;

	switch(key){
	case 'C':
		if(!object){
			loadObjectShape(event);
			//object->transform(Matrix3::Identity(),Vector3(0,0,0));
			object->initialClustersFromOneFace();
			object->alpha = 0.5;
		}
		//loadObjectShape(event);
		//object->initialClustersFromOneFace();
		object->calcClusterBinaryTree();
//		object->transform(Matrix3::Identity(),Vector3(0,0,0));
		displayClusterColor();
		//displayObjectShape(object);
		return handled;
	case 'S':
		if(!object){
			loadObjectShape(event);
			//object->transform(getPointedSceneLink()->R,Vector3(0,0,0));
			object->initialClustersFromOneFace();
			//object->alpha = 0.5;
		}else{
			object->calcClusterBinaryTreeStep();
		}
		displayClusterColor();
		return handled;
	case 'R':
		delete object;
		object=NULL;
		cout << "reset" << endl;
		return handled;
	case 'T':
		if(!object){
			loadObjectShape(event);
			object->transform(Matrix3::Identity(),Vector3(0,0,0));
			object->initialClustersFromOneFace();
			//object->initialClustersFromOneFace();
			cout << "reset cluster" << endl;
		}
		return handled;
	case 'M':
		MatchClusters::instance()->objectList.push_back(object);
		if(MatchClusters::instance()->objectList.size()>1){
			int last = MatchClusters::instance()->objectList.size()-1;
			ObjectShape outObj(MatchClusters::instance()->objectList[last]);
			MatchClusters::instance()->quadricalMatchClusters(*MatchClusters::instance()->objectList[last-1], *MatchClusters::instance()->objectList[last], outObj);
			displayClusterColor();
			//displayObjectShape(&outObj);
		}
		return handled;

	case 'D': //Clustering of object model using the old algorithm
		loadObjectShape(event);

		fp->calcClusters2(*object);

		show_all_clusters=true;
		displayClusterColor2(object);
		//fp->writeResults(*object);

		return handled;
		/*
	case 'E': //Precomputation of normal vector (This command should be performed before 'F')
		loadObjectShape(event);

		fp->precalcParallelPlane(*object);

		show_all_clusters=true;
		displayClusterColor2();

		return handled;
		*/
	case 'F': //Find a pair of parallel plane of object model (Interation)
		start = clock();
		if(fp->idCluster.size()/2.0-1<fp->counter_sub){
			bool fin;
			do{
				delete object;
				object = NULL;

				loadObjectShape(event);

				fin = fp->calcParallelPlane(*object, fp->counter++);
				fp->counter_sub=0;

			}while(fp->idCluster.size()==0 && fin);
		}
		idCluster.clear();
		idCluster.push_back(fp->idCluster[2*fp->counter_sub]);
		idCluster.push_back(fp->idCluster[2*fp->counter_sub+1]);
		cs = fp->clusters_out.size();
		fp->clusters_out[cs-1].intention = intention;
		fp->clusters_out[cs-2].intention = intention;
		cout << "Cluster " << fp->idCluster[2*fp->counter_sub] << " " << fp->idCluster[2*fp->counter_sub+1] << " was added." << endl;
		fp->counter_sub++;
		end = clock();
		cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
		//cout << "Normal vector " << fp->clusters_out[cs-1].normal.transpose() << ", " << fp->clusters_out[cs-2].normal.transpose() << endl;
		cout << "Intention parameter is set to " << intention << endl;
		methodType = 0;
		show_all_clusters=false;
		displayClusterColor2(object);

		return handled;
	case 'G': //Result output
		if(methodType==0)
			fp->writeResults2(fp->clusters_out);
		if(methodType==1)
			oe->writeResult(*object);
		return handled;

	case 'H':
		fp->clusterSeedSize++;
		cout << "ClusterSeedSize is set to " <<  fp->clusterSeedSize << endl;
		return handled;

	case 'I':
		fp->clusterSeedSize--;
		cout << "ClusterSeedSize is set to " <<  fp->clusterSeedSize << endl;
		return handled;

	case 'J':
		if(methodType==0){
			if(++intention > 3) intention = 0;
			cs = fp->clusters_out.size();
			fp->clusters_out[cs-1].intention = intention;
			fp->clusters_out[cs-2].intention = intention;
			cout << "Intention parameter is set to " << intention << endl;
		}
		if(methodType==1){
			int c = oe->calcTargetCluster(*object);
			object->clusters[c].Put = 1;
			cout << "Cluster #" << c << " was set to a putting cluster" << endl;
		}
		return handled;

	case 'K': //ObjEnvContact clustering method
		start = clock();
		loadObjectShape(event);

		ObjEnvContact::instance()->calcPlaneClusters(*object);

		show_all_clusters=true;
		methodType=1;
		displayClusterColor2(object);
		/*
		o.clear(); area.clear();
		for(size_t i=0;i<object->parentList.size();i++){
				area.push_back(1.0/object->clusters[object->parentList[i]].area);
				o.push_back(object->parentList[i]);
		}
		sort_by(o, area);

		for(size_t i=0; i<o.size(); i++)
				if(object->clusters[o[i]].area > object->clusters[o[0]].area/1000.0)
					displayPointSet(object->clusters[o[i]].boundaryList);
		*/
		end = clock();
		cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;

		return handled;

	case 'L': //Clustering algorithm for assemblyplan

		loadObjectShape(event);

		AssemblyPlan::instance()->calcPlaneClusters(*object);
		AssemblyPlan::instance()->object2.push_back(object);
		AssemblyPlan::instance()->objectNames.push_back(pointedItem->name());

		show_all_clusters=true;
		methodType=2;
		displayClusterColor2(object);

		return handled;

	case 'N' : //Show contact point locus

		AssemblyPlan::instance()->showContactPointLocus();
		return handled;

	case 'O' :

		AssemblyPlan::instance()->generateGraph();
		return handled;
    }

    return cnoid::OSGSceneBody::onKeyPressEvent(event);
}

bool GeometryHandleSceneBody::onButtonPressEvent( const OSGSceneViewEvent& event )
{

  if( ( event.modKeyMask() & GUIEventAdapter::MODKEY_CTRL )
		  && ( event.button() == GUIEventAdapter::LEFT_MOUSE_BUTTON ) ){	/* modified by qtconv.rb 3rd rule*/
		cnoid::Vector3 pressPos = Vector3( event.point().x(), event.point().y(), event.point().z() );

		FindParallelPlane::instance()->findTargetTriangle( getPointedSceneLink(), pressPos, pointedItem );

		return true;
    }

  	return cnoid::OSGSceneBody::onButtonPressEvent( event );
}



cnoid::OSGSceneBody* createGeometryHandle(cnoid::BodyItem* item) {
	return new GeometryHandleSceneBody(item);;
}


class GeometryHandlerPlugin : public cnoid::Plugin {
private:

public:

//	static GeometryHandleSceneBody* GeometryHandleSceneBody;



	GeometryHandlerPlugin() : Plugin("GeometryHandler") {
		depend("Grasp");
	}

	~GeometryHandlerPlugin() { }



	virtual bool initialize() {
		cnoid::OSGSceneBodyManager* manager = cnoid::OSGSceneBodyManager::instance();
		manage(manager->addSceneBodyFactory(createGeometryHandle));
		addToolBar(GeometryBar::instance());
		return true;
	}

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(GeometryHandlerPlugin);

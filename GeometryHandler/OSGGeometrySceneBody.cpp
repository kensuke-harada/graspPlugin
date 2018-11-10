// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*! @file
  @author Tokuo Tsuji
  @author Kensuke Harada
*/

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

#include "GeometryHandleSceneBody.h"

#include <Eigen/Core>

#include <stdio.h>
#include <time.h>
#ifndef WIN32
#include <unistd.h>
#include <sys/time.h>
#include <sys/times.h>
#endif

#include "GeometryAnalysis.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

double _drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
	}

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

GeometryHandleSceneBody::GeometryHandleSceneBody(cnoid::BodyItemPtr bodyItem) : OSGSceneBody(bodyItem), mes(*cnoid::MessageView::mainInstance()), os (cnoid::MessageView::mainInstance()->cout()) {
	pointedItem = bodyItem;
	isDragging=false;
	object = NULL;
}

#ifdef DEBUG_TIME
clock_t times_clock()
{
    struct tms t;
    return times(&t);
}
#endif
                    
bool GeometryHandleSceneBody::loadObjectShape(){

	if(object) delete object;
	
	 //Eigen::initParallel();
	//Eigen::setNbThreads(6);

//	osg::Node* shapeNode = pointedLinkShapeNode();
	osg::Node* shapeNode = linkNode(pointedItem->body()->link(0));

	//GeodeFinder geodeFinder;
	shapeNode->accept (geodeFinder);
	std::vector<osg::Geode*> gList = geodeFinder.getNodeList() ;

#if 0
	//Used for Choreonoid 1.0 and 1.1
	vector<double> vertex;
	vector<int> crd;
	if(gList.size()){
		if( gList[0]->getNumDrawables() ){
			osg::Geometry* geometry=gList[0]->getDrawable(0)->asGeometry();
			if(geometry){
				osg::Vec3Array& vertices = *(osg::Vec3Array*)geometry->getVertexArray();
				cout <<"v " <<vertices.size () << endl;
				for(int k=0;k<vertices.size();k++){
					vertex.push_back( (vertices)[k].x() );
					vertex.push_back( (vertices)[k].y() );
					vertex.push_back( (vertices)[k].z() );
				}

				osg::IntArray& vertexIndices = *((osg::IntArray*)geometry->getVertexIndices()) ;
				osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));
				cout <<"c " <<vertexIndices.size () << endl;
				int index = 0;
				for(int k=0;k<vertexIndices.size ()/3;k++){
					for(int l=0; l<3;l++){
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
	return true;
}

bool GeometryHandleSceneBody::displayClusterColor(){

	std::vector<osg::Geode*> gList = geodeFinder.getNodeList() ;
	if(!gList.size()){
		os << "no geometry data  or  not initialized" << endl;
	}


	if(gList.size()){

		osg::StateSet* stateSet =gList[0]->getStateSet();

		if(1){
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
			//stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		}
	//	osg::Materia//l* material = (osg::Material *) gList[0]->getStateSet()->getAttribute(osg::StateAttribute::MATERIAL);
		osg::Material* material = new osg::Material;
		material->setColorMode(osg::Material::OFF);
		material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.2, 0.2, 0.2, 0.1));
		material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 0.1));
		material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 0.1));
		 material->setEmission(osg::Material::FRONT_AND_BACK,osg::Vec4(0, 0, 0, 0.1));
//		 material->setShininess(osg::Material::FRONT,0);
		material->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f,0.0,0.0,0.1));
//		material->setColorMode(osg::Material::DIFFUSE);
		material->setTransparency(osg::Material::FRONT_AND_BACK, 0.1);
		gList[0]->getStateSet()->setAttribute(material, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
//		gList[0]->getStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
//		gList[0]->getStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

//		gList[i]->getOrCreateStateSet()->setAttributeAndModes(material, osg::StateAttribute::OFF);
		if( 0 ){
//		if( gList[0]->getNumDrawables() ){
//			os << i << " " << j << endl;
			osg::Geometry* geometry=gList[0]->getDrawable(0)->asGeometry();

			if(geometry){
				osg::DrawArrayLengths& lengths = *((osg::DrawArrayLengths*)(geometry->getPrimitiveSet(0)));
				osg::Vec4Array* colors = new osg::Vec4Array;
				for(int i=0;i < object->nClusterNodes;i++){
					float *c = object->clusterNodes[i].color;
//					colors->push_back(osg::Vec4(c[0],c[1],c[2],c[3]));
					colors->push_back(osg::Vec4(0,0,0,0.5));
				}
				colors->push_back(osg::Vec4(1.0, 0, 0, 1)); //dummy color for debugging clustering
				colors->push_back(osg::Vec4(0, 1, 1, 1));    //dummy color for debugging clustering
				//int index = 0;
				osg::UIntArray* indices = new osg::UIntArray;
				for(int k=0;k<object->nTriangles;k++){
//					indices->push_back(object->triangles[k].idCluster);
					indices->push_back(0);
				}
				geometry->setColorIndices(indices);
				geometry->setColorArray(colors);
geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
//				geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
/*
				if( object->quadVerticies){
					osg::Vec3Array* vertices = (osg::Vec3Array*)geometry->getVertexArray();
					for(int k=0;k<vertices->size();k++){
						(*vertices)[k] = osg::Vec3 ( object->quadVerticies[k].pos[0],object->quadVerticies[k].pos[1],object->quadVerticies[k].pos[2]);
					}
					geometry->setVertexArray(vertices);
				}
*/			}
		}
	}
	requestRedraw();
	return true;
}


bool GeometryHandleSceneBody::displayObjectShape(ObjectShape* object){

//	osg::Node* shapeNode = pointedLinkShapeNode();
	osg::Node* shapeNode = linkNode(pointedItem->body()->link(0));

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
				if(object->triangles[k].idCluster==-1)
					indices->push_back(object->nClusterNodes+1);
				else
					indices->push_back(object->triangles[k].idCluster);
			}
			geometry->setColorIndices(indices);
			geometry->setColorArray(colors);
			geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
		}
	}
	requestRedraw();
	return true;
}


int argument(const vector<int>& a, int n){

	for(unsigned int i=0; i<a.size(); i++)
		if(a[i] == n)
			return i;

	return -1;
}

bool GeometryHandleSceneBody::displayClusterColor2(ObjectShape* object){

	displayClusterColor();

//	osg::Node* shapeNode = pointedLinkShapeNode();
	osg::Node* shapeNode = linkNode(pointedItem->body()->link(0));

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
	return true;
}

void GeometryHandleSceneBody::displayPointSet(vector<vector<VertexLink*> >& pSet){

	cnoid::OSGSceneObjectPtr obj = new cnoid::OSGSceneObject();

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

		osg::Vec4 color( _drand(), _drand(), _drand(), 1.0);

		for (size_t j=0; j<pSet[i].size(); j++){
            Vector3 pos0 = pointedItem->body()->link(0)->p() + pointedItem->body()->link(0)->attitude()*pSet[i][j]->pos;
			curPoint->push_back(osg::Vec3(pos0(0), pos0(1), pos0(2) ));
			curColor->push_back(color);
            Vector3 pos1 = pointedItem->body()->link(0)->p() + pointedItem->body()->link(0)->attitude()*pSet[i][(j+1)%pSet[i].size()]->pos;
			curPoint->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2) ));
			curColor->push_back(color);
		}
	}

	geom->setVertexArray(curPoint);
	geom->setColorArray(curColor);
	geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

	pclNode->addDrawable(geom);

	viewer = cnoid::OSGSceneView::mainInstance();
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
	
	ObjectAnalysis* objectA = new ObjectAnalysis;

	switch(key){
		case 'X':
//#define OUTPUT_TIME
#ifdef OUTPUT_TIME
		clock_t start, end;
		start = clock();
#endif
		if(!object){
			loadObjectShape();
//			object->transform(getPointedSceneLink()->R,Vector3(-0.371,0.159,-1.46));
			objectA->object = object;
			objectA->LimitedInitialClustersFromOneFace();
		}
		//object->calcClusterBinaryTree();
		object->LimitedcalcClusterBinaryTree();
		//クラスタの重心を計算
		/*for(int i=0;i<object->nClusterNodes;i++){
			if(object->clusterNodes[i].isAliveNode){
				cnoid::Vector3 cluster_center = objectA->calcMesheCenter(object->clusterNodes[i].id);
				cout << "id : " << object->clusterNodes[i].id << ", cluster_center : " << cluster_center.transpose() << endl;
			}
		}*/
		displayClusterColor();
		displayObjectShape(object);
#ifdef OUTPUT_TIME
		end = clock();
		cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
#endif
//		return 0;
		return handled;		
	case 'B':
		if(!object){
			loadObjectShape();
			//object->transform(getPointedSceneLink()->R,Vector3(0,0,0));
			object->initialClustersFromOneFace();
			//object->alpha = 0.5;
		}
		objectA->object = object;
		//loadObjectShape();
		//object->initialClustersFromOneFace();
//		cout << "check1" << endl;
		objectA->createBoundaryData();
//		cout << "check2" << endl;

		return handled;
	case 'C':
#define OUTPUT_TIME
#ifdef OUTPUT_TIME
		clock_t start, end;
		start = clock();
#endif 
		if(!object){
			loadObjectShape();
			//object->transform(getPointedSceneLink()->R,Vector3(0,0,0));
			object->initialClustersFromOneFace();
			//object->alpha = 0.5;
		}
		//loadObjectShape();
		//object->initialClustersFromOneFace();
		object->calcClusterBinaryTree();
//#define OUTPUT_UTO
#ifdef OUTPUT_UTO
		//クラスタの重心を計算
		objectA->object = object;
		for(int i=0;i<object->nClusterNodes;i++){
			if(object->clusterNodes[i].isAliveNode){
				cnoid::Vector3 cluster_center = objectA->calcMesheCenter(object->clusterNodes[i].id);
//				cout << "id : " << object->clusterNodes[i].id << ", cluster_center : " << cluster_center.transpose() << endl;
			}
		}
#endif 
		displayClusterColor();
		displayObjectShape(object);
#ifdef OUTPUT_TIME
		end = clock();
		cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << endl;
#endif
		return handled;
	case 'V':
		if(!object){
			loadObjectShape();
			//object->transform(getPointedSceneLink()->R,Vector3(0,0,0));
			object->initialClustersFromOneFace();
			//object->alpha = 0.5;
		}
		objectA->object = object;
//		cout << "check1" << endl;
		objectA->createDepartData();
//		cout << "check2" << endl;

		return handled;
	case 'S':
		if(!object){
			loadObjectShape();
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
	case 'M':
		MatchClusters::instance()->objectList.push_back(object);
		if(MatchClusters::instance()->objectList.size()>1){
			int last = MatchClusters::instance()->objectList.size()-1;
			ObjectShape outObj(MatchClusters::instance()->objectList[last]);
			MatchClusters::instance()->quadricalMatchClusters(*MatchClusters::instance()->objectList[last-1], *MatchClusters::instance()->objectList[last], outObj);
			displayObjectShape(&outObj);
		}
		return handled;

	case 'D': //Clustering of object model using the old algorithm
		loadObjectShape();

		fp->calcClusters2(*object);

		show_all_clusters=true;
		displayClusterColor2(object);
		//fp->writeResults(*object);

		return handled;
		/*
	case 'E': //Precomputation of normal vector (This command should be performed before 'F')
		loadObjectShape();

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

				loadObjectShape();

				fin = fp->calcParallelPlane(*object, fp->counter++);
				fp->counter_sub=0;

			}while(fp->idCluster.size()==0 && fin);
		}
		if(!(fp->idCluster.size()>(2*fp->counter_sub+1))){
			cout << "Cluster pair does not exist!" << endl;
			return handled;
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
			cout << "Cluster #" << c;
			if(object->clusters[c].Convexity==0) cout << " (CONVEX)";
			else if(object->clusters[c].Convexity==1) cout << " (CONCAVE)";
			else if(object->clusters[c].Convexity==2) cout << " (SADDLE)";
			else if(object->clusters[c].Convexity==3) cout << " (TABLELEG)";
			cout << " was set to a putting cluster" << endl;
		}
		return handled;

	case 'K': //ObjEnvContact clustering method
		start = clock();
		loadObjectShape();

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

		loadObjectShape();

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

		PlanBase::instance()->pressItem = pointedItem;

		return true;
    }

    return cnoid::OSGSceneBody::onButtonPressEvent( event );
}





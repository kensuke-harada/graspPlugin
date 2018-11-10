#include "GeometryExecutor.h"

#include <time.h>
#ifndef WIN32
#include <unistd.h>
#include <sys/time.h>
#include <sys/times.h>
#endif

#include "../Grasp/DrawUtility.h"

#include "MatchClusters.h"
#include "FindParallelPlane.h"
#include "ObjEnvContact.h"
#include "AssemblyPlan.h"
#include "ObjectShape.h"
#include "GeometryAnalysis.h"
#include "ShapeExtractor.h"

#include <cnoid/EditableSceneBody>

using namespace grasp;
using std::cout;
using std::endl;

namespace {
	double _drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
	}
}

GeometryExecutor::GeometryExecutor(const cnoid::BodyItemPtr& item) :
	os(cnoid::MessageView::instance()->cout()),
	object(NULL) {
	pointedItem = item;
	intention = 0;
	methodType = 0;
}

GeometryExecutor::~GeometryExecutor() {
}

void GeometryExecutor::limitedCluster() {
	ObjectAnalysis* objectA = new ObjectAnalysis;
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
}

void GeometryExecutor::createBoundaryData() {
	ObjectAnalysis* objectA = new ObjectAnalysis;
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
}

void GeometryExecutor::clusterBinary() {
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
	ObjectAnalysis* objectA = new ObjectAnalysis;
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
}

void GeometryExecutor::createDepartData() {
	ObjectAnalysis* objectA = new ObjectAnalysis;
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
}

void GeometryExecutor::clusterStep() {
	if(!object){
		loadObjectShape();
		//object->transform(getPointedSceneLink()->R,Vector3(0,0,0));
		object->initialClustersFromOneFace();
		//object->alpha = 0.5;gdb
	}else{
		object->calcClusterBinaryTreeStep();
	}
	displayClusterColor();
}

void GeometryExecutor::reset() {
	delete object;
	object=NULL;
	cout << "reset" << endl;
}

void GeometryExecutor::matchCluster() {
	MatchClusters::instance()->objectList.push_back(object);
	if(MatchClusters::instance()->objectList.size()>1){
		int last = MatchClusters::instance()->objectList.size()-1;
		ObjectShape outObj(MatchClusters::instance()->objectList[last]);
		MatchClusters::instance()->quadricalMatchClusters(*MatchClusters::instance()->objectList[last-1], *MatchClusters::instance()->objectList[last], outObj);
		displayObjectShape(&outObj);
	}
}

void GeometryExecutor::clusterOld() {
	//Clustering of object model using the old algorithm
	FindParallelPlane* fp = FindParallelPlane::instance();
	loadObjectShape();

	fp->calcClusters2(*object);

	show_all_clusters=true;
	displayClusterColor2(object);
	//fp->writeResults(*object);
}

void GeometryExecutor::findPair() {
	//Find a pair of parallel plane of object model (Interation)
	FindParallelPlane* fp = FindParallelPlane::instance();
	clock_t start,end;
	int cs;
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
		return;
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
}

void GeometryExecutor::resultOutput() {
	//Result output
	FindParallelPlane* fp = FindParallelPlane::instance();
	ObjEnvContact *oe = ObjEnvContact::instance();
	if(methodType==0)
			fp->writeResults2(fp->clusters_out);
	if(methodType==1)
			oe->writeResult(*object);
}

void GeometryExecutor::incrementSeedSize() {
	FindParallelPlane* fp = FindParallelPlane::instance();
	fp->clusterSeedSize++;
	std::cout << "ClusterSeedSize is set to " <<  fp->clusterSeedSize << std::endl;
}

void GeometryExecutor::decrementSeedSize() {
	FindParallelPlane* fp = FindParallelPlane::instance();
	fp->clusterSeedSize--;
	std::cout << "ClusterSeedSize is set to " <<  fp->clusterSeedSize << std::endl;
}

void GeometryExecutor::clusterOutput() {
	int cs;
	if(methodType==0){
		if(++intention > 3) intention = 0;
		FindParallelPlane* fp = FindParallelPlane::instance();
		cs = fp->clusters_out.size();
		fp->clusters_out[cs-1].intention = intention;
		fp->clusters_out[cs-2].intention = intention;
		cout << "Intention parameter is set to " << intention << endl;
	}
	if(methodType==1){
		ObjEnvContact *oe = ObjEnvContact::instance();
		int c = oe->calcTargetCluster(*object);
		object->clusters[c].Put = 1;
		cout << "Cluster #" << c;
		if(object->clusters[c].Convexity==0) cout << " (CONVEX)";
		else if(object->clusters[c].Convexity==1) cout << " (CONCAVE)";
		else if(object->clusters[c].Convexity==2) cout << " (SADDLE)";
		else if(object->clusters[c].Convexity==3) cout << " (TABLELEG)";
		cout << " was set to a putting cluster" << endl;
	}
}

void GeometryExecutor::clusterObjEnvContact() {
	//ObjEnvContact clustering method
	clock_t start,end;
	start = clock();
	loadObjectShape();

	ObjEnvContact::instance()->calcPlaneClusters(*object);

	show_all_clusters=true;
	methodType=1;
	displayClusterColor2(object);
	// vector<double> area;
	// vector<int> o;
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
	std::cout << "It took "<< (double)(end-start)/CLOCKS_PER_SEC << "(s)" << std::endl;
}

void GeometryExecutor::clusterForAssembly() {
	//Clustering algorithm for assemblyplan
	loadObjectShape();

	AssemblyPlan::instance()->calcPlaneClusters(*object);
	AssemblyPlan::instance()->object2.push_back(object);
	AssemblyPlan::instance()->objectNames.push_back(pointedItem->name());

	show_all_clusters=true;
	methodType=2;
	displayClusterColor2(object);
}

void GeometryExecutor::assemblyShowContactPointLocus() {
	//Show contact point locus
	AssemblyPlan::instance()->showContactPointLocus();
}

void GeometryExecutor::assemblyGenerateGraph() {
	AssemblyPlan::instance()->generateGraph();
}

bool GeometryExecutor::loadObjectShape() {
	if (object) delete object;

	cnoid::MeshExtractor extractor;
	cnoid::SgNode* node = pointedItem->body()->link(0)->collisionShape();
	cnoid::SgMesh* mesh = extractor.integrate(node);
	cnoid::SgVertexArrayPtr vertices = mesh->vertices();

	std::vector<double> vertex;
	for (int k = 0; k < vertices->size(); k++) {
		cnoid::Vector3f& vec = vertices->at(k);
		vertex.push_back(vec[0]);
		vertex.push_back(vec[1]);
		vertex.push_back(vec[2]);
	}

	std::vector<int> crd;
	for (int k = 0; k < mesh->numTriangles(); k++) {
		cnoid::SgMesh::TriangleRef tri = mesh->triangle(k);
		crd.push_back(tri[0]);
		crd.push_back(tri[1]);
		crd.push_back(tri[2]);
		crd.push_back(-1);
	}

	object = new ObjectShape(vertex, crd);
	object->makeNbr();

	object->name = pointedItem->name();

	delete mesh;
	return true;
}

bool GeometryExecutor::displayObjectShape(ObjectShape* object) {
	cnoid::SgMeshPtr mesh = new cnoid::SgMesh();

	cnoid::SgVertexArray* vertices = mesh->getOrCreateVertices();
	for (int i = 0; i < object->nVerticies; i++) {
		vertices->push_back(object->verticies[i].pos.cast<float>());
	}

	cnoid::SgIndexArray& indices = mesh->triangleVertices();
	for (int i = 0; i < object->nTriangles; i++) {
		for (int j = 0; j < 3; j++) {
			indices.push_back(object->triangles[i].ver[j]->id);
		}
	}

	cnoid::SgShapePtr shape = new cnoid::SgShape();
	shape->setMesh(mesh);
	mesh->updateBoundingBox();

	if (object->clusterNodes) {
		cnoid::SgMaterial* material = new cnoid::SgMaterial();
		material->setDiffuseColor(cnoid::Vector3f(0.3, 0.3, 0.3));
		material->setAmbientIntensity(0);
		material->setSpecularColor(cnoid::Vector3f(0, 0, 0));
		material->setEmissiveColor(cnoid::Vector3f(0, 0, 0));
		material->setTransparency(0);
		shape->setMaterial(material);

		cnoid::SgColorArray* colorArray = mesh->getOrCreateColors();
		std::vector<cnoid::Vector3f> colors;
		for (int i = 0; i < object->nClusterNodes; i++) {
			float* c = object->clusterNodes[i].color;
			colors.push_back(cnoid::Vector3f(c[0], c[1], c[2]));
		}
		colors.push_back(cnoid::Vector3f(1.0, 0, 0)); //dummy color for debugging clustering
		colors.push_back(cnoid::Vector3f(0.1, 1, 1)); //dummy color for debugging clustering

		for (int i = 0; i < object->nTriangles; i++) {
			for (int j = 0; j < 3; j++) {
				if (object->triangles[i].idCluster == -1) {
					colorArray->push_back(colors[object->nClusterNodes+1]);
				} else {
					colorArray->push_back(colors[object->triangles[i].idCluster]);
				}
			}
		}
	}

	cnoid::MeshNormalGenerator normalGenerator;
	normalGenerator.generateNormals(mesh);

	pointedItem->sceneBody()->sceneLink(0)->addChild(shape, true);

	return true;
}

bool GeometryExecutor::displayClusterColor() {
	ShapeExtractor extract;
	std::vector<cnoid::SgShape*> shapes;
	extract.collect(pointedItem->body()->link(0)->visualShape() ,shapes);

	if (shapes.empty()) return true;

	if (shapes[0]->mesh()->hasColors()) {
		shapes[0]->mesh()->colors()->clear();
	}
	cnoid::SgMaterial* material = shapes[0]->getOrCreateMaterial();
	material->setDiffuseColor(cnoid::Vector3f(0.2, 0.2, 0.2));
	material->setAmbientIntensity(0);
	material->setSpecularColor(cnoid::Vector3f(0, 0, 0));
	material->setEmissiveColor(cnoid::Vector3f(0, 0, 0));
	material->setTransparency(0.1);

	if (0) {
		cnoid::SgColorArray* colorArray = shapes[0]->mesh()->getOrCreateColors();
		std::vector<cnoid::Vector3f> colors;
		for (int i = 0; i < object->nClusterNodes; i++) {
			float* c = object->clusterNodes[i].color;
			// colors.push_back(cnoid::Vector3f(c[0], c[1], c[2]));
			colors.push_back(cnoid::Vector3f(0, 0, 0));
		}
		colors.push_back(cnoid::Vector3f(1.0, 0, 0));
		colors.push_back(cnoid::Vector3f(0.1, 1, 1));

		for (int i = 0; i < object->nTriangles; i++) {
			for (int j = 0; j < 3; j++) {
				// colorArray->push_back(colors[object->triangles[i].idCluster]);
				colorArray->push_back(colors[0]);
			}
		}
	}

	// TODO
	cnoid::ItemTreeView::instance()->checkItem(pointedItem, false);
#ifndef  CNOID_GE_16
	cnoid::SceneView::instance()->sceneWidget()->renderer().render();
#endif
	cnoid::ItemTreeView::instance()->checkItem(pointedItem, true);

	return true;
}

bool GeometryExecutor::displayClusterColor2(ObjectShape* object) {
	displayClusterColor();

	cnoid::SgMeshPtr mesh = new cnoid::SgMesh();

	cnoid::SgVertexArray* vertices = mesh->getOrCreateVertices();
	for (int i = 0; i < object->nVerticies; i++) {
		vertices->push_back(object->verticies[i].pos.cast<float>());
	}

	cnoid::SgIndexArray& indices = mesh->triangleVertices();
	for (int i = 0; i < object->nTriangles; i++) {
		for (int j = 0; j < 3; j++) {
			indices.push_back(object->triangles[i].ver[j]->id);
		}
	}

	cnoid::SgShapePtr shape = new cnoid::SgShape();
	shape->setMesh(mesh);
	mesh->updateBoundingBox();

	cnoid::SgMaterial* material = new cnoid::SgMaterial();
	material->setDiffuseColor(cnoid::Vector3f(0.3, 0.3, 0.3));
	material->setAmbientIntensity(0);
	material->setSpecularColor(cnoid::Vector3f(0, 0, 0));
	material->setEmissiveColor(cnoid::Vector3f(0, 0, 0));
	material->setTransparency(0);
	shape->setMaterial(material);

	cnoid::SgColorArray* colorArray = mesh->getOrCreateColors();
	std::vector<cnoid::Vector3f> colors;
	for (size_t i = 0; i < object->parentList.size(); i++) {
		float *c = object->clusters[object->parentList[i]].color;

		if(show_all_clusters && object->clusters[object->parentList[i]].closed)
			colors.push_back(cnoid::Vector3f(c[0], c[1], c[2]));
		else if (!show_all_clusters && included(object->parentList[i], idCluster))
			colors.push_back(cnoid::Vector3f(0.0, 0.0, 1.0));
		else
			colors.push_back(cnoid::Vector3f(0.5, 1.0, 0.5));
	}
	colors.push_back(cnoid::Vector3f(1.0, 0, 0)); //dummy color for debugging clustering
	colors.push_back(cnoid::Vector3f(0.1, 1, 1)); //dummy color for debugging clustering

	for (int i = 0; i < object->nTriangles; i++) {
		for (int j = 0; j < 3; j++) {
			colorArray->push_back(colors[grasp::argument(object->parentList, object->triangles[i].idCluster)]);
		}
	}

	cnoid::MeshNormalGenerator normalGenerator;
	normalGenerator.generateNormals(mesh);

	pointedItem->sceneBody()->sceneLink(0)->addChild(shape, true);

	return true;
}

void GeometryExecutor::displayPointSet(std::vector<std::vector<VertexLink*> >& pSet) {
	DrawUtility* draw = DrawUtility::instance();
	draw->points.clear();
	draw->colors.clear();

	for (size_t i = 0; i < pSet.size(); i++) {
		cnoid::Vector3 color(_drand(), _drand(), _drand());
		for (size_t j = 0; j < pSet[i].size(); j++) {
			Vector3 pos0 = pointedItem->body()->link(0)->p() + pointedItem->body()->link(0)->attitude()*pSet[i][j]->pos;
			draw->points.push_back(pos0);
			draw->colors.push_back(color);
			Vector3 pos1 = pointedItem->body()->link(0)->p() + pointedItem->body()->link(0)->attitude()*pSet[i][(j+1)%pSet[i].size()]->pos;
			draw->points.push_back(pos1);
			draw->colors.push_back(color);
		}
	}

	draw->displayPoints();
}

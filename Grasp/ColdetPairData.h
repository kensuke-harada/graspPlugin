
#ifndef _COLDET_PAIR_DATA_H
#define _COLDET_PAIR_DATA_H

#include <vector>

#include <cnoid/BodyItem>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define DEBUG_VIEW_MODE

#ifdef DEBUG_VIEW_MODE
#include <cnoid/SceneGraph>
#include <cnoid/SceneView>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#endif

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/SceneShape>
#include <cnoid/MeshNormalGenerator>
#include "ColdetConverter.h"
#include "ColdetModelGetter.h"
#endif

#include "exportdef.h"
#include "InterObject.h"

#include "ClearanceBoundingVolume.h"

namespace grasp{
	
class LinkStateNode{
	public:
	LinkStateNode(){
		children[0] = -1;
		children[1] = -1;
		parents[0] = -1;
		parents[1] = -1;
		existSweep = 0;
		belongNode=-1;
	}
	int id;
	double t;
	double level;
	int children[2];
	int parents[2];
	cnoid::Vector3 p;
	cnoid::Matrix3 R;
	cnoid::ColdetModelPtr sweepbv;
	int existSweep;
	int belongNode;
};
	
class EXCADE_API ColdetPairData{
	public:
	ColdetPairData(cnoid::BodyItemPtr bodyItem1,cnoid::BodyItemPtr bodyItem2,bool doComputeDistance=false, bool useRobotSafeBoundingBox=false){
		this->bodyItem1 = bodyItem1;
		this->bodyItem2 = bodyItem2;
		for(int j=0;j<bodyItem1->body()->numLinks();j++){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			cnoid::ColdetModelPtr backup = bodyItem1->body()->link(j)->coldetModel();
			if(useRobotSafeBoundingBox){
				cnoid::ColdetModelPtr temp = getSafeBoundingBox(bodyItem1->body()->link(j)->coldetModel(), cnoid::Vector3(0.03,0.03,0.03) );
				if(temp != NULL) bodyItem1->body()->link(j)->setColdetModel(temp);
			}
#else
			cnoid::SgNode* backup = bodyItem1->body()->link(j)->collisionShape();
			if(useRobotSafeBoundingBox){
				cnoid::SgShapePtr temp = getSafeBoundingBox(bodyItem1->body()->link(j)->collisionShape(), cnoid::Vector3(0.03,0.03,0.03) );
				if(temp != NULL) bodyItem1->body()->link(j)->setCollisionShape(temp);
			}
#endif
			for(int i=0;i<bodyItem2->body()->numLinks();i++){
	#ifdef  CNOID_10_11_12_13
				cnoid::ColdetLinkPairPtr temp= new cnoid::ColdetLinkPair(bodyItem1->body()->link(j), bodyItem2->body()->link(i));
	#elif defined(CNOID_14)
				cnoid::ColdetLinkPairPtr temp = boost::make_shared<cnoid::ColdetLinkPair>(bodyItem1->body(),bodyItem1->body()->link(j), bodyItem2->body(), bodyItem2->body()->link(i) );
	#else
				grasp::ColdetLinkPairPtr temp = boost::make_shared<grasp::ColdetLinkPair>(bodyItem1->body(),bodyItem1->body()->link(j), bodyItem2->body(), bodyItem2->body()->link(i) );
	#endif
				if(doComputeDistance){
					temp->updatePositions();
					int t1,t2;
					double p1[3],p2[3];
					double distance = temp->computeDistance(t1,p1,t2,p2);
					if(distance <1.0e-04) continue;
				}
				coldetLinkPairs.push_back(temp);
			}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			bodyItem1->body()->link(j)->setColdetModel(backup);
#else
			bodyItem1->body()->link(j)->setCollisionShape(backup);
#endif
		}
	}
	ColdetPairData(cnoid::BodyItemPtr bodyItem1, const std::vector<cnoid::Link*>& links, cnoid::BodyItemPtr bodyItem2, bool doComputeDistance=false, bool useRobotSafeBoundingBox=false) {
		this->bodyItem1 = bodyItem1;
		this->bodyItem2 = bodyItem2;
		for(int j=0;j<links.size();j++){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			cnoid::ColdetModelPtr backup = links[j]->coldetModel();
			if(useRobotSafeBoundingBox){
				cnoid::ColdetModelPtr temp = getSafeBoundingBox(links[j]->coldetModel(), cnoid::Vector3(0.03,0.03,0.03) );
				if(temp != NULL) links[j]->setColdetModel(temp);
			}
#else
			cnoid::SgNode* backup = links[j]->collisionShape();
			if(useRobotSafeBoundingBox){
				cnoid::SgShapePtr temp = getSafeBoundingBox(links[j]->collisionShape(), cnoid::Vector3(0.03,0.03,0.03) );
				if(temp != NULL) links[j]->setCollisionShape(temp);
			}
#endif
			for(int i=0;i<bodyItem2->body()->numLinks();i++){
#ifdef CNOID_10_11_12_13
				cnoid::ColdetLinkPairPtr temp= new cnoid::ColdetLinkPair(links[j], bodyItem2->body()->link(i));
#elif defined(CNOID_14)
				cnoid::ColdetLinkPairPtr temp = boost::make_shared<cnoid::ColdetLinkPair>(bodyItem1->body(),links[j], bodyItem2->body(), bodyItem2->body()->link(i) );
#else
				grasp::ColdetLinkPairPtr temp = boost::make_shared<grasp::ColdetLinkPair>(bodyItem1->body(),links[j], bodyItem2->body(), bodyItem2->body()->link(i) );
#endif
				if(doComputeDistance){
					temp->updatePositions();
					int t1,t2;
					double p1[3],p2[3];
					double distance = temp->computeDistance(t1,p1,t2,p2);
					if(distance <1.0e-04) continue;
				}
				coldetLinkPairs.push_back(temp);
			}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			links[j]->setColdetModel(backup);
#else
			links[j]->setCollisionShape(backup);
#endif
		}
	}
	~ColdetPairData(){
		coldetLinkPairs.clear();
	}

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	static cnoid::ColdetModelPtr getSafeBoundingBox(cnoid::ColdetModelPtr model, cnoid::Vector3 boundingBoxSafetySize){
//		cnoid::ColdetModelPtr safeBoundingBox;
#ifdef  CNOID_10_11_12_13
		cnoid::ColdetModelPtr safeBoundingBox = new cnoid::ColdetModel();
#else
		cnoid::ColdetModelPtr safeBoundingBox =  boost::make_shared<cnoid::ColdetModel>();
#endif

		int nVerticies = model->getNumVertices();
#ifdef  CNOID_10_11_12_13
		if(nVerticies == 0) return NULL;
#else
		if(nVerticies == 0) {
			safeBoundingBox.reset();
			return safeBoundingBox;
		}
#endif
		
		float out_x, out_y, out_z;
		model->getVertex(0, out_x, out_y, out_z);
		cnoid::Vector3 min(out_x, out_y, out_z),max(out_x, out_y, out_z);
		for(int i=0;i<nVerticies;i++){
			model->getVertex(i, out_x, out_y, out_z);
			if(min[0] > out_x) min[0] = out_x;
			if(min[1] > out_y) min[1] = out_y;
			if(min[2] > out_z) min[2] = out_z;
			if(max[0] < out_x) max[0] = out_x;
			if(max[1] < out_y) max[1] = out_y;
			if(max[2] < out_z) max[2] = out_z;
		}
		min -= boundingBoxSafetySize;
		max += boundingBoxSafetySize;

		for(int i=0;i<8;i++){
			cnoid::Vector3 temp;
			if( (i&1) == 0) temp[0] = min[0];
			else temp[0] = max[0];
			if( (i&2) == 0) temp[1] = min[1];
			else temp[1] = max[1];
			if( (i&4) == 0) temp[2] = min[2];
			else temp[2] = max[2];
			safeBoundingBox->addVertex(temp[0],temp[1],temp[2]);
		}
		
		safeBoundingBox->addTriangle(0,1,2);
		safeBoundingBox->addTriangle(1,3,2);
		safeBoundingBox->addTriangle(5,4,7);
		safeBoundingBox->addTriangle(4,6,7);
		safeBoundingBox->addTriangle(0,2,4);
		safeBoundingBox->addTriangle(2,6,4);
		safeBoundingBox->addTriangle(1,5,3);
		safeBoundingBox->addTriangle(5,7,3);
		safeBoundingBox->addTriangle(0,4,1);
		safeBoundingBox->addTriangle(4,5,1);
		safeBoundingBox->addTriangle(2,3,6);
		safeBoundingBox->addTriangle(3,7,6);
		
		safeBoundingBox->build();
		
		return safeBoundingBox;
	}
#else
	static cnoid::SgShapePtr getSafeBoundingBox(cnoid::SgNode* model, cnoid::Vector3 boundingBoxSafetySize){
        	cnoid::SgShapePtr shape = new cnoid::SgShape();
        	cnoid::SgMeshPtr mesh = shape->setMesh(new cnoid::SgMesh());
		cnoid::SgMeshPtr modelMesh = ColdetConverter::ExtractMesh(model);
		if (!modelMesh->hasVertices()) return shape;
		modelMesh->updateBoundingBox();
		cnoid::BoundingBox bbox = modelMesh->boundingBox();
		cnoid::Vector3 min = bbox.min() - boundingBoxSafetySize;
		cnoid::Vector3 max = bbox.max() + boundingBoxSafetySize;
        	cnoid::SgVertexArrayPtr vertices = mesh->setVertices(new cnoid::SgVertexArray());
        	vertices->resize(8);
		for (int i = 0; i < 8; i++) {
			cnoid::Vector3 temp;
			if( (i&1) == 0) temp[0] = min[0];
			else temp[0] = max[0];
			if( (i&2) == 0) temp[1] = min[1];
			else temp[1] = max[1];
			if( (i&4) == 0) temp[2] = min[2];
			else temp[2] = max[2];
        		(*vertices)[i] << temp[0], temp[1], temp[2];
		}

		mesh->addTriangle(0,1,2);
		mesh->addTriangle(1,3,2);
		mesh->addTriangle(5,4,7);
		mesh->addTriangle(4,6,7);
		mesh->addTriangle(0,2,4);
		mesh->addTriangle(2,6,4);
		mesh->addTriangle(1,5,3);
		mesh->addTriangle(5,7,3);
		mesh->addTriangle(0,4,1);
		mesh->addTriangle(4,5,1);
		mesh->addTriangle(2,3,6);
		mesh->addTriangle(3,7,6);

        	mesh->texCoordIndices() = mesh->triangleVertices();
		
		return shape;
	}
#endif
	cnoid::BodyItemPtr bodyItem1,bodyItem2;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	std::vector<cnoid::ColdetLinkPairPtr> coldetLinkPairs;
#else
	std::vector<grasp::ColdetLinkPairPtr> coldetLinkPairs;
#endif
//	cnoid::Vector3 boundingBoxSafetySize;
};

class EXCADE_API LinkRoughClearance
{
	public:
		struct OffsetBoundingBox{
			double tolerance;
			cnoid::ColdetModelPtr boundingBox;
			std::vector<clearance::Edge> edges;
		};
		struct ColdetLink{
			cnoid::Link *link;
			std::vector<cnoid::ColdetModelPairPtr> coldetModelList; 
		};
		struct ColdetBodyItem{
			cnoid::BodyItemPtr bodyItem;
			std::vector< ColdetLink > coldetLinkList;
		};
		typedef boost::shared_ptr<ColdetBodyItem> ColdetBodyItemPtr;

		LinkRoughClearance(cnoid::Link* link){
			coldetBodyItemList.clear();
			double offsetsize[] = {0.04,0.02,0.01,0.005};
			this->link = link;
			for(int i=0;i<4;i++){
				double tol = offsetsize[i];
				OffsetBoundingBox bb;
				bb.tolerance = tol;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				//bb.boundingBox = ColdetPairData::getSafeBoundingBox(link->coldetModel(), cnoid::Vector3(tol,tol,tol));
				bb.boundingBox = grasp::clearance::ClearanceBoundingVolume::getSafeBoundingBox(link->coldetModel(), cnoid::Vector3(tol,tol,tol), bb.edges);
#else
				bb.boundingBox = grasp::clearance::ClearanceBoundingVolume::getSafeBoundingBox(link->collisionShape(), cnoid::Vector3(tol,tol,tol), bb.edges);
#endif
				toleranceBoundingBoxList.push_back(bb);
//				if(link->index() == 1 &&i==2){
				if(i==3){
					viewColdetModel(bb.boundingBox,link->p(),link->R());
				}
			}
		}
		
		void initialRoughClearance(std::list<cnoid::BodyItemPtr>itemlist){
			testPairs.clear();
			for( std::list<cnoid::BodyItemPtr>::iterator it = itemlist.begin(); it !=itemlist.end(); it++){
				cnoid::BodyItemPtr bodyItem2= *it;
				bool find=false;
				for(int i=0;i<coldetBodyItemList.size();i++){
					if(coldetBodyItemList[i]->bodyItem == bodyItem2){
						find = true;
						testPairs.insert( testPairs.end(), coldetBodyItemList[i]->coldetLinkList.begin(), coldetBodyItemList[i]->coldetLinkList.end() );
					}
				}
				if(find) continue;
				ColdetBodyItemPtr cbody = boost::make_shared<ColdetBodyItem>();
				cbody->bodyItem = bodyItem2;
				for(int i=0;i<bodyItem2->body()->numLinks();i++){
					ColdetLink mplist;
					mplist.link = bodyItem2->body()->link(i);
					for(int j=0;j< toleranceBoundingBoxList.size();j++){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
						cnoid::ColdetModelPairPtr temp = boost::make_shared<cnoid::ColdetModelPair>(toleranceBoundingBoxList[j].boundingBox, bodyItem2->body()->link(i)->coldetModel() );

#else
						cnoid::ColdetModelPtr tmpModel = ColdetModelGetter::get(bodyItem2->body()->link(i));
						cnoid::ColdetModelPairPtr temp = createColdetModelPair(toleranceBoundingBoxList[j].boundingBox, tmpModel);
#endif
						mplist.coldetModelList.push_back(temp);
					}
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
					cnoid::ColdetModelPairPtr temp = boost::make_shared<cnoid::ColdetModelPair>(link->coldetModel(), bodyItem2->body()->link(i)->coldetModel() );
#else
					cnoid::ColdetModelPairPtr temp = createColdetModelPair(
					ColdetModelGetter::get(link),
					ColdetModelGetter::get(bodyItem2->body()->link(i)));
#endif
					if(0){
						temp->model(0)->setPosition(link->T());
						temp->model(1)->setPosition(bodyItem2->body()->link(i)->T());
						int t1,t2;
						double p1[3],p2[3];
						double distance = temp->computeDistance(t1,p1,t2,p2);
						if(distance <1.0e-04){
							std::cout <<"collide at initial position" <<link->name() << " " << bodyItem2->name() << ":"<< bodyItem2->body()->link(i)->name() << std::endl;
							continue;
						}
					}
					mplist.coldetModelList.push_back(temp);
					cbody->coldetLinkList.push_back(mplist);
				}
				coldetBodyItemList.push_back(cbody);
				testPairs.insert( testPairs.end(), cbody->coldetLinkList.begin(), cbody->coldetLinkList.end() );
			}
		}
		double roughClearance(bool initial){
			int start= (initial) ? 0 : preLevel;
			if( initial ||  (preR != link->R())  || (prep != link->p())  ){ 
				for(int i=0; i< testPairs.size(); i++){
					for(int j=start;j<testPairs[i].coldetModelList.size();j++){
						cnoid::ColdetModelPairPtr c = testPairs[i].coldetModelList[j];
						c->model(0)->setPosition(link->T());
						c->model(1)->setPosition(testPairs[i].link->T());
						bool col = c->checkCollision();
						if(col){
							if(j==testPairs[i].coldetModelList.size()-1){
								return 0;
							}
							start = j+1;
						}
						else{
							break;
						}
					}
				}
			}
			preR = link->R();
			prep = link->p();
			preLevel = start;
			if(start < toleranceBoundingBoxList.size() ){
				return toleranceBoundingBoxList[start].tolerance;
			}else{
//				return toleranceBoundingBoxList[start-1].tolerance/2.0;
				return 0;
			}
			
		}
		double sweepRoughClearance(){
			int start=preLevel;
			if( (preR != link->R())  || (prep != link->p()) ){ 
				//cnoid::ColdetModelPtr sweepbb = getSweepSafeBoundingBox(temp->model(0), preR, prep, link->R(), link->p());
				cnoid::ColdetModelPtr sweepbb = getSweepSafeBoundingVolume(toleranceBoundingBoxList[start].boundingBox,toleranceBoundingBoxList[start].edges, preR, prep, link->R(), link->p());
				for(int i=0; i< testPairs.size(); i++){
					for(int j=start;j<testPairs[i].coldetModelList.size()-1;j++){
						cnoid::ColdetModelPairPtr temp = testPairs[i].coldetModelList[j];
						temp->model(0)->setPosition(link->T());
						temp->model(1)->setPosition(testPairs[i].link->T());
						bool col0 = temp->checkCollision();
						cnoid::ColdetModelPairPtr c = createColdetModelPair(sweepbb, temp->model(1) );
						c->model(0)->setPosition(cnoid::Position::Identity());
						c->model(1)->setPosition(testPairs[i].link->T());
						bool col = c->checkCollision() || col0;
//						bool col = col0;
						if(col){
							start = j+1;
							return 0;
							break;
						}else{
							if(link->name()== "HAND_R"){
								viewColdetModel(sweepbb,cnoid::Vector3(0,0,0),cnoid::Matrix3::Identity());
							}
							break;
						}
					}
				}
			}
			preR = link->R();
			prep = link->p();
			preLevel = start;
			if(start < toleranceBoundingBoxList.size() ){
				return toleranceBoundingBoxList[start].tolerance;
			}else{
//				return toleranceBoundingBoxList[start-1].tolerance/2.0;
				return 0;
			}
		}
		
		double sweepRoughClearanceFast(int id){
			const int start=preLevel;
			LinkStateNode node;
			LinkStateNode* mainp;
			node.p=link->p();
			node.R=link->R();
			node.id = id;
			
			switch (id){
				case 0:
					linkStateNodeList.clear();
				case 1:
					linkStateNodeList.push_back(node);
					return 0;
				case 2:
					node.parents[0]=0;
					node.parents[1]=1;
					mainp = &linkStateNodeList[0];
					node.belongNode = 2;
					break;
				default: 
					if( id%2 ){
						node.parents[1] = (id+1)/2;
						mainp = &linkStateNodeList[node.parents[1]];
						mainp->children[0] = id;
						node.parents[0] = mainp->parents[0];
					}
					else{
						node.parents[0] = id/2;
						mainp = &linkStateNodeList[node.parents[0]];
						mainp->children[1] = id;
						node.parents[1] = mainp->parents[1];
					}
					if(mainp->belongNode == -1){
						node.belongNode=id;
					}else{
						node.belongNode = mainp->belongNode;
					}
			}
			linkStateNodeList.push_back(node); // pointer will change
			LinkStateNode* bnode = &linkStateNodeList[node.belongNode];
			LinkStateNode* pp0 =  &linkStateNodeList[bnode->parents[0]];
			LinkStateNode* pp1 =  &linkStateNodeList[bnode->parents[1]];

			cnoid::ColdetModelPtr model;
			double tolerance;
			if(start+1 < toleranceBoundingBoxList.size() ){
				model = toleranceBoundingBoxList[start+1].boundingBox;
				tolerance = toleranceBoundingBoxList[start].tolerance - toleranceBoundingBoxList[start+1].tolerance;
			}else{
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				model = link->coldetModel();
#else
				model = ColdetModelGetter::get(link);
#endif
				tolerance = toleranceBoundingBoxList.back().tolerance/2.0;
			}
			bool isInside = isInsideSweep(model, tolerance, link->R(), link->p(), pp0->R, pp0->p, pp1->R, pp1->p); 

			
			if(!isInside){
				std::vector<int>nodelist;
				nodelist.push_back(node.belongNode);
				while(nodelist.size() >0){
					LinkStateNode* tempnode = &linkStateNodeList[nodelist.back()];
					nodelist.pop_back();
					cnoid::Position T;
					T.linear() = tempnode->R;
					T.translation() = tempnode->p;
					
					for(int i=0; i< testPairs.size(); i++){
						cnoid::ColdetModelPairPtr temp = testPairs[i].coldetModelList[start];
						temp->model(0)->setPosition(T);
						temp->model(1)->setPosition(testPairs[i].link->T());
						bool col = temp->checkCollision();
						if(col){
							return 0;
						}
						if(link->name()== "HAND_R"){
							//viewColdetModel(temp->model(0),tempnode->p,tempnode->R);
						}
					}
					tempnode->belongNode=-1;
					if(tempnode->children[0] != -1) 	nodelist.push_back(tempnode->children[0]);
					if(tempnode->children[1] != -1) 	nodelist.push_back(tempnode->children[1]);
				}
			}
				
			if(start < toleranceBoundingBoxList.size() ){
				return toleranceBoundingBoxList[start].tolerance;
			}else{
				return 0;
			}
		}
		
		double sweepRoughClearanceFastTest(){
			const int start=preLevel;
			std::vector<int>nodelist;
			nodelist.push_back(2);
			while(nodelist.size() >0){
				LinkStateNode* tempnode = &linkStateNodeList[nodelist.back()];
				nodelist.pop_back();
				if(tempnode->id != tempnode->belongNode){
					if(tempnode->children[0] != -1) 	nodelist.push_back(tempnode->children[0]);
					if(tempnode->children[1] != -1) 	nodelist.push_back(tempnode->children[1]);
					continue;
				}
				LinkStateNode* pp0 =  &linkStateNodeList[tempnode->parents[0]];
				LinkStateNode* pp1 =  &linkStateNodeList[tempnode->parents[1]];
				cnoid::ColdetModelPtr sweepbb = getSweepSafeBoundingVolume(toleranceBoundingBoxList[start].boundingBox,toleranceBoundingBoxList[start].edges, pp0->R, pp0->p, pp1->R, pp1->p);
				for(int i=0; i< testPairs.size(); i++){
					cnoid::ColdetModelPairPtr temp = testPairs[i].coldetModelList[start];
					cnoid::ColdetModelPairPtr c = createColdetModelPair(sweepbb, temp->model(1) );
					c->model(0)->setPosition(cnoid::Position::Identity());
					c->model(1)->setPosition(testPairs[i].link->T());
					bool col = c->checkCollision();
					if(col){
						return 0;
					}
				}
				if(link->name()== "HAND_R"){
					viewColdetModel(sweepbb,cnoid::Vector3(0,0,0),cnoid::Matrix3::Identity());
				}
			}
			if(start < toleranceBoundingBoxList.size() ){
				return toleranceBoundingBoxList[start].tolerance;
			}else{
				return 0;
			}
		}
		
		cnoid::ColdetModelPtr getSweepSafeBoundingBox(cnoid::ColdetModelPtr model, cnoid::Matrix3 preR, cnoid::Vector3 prep, cnoid::Matrix3 R, cnoid::Vector3 p){

			cnoid::ColdetModelPtr safeBoundingBox = createColdetModel();
			int nVerticies = model->getNumVertices();
			if(nVerticies != 8) {
				safeBoundingBox.reset();
				return safeBoundingBox;
			}
			float out_x, out_y, out_z;
			cnoid::Vector3 vertex[8];
			for(int i=0;i<nVerticies;i++){
				model->getVertex(i, out_x, out_y, out_z);
				vertex[i] = cnoid::Vector3(out_x, out_y, out_z);
			}

			for(int i=0;i<8;i++){
				cnoid::Vector3 tmp = preR*vertex[i]+prep;
				safeBoundingBox->addVertex(tmp[0],tmp[1],tmp[2]);
			}
			for(int i=0;i<8;i++){
				cnoid::Vector3 tmp = R*vertex[i]+p;
				safeBoundingBox->addVertex(tmp[0],tmp[1],tmp[2]);
			}
			int edges[12][2];
			edges[0][0] =0; edges[0][1] =1;
			edges[1][0] =0; edges[1][1] =2;
			edges[2][0] =1; edges[2][1] =3;
			edges[3][0] =2; edges[3][1] =3;
			for(int i=0;i<4;i++){
				edges[i+4][0] = edges[i][0]+4;
				edges[i+4][1] = edges[i][1]+4;
			}
			for(int i=0;i<4;i++){
				edges[i+8][0] = i;
				edges[i+8][1] = i+4;
			}
			for(int i=0;i<12;i++){
				safeBoundingBox->addTriangle(edges[i][0],edges[i][1],edges[i][0]+8);
				safeBoundingBox->addTriangle(edges[i][1],edges[i][1]+8,edges[i][0]+8);
			}
			safeBoundingBox->build();
		
			return safeBoundingBox;
		}
		
		cnoid::ColdetModelPtr getSweepSafeBoundingVolume(cnoid::ColdetModelPtr model, std::vector<clearance::Edge>edges, cnoid::Matrix3 preR, cnoid::Vector3 prep, cnoid::Matrix3 R, cnoid::Vector3 p){

			cnoid::ColdetModelPtr safeBoundingBox = createColdetModel();
			int nVerticies = model->getNumVertices();
			if(nVerticies == 0) {
				safeBoundingBox.reset();
				return safeBoundingBox;
			}
			float out_x, out_y, out_z;
			std::vector<cnoid::Vector3> vertex;
			for(int i=0;i<nVerticies;i++){
				model->getVertex(i, out_x, out_y, out_z);
				vertex.push_back(cnoid::Vector3(out_x, out_y, out_z));
			}

			for(int i=0;i<nVerticies;i++){
				cnoid::Vector3 tmp = preR*vertex[i]+prep;
				safeBoundingBox->addVertex(tmp[0],tmp[1],tmp[2]);
			}
			for(int i=0;i<nVerticies;i++){
				cnoid::Vector3 tmp = R*vertex[i]+p;
				safeBoundingBox->addVertex(tmp[0],tmp[1],tmp[2]);
			}
			
			for(int i=0;i<edges.size();i++){
//				bool detect = true;
				int id0 = edges[i].id[0];
				int id1 = edges[i].id[1];
				cnoid::Vector3 e1 = preR*vertex[id1] - preR*vertex[id0];
				cnoid::Vector3 e2 = R*vertex[id1] - R*vertex[id0];
				cnoid::Vector3 bnml[4];
				bnml[0] = e1.cross( (R*vertex[id0]+p) - (preR*vertex[id0]+prep) );
				bnml[1] = e1.cross( (R*vertex[id1]+p) - (preR*vertex[id0]+prep) );
				bnml[2] = e2.cross( (R*vertex[id0]+p) - (preR*vertex[id0]+prep) );
				bnml[3] = e2.cross( (R*vertex[id0]+p) - (preR*vertex[id1]+prep) );
				
				cnoid::Vector3 s1[6];
				cnoid::Vector3 s2[6];

				s1[0] = e1.cross( preR*edges[i].nml[0] );
				s2[0] = e1.cross( preR*edges[i].nml[1] );
				s1[1] = e2.cross( R*edges[i].nml[0] );
				s2[1] = e2.cross( R*edges[i].nml[1] );
				
				s1[2] = e1.cross( preR*edges[i].nml[0] );
				s2[2] = e1.cross( R*edges[i].nml[0] );
				s1[3] = e1.cross( preR*edges[i].nml[1] );
				s2[3] = e1.cross( R*edges[i].nml[1] );

				s1[4] = e2.cross( preR*edges[i].nml[0] );
				s2[4] = e2.cross( R*edges[i].nml[0] );
				s1[5] = e2.cross( preR*edges[i].nml[1] );
				s2[5] = e2.cross( R*edges[i].nml[1] );

				bool detect=false;
				for(int j=0;j<4;j++){
					for(int k=0;k<6;k++){
						if( (bnml[j].dot(s1[k])*bnml[j].dot(s2[k]) ) <= 0 ) detect = true;
					}
				}
				
				if(detect){
					safeBoundingBox->addTriangle(id0,id1,id0+nVerticies);
					safeBoundingBox->addTriangle(id1,id1+nVerticies,id0+nVerticies);
				}
			}
			safeBoundingBox->build();
		
			return safeBoundingBox;
		}
		
		bool isInsideSweep(cnoid::ColdetModelPtr model, double torelance, cnoid::Matrix3 R, cnoid::Vector3 p, cnoid::Matrix3 pR0, cnoid::Vector3 pp0, cnoid::Matrix3 pR1, cnoid::Vector3 pp1){

			int nVerticies = model->getNumVertices();
			float out_x, out_y, out_z;
			for(int i=0;i<nVerticies;i++){
				model->getVertex(i, out_x, out_y, out_z);
				cnoid::Vector3 v(out_x, out_y, out_z);
				cnoid::Vector3 vt = R*v+p;
				cnoid::Vector3 v0 = pR0*v+pp0;
				cnoid::Vector3 v1 = pR1*v+pp1;
				if( ( (vt-v0).cross( (v1-v0).normalized() ) ).norm() > torelance){
					//std::cout << "outside" << std::endl;
					return false;
				}
			}
			return true;
		}
		
		
		void viewColdetModel(cnoid::ColdetModelPtr model, cnoid::Vector3 p, cnoid::Matrix3 R){
#ifdef DEBUG_VIEW_MODE
			int nVerticies = model->getNumVertices();
			
			cnoid::SgVertexArrayPtr vertices = new cnoid::SgVertexArray();
			vertices->reserve(nVerticies);
			cnoid::SgMeshPtr sgMesh = new cnoid::SgMesh();

			float out_x, out_y, out_z;
			for(int i=0;i<nVerticies;i++){
				model->getVertex(i, out_x, out_y, out_z);
				cnoid::Vector3 temp(out_x, out_y, out_z);
				cnoid::Vector3 temp2 = R*temp+p;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				cnoid::SgVector3 sgvertex = cnoid::SgVector3(temp2[0],temp2[1],temp2[2]);
				vertices->push_back(sgvertex);
#else
				vertices->push_back(cnoid::Vector3f(temp2[0],temp2[1],temp2[2]));
#endif
			}
			sgMesh->setVertices(vertices);
			
			int t0,t1,t2;
			for(int i=0;i<model->getNumTriangles();i++){
				model->getTriangle(i,t0,t1,t2);
				sgMesh->addTriangle(t0,t1,t2);
			}
			if(sgMesh){
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				sgMesh->generateNormals(3.141592);
#else
				cnoid::MeshNormalGenerator meshGen;
				meshGen.generateNormals((cnoid::SgMesh*)sgMesh);
#endif
				cnoid::SgShapePtr shape = new cnoid::SgShape;
				cnoid::SgMaterialPtr material= new cnoid::SgMaterial;
				material->setTransparency(0.5);
				shape->setMesh(sgMesh);
				shape->setMaterial(material);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
				cnoid::SceneView::instance()->addEntity(shape);
#else
				cnoid::SceneView::instance()->scene()->addChild(shape, true);
#endif
			}
#endif
		}
		void setStartPosition(){
			preR = link->R();
			prep = link->p();
		}
		cnoid::Link* link;
		cnoid::Matrix3 preR;
		cnoid::Vector3 prep;
		int preLevel;
		std::vector<OffsetBoundingBox> toleranceBoundingBoxList;
		std::vector< ColdetBodyItemPtr >  coldetBodyItemList;
		std::vector< ColdetLink > testPairs;
		std::vector < LinkStateNode > linkStateNodeList;
	
};

}

#endif

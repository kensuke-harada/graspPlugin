

#ifndef _CLEARNCE_BOUNDING_VOLUME_H
#define _CLEARNCE_BOUNDING_VOLUME_H

#include <vector>

#include <cnoid/BodyItem>
#include <boost/make_shared.hpp>

#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
#include <cnoid/SceneShape>
#include "ColdetConverter.h"
#endif

#include "exportdef.h"
#include "InterObject.h"
#include "UtilFunction.h"

namespace grasp{

namespace clearance{	
	
struct EXCADE_API Vertex{
	cnoid::Vector3 pos;
	Vertex* child;
	Vertex* parents[2];
	bool inside;
	int id;
};

	
struct EXCADE_API Polygon{
	std::list<Vertex*> verticies;
	cnoid::Vector3 nml;
};

struct EXCADE_API Edge{
	int id[2];
	cnoid::Vector3 pos[2];
	cnoid::Vector3 nml[2];
};

	
class EXCADE_API ClearanceBoundingVolume{
public:
	ClearanceBoundingVolume(){ }
	~ClearanceBoundingVolume(){  }

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	static cnoid::ColdetModelPtr getSafeBoundingBox(cnoid::ColdetModelPtr model, cnoid::Vector3 boundingBoxSafetySize, std::vector<Edge>& edges){
#else
	static cnoid::ColdetModelPtr getSafeBoundingBox(cnoid::SgNode* model, cnoid::Vector3 boundingBoxSafetySize, std::vector<Edge>& edges){
#endif
		double safetyRangeMin = boundingBoxSafetySize[0];
		double safetyRangeMax = safetyRangeMin * 1.5;
		cnoid::ColdetModelPtr safeBoundingBox = createColdetModel();

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		int nVerticies = model->getNumVertices();
  #ifdef  CNOID_10_11_12_13
		if(nVerticies == 0) return NULL;
  #else
		if(nVerticies == 0) {
			safeBoundingBox.reset();
			return safeBoundingBox;
		}
  #endif
#else
		cnoid::SgMeshPtr modelMesh = ColdetConverter::ExtractMesh(model);
		int nVerticies = modelMesh->vertices()->size();
		if(nVerticies == 0) return safeBoundingBox;
#endif

		std::vector<cnoid::Vector3> mvertex;

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
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
			mvertex.push_back(cnoid::Vector3(out_x,out_y,out_z));
		}
		min -= boundingBoxSafetySize;
		max += boundingBoxSafetySize;
#else
		modelMesh->updateBoundingBox();
		cnoid::BoundingBox bbox = modelMesh->boundingBox();
		cnoid::Vector3 min = bbox.min() - boundingBoxSafetySize;
		cnoid::Vector3 max = bbox.max() + boundingBoxSafetySize;
		for(int i = 0; i < nVerticies; i++){
			cnoid::Vector3f vec = modelMesh->vertices()->at(i);
			mvertex.push_back(cnoid::Vector3(vec[0],vec[1],vec[2]));
		}
#endif
		std::vector<Vertex> bvertex;
		bvertex.reserve(1000);
		for(int i=0;i<8;i++){
			cnoid::Vector3 temp;
			if( (i&1) == 0) temp[0] = min[0];
			else temp[0] = max[0];
			if( (i&2) == 0) temp[1] = min[1];
			else temp[1] = max[1];
			if( (i&4) == 0) temp[2] = min[2];
			else temp[2] = max[2];
			Vertex v;
			v.pos = cnoid::Vector3(temp[0],temp[1],temp[2]);
			v.child = NULL; 
			v.parents[0] = v.parents[1] = NULL;
			v.inside=true;
			v.id = i;
			bvertex.push_back(v);
		}
		
		std::vector<Polygon> polygons;
		//polygons.reserve(1000*2);
		int initial_polygon[6][4] = { {0,1,3,2},{5,4,6,7},{0,2,6,4},{1,5,7,3},{0,4,5,1},{2,3,7,6} };
		for(int i=0;i<6;i++){
			Polygon temp;
			Vertex* v[4];
			for(int j=0;j<4;j++){
				temp.verticies.push_back(&bvertex[initial_polygon[i][j]]);
				v[j] = &bvertex[initial_polygon[i][j]];
			}
			temp.nml = -((v[1]->pos -v[0]->pos).cross(v[2]->pos -v[0]->pos));
			temp.nml.normalize();
			polygons.push_back(temp);
		}
		
		std::vector<int> neighborPolygons;
		for(int i=0;i<6;i++){
			for(int j=0;j<6;j++){
				if( i/2 == j/2) continue;
				if(i>j) continue;
				neighborPolygons.push_back(i);
				neighborPolygons.push_back(j);
			}
		}
		
		int np=0;
		for (int steps=0;steps<100;steps++){
			cnoid::Vector3 cutNml(1,0,0);
			cnoid::Vector3 cutPnt(0,0,0);
			if( steps*2+1 >= neighborPolygons.size() ) break;
			
			cutNml = polygons[neighborPolygons[steps*2]].nml+polygons[neighborPolygons[steps*2+1]].nml;
			cutNml.normalize();
			cutPnt = mvertex[0];
			for(int i=0;i<nVerticies;i++){
				if( mvertex[i].dot(cutNml) > cutPnt.dot(cutNml) ) cutPnt = mvertex[i];
			}
			cutPnt += safetyRangeMin*cutNml;
			
			double dist=0;
			for(int i=0;i<bvertex.size();i++){
				if( !bvertex[i].inside) continue;
				if( (bvertex[i].pos.dot(cutNml) -  cutPnt.dot(cutNml)) > dist){
					dist = bvertex[i].pos.dot(cutNml) -  cutPnt.dot(cutNml);
				}
			}
			//std::cout << "dist "<< dist << std::endl;
			if(dist < (safetyRangeMax-safetyRangeMin) ) continue;
			//std::cout << bvertex.size() << std::endl;

			for(int i=0;i<bvertex.size();i++){
				if(bvertex[i].pos.dot(cutNml) > cutPnt.dot(cutNml) ){
					bvertex[i].inside=false;
				}
			}

			
			//std::cout << steps << std::endl;
			
			std::vector<Vertex*>newVertex;
			
			std::vector<int>newNeighbor;
		
			for(int i=0;i<polygons.size();i++){
				Vertex* pre= polygons[i].verticies.back();
				std::vector<Vertex*>outside;
				Vertex *v1=NULL, *v2=NULL;
				for( std::list<Vertex*>::iterator it = polygons[i].verticies.begin(); it != polygons[i].verticies.end(); it++ ){
					if(pre->inside != (*it)->inside){
						Vertex* child=NULL;
						for(int j=0;j<newVertex.size();j++){
							if( (newVertex[j]->parents[1]==pre) && (newVertex[j]->parents[0]==*it) ) child =  newVertex[j];	
						}
						if(!child){
							Vertex v;
							double a = fabs(cutNml.dot(pre->pos-cutPnt));
							double b = fabs(cutNml.dot((*it)->pos-cutPnt));
							v.pos = (b*pre->pos+a*(*it)->pos)/(a+b);
							//std::cout << v.pos.transpose() << std::endl;
							v.child=NULL;
							v.inside = true;
							bvertex.push_back(v);
							child = &bvertex.back();
							newVertex.push_back(child);
							pre->child = (*it)->child = child;
							child->parents[0] = pre;
							child->parents[1] = (*it);
						}
						polygons[i].verticies.insert(it, child);
						if(pre->inside) v1=child;
						else	v2=child;
					}
					if(!(*it)->inside) outside.push_back(*it);
					pre = *it;
				}
				if(v1 && v2){
					v2->child = v1;
					newNeighbor.push_back(i);
				}
				//std::cout <<"v1 v2 " << v1 << " " <<  v2 << std::endl;
				for(int j=0;j<outside.size();j++){
					polygons[i].verticies.remove(outside[j]);
				}
			}
			if(!newVertex.empty()){
				for(int i=0;i<newVertex.size();i++){
					//std::cout <<"newver " << newVertex[i] << " " << newVertex[i]->child << std::endl;
				}
				Vertex* v0 = newVertex[0];
				Polygon p;
				p.nml = cutNml;
				do{
					p.verticies.push_back(v0);
					v0 = v0->child;
				}while(v0 != newVertex[0]);				
				polygons.push_back(p);
				for(int i=0;i<newNeighbor.size();i++){
					neighborPolygons.push_back(polygons.size()-1);
					neighborPolygons.push_back(newNeighbor[i]);
				}
			}
		}
		
		int id=0;
		for(int i=0;i<bvertex.size();i++){
			if(!bvertex[i].inside) continue;
			bvertex[i].id = id++;
			cnoid::Vector3 pos= bvertex[i].pos;
			safeBoundingBox->addVertex(pos[0],pos[1],pos[2]);
		}
		int ecnt=0;
		for(int i=0;i<polygons.size();i++){
			if(polygons[i].verticies.size() == 0) continue;
			ecnt += polygons[i].verticies.size();
			std::list<Vertex*>::iterator it = polygons[i].verticies.begin();
			Vertex *pivot= *it;
			it++;
			Vertex *first = *it;
			for( ; it != polygons[i].verticies.end(); it++ ){
				//std::cout <<  pivot->id << " " << first->id <<" " << (*it)->id <<  std::endl;
				safeBoundingBox->addTriangle(pivot->id,first->id,(*it)->id);
				first = *it;
			}
		}
		
		for(int i=0;i<neighborPolygons.size();i+=2){
			Polygon* p1= &polygons[neighborPolygons[i]];
			Polygon* p2= &polygons[neighborPolygons[i+1]];
			if(p1->verticies.size()==0) continue;
			if(p2->verticies.size()==0) continue;
			
			if( p1->verticies.size() < 3) std::cout << "n ver of poly"<< p1->verticies.size() << std::endl; 
			if( p2->verticies.size() < 3) std::cout << "n ver of poly" << p2->verticies.size() << std::endl; 
			
			Vertex *v1 = (p1->verticies.back());
			for(std::list<Vertex*>::iterator it1 = p1->verticies.begin() ; it1 != p1->verticies.end(); it1++ ){
				Vertex *v2 = (p2->verticies.back());
				for(std::list<Vertex*>::iterator it2 = p2->verticies.begin() ; it2 != p2->verticies.end(); it2++ ){
					if( (v1==*it2) && (v2==*it1) ){
						Edge e;
						e.id[0] = v1->id;
						e.id[1] = v2->id;
						e.pos[0] = v1->pos;
						e.pos[1] = v2->pos;
						e.nml[0] = p1->nml;
						e.nml[1] = p2->nml;
						edges.push_back(e);
					}
					v2=*it2;
				}
				v1 = *it1;
			}
		}

		std::cout << "ver "<< id << " tri " << safeBoundingBox->getNumTriangles() <<  " edge "<<  edges.size()  << " " << ecnt << std::endl;
		safeBoundingBox->build();

		return safeBoundingBox;
	}
	
	
//	cnoid::Vector3 boundingBoxSafetySize;
};


}


}

#endif

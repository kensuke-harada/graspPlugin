#include "ShapeElements.h"

using namespace grasp;

bool VertexLink::isCorner(int id){
	int temp=-1;
	
	for(unsigned int i=0; i<tlist.size();i++){
		if( tlist[i]->idCluster ==id) continue;
		if(temp == -1) temp = tlist[i]->idCluster;
		if(temp != tlist[i]->idCluster) return true;
	}
	return false;
}


double Triangle::pointInTriangleDistance(const cnoid::Vector3& p){
	double eps = min( min( norm2(ver[0]->pos -ver[1]->pos),  norm2(ver[1]->pos -ver[2]->pos) ), norm2(ver[2]->pos -ver[0]->pos) );
	eps = (eps > 0.001) ? eps : 0.001;
	double distance = (norm2(ver[0]->pos -ver[1]->pos) + norm2(ver[1]->pos -ver[2]->pos) + norm2(ver[2]->pos -ver[0]->pos) )*3.0;

	for(int i=0; i<3; i++){
			
			cnoid::Vector3 q1, q2;
			if(minDistancePoints(ver[i%3]->pos, cnoid::Vector3(p - ver[i%3]->pos), ver[(i+1)%3]->pos, cnoid::Vector3(ver[(i+2)%3]->pos - ver[(i+1)%3]->pos), q1, q2) > eps)
					return -1.0;

			double d0 = norm2(q1 - ver[i%3]->pos) - norm2(p - ver[i%3]->pos);

			if(d0 < distance)
					distance = d0; 
	}

	return distance;
}

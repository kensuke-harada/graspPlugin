#include "GeometryHandle.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include <list>

using namespace std;
using namespace cnoid;
using namespace grasp;


OverlapVolumeCalculator::BBox::BBox(Vector3 _edge)
{
	edge = _edge;
	v[0] = 0.5*Vector3(   edge(0),   edge(1),   edge(2));
	v[1] = 0.5*Vector3(   edge(0),   edge(1),-1*edge(2));
	v[2] = 0.5*Vector3(   edge(0),-1*edge(1),   edge(2));
	v[3] = 0.5*Vector3(   edge(0),-1*edge(1),-1*edge(2));
	v[4] = 0.5*Vector3(-1*edge(0),   edge(1),   edge(2));
	v[5] = 0.5*Vector3(-1*edge(0),   edge(1),-1*edge(2));
	v[6] = 0.5*Vector3(-1*edge(0),-1*edge(1),   edge(2));
	v[7] = 0.5*Vector3(-1*edge(0),-1*edge(1),-1*edge(2));

	l[0].p = v[0]; l[0].n = Vector3(0,0,1); l[0].v[0] = v[0]; l[0].v[1] = v[1];
	l[1].p = v[1]; l[1].n = Vector3(0,1,0); l[1].v[0] = v[1]; l[1].v[1] = v[3];
	l[2].p = v[3]; l[2].n = Vector3(0,0,1); l[2].v[0] = v[2]; l[2].v[1] = v[3];
	l[3].p = v[2]; l[3].n = Vector3(0,1,0); l[3].v[0] = v[0]; l[3].v[1] = v[2];
	l[4].p = v[0]; l[4].n = Vector3(1,0,0); l[4].v[0] = v[0]; l[4].v[1] = v[4];
	l[5].p = v[1]; l[5].n = Vector3(1,0,0); l[5].v[0] = v[1]; l[5].v[1] = v[5];
	l[6].p = v[3]; l[6].n = Vector3(1,0,0); l[6].v[0] = v[3]; l[6].v[1] = v[7];
	l[7].p = v[2]; l[7].n = Vector3(1,0,0); l[7].v[0] = v[2]; l[7].v[1] = v[6];
	l[8].p = v[4]; l[8].n = Vector3(0,0,1); l[8].v[0] = v[4]; l[8].v[1] = v[5];
	l[9].p = v[5]; l[9].n = Vector3(0,1,0); l[9].v[0] = v[5]; l[9].v[1] = v[7];
	l[10].p = v[7]; l[10].n = Vector3(0,0,1); l[10].v[0] = v[6]; l[10].v[1] = v[7];
	l[11].p = v[6]; l[11].n = Vector3(0,1,0); l[11].v[0] = v[4]; l[11].v[1] = v[6];

	p[0].p = v[0]; p[0].n = Vector3(1,0,0);
	p[0].l[0] = &l[0]; p[0].l[1] = &l[1]; p[0].l[2] = &l[2]; p[0].l[3] = &l[3];
	p[0].v[0] = v[0]; p[0].v[1] = v[1]; p[0].v[2] = v[2]; p[0].v[3] = v[3]; 
	p[1].p = v[4]; p[1].n = Vector3(0,1,0);
	p[1].l[0] = &l[0]; p[1].l[1] = &l[4]; p[1].l[2] = &l[5]; p[1].l[3] = &l[8];
	p[1].v[0] = v[0]; p[1].v[1] = v[1]; p[1].v[2] = v[4]; p[1].v[3] = v[5];
	p[2].p = v[6]; p[2].n = Vector3(-1,0,0);
	p[2].l[0] = &l[8]; p[2].l[1] = &l[9]; p[2].l[2] = &l[10]; p[2].l[3] = &l[11];
	p[2].v[0] = v[4]; p[2].v[1] = v[5]; p[2].v[2] = v[6]; p[2].v[3] = v[7];
	p[3].p = v[2]; p[3].n = Vector3(0,-1,0);
	p[3].l[0] = &l[2]; p[3].l[1] = &l[6]; p[3].l[2] = &l[7]; p[3].l[3] = &l[10];
	p[3].v[0] = v[2]; p[3].v[1] = v[3]; p[3].v[2] = v[6]; p[3].v[3] = v[7];
	p[4].p = v[6]; p[4].n = Vector3(0,0,1);
	p[4].l[0] = &l[3]; p[4].l[1] = &l[4]; p[4].l[2] = &l[7]; p[4].l[3] = &l[11];
	p[4].v[0] = v[0]; p[4].v[1] = v[2]; p[4].v[2] = v[4]; p[4].v[3] = v[6];
	p[5].p = v[5]; p[5].n = Vector3(0,0,-1);
	p[5].l[0] = &l[1]; p[5].l[1] = &l[5]; p[5].l[2] = &l[6]; p[5].l[3] = &l[9];
	p[5].v[0] = v[1]; p[5].v[1] = v[3]; p[5].v[2] = v[5]; p[5].v[3] = v[7];
}

double OverlapVolumeCalculator::calcOverlapVolume(ClusterImpl& c1,ClusterImpl& c2)
{
	if(norm2(c1.bbedge) + norm2(c2.bbedge) < norm2(c1.bbcenter-c2.bbcenter)) return 0;
	if(c2.bbedge(0) < 10e-10 || c2.bbedge(1) < 10e-10 || c2.bbedge(2) < 10e-10) return 0;

	vertices.clear();

	addPolyhedronVertex(c2,c1);

	for(int i=0;i<vertices.size();i++){
		normals[i] = c1.bbR.transpose() * c2.bbR * normals[i];
		for(int j=0;j<vertices[i].size();j++){
			vertices[i][j] = c1.bbR.transpose() * (c2.bbcenter + c2.bbR * vertices[i][j] - c1.bbcenter);
		}
	}
	addPolyhedronVertex(c1,c2);

	double vol = 0;

	if(vertices.size() < 3) return 0;
	
	for(int i=0;i<vertices.size();i++){
		int sign = (normals[i].dot(vertices[i][0]) < 0 ) ? -1 : 1;
		for(int j=1;j<vertices[i].size()-1;j++){
			vol += sign * fabs(dot(vertices[i][0],cross(vertices[i][j],vertices[i][j+1]))/6);
		}
	}
	return vol;
}

void OverlapVolumeCalculator::addPolyhedronVertex(ClusterImpl& c1,ClusterImpl& c2)
{

	BBox b1(c1.bbedge);
	BBox b2(c2.bbedge);

	for(int i=0;i<6;i++){
		std::vector<Vector3> vertex;
		std::vector<bool> isAlive;
		std::vector<Vector3> sorted_vertex;
		vertex.clear();
		isAlive.clear();
		sorted_vertex.clear();
		for(int j=0;j<4;j++){
			Vector3 p = c2.bbR.transpose() * (c1.bbcenter + c1.bbR * b1.p[i].v[j] - c2.bbcenter);
			if((-0.5*c2.bbedge(0) <= p(0) && p(0) <= 0.5*c2.bbedge(0)) &&
				(-0.5*c2.bbedge(1) <= p(1) && p(1) <= 0.5*c2.bbedge(1)) &&
				(-0.5*c2.bbedge(2) <= p(2) && p(2) <= 0.5*c2.bbedge(2))){
					vertex.push_back(b1.p[i].v[j]);
			}
		}


		for(int j=0;j<12;j++){	
			Vector3 p2 = c1.bbR.transpose() * (c2.bbcenter + c2.bbR * b2.l[j].p - c1.bbcenter);
			Vector3 n2 = c1.bbR.transpose() * c2.bbR * b2.l[j].n;
			Vector3 v1 = c1.bbR.transpose() * (c2.bbcenter + c2.bbR * b2.l[j].v[0] - c1.bbcenter);
			Vector3 v2 = c1.bbR.transpose() * (c2.bbcenter + c2.bbR * b2.l[j].v[1] - c1.bbcenter);
			Vector3 intersect_p = intersectionPoint(p2,n2,b1.p[i].p,b1.p[i].n);

			if(isPointOnBox(b1,intersect_p,i,v1,v2)){
				vertex.push_back(intersect_p);
			}

		}

		for(int j=0;j<6;j++){
			Vector3 p2 = b2.p[j].p;
			Vector3 n2 = b2.p[j].n;

			for(int k=0;k<4;k++){
				Vector3 p1 = c2.bbR.transpose() * (c1.bbcenter + c1.bbR * b1.p[i].l[k]->p - c2.bbcenter);
				Vector3 n1 = c2.bbR.transpose() * c1.bbR * b1.p[i].l[k]->n;
				Vector3 v1 = c2.bbR.transpose() * (c1.bbcenter + c1.bbR * b1.p[i].l[k]->v[0] - c2.bbcenter);
				Vector3 v2 = c2.bbR.transpose() * (c1.bbcenter + c1.bbR * b1.p[i].l[k]->v[1] - c2.bbcenter);
				Vector3 intersect_p = intersectionPoint(p1,n1,p2,n2);
				Vector3 intersect_p1 = c1.bbR.transpose() * (c2.bbcenter + c2.bbR * intersect_p - c1.bbcenter);

				if(isPointOnBox(b2,intersect_p,j,v1,v2)){
					vertex.push_back(intersect_p1);
				}
			}
		}


		if(vertex.size() < 3) continue;
		Vector3 tan;
		int axis = 0;
		if(b1.p[i].n(0) > 0){ tan = Vector3(0,1,0); axis = 2;}
		else if(b1.p[i].n(1) > 0){tan = Vector3(0,0,1); axis = 0;}
		else if(b1.p[i].n(2) > 0){tan = Vector3(1,0,0); axis = 1;}

		double min = c1.bbedge(axis);
		int min_index = -1;
		for(int j=0;j<vertex.size();j++){
			if(min > vertex[j](axis)){
				min = vertex[j](axis);
				min_index = j;
			}
		}

		if(min_index == -1) continue;

		for(int j=0;j<vertex.size();j++){
			isAlive.push_back(true);
		}
		sorted_vertex.push_back(vertex[min_index]);
		isAlive[min_index] = false;
		int num_alive = vertex.size() - 1;

		Vector3 ref_n = tan;

		while(num_alive > 0){
			int ref_p_index = min_index;
			double min_angle = -DBL_MAX;
			for(int j=0;j<vertex.size();j++){
				if(!isAlive[j]) continue;
				double angle = dot(ref_n,vertex[j]-vertex[ref_p_index])/(norm2(ref_n)*norm2(vertex[ref_p_index]));
				if(angle > min_angle){
					min_angle = angle;
					min_index = j;
				}
			}

			ref_n = vertex[min_index]-vertex[ref_p_index];
			sorted_vertex.push_back(vertex[min_index]);
			isAlive[min_index] = false;
			num_alive--;
		}
		vertices.push_back(sorted_vertex);
		normals.push_back(b1.p[i].n);
	}

}

bool OverlapVolumeCalculator::isPointOnBox(BBox& box,Vector3& p,int plane_id,Vector3& line_end1,Vector3& line_end2) const
{
	if(fabs(box.p[plane_id].n(0)) > 0){
		if(!(line_end1(0) <= p(0) && p(0) <= line_end2(0) || line_end2(0) <= p(0) && p(0) <= line_end1(0))){
			return false;
		}
		if((-0.5*box.edge(1) <= p(1) && p(1) <= 0.5*box.edge(1)) &&
			(-0.5*box.edge(2) <= p(2) && p(2) <= 0.5*box.edge(2))){
				return true;
		}
	}else if(fabs(box.p[plane_id].n(1)) > 0){
		if(!(line_end1(1) <= p(1) && p(1) <= line_end2(1) || line_end2(1) <= p(1) && p(1) <= line_end1(1))){
			return false;
		}
		if((-0.5*box.edge(0) <= p(0) && p(0) <= 0.5*box.edge(0)) &&
			(-0.5*box.edge(2) <= p(2) && p(2) <= 0.5*box.edge(2))){
				return true;
		}
	}else if(fabs(box.p[plane_id].n(2)) > 0){
		if(!(line_end1(2) <= p(2) && p(2) <= line_end2(2) || line_end2(2) <= p(2) && p(2) <= line_end1(2))){
			return false;
		}
		if((-0.5*box.edge(1) <= p(1) && p(1) <= 0.5*box.edge(1)) &&
			(-0.5*box.edge(0) <= p(0) && p(0) <= 0.5*box.edge(0))){
				return true;
		}
	}
	return false;
}



#ifndef __ShapeElements_H__
#define __ShapeElements_H__

#include <map>
#include "../Grasp/VectorMath.h"


namespace grasp{

class Triangle;
class Vertex;


class Vertex{
public:
	int id;
	cnoid::Vector3 pos;
	cnoid::Vector3 normal;
};


class VertexLink: public Vertex
{
public:
	int check;
	VertexLink *next;
	std::map<int, VertexLink*> nextlist;
	std::vector <Triangle*> tlist;
	int nbrId;

	bool isCorner(int id);
};


class Triangle{
public:
//	Vertex* ver[3];
	VertexLink* ver[3];
	Triangle* nbr[3];
	float area;
	cnoid::Vector3 normal;
	int idCluster;
	int id;

	double pointInTriangleDistance(const cnoid::Vector3& p);
};

}

#endif
#ifndef  __OverlapVolumeCalculator_H__
#define __OverlapVolumeCalculator_H__

#include "Cluster.h"

namespace grasp{

class OverlapVolumeCalculator{
public:
	OverlapVolumeCalculator(){;}
	~OverlapVolumeCalculator(){;}

	double calcOverlapVolume(ClusterImpl& c1,ClusterImpl& c2);
private:
	class BBox{
	public:
		BBox(cnoid::Vector3 _edge);
		class Line{
		public:
			cnoid::Vector3 p;
			cnoid::Vector3 n;
			cnoid::Vector3 v[2];
		};
		class Plane{
		public:
			cnoid::Vector3 p;
			cnoid::Vector3 n;
			Line* l[4];
			cnoid::Vector3 v[4];
		};
		cnoid::Vector3 edge;
		cnoid::Vector3 v[8];
		Line l[12];
		Plane p[6];
	};

	void addPolyhedronVertex(ClusterImpl& c1,ClusterImpl& c2);
	bool isPointOnBox(BBox& box,cnoid::Vector3& intersect_p,int plane_id,cnoid::Vector3& line_end1,cnoid::Vector3& line_end2) const;

	std::vector<std::vector<cnoid::Vector3> > vertices;
	std::vector<cnoid::Vector3> normals;
};

}

#endif
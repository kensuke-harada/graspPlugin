#ifndef _GRASPDATAGENRATOR_CONVEXHULL_H_
#define _GRASPDATAGENRATOR_CONVEXHULL_H_

#include "../Grasp/ConvexAnalysis.h"
#include "exportdef.h"

namespace grasp {
	/**
	* Face of convex hull
	*/
	class EXCADE_API Face {
	public:
		typedef std::vector<cnoid::Vector3> PointSet;
		typedef int                         FaceID;
		typedef std::vector<Face*>          FaceList;
		typedef FaceList::const_iterator    FaceListIterator;
		typedef std::vector<FaceID>         FaceIDList;
		typedef int                         PointID;
		typedef std::vector<PointID>        PointIDList;

		Face(int _id, PointIDList _pids, PointSet* _points) : id(_id), pids(_pids), points(_points) {;}
		virtual ~Face() {;}

		FaceID                   getID() const;
		std::vector<PointIDList> getTrianglePids() const;
		PointIDList              getPids() const;
		PointSet                 getPoints() const;
		cnoid::Vector3           getCenter() const;
		cnoid::Vector3           getNormal() const;
		bool                     isNeighbor(const Face* face, int& pid1, int& pid2) const;

	private:
		FaceID      id;
		PointIDList pids;
		PointSet*   points;
	};

	/**
	* Convex hull
	*/
	class EXCADE_API ConvexHull {
	public:
		ConvexHull();
		virtual ~ConvexHull();

		Face::FaceList getFaces() const;
		int            getFaceSize() const;
		Face*    getFace(Face::FaceID fid) const;
		cnoid::Vector3 getPoint(Face::PointID pid) const;
		void           createConvexHull(std::vector<double>& vertex);
		cnoid::Vector3 getIntersectPoint(const cnoid::Vector3& N, const cnoid::Vector3& com) const;
		void           showConvexHull(const cnoid::Matrix3& R, const cnoid::Vector3& p) const;
		void           showConvexHull(const cnoid::Matrix3& R, const cnoid::Vector3& p, const std::vector<Face::FaceIDList>& fids, const std::vector<cnoid::Vector3>& color) const;
		bool           isStable(Face::FaceID fid, const cnoid::Vector3& com) const;
	private:
		bool           lineIntersectToPlane(const cnoid::Vector3& p, const cnoid::Vector3& n, const Face::PointSet& Pset, cnoid::Vector3& p_out) const;

		Face::FaceList faces;
		Face::PointSet points;
	};

	
}

#endif /* _GRASPDATAGENRATOR_CONVEXHULL_H_ */

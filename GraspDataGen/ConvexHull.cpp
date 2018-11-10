#include "ConvexHull.h"
#include "../Grasp/DrawUtility.h"
#include "../Grasp/VectorMath.h"
#include <vector>

using namespace std;
using namespace cnoid;
using namespace grasp;

/**
* Get faceID.
* @return FaceID
*/
Face::FaceID Face::getID() const {
	return id;
}

/**
* Get point ID list of triangles obtained by dividing this face.
* @return point ID list
*/
vector<Face::PointIDList> Face::getTrianglePids() const {
	vector<PointIDList> triangles;
	for (int i = 2; i < pids.size(); i++) {
		PointIDList triangle;
		triangle.push_back(pids[i]);
		triangle.push_back(pids[i-1]);
		triangle.push_back(pids[0]);
		triangles.push_back(triangle);
	}
	return triangles;
}

/**
* Get point IDs of vertex.
* @return point IDs
*/
Face::PointIDList Face::getPids() const {
	return pids;
}

/**
* Get points of vertex.
* @return point
*/
Face::PointSet Face::getPoints() const {
	PointSet pset;
	for (int i = 0; i < pids.size(); i++) {
		pset.push_back(points->at(pids[i]));
	}
	return pset;
}

/**
* Get the center of this face.
* @return center point
*/
Vector3 Face::getCenter() const {
	Vector3 v(0, 0, 0);
	for(int i = 0; i < pids.size(); i++) {
		v = v + points->at(pids[i]);
	}
	return v/static_cast<double>(pids.size());
}

/**
* Get a normal vector.
* @return normal vector
*/
Vector3 Face::getNormal() const {
	Vector3 N = cross(points->at(pids[2])-points->at(pids[1]), points->at(pids[0])-points->at(pids[1]));
	return N/norm2(N);
}

/**
* Check if the target face is adjacent to this face.
* @param[in]  face pointer of target face
* @param[out] pid1,pid2 IDs of adjacent vertex
*/
bool Face::isNeighbor(const Face* face, int& pid1, int& pid2) const  {
	for (int i = 0; i < pids.size(); i++) {
		pid1 = pids[i];
		pid2 = pids[(i+1)%pids.size()];
		for (int j = 0; j < face->pids.size(); j++) {
			PointID t_pid1 = face->pids[j];
			PointID t_pid2 = face->pids[(j+1)%face->pids.size()];
			if ((pid1 == t_pid1 && pid2 == t_pid2) || (pid1 == t_pid2 && pid2 == t_pid1)) {
				return true;
			}
		}
	}
	pid1 = -1; pid2 = -1;
	return false;
}

ConvexHull::ConvexHull() {
	faces.clear();
	points.clear();
}

ConvexHull::~ConvexHull() {
	for (unsigned int i = 0; i < faces.size(); i++) {
		delete faces[i];
	}
}

/**
* Get faces.
* @return face list
*/
Face::FaceList ConvexHull::getFaces() const {
	return faces;
}

/**
* Get the size of face list
* @return size of face list
*/
int ConvexHull::getFaceSize() const {
	return faces.size();
}

/**
* Get a face.
* @param[in] fid id of target face
* @return Face pointer
*/
Face* ConvexHull::getFace(Face::FaceID fid) const {
	if (fid > faces.size()) return NULL;
	return faces[fid];
}

/**
* Get a point.
* @param[in] pid id of target point
* @return point
*/
Vector3 ConvexHull::getPoint(Face::PointID pid) const {
	if (pid >= points.size()) {
		cerr << "err:refer index out of range in ConvexHull.getPoint" << endl; 
		return Vector3(0, 0, 0);
	}
	return points[pid];
}

/**
* Create a convex hull.
* @param[in] vertex points
*/
void ConvexHull::createConvexHull(vector<double>& vertex) {
	// initialize
	for (int i = 0; i < faces.size(); i++) {
		delete faces[i];
	}
	faces.clear();
	points.clear();

	// convert points (double list) to Vector3 list
	for (unsigned int i = 0; i < vertex.size(); i = i+3) {
		Vector3 p(vertex[i], vertex[i+1], vertex[i+2]);
		points.push_back(p);
	}

	// create faces
	vector<double> pt_out;
	vector<int> idx_out;

	ConvexAnalysis ca;
	ca.calcConvexHull(3, vertex, pt_out, idx_out, false);

	int count = 0;
	for (unsigned int i = 0; i < idx_out.size(); i++) {
		int size = idx_out[i];
		Face::PointIDList face_indeces;
		int start = i+1;
		for (int j = start; j < start+size; j++) {
			face_indeces.push_back(idx_out[j]);
			i = j;
		}
		faces.push_back(new Face(count++, face_indeces, &points));
	}
}

/**
* Get an intersection of a face and vector @c N from the center of mass.
* @param[in] N normal vector
* @param[in] com center of mass
* @return intersection point
*/
Vector3 ConvexHull::getIntersectPoint(const Vector3& N, const Vector3& com) const {
	for (Face::FaceListIterator fli = faces.begin(); fli != faces.end(); ++fli) {
		Vector3 p_out;
		Face::PointSet pset = (*fli)->getPoints();
		if (lineIntersectToPlane(com, N, pset, p_out)) {
			return p_out;
		}
	}
	return Vector3(0, 0, 0);
}

/**
* Display a convex hull.
* @param[in] R rotation matrix
* @param[in] p position
*/
void ConvexHull::showConvexHull(const Matrix3& R, const Vector3& p) const {
	vector<Face::FaceIDList> dummy1;
	vector<Vector3> dummy2;
	dummy1.clear(); dummy2.clear();
	showConvexHull(R, p, dummy1, dummy2);
}

/**
* Display a convex hull.
* @param[in] R     rotation matrix
* @param[in] p     position
* @param[in] fids  ID list of faces to be colored
* @param[in] color color list 
*/
void ConvexHull::showConvexHull(const Matrix3& R, const Vector3& p, const vector<Face::FaceIDList>& fids, const vector<Vector3>& color) const {
	DrawUtility* draw = DrawUtility::instance();
	draw->clear();

	for (int i = 0; i < points.size(); i++) {
		draw->points.push_back(R*points[i]+p);
	}

	Face::PointIDList index_default;
	vector<Face::PointIDList> index_special;
	for (int i = 0; i < fids.size(); i++) {
		index_special.push_back(vector<int>());
	}

	// assign color to each triangle
	for (Face::FaceListIterator fli = faces.begin(); fli != faces.end(); ++fli) {
		int is_special = -1;
		for (int j = 0 ; j < fids.size(); j++) {
			for (int k = 0; k < fids[j].size(); k++) {
				if ((*fli)->getID() == fids[j][k]) {
					is_special = j;
					break;
				}
			}
		}
		vector<Face::PointIDList> triangles = (*fli)->getTrianglePids();
		for (int j = 0; j < triangles.size(); j++) {
			for (int k = 0; k < 3; k++) {
				if (is_special != -1) {
					index_special[is_special].push_back(triangles[j][k]);
				} else {
					index_default.push_back(triangles[j][k]);
				}
			}
		}
	}

	draw->triangles.push_back(index_default);
	draw->colors.push_back(Vector3(0.8, 0.8, 0));

	for (int i = 0; i < fids.size(); i++) {
		if (fids[i].size() < 1) continue;
		draw->triangles.push_back(index_special[i]);
		draw->colors.push_back(color[i]);
	}

	draw->displayTriangles(0.8);
}

/**
* Check if face @c fid is a gravity stable face.
* @param[in] fid target face ID
* @param[in] com center of mass
* @return true if stable
*/
bool ConvexHull::isStable(Face::FaceID fid, const Vector3& com) const {
	Face* target_face = getFace(fid);
	Vector3 N = target_face->getNormal();
	Face::PointSet pset = target_face->getPoints();
	Vector3 p_out;
	return (lineIntersectToPlane(com, -N, pset, p_out) || lineIntersectToPlane(com, N, pset, p_out));
}

/**
* Get point where a ray and a face intersect.
* @param[in]  p end point of the ray
* @param[in]  n slope of the ray
* @param[in]  Pset vertices of the face
* @param[out] p_out intersection point
* @return true if a ray and a face intersect
*/
bool ConvexHull::lineIntersectToPlane(const Vector3& p, const Vector3& n, const Face::PointSet& Pset, Vector3& p_out) const {
	if(Pset.size() < 3) return false;

	Vector3 p0 = Pset[0];
	Vector3 p1 = Pset[1];
	Vector3 p2 = Pset[2];
	Vector3 planeN = cross(p2 - p1, p0 - p1)/norm2(cross(p2 - p1, p0 - p1));

	p_out = intersectionPoint(p, n, p0, planeN);

	// check if the ray and the plane of the face intersect
	if(dot(p_out-p, n) < 0) return false;

	// check if the intersection point is in the face
	for (int i = 2; i < Pset.size(); i++) {
		p0 = Pset[0];
		p1 = Pset[i-1];
		p2 = Pset[i];
		Vector3 cross1 = cross((p_out-p0), (p0-p1));
		Vector3 cross2 = cross((p_out-p1), (p1-p2));
		Vector3 cross3 = cross((p_out-p2), (p2-p0));
		if (dot(cross1, cross2) >= 0 && dot(cross2, cross3) >= 0) {
			return true;
		}
	}
	return false;
}

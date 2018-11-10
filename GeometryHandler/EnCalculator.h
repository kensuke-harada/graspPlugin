#ifndef _GEOMETRYHANDLER_ENCALCULATOR_H_
#define _GEOMETRYHANDLER_ENCALCULATOR_H_

#include <float.h>
#include <map>
#include <vector>
#include "GeometryHandle.h"
#include "ObjectShape.h"
#include "exportdef.h"

namespace grasp {
	class EXCADE_API OverlapRegionCalculator {
	public:
		typedef std::vector<cnoid::Vector3>  PointVec;
		typedef PointVec                     Boundary;
		typedef std::vector<PointVec>        PointVecVec;
		typedef PointVecVec                  BoundaryVec;

		OverlapRegionCalculator() : is_overlap(false) {;}
		virtual ~OverlapRegionCalculator() {;}

		void calc(const Triangle* triangle, const Boundary& _boundary,
		 const cnoid::Vector3& plane_p, const cnoid::Vector3& plane_n,
		 const cnoid::Vector3& boundary_n);

		bool isOverlap() const {return is_overlap;}
		BoundaryVec getOvelapBoundary() const {return overlap_boundary;}
		BoundaryVec getNotOverlapBoundary() const {return not_overlap_boundary;}
		void clear();

		static void dividePolygon(const Boundary& boundary, PointVecVec& trinagles);
		static bool isInsideTriangle(const PointVec& triangle, const cnoid::Vector3& p);
		static double getArea(const BoundaryVec& boundaries, const cnoid::Vector3& n_bound);
		static bool twoSegmentsIntersect(const cnoid::Vector3& P1,const cnoid::Vector3& P2,const cnoid::Vector3& P3,const cnoid::Vector3& P4,const cnoid::Vector3& n,cnoid::Vector3& Pout);

	private:
		void makeVertexList(const Triangle* triangle, const cnoid::Vector3& plane_p, const cnoid::Vector3& plane_n);
		void makeBoundaries(bool is_target_overlap);

    void removeDuplicatedPoints(PointVec& vec) const;

		enum STATE {S_TRIANGLE_VERTEX, S_TRIANGLE_BOUNDARY, S_BOUNDARY_VERTEX, S_BOUNDARY_TRIANGLE};
		enum DIRECTION {D_ASCENDING, D_DESCENDING};
		Boundary boundary_v;
		BoundaryVec overlap_boundary;
		BoundaryVec not_overlap_boundary;
		bool is_overlap;

		PointVec triangle_v;
		std::vector<int> in_tri_vids;
		std::vector<int> out_tri_vids;
		std::vector<int> bound_vids;
		std::vector<std::vector<int> > intersect_vids;
		std::vector<std::map<int, cnoid::Vector3> > intersect_pos;
	};

	class ContactTriangle {
	public:
		explicit ContactTriangle(const Triangle* _tri) : tri(_tri), splitted(false) {;}
		virtual ~ContactTriangle() {;}

		static bool greater(ContactTriangle* l, ContactTriangle* r) {
			return (l->sorted_depth[0] < r->sorted_depth[0]);
		}

		const Triangle* tri;  /// target trinagle
		double depth;  /// depth from contact point
		double overlap_area;  /// area of overlap region
		double dist;   /// distance from contact point
		double sorted_depth[3];
		Vertex* sorted_ver[3];
		bool splitted;
	};

	class EXCADE_API EnCalculator {
	public:
		typedef std::vector<ContactTriangle*>        ContactTriangleVec;
		typedef ContactTriangleVec::iterator         ContactTriangleVecIte;
		typedef ContactTriangleVec::const_iterator   ContactTriangleVecConstIte;
		typedef OverlapRegionCalculator::Boundary    Boundary;
		typedef OverlapRegionCalculator::BoundaryVec BoundaryVec;

		EnCalculator();
		virtual ~EnCalculator();

		double calc(
			const cnoid::Vector3& p_obj,
			const cnoid::Vector3& p_fing, const cnoid::Vector3& n_fing,
			const BoundaryVec& boundary_fing,
			double fmax);

		double calc(
			const cnoid::Vector3& p_obj, const cnoid::Vector3& p_obj_top,
			const cnoid::Vector3& p_fing, const cnoid::Vector3& n_fing,
			const BoundaryVec& boundary_fing,
			double fmax);

		double getEn() const {return en;}
		double getForce() const {return force;}
		double getArea() const {return sum_area;}
		cnoid::Vector3 getNormal() const {return normal;}
		void setHmax(double _hmax) {hmax = _hmax;}
		void setHDivisionNum(unsigned int _num) {h_division_num = _num;}
		void setK(double _k) {elastic = _k;}
		void setObject(const ObjectShape* _obj) {obj = _obj;}

	protected:
		const ObjectShape* obj;
		double hmax;  /// depth limit
		double elastic;  /// elastic modulus
		unsigned int h_division_num;

		double en;    /// eccentricity paramter
		double sum_area;   /// area
		cnoid::Vector3 normal;  /// normal
		double force;  /// contact force

		cnoid::Vector3 p_top;
		cnoid::Vector3 n_contact;
		cnoid::Vector3 p_contact;
		cnoid::Vector3 p_plane;
		BoundaryVec boundaries;
		ContactTriangleVec candidate_triangles;
		ContactTriangleVec splitted_candidate_triangles;
		ContactTriangleVec splitted_nocandidate_triangles;

		class ConvexPoint {
		public:
			ConvexPoint() {;}
			ConvexPoint(cnoid::Vector3 _p, double _depth) : p(_p), depth(_depth) {;}
			cnoid::Vector3 p;
			double depth;
			bool operator<(const ConvexPoint& l) const {
				return depth < l.depth;
			}
		};

		void init();
		void generateCandidateTrianglesList();
		void sortByDepth();
		bool calcProc(double h);
		void calcOverlapArea(ContactTriangleVec& target_list, bool remove_unoverlaptri = true);
		void generateTargetTriangleList(double h, ContactTriangleVec& target_list);
		bool makeSplittedTriangle(std::vector<ConvexPoint> p, const Triangle* ori_tri, std::vector<ContactTriangle*>& triangles) const;
		void clearSplittedTriangles();
	};

}

#endif /* _GEOMETRYHANDLER_ENCALCULATOR_H_ */

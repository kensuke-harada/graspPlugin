/**
 * @file   SweptVolume.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACEPLANNER_SWEPTVOLUME_H_
#define _PICKANDPLACEPLANNER_SWEPTVOLUME_H_

#include <vector>
#include <string>

#include <cnoid/EigenTypes>
#include <cnoid/Link>
#include <cnoid/ColdetModel>

#include "../PCL/PrimitiveShapeParameter.h"
#include "../Grasp/OccupiedVoxel.h"
#include "exportdef.h"

namespace grasp {
	class PlanBase;
}

struct Triangle {
	int vid[3];
	cnoid::Vector3 e1;  // v1 - v0
	cnoid::Vector3 e2;  // v2 - v0
	cnoid::Vector3 normal;
	double x_max, x_min;
	double z_max, z_min;
	Triangle* nbr[3];
};

class Polyhedron {
 public:
	typedef std::vector<cnoid::Vector3> VertexVec;
	typedef std::vector<Triangle*> TrianglePtrVec;
	typedef std::vector<const Triangle*> TriangleConstPtrVec;
	typedef std::vector<Polyhedron*> PolyhedronPtrVec;

	Polyhedron();
	virtual ~Polyhedron();

	void addVertex(const cnoid::Vector3& v);
	void addTrianglePtr(Triangle* tri);

	void setClosed(bool closed);


	VertexVec& vertices();
	const VertexVec& vertices() const;
	cnoid::Vector3& vertex(int i);
	const cnoid::Vector3& vertex(int i) const;
	TrianglePtrVec& triangles();
	const TrianglePtrVec& triangles() const;
	Triangle* triangle(int i);
	const Triangle* triangle(int i) const;
	cnoid::ColdetModelPtr coldet_model();

	bool isClosed() const;
	bool isConvex() const;
	void getAABB(cnoid::Vector3& min, cnoid::Vector3& max) const;

	void construct();

	void computeNormals();
	void computeBBs();
	void convexityTest();
	void computeAABB();
	void buildColdet();
	void bindNeighbors();

	void clear();

 private:
	VertexVec vertices_;
	TrianglePtrVec triangles_;
	cnoid::ColdetModelPtr coldet_model_;

	bool is_closed_;
	bool is_convex_;
	cnoid::Vector3 aabb_min_;
	cnoid::Vector3 aabb_max_;
};

class SweptVolumePolyhedron :
public Polyhedron {
 public:
	SweptVolumePolyhedron();
	virtual ~SweptVolumePolyhedron();

	enum TYPE {MODEL, APPROACH, GRASP, };

	void setType(TYPE type);
	bool isModelPart() const;
	bool isApproachPart() const;
	bool isGraspPart() const;

	void setFingerPart(bool is_finger);
	bool isFingerPart() const;

 private:
	TYPE type_;
	bool is_finger_;
};

class ModelDivider {
 public:
	typedef Polyhedron::VertexVec VertexVec;
	typedef Polyhedron::TrianglePtrVec TrianglePtrVec;
	typedef Polyhedron::PolyhedronPtrVec PolyhedronPtrVec;

	ModelDivider();
	virtual ~ModelDivider();

	void divide(const cnoid::Link* target_link, PolyhedronPtrVec& polyhedra);
 private:
	VertexVec vertices_;
	TrianglePtrVec triangles_;

	void makeTriangles(const cnoid::Link* target_lik);
	void bindNeighbors();
	void makePolyhedra(PolyhedronPtrVec& polyhedra);
	void makePolyhedron(const TrianglePtrVec& triangles, Polyhedron* poly);
};

class EXCADE_API SweptVolume {
 public:
	typedef Polyhedron::TrianglePtrVec TrianglePtrVec;
	typedef Polyhedron::TriangleConstPtrVec TriangleConstPtrVec;
	typedef std::vector<Polyhedron*> PolyhedronPtrVec;
	typedef std::vector<SweptVolumePolyhedron*> SVPolyhedronPtrVec;

	SweptVolume();
	virtual ~SweptVolume();

	void setGRCR(const cnoid::Matrix3& palm_R_GRC);
	cnoid::Matrix3 getGRCR() const;

	double getAppLength() const;
	void setAppLength(double len);
	void setNumGraspStep(unsigned int n);

	void makeSweptVolume();

	SVPolyhedronPtrVec& polyhedra();
	const SVPolyhedronPtrVec& polyhedra() const;

	void clear();

	std::string hand_name() const;

 protected:
	grasp::PlanBase* gc;
	cnoid::Matrix3 palm_R_GRC_;
	double app_len_;
	std::vector<double> finger_q_;
	unsigned int num_grasp_step_;
	std::string hand_name_;

	ModelDivider divider_;

	SVPolyhedronPtrVec polyhedra_;

	void storeFinger();
	void restoreFinger();
	void setCloseFinger();
	void openFinger();
	void closeFinger();
	void graspFinger(int n, int n_close);

	void makeLinkPolyhedra(const cnoid::Link* target_link);
	void makeLinkModelPolyhedron(const cnoid::Link* target_link, const Polyhedron* base_poly, SweptVolumePolyhedron* model_poly) const;
	void makeApproachPolyhedra(const SweptVolumePolyhedron* model_polyhedron, SVPolyhedronPtrVec& app_polyhedra) const;
	void makeApproachPolyhedronByFacet(const SweptVolumePolyhedron* model_polyhedron, const TriangleConstPtrVec& facet, SweptVolumePolyhedron* app_polyhedron) const;
	void makeGraspPolyhedra(const cnoid::Link* target_link, const Polyhedron* base_poly, SVPolyhedronPtrVec& grasp_polyhedra, int n_divide);
};

namespace grasp {
	class ObjPoseEstimateSol;
}

class PenaltyCalculator;

class HistogramParameter;

class EXCADE_API SweptVolumeChecker {
 public:
	typedef std::vector<SweptVolumePolyhedron*> SVPolyhedronPtrVec;

	explicit SweptVolumeChecker(const SweptVolume* sv);
	virtual ~SweptVolumeChecker();

	void setObjPoseEstimateSol(const grasp::ObjPoseEstimateSol* est_sol);

	void setMargin(double margin);
	void setObjName(const std::string& obj_name);
	void setUnknownRegionCheckMode(bool mode);
	void setOccupiedVoxelVec(grasp::OccupiedVoxelVec* voxel);

	double check(const cnoid::Vector3& palm_p, const cnoid::Matrix3& palm_R);
	double computeScore();

	void getInlierIndices(std::vector<int>& indices);

	void getFeatureVector(std::vector<double>& feature) const;

	void outputPointsGRC(const std::string& filepath) const;

 protected:
	const SweptVolume* sv_;
	const grasp::ObjPoseEstimateSol* est_sol_;
	PenaltyCalculator* penalty_calc_;
	double margin_;
	bool unknown_region_check_mode_;
	grasp::OccupiedVoxelVec* voxel_;

	std::vector<int> in_indices_;
	std::vector<double> feature_;

	std::vector<cnoid::Vector3> points_GRC_;
	std::vector<cnoid::Vector3> normals_GRC_;
	std::vector<int> indices_app_palm_;
	std::vector<int> indices_app_finger_;
	std::vector<int> indices_grasp_;
	cnoid::Vector3 bb_max_, bb_min_;
	cnoid::Vector3 bb_finger_max_, bb_finger_min_;
	SVPolyhedronPtrVec app_poly_;

	void computeAABB();
	void getInsideRotatedAABBIndices(const cnoid::Matrix3& palm_R, const cnoid::Vector3& palm_p, std::vector<int>& indices) const;
	void transToGRCcoordinate(const cnoid::Matrix3& palm_R, const cnoid::Vector3& palm_p, const std::vector<int>& indices);
	void transToGRCcoordinateProc(const cnoid::Matrix3& tipRw, const cnoid::Vector3& palm_p, const std::vector<int>& indices, int start, int end);
	void getInsideAABBIndices(const std::vector<int>& target_indices, std::vector<int>& indices) const;
	void labelingInsidePoints(const std::vector<int>& indices);

	void computeFeatureSum();
	void computeFeatureHistogram(const HistogramParameter* param);
	void computeFeatureHistogramSub(const std::vector<int>& indices,
																	const cnoid::Vector3& bb_max, const cnoid::Vector3& bb_min,
																	const HistogramParameter* param);


	bool isInsidePolyhedra(const SVPolyhedronPtrVec& polyhedra, const cnoid::Vector3& p) const;
	bool isInsideConvexPolyhedron(const cnoid::Vector3& p, Polyhedron* poly) const;
	bool isInsideConcavePolyhedron(const cnoid::Vector3& p, Polyhedron* poly) const;
	double computeHeightPenalty(const std::vector<cnoid::Vector3>& points_GRC, const std::vector<int>& indices,
															const cnoid::Vector3& bb_max) const;
	double computeDistancePenalty(const std::vector<cnoid::Vector3>& points_GRC, const std::vector<int>& indices,
																const cnoid::Vector3& bb_min, const cnoid::Vector3& bb_max) const;
	double computeApproachRegionPenalty(const std::vector<cnoid::Vector3>& points_GRC, const std::vector<cnoid::Vector3>& normals_GRC,
																			const std::vector<int>& indices, const cnoid::Vector3& bb_min, const cnoid::Vector3& bb_max,
																			const SVPolyhedronPtrVec& polyhedra) const;
	void computeHeights(const std::vector<cnoid::Vector3>& points_GRC, const std::vector<int>& indices, const SVPolyhedronPtrVec& polyhedra,
										 std::vector<double>& height_vec) const;

	double computeUnknownRegionHeuristicScore(const cnoid::Matrix3& palm_R, const cnoid::Vector3& palm_p) const;
};

class EXCADE_API SweptVolumeDrawer {
 public:
	explicit SweptVolumeDrawer(SweptVolume* sv);
	virtual ~SweptVolumeDrawer();

	void draw(const cnoid::Vector3& offset = cnoid::Vector3::Zero());
	void clear();

	void addPoints(const std::vector<cnoid::Vector3>& points);
	void addPointsPtr(const std::vector<cnoid::Vector3>* points, const std::vector<int>* indices);

 protected:
	SweptVolume* sv_;

	std::vector<cnoid::Vector3> points_;
	const std::vector<cnoid::Vector3>* points_ptr_;
	const std::vector<int>* indices_ptr_;
};

#endif /* _PICKANDPLACEPLANNER_SWEPTVOLUME_H_ */


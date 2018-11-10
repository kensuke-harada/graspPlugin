#ifndef  __Cluster_H__
#define __Cluster_H__


#include <stdio.h>
#include <list>
#include <vector>
#include <map>
#include <iostream>
#ifndef WIN32
#include <float.h>
#endif
#include "../Grasp/VectorMath.h"
#include "exportdef.h"
//#include "Boundary.h"
//#define HEURISTICS_FOR_PCL //Used for incomplete mesh models
#include "ShapeElements.h"

namespace grasp{

double _drand();

class Triangle;
class Vertex;

class ClusterData{
public:
    int id;
    std::vector<Triangle*> triangleList;
    std::vector<std::vector<VertexLink*> > boundaryList;
    std::vector<cnoid::Vector3> controlPoints;
    double controlPointInterval;

    cnoid::Matrix3 sumDistribute;
    cnoid::Vector3 sumCenter;
    cnoid::Vector3 center, bottom, top;
    double area;
    cnoid::Vector3 normal;
    cnoid::Vector3 eigenVector[3];
    cnoid::Vector3 eval;
    float color[4];
    //
    int intention;
    std::vector<cnoid::Vector3> approach;
    bool closed;
    std::vector<Triangle*> neighborList;
    cnoid::Vector3 bbedge, bbcenter;
    cnoid::Matrix3 bbR;
    std::vector<int> idPair;
    double thickness;

    //option
    std::vector<cnoid::Vector3> convexHull;
    enum ConvexityParameters {CONVEX, CONCAVE, SADDLE, TABLELEG};
    int Convexity, Put;

    //for quadrics
    cnoid::MatrixXd sumQuadDistribution;
    cnoid::MatrixXd sumQuadError;
    double quadError;
    cnoid::VectorXd quadCoefficient;
    cnoid::MatrixXd quadDistribution;
	cnoid::VectorXd planeCoefficient;
    void calcQuadricSurfaceParameter();
    void calcQuadricSurfaceParameterSecond(cnoid::VectorXd Coefficient, cnoid::MatrixXd sumDistribution, cnoid::MatrixXd sumError);
    void calcQuadricSurfaceParameterOutput();
    void calcMainAxis( cnoid::Vector3& v1,  cnoid::Vector3& v2);

    cnoid::Vector3 quadCenter;
    cnoid::Vector3 quadRadius;
    cnoid::Matrix3 quadRot;
    double quadScale;

    cnoid::MatrixXd sumQuadDistributionShift;
    cnoid::MatrixXd sumQuadErrorShift;
    cnoid::MatrixXd sumQuadDistributionLow[16];
    cnoid::MatrixXd sumQuadErrorLow[4];
    cnoid::Vector3 shift;

};

class EXCADE_API ClusterImpl : public ClusterData
{
public:
    ClusterImpl(int id_=-1){
        id = id_;
        sumDistribute << 0,0,0,0,0,0,0,0,0;
        sumCenter = cnoid::Vector3(0,0,0);
        area=0;
        color[0] = _drand();
        color[1] = _drand();
        color[2] = _drand();
        color[3] = 1.0;

        closed = false;
        intention = 0;
        controlPointInterval = 0.02;

        isAliveNode=true;

        //for quadrics
        //sumQuadDistribution=dzeromatrix(10,10);
        //sumQuadError=dzeromatrix(9,9);
        quadError = 0;
        k0=100;
        m0=100;
        quadCoefficient = cnoid::VectorXd::Zero(10);
        //quadCoefficient=dzerovector(10);

    }
    ~ClusterImpl(){
        triangleList.clear();
        neighborClusters.clear();
//		sumQuadDistribution.clear();
//		sumQuadError.clear();
//		quadCoefficient.clear();
//		delete sumQuadDistribution;
//		delete sumQuadError;
    }

    int clusterGrowingFromSeed(Triangle& st, double thickness = 0.005);
    void calcClusterNormal();
    int calcClusterBoundary();
    int calcControlPoints(double param);
		int calcControlPointsWithBoundSamres(double param,
										 double bounddist,
										 double principle0dist,
										 double principle1dist,
										 double samres);
    int calcClusterBoundaryWithHole();

    //private:

    void addTriangleToCluster(Triangle &t);
    double clusterTriangleDistance(Triangle t);
    //
    void addTriangleToNeighborsList(Triangle &t);
    void calcTriangleList(ClusterImpl& c1,  ClusterImpl& c2);
    void calcNeighborList(ClusterImpl& c1,  ClusterImpl& c2);
    bool isaMember(Triangle* a);
    void calcClusterBoundingBox();
    bool lineIntersectToCluster(const cnoid::Vector3& p, const cnoid::Vector3& n, cnoid::Vector3& p_out);

		double pointDistToBoundary(const cnoid::Vector3 &p);


    //for cluster tree
    ClusterImpl *parentNode;
    ClusterImpl* childNodes[2];
    std::list <ClusterImpl*> neighborClusters;
    bool isAliveNode;
    void copyParameter(ClusterData& c);
    void mergeClusterForBinaryTree(ClusterImpl& c1,  ClusterImpl& c2);
    void calcQuadricParameter(Triangle& t);
    void calcQuadricAreaParameter();
    void calcQuadricAreaParameterShift(cnoid::Vector3 shift);
    void calcQuadricAreaParameterLow();
    void calcQuadricAreaParameterLowShift(cnoid::Vector3 shift);
    void writeData(std::ofstream & fout);

    double k0;
    double m0;


};

class EXCADE_API ClusterPair : public ClusterData
{
public:
    ClusterPair(ClusterImpl& c1, ClusterImpl& c2){
        clusters[0] = &c1;
        clusters[1] = &c2;
        mergeClusterParameter(c1, c2);
        error = quadError;
    }
    int id;
    double error;
    ClusterImpl* clusters[2];

    void mergeClusterParameter(ClusterImpl& c1,  ClusterImpl& c2);
//	static bool compClusterHeap (ClusterPair& cp1, ClusterPair& cp2);
    static bool compClusterHeap (ClusterPair* cp1, ClusterPair* cp2);
};




}


#endif

#ifndef  __ObjectShape_H__
#define __ObjectShape_H__


#include <stdio.h>
#include <list>
#include <vector>
#include <map>
#include <iostream>
#ifndef WIN32
#include <float.h>
#endif
#include <cnoid/ColdetModel>
#include <cnoid/ValueTree>

#include <boost/shared_ptr.hpp>

#include "../Grasp/VectorMath.h"
#include "GeometryHandle.h"

#include "exportdef.h"

namespace grasp {

class EXCADE_API ObjectShape{
public:
    ObjectShape(std::vector<double> vertex, std::vector<int> crd);
    ObjectShape(cnoid::ColdetModelPtr c);
    ObjectShape(){
        clusterNodes=NULL;
        quadVerticies=NULL;
        thickness = 0.005;
        alpha=1.0;
    };
    ObjectShape(ObjectShape* object);

    ~ObjectShape();

    class LessAbs{
    public:
        bool operator()(const double& l, const double& r) const{
            return fabs(l)<fabs(r);
        }
    };

    void makeNbr();
    void transform(cnoid::Matrix3 R, cnoid::Vector3 );
    std::string name;

    int nVerticies;
    int nTriangles;
//	Vertex* verticies;
    VertexLink* verticies;
    Triangle* triangles;
    std::vector <ClusterImpl> clusters;
    //quadric approximation
    int nClusterNodes;
    ClusterImpl* clusterNodes;
    std::vector <ClusterPair*> clusterPairHeap;
    Vertex* quadVerticies;
    double thickness;

    double alpha;
    //tajima
    int petid[2];
    int cupid;


    //
    std::vector <int> parentList;
    bool closed;
    void generateClusterSeeds(int size=2);
    bool mergeClusters2(int id, const cnoid::Vector3& normal, bool PlaneMode=false);
    void mergeClustersFromSeed(bool PlaneMode=false);
    void mergeClustersFromSeed(const cnoid::Vector3& normal, const cnoid::Vector3& position);

    //quadric approximation
    void initialClustersFromOneFace();
    void calcClusterBinaryTree(const int nNodes=0);
    //tajima
    void calcClusterBinaryTreeFV(const int nNodes=0);
    void ClusterClass();
    void ShapeJudge(const int id=0);

    void LimitedcalcClusterBinaryTree(const int nNodes=0);
    void calcClusterBinaryTreeStep();
    void calcVertexPositionOnQuadSurface();
    void calcVertexPositionOnOtherSurface(const cnoid::MatrixXd& qAffine, const cnoid::VectorXd& qTrans);


    void calcClusterIncompleteModel(const int nClusters=0,double volRatioThreshold=0.0,double magnification=1.0);
    void generateInitialClusterCandidate(double dist_threshold=0);
    void calcClusterBinaryTreeFromClusteredNodes(const int nNodes=0,double volRatioThreshold=0.0);
    bool isNeighbor(ClusterImpl* c1,ClusterImpl* c2,double threshold,bool useVer2TriDist = true);
    double calcOverlapVolumeRatio();
    void calcBoundingBox(cnoid::Vector3 &edge,cnoid::Vector3& center,cnoid::Matrix3& Rot);
    void calcBoundingBox(int id,cnoid::Vector3 &edge,cnoid::Vector3& center,cnoid::Matrix3& Rot);
    void calcBoundingBoxImpl(std::vector<Triangle*>& triangles,cnoid::Vector3 &edge,cnoid::Vector3& center,cnoid::Matrix3& Rot);
    void getClusterParameters(int clusters,std::vector<cnoid::Vector3>* edges,std::vector<cnoid::Vector3>* centers,std::vector<cnoid::Matrix3>* rots,double volRatioThreshold,double magnification=1.0);
    void getClusterTrianglePoints(int clusters,std::vector<cnoid::Vector3>& points,std::vector<std::vector<int> >&pids,double volRatioThreshold,double magnification=1.0);
    void getClusterQuadParameters(int clusters,std::vector<cnoid::Matrix3>* qrot,std::vector<cnoid::Vector3>* qcenter,std::vector<cnoid::Vector3>* qradius,std::vector<double>* qscale,double volRatioThreshold,double magnification=1.0);

    void generateContactCluster(const cnoid::Vector3& contact_point,const cnoid::Vector3& N,int& contact_cluster_id,bool PlaneMode=false);
    bool isCornerPoint(const cnoid::Vector3& contact_point,std::vector<int>& contact_tids) const;
    double calcContactArea(const cnoid::Vector3& contact_point,const cnoid::Vector3& N);
    std::vector<std::vector<cnoid::Vector3> > getContactRegionBoundaryPoints(const cnoid::Vector3& contact_point,const cnoid::Vector3& N,int& tid);
    std::vector<std::vector<cnoid::Vector3> > getBoundaryPoints(int cid);
    double minDistancePointLine(const cnoid::Vector3& lp1,const cnoid::Vector3& lp2,const cnoid::Vector3& p) const;

    void saveClusters(std::ofstream& fout, int depth);
    bool loadClusters(const cnoid::Mapping& clusterSetting);

};

typedef boost::shared_ptr<ObjectShape> ObjectShapePtr;

}

#endif

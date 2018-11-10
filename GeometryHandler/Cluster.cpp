// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-

//#include </opt/intel/composerxe-2011.0.084/mkl/include/mkl.h>

#include "GeometryHandle.h"
#include "ObjectShape.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include <list>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <Eigen/Eigenvalues>

#include "../Grasp/PlanBase.h"
#include "../Grasp/VectorMath.h"
//#define THREAD
#define OUTPUT_COMMENT

//#define CENTER_SHIFT
//#define CYLINDER_FITTING

//#define DEBUG2
//#define LOG

extern "C" {
//#include <cblas.h>
//#include <clapack.h>
};

//extern "C" void dggev_(const char* JOBVL, const char* JOBVR, const int* N, const double* A, const int* LDA, const double* B, const int* LDB,
//									double* ALPHAR, double* ALPHAI, double* BETA, double* VL, const int* LDVL, double* VR, const int* LDVR, double* WORK, const int* LWORK, int* INFO);

//extern "C" void dsygv_ (int *itype, char *jobz, char *uplo, int *n, double *a, int *lda, double *b, int *ldb, double *w, double *work, int *lwork, int *info);

double grasp::_drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
	}

using namespace std;
using namespace cnoid;
using namespace grasp;

bool ClusterImpl::isaMember(Triangle* a) { //, const vector<Triangle*> b){

	for(unsigned int i=0; i<triangleList.size(); i++)
		if(a->id==triangleList[i]->id) return true;

	return false;
}

bool ClusterImpl::lineIntersectToCluster(const Vector3& p, const Vector3& n, Vector3& p_out){

	if(norm2(normalPoint(center, p, n)-center) > 0.5*norm2(bbedge) ){
		p_out << 0,0,0;
		return false;
	}

	for(unsigned int i=0; i<triangleList.size(); i++){

		p_out = intersectionPoint(p, n, triangleList[i]->ver[0]->pos, triangleList[i]->normal) ;
		if(triangleList[i]->pointInTriangleDistance(p_out) > -1.0e-8)
			return true;
	}

	return false;
}

void ClusterImpl::calcTriangleList(ClusterImpl& c1,  ClusterImpl& c2)
{

	area = c1.area + c2.area;
	sumCenter = c1.area* c1.center + c2.area* c2.center;
	sumDistribute = c1.sumDistribute + c2.sumDistribute;

	for(unsigned int j=0; j<c1.triangleList.size(); j++){
		c1.triangleList[j]->idCluster = id;
		triangleList.push_back(c1.triangleList[j]);
	}
	for(unsigned int j=0; j<c2.triangleList.size(); j++){
		c2.triangleList[j]->idCluster = id;
		triangleList.push_back(c2.triangleList[j]);
	}

	center = sumCenter/area;

	dmatrix distribute(3, 3);
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = sumDistribute(j,k) - center[j] * center[k] * area;
		}
	}
	dmatrix evec(3, 3);
	dvector eval(3);
	int info;
	info = calcEigenVectors(distribute, evec, eval);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			eigenVector[j][i] = evec(i, j);

	for(int i=0;i<3;i++){
		int j=i%2;
		if(eval[j] < eval[j+1]){
			double temp=eval[j];
			Vector3 tvec=eigenVector[j];
			eval[j] = eval[j+1];
			eigenVector[j] = eigenVector[j+1];
			eval[j+1] = temp;
			eigenVector[j+1] = tvec;
		}
	}
	for(int i=0;i<3;i++){
		this->eval[i]= eval[i];
	}

	return;
}

void ClusterImpl::calcNeighborList(ClusterImpl& c1,  ClusterImpl& c2)
{

	for(unsigned int j=0; j<c1.neighborList.size(); j++)
		if( ! isaMember(c1.neighborList[j]))
			neighborList.push_back(c1.neighborList[j]);

	for(unsigned int j=0; j<c2.neighborList.size(); j++)
		if( ! isaMember(c2.neighborList[j]))
			neighborList.push_back(c2.neighborList[j]);

	return;
}

void ClusterImpl::addTriangleToNeighborsList(Triangle &t){

	if(neighborList.size()>0){

		vector<Triangle*>::iterator it=neighborList.begin();
		while(it!=neighborList.end()){

			while((*it) ==NULL)	++it;

			if( (*it)->id == t.id)
				it = neighborList.erase(it);
			else
				++it;
		}

		for(int i=0; i<3; i++){

			if(t.nbr[i]==NULL) continue;

			bool included = false;

			for(it=neighborList.begin(); it!=neighborList.end(); ++it)
				if((*it)->id == t.nbr[i]->id)
					included = true;
			for(it=triangleList.begin(); it!=triangleList.end(); ++it)
				if((*it)->id == t.nbr[i]->id)
					included = true;

			if(!included)
				neighborList.push_back(t.nbr[i]);
		}
	}
	else{
		for(int i=0; i<3; i++){
			if(t.nbr[i] == NULL) continue;
			neighborList.push_back(t.nbr[i]);
		}
	}

	return;
}

int ClusterImpl::clusterGrowingFromSeed(Triangle& st, double thickness){

	int top=0,end=0;
	Triangle *t,*tn;

	addTriangleToCluster(st);

	top = 0;
	end = triangleList.size();
	while(top!=end){
		t = triangleList[top++];
		for(int i=0;i<3;i++){
			tn = t->nbr[i];
			if(tn == NULL) continue;
			if(tn->idCluster != -1) continue;
			if(clusterTriangleDistance(*tn) > thickness) continue;
			addTriangleToCluster(*tn);
			end++;
		}
	}
	return end;


}

int ClusterImpl::calcClusterBoundary(){
	Triangle *t,*tn;
	VertexLink *v,*e,*e0;
	VertexLink *v0;

	std::vector<VertexLink*> boundaryVertices;

	for(unsigned int i=0;i<triangleList.size();i++){
		t = triangleList[i];
		for(int j=0;j<3;j++){
			tn = t->nbr[j];
#ifndef HEURISTICS_FOR_PCL
			if(tn == NULL) continue;
#endif
			if(tn != NULL && tn->idCluster == id) continue;
			t->ver[j]->next = t->ver[(j+1)%3];
			boundaryVertices.push_back(t->ver[j]);
			t->ver[j]->check = id;
		}
	}

	for(unsigned int k=0; k<boundaryVertices.size(); k++)
		boundaryVertices[k]->check = 1;

	boundaryList.clear();

	for(unsigned int k=0; k<boundaryVertices.size(); k++){

		v = boundaryVertices[k];
		vector<VertexLink*> b;

		while(v->check == 1){
			b.push_back(v);
			v->check = -1;
			v = v->next;
		}

		if(b.size()>0)
			boundaryList.push_back(b);
	}

	return 0;
}

int ClusterImpl::calcClusterBoundaryWithHole(){
  Triangle *t,*tn;
  VertexLink *v,*e,*e0;
  VertexLink *v0;

  std::vector<VertexLink*> boundaryVertices;

  for(unsigned int i=0;i<triangleList.size();i++){
    triangleList[i]->ver[0]->nextlist.clear();
    triangleList[i]->ver[1]->nextlist.clear();
    triangleList[i]->ver[2]->nextlist.clear();
  }

  for(unsigned int i=0;i<triangleList.size();i++){
    t = triangleList[i];
    for(int j=0;j<3;j++){
      tn = t->nbr[j];
#ifndef HEURISTICS_FOR_PCL
      if(tn == NULL) continue;
#endif
      if(tn != NULL && tn->idCluster == id) continue;
      t->ver[j]->next = t->ver[(j+1)%3];
      if(tn != NULL) t->ver[j]->nextlist[tn->id] = t->ver[(j+1)%3];
      boundaryVertices.push_back(t->ver[j]);
      t->ver[j]->check = id;
    }
  }

  for(unsigned int k=0; k<boundaryVertices.size(); k++)
    boundaryVertices[k]->check = boundaryVertices[k]->nextlist.size()-1;

  boundaryList.clear();

  for(unsigned int k=0; k<boundaryVertices.size(); k++){

    v = boundaryVertices[k];
    vector<VertexLink*> b;
    int pre_vid = v->id;
    while(v->check > -1){
      b.push_back(v);
      v->check--;

      int cur_vid = v->id;
      if(v->nextlist.size() > 1){
        // In case target vertex has multiple next boundary vertex candidates

        // Find a triangle which has a boundary edge from the last visited vertex to the target vertex
        Triangle* t=NULL;
        for (int j = 0; j < v->tlist.size(); j++) {
          if (v->tlist[j]->idCluster != id) continue;
          for (int n = 0; n < 3; n++) {
            if (v->tlist[j]->ver[n]->id == pre_vid){
              t = v->tlist[j];
              break;
            }
          }
        }

        bool is_target = false;

        // Search next boundary vertex in a triangle which is adjacent to the triangle found in above procedure
        while(t != NULL){
          if(v->nextlist.find(t->id) != v->nextlist.end()){
            v = v->nextlist[t->id];
            break;
          }
          int vn = 0;
          for(int n = 0;n<3;n++){
            if(t->ver[n]->id == v->id){
              vn = n;
            }
          }
          t = t->nbr[vn];
        }
      }else{
        v = v->next;
      }
      pre_vid = cur_vid;
    }

    if(b.size()>0)
      boundaryList.push_back(b);
  }

  return 0;
}

void ClusterImpl::calcClusterBoundingBox() {

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for(size_t k=0;k<boundaryList.size();k++){
		for(size_t l=0;l<boundaryList[k].size();l++){
			Vector3 pt = boundaryList[k][l]->pos;
			for (int j = 0; j < 3; j++) {
				double tmp = dot(eigenVector[j], Vector3(pt - center));
				if (tmp > pt_max[j]) pt_max[j] = tmp;
				if (tmp < pt_min[j]) pt_min[j] = tmp;
			}
		}
	}

	bbedge =  (pt_max - pt_min);
	bbcenter = 0.5*(pt_max + pt_min) + center;

	return;
}

void ClusterImpl::addTriangleToCluster(Triangle& t) {

	area += t.area;
	( sumCenter)  =  sumCenter + t.area/3.0* Vector3 ( t.ver[0]->pos + t.ver[1]->pos + t.ver[2]->pos);
	if(isfinite(1.0/t.area))
		center = (sumCenter/area);

	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			for(int k=0;k<3;k++){
				sumDistribute(j,k) += t.area*(t.ver[i]->pos[j] * t.ver[i]->pos[k])/6.0;
				sumDistribute(j,k) += t.area*(t.ver[i]->pos[j] * t.ver[(i+1)%3]->pos[k])/12.0;
				sumDistribute(j,k) += t.area*(t.ver[(i+1)%3]->pos[j] * t.ver[i]->pos[k])/12.0;
			}
		}
	}
	dmatrix distribute(3, 3);
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = sumDistribute(j,k) - center[j] * center[k] * area;
		}
	}
	dmatrix evec(3, 3);
	dvector eval(3);
	int info;
	info = calcEigenVectors(distribute, evec, eval);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			eigenVector[j][i] = evec(i, j);

	for(int i=0;i<3;i++){
		int j=i%2;
		if(eval[j] < eval[j+1]){
			double temp=eval[j];
			Vector3 tvec=eigenVector[j];
			eval[j] = eval[j+1];
			eigenVector[j] = eigenVector[j+1];
			eval[j+1] = temp;
			eigenVector[j+1] = tvec;
		}
	}

	for(int i=0;i<3;i++){
		this->eval[i]= eval[i];
	}

	triangleList.push_back(&t);
	t.idCluster = id;

	return;
}



double ClusterImpl::clusterTriangleDistance(Triangle t){
	double max=0;
	double min=0;

	max = min =  dot( center, eigenVector[2]);
	for(int i=0;i<3;i++){
		double temp = dot( t.ver[i]->pos, eigenVector[2]);
		if(temp > max)
			max =temp;
		if(temp < min)
			min = temp;
	}
	for(unsigned int i=0;i<triangleList.size();i++){
		for(int j=0;j<3;j++){
			double temp = dot ( (*triangleList[i]).ver[j]->pos, eigenVector[2]);
			if(temp > max)
				max =temp;
			if(temp < min)
				min = temp;
		}
	}

	return max-min;
}


void ClusterImpl::calcClusterNormal(){

	Vector3 sum(0,0,0);

	for(unsigned int i=0;i<triangleList.size();i++){
		(sum) = sum+triangleList[i]->area*triangleList[i]->normal;
	}
	if(dot (sum, eigenVector[2]) >0)  normal = eigenVector[2];
	else	                          normal = -eigenVector[2];

	double max, min;
	max = min =  dot( center, normal);
	top = bottom = (*triangleList[0]).ver[0]->pos;

	for(unsigned int i=0;i<triangleList.size();i++){
		for(int j=0;j<3;j++){
			double temp = dot ( (*triangleList[i]).ver[j]->pos, normal);
			if(temp > max){
				max =temp;
				top = (*triangleList[i]).ver[j]->pos;
			}
			if(temp < min){
				min = temp;
				bottom = (*triangleList[i]).ver[j]->pos;
			}
		}
	}

	(top) = center + dot(top-center, normal)*normal;
	(bottom) = center + dot(bottom-center, normal)*normal;
}

int ClusterImpl::calcControlPoints(double param){

	controlPoints.clear();

       Vector3 movedCenter;
       if(param>0.0)
	       movedCenter = center + param * dot(Vector3(top - center), normal) * normal;
       else
	       movedCenter = center + param * dot(Vector3(center - bottom), normal) * normal;

	double min0, max0, min1, max1,center0,center1,center2;
	center0 = min0 = max0 = dot(center,eigenVector[0]);
	center1 = min1 = max1 = dot(center,eigenVector[1]);
	center2 = dot(movedCenter,eigenVector[2]);

	for(unsigned int i=0;i<triangleList.size();i++){
		for(int j=0;j<3;j++){
			double temp = dot ( triangleList[i]->ver[j]->pos, eigenVector[0]);
			if(temp > max0){
				max0 =temp;
			}
			if(temp < min0){
				min0 = temp;
			}
			temp = dot ( triangleList[i]->ver[j]->pos, eigenVector[1]);
			if(temp > max1){
				max1 =temp;
			}
			if(temp < min1){
				min1 = temp;
			}
		}
	}

	int s0 = int ( -(center0-min0)/controlPointInterval ) -1 ;
	int s1 = int ( -(center1-min1)/controlPointInterval ) -1 ;

	for(int i = s0; double(i)*controlPointInterval+center0 < max0;i++){
		for(int j = s1; double(j)*controlPointInterval+center1 < max1;j++){

			double pos0 = double(i)*controlPointInterval+center0;
			double pos1 = double(j)*controlPointInterval+center1;
			Vector3 pos (pos0*eigenVector[0]+pos1*eigenVector[1]+center2*eigenVector[2]);

			Vector3 pt;
			if((i==0 && j==0) || lineIntersectToCluster(pos, normal, pt)){ //Confirm later!
				Vector3* temp= new Vector3(pos);
				controlPoints.push_back(*temp);
			}

		}
	}

	return controlPoints.size();
}

/**
 * @brief ClusterImpl::calcControlPointsWithBoundSamres
 * @param param
 * @param bounddist the distance to boundary
 * @param principle0dist the distance to the end of the first principle axis
 * @param principle1dist the distance to the end of the second
 * @param samres sampling resolution
 * @return
 */
int ClusterImpl::calcControlPointsWithBoundSamres(double param,
												  double bounddist,
												  double principle0dist,
												  double principle1dist,
												  double samres)
{
	controlPoints.clear();

	Vector3 movedCenter;
	if(param>0.0) {
		movedCenter = center + param *
				dot(Vector3(top - center), normal) * normal;
	}
	else {
		movedCenter = center + param *
				dot(Vector3(center - bottom), normal) * normal;
	}

	double min0, max0, min1, max1, center0, center1, center2;
	center0 = min0 = max0 = dot(center,eigenVector[0]);
	center1 = min1 = max1 = dot(center,eigenVector[1]);
	center2 = dot(movedCenter,eigenVector[2]);

	// find the max and min edges of this cluster
	for(unsigned int i=0;i<triangleList.size();i++){
		for(int j=0;j<3;j++){
			double temp = dot(triangleList[i]->ver[j]->pos, eigenVector[0]);
			if(temp > max0){
				max0 =temp;
			}
			if(temp < min0){
				min0 = temp;
			}
			temp = dot(triangleList[i]->ver[j]->pos, eigenVector[1]);
			if(temp > max1){
				max1 =temp;
			}
			if(temp < min1){
				min1 = temp;
			}
		}
	}

	// 20150603, weiwei, set the sample rate according to eigen length
	min0 = min0+principle0dist;
	max0 = max0-principle0dist;
	min1 = min1+principle1dist;
	max1 = max1-principle1dist;
	// the original one has a -1 at the end
	// it seemed to be incorrect -- weiwei
	controlPointInterval=samres;
	int s0 = int(-(center0-min0)/controlPointInterval);
	int s1 = int(-(center1-min1)/controlPointInterval);

	for(int i = s0; double(i)*controlPointInterval+center0 <= max0;i++) {
		for(int j = s1; double(j)*controlPointInterval+center1 <= max1;j++) {

			double pos0 = double(i)*controlPointInterval+center0;
			double pos1 = double(j)*controlPointInterval+center1;
			Vector3 pos(pos0*eigenVector[0]+
					pos1*eigenVector[1]+center2*eigenVector[2]);

			Vector3 pt;
			// the original one has i==0&&j==0,
			// I didn't know why (0,0) is assumed to be right
			// at least in the w0r.wrl case, this is wrong --weiwei
			if(lineIntersectToCluster(pos, normal, pt)) {
				double mindist_boundary = this->pointDistToBoundary(pos);
				if(mindist_boundary>bounddist) {
					Vector3* temp = new Vector3(pos);
					controlPoints.push_back(*temp);
				}
			}
		}
	}

	return controlPoints.size();
}

// quadric approximation
/*
void ClusterImpl::calcQuadricParameter(Triangle& t) {
	// This does not work now
	const double k0=0.1;

	for(int i=0; i<3;i++){
		dvector  p = VectorXd::Zero(10);
		double x= t.ver[i]->pos[0];
		double y= t.ver[i]->pos[1];
		double z= t.ver[i]->pos[2];
		p[0] = x*x;
		p[1] = y*y;
		p[2] = z*z;
		p[3] = x*y;
		p[4] = y*z;
		p[5] = z*x;
		p[6] = k0*x;
		p[7] = k0*y;
		p[8] = k0*z;
		p[9] = k0*k0;
		for(int j=0;j<10;j++) for(int k=0;k<10;k++) sumQuadDistribution(j,k) += p[j]*p[k];
	}

	//dmatrix N=MatrixXd::Zero(10, 10);
	for(int i=0; i<3;i++){
		dmatrix dp = MatrixXd::Zero(3,10);
		double x= t.ver[i]->pos[0];
		double y= t.ver[i]->pos[1];
		double z= t.ver[i]->pos[2];
		dp(0,0) = 2.0*x;
		dp(1,1) = 2.0*y;
		dp(2,2) = 2.0*z;
		dp(0,3) = y; dp(1,3) = x;
		dp(1,4) = z; dp(2,4) = y;
		dp(2,5) = x; dp(0,5) = z;
		dp(0,6) = k0;
		dp(1,7) = k0;
		dp(2,8) = k0;
		dmatrix N_ ( prod ( trans(dp),dp) );
		sumQuadError += N_;
	}

	const int n0=10;
	const int n0w=n0*8;

	dmatrix M (sumQuadDistribution);
	dmatrix N (sumQuadError) ;

	dvector alphar=VectorXd::Zero(10);
	dvector alphai=VectorXd::Zero(10);
	dvector beta=VectorXd::Zero(10);
//	dmatrix alphai=MatrixXd::Zero(1, 10);
//	dmatrix beta=MatrixXd::Zero(1, 10);
	dmatrix evecl=MatrixXd::Zero(10, 10);
	dmatrix evecr=MatrixXd::Zero(10, 10);
	dmatrix work=MatrixXd::Zero(10*8, 10*8);
	int info;
	dggev_("V","V",&n0, &M(0,0), &n0, &N(0,0), &n0, &alphar[0], &alphai[0], &beta[0], &evecl(0,0), &n0, &evecr(0,0), &n0, &work(0,0), &n0w, &info);


	dvector eval=VectorXd::Zero(10);
	for(int i=0;i<10;i++){
		if(beta[i] != 0) eval[i] = alphar[i]/beta[i];
	}

	return;
}
*/
void ClusterImpl::calcQuadricAreaParameter() {

	const int n10=10;

	static double weight[3][3][3][3];
	static bool isFirst = true;


	if(isFirst){

		isFirst=false;

		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					for(int l=0;l<3;l++){
						int flag[3];
						flag[0] = flag[1] = flag[2] = 1;
						flag[i]++; flag[j]++; flag[k]++; flag[l]++;
						int c= flag[0]*flag[1]*flag[2];
						double mul=0;
						switch(c){
							case 5:
								mul=1.0;
								break;
							case 8:
								mul=1.0/4.0;
								break;
							case 9:
								mul=1.0/6.0;
								break;
							case 12:
								mul=1.0/12.0;
								break;
							default:
								cout << "program error" << endl;
						}
						weight[i][j][k][l] = 1.0/15.0*mul;
					}
				}
			}
		}
	}

	sumQuadDistribution=MatrixXd::Zero(10,10);
	sumQuadError=MatrixXd::Zero(9,9);
	for(int tri=0;tri<triangleList.size();tri++){

		Triangle& t=  *triangleList[tri];
		// xx yy zz xy yz zx kx ky kz kk (10 variables)            k=0.1


		VectorXd  p[3],q[3];
		p[0]= VectorXd::Zero(10);
		p[1]= VectorXd::Zero(10);
		p[2]= VectorXd::Zero(10);
		q[0]= VectorXd::Zero(10);
		q[1]= VectorXd::Zero(10);
		q[2]= VectorXd::Zero(10);
		for(int i=0; i<3;i++){
			double x= t.ver[i]->pos[0]*m0;
			double y= t.ver[i]->pos[1]*m0;
			double z= t.ver[i]->pos[2]*m0;
			p[i][0] = x;
			p[i][1] = y;
			p[i][2] = z;
			p[i][3] = x;
			p[i][4] = y;
			p[i][5] = z;
			p[i][6] = k0;
			p[i][7] = k0;
			p[i][8] = k0;
			p[i][9] = k0;

			q[i][0] = x;
			q[i][1] = y;
			q[i][2] = z;
			q[i][3] = y;
			q[i][4] = z;
			q[i][5] = x;
			q[i][6] = x;
			q[i][7] = y;
			q[i][8] = z;
			q[i][9] = k0;
		}

		MatrixXd M_ = MatrixXd::Zero(10,10);
		for(int m=0;m<10;m++){
			for(int n=0;n<10;n++){
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						for(int k=0;k<3;k++){
							for(int l=0;l<3;l++){
								M_(m,n)  = M_(m,n) + weight[i][j][k][l]*p[i][m]*q[j][m]*p[k][n]*q[l][n];
							}
						}
					}
				}
			}
		}
		sumQuadDistribution += t.area*M_;
//	//	exit(0);

		dmatrix dp[3];
		dp[0] = MatrixXd::Zero(3,9);
		dp[1] = MatrixXd::Zero(3,9);
		dp[2] = MatrixXd::Zero(3,9);
		for(int i=0; i<3;i++){
			double x= t.ver[i]->pos[0]*m0;
			double y= t.ver[i]->pos[1]*m0;
			double z= t.ver[i]->pos[2]*m0;
			dp[i](0,0) = 2.0*x;
			dp[i](1,1) = 2.0*y;
			dp[i](2,2) = 2.0*z;
			dp[i](0,3) = y; dp[i](1,3) = x;
			dp[i](1,4) = z; dp[i](2,4) = y;
			dp[i](2,5) = x; dp[i](0,5) = z;
			dp[i](0,6) = k0;
			dp[i](1,7) = k0;
			dp[i](2,8) = k0;
		}
		dmatrix N_=MatrixXd::Zero(9, 9);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				if(i==j) N_ += 1.0/6.0 * prod ( trans(dp[i]),dp[j]) ;
				else N_ += 1.0/12.0 * prod ( trans(dp[i]),dp[j]) ;
			}
		}
		sumQuadError += t.area*N_;
	}

	return;
}

void ClusterImpl::calcQuadricAreaParameterShift(Vector3 shift) {

	const int n10=10;

	static double weight[3][3][3][3];
	static bool isFirst = true;


	if(isFirst){

		isFirst=false;

		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					for(int l=0;l<3;l++){
						int flag[3];
						flag[0] = flag[1] = flag[2] = 1;
						flag[i]++; flag[j]++; flag[k]++; flag[l]++;
						int c= flag[0]*flag[1]*flag[2];
						double mul=0;
						switch(c){
							case 5:
								mul=1.0;
								break;
							case 8:
								mul=1.0/4.0;
								break;
							case 9:
								mul=1.0/6.0;
								break;
							case 12:
								mul=1.0/12.0;
								break;
							default:
								cout << "program error" << endl;
						}
						weight[i][j][k][l] = 1.0/15.0*mul;
					}
				}
			}
		}
	}

	sumQuadDistributionShift=MatrixXd::Zero(10,10);
	sumQuadErrorShift=MatrixXd::Zero(9,9);
	for(int tri=0;tri<triangleList.size();tri++){

		Triangle& t=  *triangleList[tri];
		// xx yy zz xy yz zx kx ky kz kk (10 variables)            k=0.1


		VectorXd  p[3],q[3];
		p[0]= VectorXd::Zero(10);
		p[1]= VectorXd::Zero(10);
		p[2]= VectorXd::Zero(10);
		q[0]= VectorXd::Zero(10);
		q[1]= VectorXd::Zero(10);
		q[2]= VectorXd::Zero(10);
		for(int i=0; i<3;i++){
			double x= (t.ver[i]->pos[0]-shift[0])*m0;
			double y= (t.ver[i]->pos[1]-shift[1])*m0;
			double z= (t.ver[i]->pos[2]-shift[2])*m0;
			p[i][0] = x;
			p[i][1] = y;
			p[i][2] = z;
			p[i][3] = x;
			p[i][4] = y;
			p[i][5] = z;
			p[i][6] = k0;
			p[i][7] = k0;
			p[i][8] = k0;
			p[i][9] = k0;

			q[i][0] = x;
			q[i][1] = y;
			q[i][2] = z;
			q[i][3] = y;
			q[i][4] = z;
			q[i][5] = x;
			q[i][6] = x;
			q[i][7] = y;
			q[i][8] = z;
			q[i][9] = k0;
		}

		MatrixXd M_ = MatrixXd::Zero(10,10);
		for(int m=0;m<10;m++){
			for(int n=0;n<10;n++){
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						for(int k=0;k<3;k++){
							for(int l=0;l<3;l++){
								M_(m,n)  = M_(m,n) + weight[i][j][k][l]*p[i][m]*q[j][m]*p[k][n]*q[l][n];
							}
						}
					}
				}
			}
		}
		sumQuadDistributionShift += t.area*M_;
//	//	exit(0);

		dmatrix dp[3];
		dp[0] = MatrixXd::Zero(3,9);
		dp[1] = MatrixXd::Zero(3,9);
		dp[2] = MatrixXd::Zero(3,9);
		for(int i=0; i<3;i++){
			double x= (t.ver[i]->pos[0]-shift[0])*m0;
			double y= (t.ver[i]->pos[1]-shift[1])*m0;
			double z= (t.ver[i]->pos[2]-shift[2])*m0;
			dp[i](0,0) = 2.0*x;
			dp[i](1,1) = 2.0*y;
			dp[i](2,2) = 2.0*z;
			dp[i](0,3) = y; dp[i](1,3) = x;
			dp[i](1,4) = z; dp[i](2,4) = y;
			dp[i](2,5) = x; dp[i](0,5) = z;
			dp[i](0,6) = k0;
			dp[i](1,7) = k0;
			dp[i](2,8) = k0;
		}
		dmatrix N_=MatrixXd::Zero(9, 9);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				if(i==j) N_ += 1.0/6.0 * prod ( trans(dp[i]),dp[j]) ;
				else N_ += 1.0/12.0 * prod ( trans(dp[i]),dp[j]) ;
			}
		}
		sumQuadErrorShift += t.area*N_;
	}

	return;
}

void ClusterImpl::calcQuadricAreaParameterLow() {

	const int n10=10;

	static double weight[3][3][3][3];
	static bool isFirst = true;


	if(isFirst){

		isFirst=false;

		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					for(int l=0;l<3;l++){
						int flag[3];
						flag[0] = flag[1] = flag[2] = 1;
						flag[i]++; flag[j]++; flag[k]++; flag[l]++;
						int c= flag[0]*flag[1]*flag[2];
						double mul=0;
						switch(c){
							case 5:
								mul=1.0;
								break;
							case 8:
								mul=1.0/4.0;
								break;
							case 9:
								mul=1.0/6.0;
								break;
							case 12:
								mul=1.0/12.0;
								break;
							default:
								cout << "program error" << endl;
						}
						weight[i][j][k][l] = 1.0/15.0*mul;
					}
				}
			}
		}
	}

	for(int i=0;i<16;i++){
		sumQuadDistributionLow[i]=MatrixXd::Zero(10,10);
	}
	sumQuadErrorLow[0]=MatrixXd::Zero(9,9);
	sumQuadErrorLow[1]=MatrixXd::Zero(3,9);
	sumQuadErrorLow[2]=MatrixXd::Zero(3,9);
	sumQuadErrorLow[3]=MatrixXd::Zero(1,1);
	for(int tri=0;tri<triangleList.size();tri++){

		Triangle& t=  *triangleList[tri];
		// xx yy zz xy yz zx kx ky kz kk (10 variables)            k=0.1


		VectorXd  p[3],q[3];
		p[0]= VectorXd::Zero(10);
		p[1]= VectorXd::Zero(10);
		p[2]= VectorXd::Zero(10);
		q[0]= VectorXd::Zero(10);
		q[1]= VectorXd::Zero(10);
		q[2]= VectorXd::Zero(10);
		for(int i=0; i<3;i++){
			double x= (t.ver[i]->pos[0])*m0;
			double y= (t.ver[i]->pos[1])*m0;
			double z= (t.ver[i]->pos[2])*m0;
			p[i][0] = x;
			p[i][1] = y;
			p[i][2] = z;
			p[i][3] = x;
			p[i][4] = y;
			p[i][5] = z;
			p[i][6] = k0;
			p[i][7] = k0;
			p[i][8] = k0;
			p[i][9] = k0;

			q[i][0] = x;
			q[i][1] = y;
			q[i][2] = z;
			q[i][3] = y;
			q[i][4] = z;
			q[i][5] = x;
			q[i][6] = x;
			q[i][7] = y;
			q[i][8] = z;
			q[i][9] = k0;
		}

		MatrixXd M_ = MatrixXd::Zero(10,10);
		for(int m=0;m<10;m++){
			for(int n=0;n<10;n++){
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						for(int k=0;k<3;k++){
							for(int l=0;l<3;l++){
								double a=p[i][m];
								double b=q[j][m];
								double c=p[k][n];
								double d=q[l][n];
								double w = t.area*weight[i][j][k][l];
								sumQuadDistributionLow[0](m,n) += w*a*b*c*d;

								sumQuadDistributionLow[1](m,n) += w*b*c*d;
								sumQuadDistributionLow[2](m,n) += w*a*c*d;
								sumQuadDistributionLow[3](m,n) += w*a*b*d;
								sumQuadDistributionLow[4](m,n) += w*a*b*c;

								sumQuadDistributionLow[5](m,n) += w*c*d;
								sumQuadDistributionLow[6](m,n) += w*a*d;
								sumQuadDistributionLow[7](m,n) += w*a*b;
								sumQuadDistributionLow[8](m,n) += w*b*c;
								sumQuadDistributionLow[9](m,n) += w*b*d;
								sumQuadDistributionLow[10](m,n) += w*a*c;

								sumQuadDistributionLow[11](m,n) += w*a;
								sumQuadDistributionLow[12](m,n) += w*b;
								sumQuadDistributionLow[13](m,n) += w*c;
								sumQuadDistributionLow[14](m,n) += w*d;

								sumQuadDistributionLow[15](m,n) += w;
							}
						}
					}
				}
			}
		}

		MatrixXd dp[3];
		dp[0] = MatrixXd::Zero(3,9);
		dp[1] = MatrixXd::Zero(3,9);
		dp[2] = MatrixXd::Zero(3,9);
		for(int i=0; i<3;i++){
			double x= (t.ver[i]->pos[0])*m0;
			double y= (t.ver[i]->pos[1])*m0;
			double z= (t.ver[i]->pos[2])*m0;
			dp[i](0,0) = 2.0*x;
			dp[i](1,1) = 2.0*y;
			dp[i](2,2) = 2.0*z;
			dp[i](0,3) = y; dp[i](1,3) = x;
			dp[i](1,4) = z; dp[i](2,4) = y;
			dp[i](2,5) = x; dp[i](0,5) = z;
			dp[i](0,6) = k0;
			dp[i](1,7) = k0;
			dp[i](2,8) = k0;
		}
		MatrixXd N_=MatrixXd::Zero(9, 9);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				double w = (i==j) ? 1.0/6.0 : 1.0/12.0;
				w = t.area*w;
				sumQuadErrorLow[0] += w*dp[i].transpose()*dp[j];
				sumQuadErrorLow[1] += w*dp[i];
				sumQuadErrorLow[2] += w*dp[j];
				sumQuadErrorLow[3](0,0) += w;
			}
		}
	}

	return;
}


void ClusterImpl::calcQuadricAreaParameterLowShift(Vector3 shift) {

	const int n10=10;

	VectorXd  p,q;
	p= VectorXd::Zero(10);
	q= VectorXd::Zero(10);
	double x= shift[0]*m0;
	double y= shift[1]*m0;
	double z= shift[2]*m0;
	p[0] = x;
	p[1] = y;
	p[2] = z;
	p[3] = x;
	p[4] = y;
	p[5] = z;
	p[6] = 0;
	p[7] = 0;
	p[8] = 0;
	p[9] = 0;

	q[0] = x;
	q[1] = y;
	q[2] = z;
	q[3] = y;
	q[4] = z;
	q[5] = x;
	q[6] = x;
	q[7] = y;
	q[8] = z;
	q[9] = 0;

	MatrixXd shiftMatrix[16];

	for(int i=0;i<16;i++){
		shiftMatrix[i]=MatrixXd::Zero(10,10);
	}


	for(int m=0;m<10;m++){
		for(int n=0;n<10;n++){
			double a=p[m];
			double b=q[m];
			double c=p[n];
			double d=q[n];
			shiftMatrix[0](m,n) = 1;

			shiftMatrix[1](m,n) = a;
			shiftMatrix[2](m,n) = b;
			shiftMatrix[3](m,n) = c;
			shiftMatrix[4](m,n) = d;

			shiftMatrix[5](m,n) = a*b;
			shiftMatrix[6](m,n) = b*c;
			shiftMatrix[7](m,n) = c*d;
			shiftMatrix[8](m,n) = d*a;
			shiftMatrix[9](m,n) = a*c;
			shiftMatrix[10](m,n) =b*d;

			shiftMatrix[11](m,n) =b*c*d;
			shiftMatrix[12](m,n) =a*c*d;
			shiftMatrix[13](m,n) =a*b*d;
			shiftMatrix[14](m,n) =a*b*c;

			shiftMatrix[15](m,n) = a*b*c*d;
		}
	}

	sumQuadDistributionShift = MatrixXd::Zero(10,10);
	for(int i=0;i<16;i++){
		MatrixXd temp =  sumQuadDistributionLow[i].array()*shiftMatrix[i].array();
		sumQuadDistributionShift = sumQuadDistributionShift+ temp;
	}

	MatrixXd dp;
	dp = MatrixXd::Zero(3,9);
	dp(0,0) = 2.0*x;
	dp(1,1) = 2.0*y;
	dp(2,2) = 2.0*z;
	dp(0,3) = y; dp(1,3) = x;
	dp(1,4) = z; dp(2,4) = y;
	dp(2,5) = x; dp(0,5) = z;
	dp(0,6) = 0;
	dp(1,7) = 0;
	dp(2,8) = 0;
	sumQuadErrorShift =  sumQuadErrorLow[0] + sumQuadErrorLow[1].transpose()*dp+dp.transpose()*sumQuadErrorLow[2]+sumQuadErrorLow[3](0,0)*dp.transpose()*dp;

	return;
}

/*bool ClusterPair::compClusterHeap (ClusterPair& cp1, ClusterPair& cp2) {
//	return (cp1.error > cp2.error);
//}*/
bool ClusterPair::compClusterHeap (ClusterPair* cp1, ClusterPair* cp2) {
	return (cp1->error > cp2->error);
}


void ClusterPair::mergeClusterParameter(ClusterImpl& c1,  ClusterImpl& c2){

	area = c1.area + c2.area;

	//sumCenter =  c1.sumCenter +  c2.sumCenter ;
	//center = (sumCenter/area);

	//sumDistribute = c1.sumDistribute + c2.sumDistribute;

	//for quadrics
	sumQuadDistribution = c1.sumQuadDistribution + c2.sumQuadDistribution;
	sumQuadError = c1.sumQuadError + c2.sumQuadError;

	Vector3 shift(sumQuadDistribution(6,9)/sumQuadDistribution(9,9), sumQuadDistribution(7,9)/sumQuadDistribution(9,9), sumQuadDistribution(8,9)/sumQuadDistribution(9,9) );

#ifdef CENTER_SHIFT
//	c1.calcQuadricAreaParameterShift(shift);
//	c2.calcQuadricAreaParameterShift(shift);
	c1.calcQuadricAreaParameterLowShift(-shift);
	c2.calcQuadricAreaParameterLowShift(-shift);
	sumQuadDistributionShift = c1.sumQuadDistributionShift+c2.sumQuadDistributionShift;
	sumQuadErrorShift = c1.sumQuadErrorShift+c2.sumQuadErrorShift;
#endif

	calcQuadricSurfaceParameter();
}


void ClusterData::calcQuadricSurfaceParameter(){

	//for quadrics
	int nS=9;

	dmatrix M (nS,nS);
	dmatrix N (sumQuadError);


	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)=sumQuadDistribution(i,j) - sumQuadDistribution(i,9)*sumQuadDistribution(9,j)/sumQuadDistribution(9,9);
		}
	}
#ifdef CENTER_SHIFT
	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)=sumQuadDistributionShift(i,j) - sumQuadDistributionShift(i,9)*sumQuadDistributionShift(9,j)/sumQuadDistributionShift(9,9);
		}
	}
	N = sumQuadErrorShift;
#endif


	VectorXd* eigenVecs = new VectorXd[nS+3];
	//VectorXd eigenVecs[nS+3];
	VectorXd eigenVal = VectorXd::Zero(nS+3);
	for (int i = 0; i < nS+3; i++) eigenVecs[i] =VectorXd::Zero(nS);


	// plane approximation
	const int n3=3;
	dmatrix M2 (3,3);
	dmatrix N2 (3,3);

	for (int i = 6; i < 9; i++){
		for (int j = 6; j < 9; j++){
			M2(i-6,j-6)=sumQuadDistribution(i,j) - sumQuadDistribution(i,9)*sumQuadDistribution(9,j)/sumQuadDistribution(9,9);
			N2(i-6,j-6)=sumQuadError(i,j);
		}
	}
#ifdef CENTER_SHIFT
	for (int i = 6; i < 9; i++){
		for (int j = 6; j < 9; j++){
			M2(i-6,j-6)=sumQuadDistributionShift(i,j) - sumQuadDistributionShift(i,9)*sumQuadDistributionShift(9,j)/sumQuadDistributionShift(9,9);
			N2(i-6,j-6)=sumQuadErrorShift(i,j);
		}
	}
#endif

	Eigen::SelfAdjointEigenSolver<MatrixXd> es3(N2.inverse()*M2);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			eigenVecs[j+nS][i+6] = es3.eigenvectors().col(j)[i];

	for(int i=0;i<3;i++){
		eigenVal[i+nS] = es3.eigenvalues()[i];
	}

	//Select coefficient
	int imin=nS;
	double min=fabs(eigenVal[imin]);
	for(int i=nS;i<nS+3;i++){
		if(min > fabs(eigenVal[i])){
			imin=i;
			min=fabs(eigenVal[i]);
		}
	}

	planeCoefficient = VectorXd::Zero(10);
	for(int i=0;i<9;i++){
		planeCoefficient[i] =eigenVecs[imin][i];
	}
	double planeoffset = 0;
	for(int i=0;i<9;i++){
		planeoffset += - sumQuadDistribution(i,9)*eigenVecs[imin][i];
	}
	planeoffset /= sumQuadDistribution(9,9);
	planeCoefficient[9]=planeoffset;

	quadDistribution = MatrixXd::Zero(9,9);
	quadDistribution = M.block<9,9>(0,0);

#ifdef CYLINDER_FITTING
	double dista = es3.eigenvalues()[(imin-nS+1)%3];
	double distb = es3.eigenvalues()[(imin-nS+2)%3];
	min *= (dista > distb) ? dista/distb : distb/dista;
#endif

	quadRadius = Vector3(0,0,0);
	quadRot = es3.eigenvectors();
	if(quadRot.determinant() < 0) quadRot = -quadRot;
	quadCenter=sumQuadDistribution.col(9).segment<3>(6)/sumQuadDistribution(9,9);;
#ifdef CENTER_SHIFT
	quadCenter=sumQuadDistributionShift.col(9).segment<3>(6)/sumQuadDistributionShift(9,9);;
#endif
	quadScale=0;

	//Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> es(M,N, Eigen::EigenvaluesOnly|Eigen::Ax_lBx);


	if( (min> 0.001) && (area > 0.00001)  ){
//	if( min> 0.000000001   ){
//	if(min> 100000000000){

		Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> es(M,N);
		eigenVal = es.eigenvalues();//lose the error of the plane
		for(int i=0;i<nS;i++){
//			eigenVecs[i] = es.eigenvectors().col(i).normalized();
			eigenVecs[i] = es.eigenvectors().col(i);
		}
		int i;
		eigenVal.minCoeff(&i);
		//for(int i=0;i<nS;i++){
		//	if(min <= fabs(eigenVal[i])) continue;
		if(min > fabs(eigenVal[i])){

			Matrix3 M3 = Matrix3::Zero();
			Vector3 eval;
			M3(0,0)=eigenVecs[i][0];
			M3(1,1)=eigenVecs[i][1];
			M3(2,2)=eigenVecs[i][2];
			M3(0,1)=eigenVecs[i][3]/2.0;
			M3(1,2)=eigenVecs[i][4]/2.0;
			M3(0,2)=eigenVecs[i][5]/2.0;
			M3(1,0)=eigenVecs[i][3]/2.0;
			M3(2,1)=eigenVecs[i][4]/2.0;
			M3(2,0)=eigenVecs[i][5]/2.0;
			//M3 = (M3 + M3.transpose())/2.0;
			Eigen::SelfAdjointEigenSolver<MatrixXd> M3a ( M3 );
			eval = M3a.eigenvalues();
			if( ( fabs(eval[0])+fabs(eval[1])+fabs(eval[2]) ) > 0 ) eval.normalize();

			int cnt=0;
			double sign=1;
			for(int j=0;j<n3;j++){
#ifdef CYLINDER_FITTING
				if( fabs(eval[j]) < 0.3) cnt++;
#else
				if( fabs(eval[j]) < 0.2) cnt++;
#endif
				else sign *= eval[j];
			}
#ifdef CYLINDER_FITTING
			if(  ( (sign < 0)&&(cnt == 1) )  || (cnt  != 1) ); //continue;
#else
			if(  ( (sign < 0)&&(cnt == 1) )  || (cnt > 1)  ); //continue;
#endif
			else{
				imin=i;
				min=fabs(eigenVal[i]);

				quadRadius = M3a.eigenvalues();
				quadRot = M3a.eigenvectors();

				MatrixXd M3i= M3.inverse();
				//calcPseudoInverse((MatrixXd)M3,M3i,1.0e-20);
				//quadCenter = -0.5*M3.selfadjointView<Eigen::Upper>().ldlt().solve(Vector3(eigenVecs[i][6],eigenVecs[i][7],eigenVecs[i][8]));
				quadCenter=-0.5*Matrix3(M3i)*Vector3(eigenVecs[i][6],eigenVecs[i][7],eigenVecs[i][8]);
				//quadScale = quadCenter.transpose()*M3*quadCenter;
				quadScale = -0.5*quadCenter.dot( Vector3(eigenVecs[i][6],eigenVecs[i][7],eigenVecs[i][8]) );


			}
		}
	}

	quadError = min;
	quadCoefficient = VectorXd::Zero(10);
	for(int i=0;i<9;i++){
		quadCoefficient[i] =eigenVecs[imin][i];
	}
	double offset = 0;
	for(int i=0;i<9;i++){
		offset += - sumQuadDistribution(i,9)*eigenVecs[imin][i];
	}
	offset /= sumQuadDistribution(9,9);
	quadCoefficient[9]=offset;
#ifdef CENTER_SHIFT
	offset = 0;
	for(int i=0;i<9;i++){
		offset += - sumQuadDistributionShift(i,9)*eigenVecs[imin][i];
	}
	offset /= sumQuadDistributionShift(9,9);
	quadCoefficient[9]=offset;
#endif

	quadScale -= offset;

	if(imin <nS){
		if(quadScale < 0){
			quadScale = -quadScale;
			quadRadius = -quadRadius;
		}

		int itemp;
		double dtemp;
		Vector3 vtemp;
		dtemp = quadRadius.minCoeff(&itemp);
		vtemp = quadRot.col(itemp);
		quadRadius[itemp] = quadRadius[2];
		quadRot.col(itemp) = quadRot.col(2);
		quadRadius[2] = dtemp;
		quadRot.col(2) = vtemp;
		dtemp = quadRadius.maxCoeff(&itemp);
		vtemp = quadRot.col(itemp);
		quadRadius[itemp] = quadRadius[0];
		quadRot.col(itemp) = quadRot.col(0);
		quadRadius[0] = dtemp;
		quadRot.col(0) = vtemp;
		if(quadRot.determinant() < 0) quadRot = - quadRot;

	}

	delete[] eigenVecs;
	return ;
}


void ClusterData::calcQuadricSurfaceParameterSecond(cnoid::VectorXd Coefficient, cnoid::MatrixXd sumDistribution, cnoid::MatrixXd sumError){

	//for quadrics
	int nS=9;

	dmatrix M (nS,nS);
	dmatrix N (sumQuadError);

	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)=sumDistribution(i,j) - sumDistribution(i,9)*sumDistribution(9,j)/sumDistribution(9,9);
		}
	}
#ifdef CENTER_SHIFT
	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)=sumQuadDistributionShift(i,j) - sumQuadDistributionShift(i,9)*sumQuadDistributionShift(9,j)/sumQuadDistributionShift(9,9);
		}
	}
	N = sumQuadErrorShift;
#endif


	VectorXd* eigenVecs = new VectorXd[nS+3];
	//VectorXd eigenVecs[nS+3];
	VectorXd eigenVal = VectorXd::Zero(nS+3);
	for (int i = 0; i < nS+3; i++) eigenVecs[i] =VectorXd::Zero(nS);


	// plane approximation
	const int n3=3;
	dmatrix M2 (3,3);
	dmatrix N2 (3,3);

	for (int i = 6; i < 9; i++){
		for (int j = 6; j < 9; j++){
			M2(i-6,j-6)=sumDistribution(i,j) - sumDistribution(i,9)*sumDistribution(9,j)/sumDistribution(9,9);
			N2(i-6,j-6)=sumError(i,j);
		}
	}
#ifdef CENTER_SHIFT
	for (int i = 6; i < 9; i++){
		for (int j = 6; j < 9; j++){
			M2(i-6,j-6)=sumQuadDistributionShift(i,j) - sumQuadDistributionShift(i,9)*sumQuadDistributionShift(9,j)/sumQuadDistributionShift(9,9);
			N2(i-6,j-6)=sumQuadErrorShift(i,j);
		}
	}
#endif

	Eigen::SelfAdjointEigenSolver<MatrixXd> es3(N2.inverse()*M2);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			eigenVecs[j+nS][i+6] = es3.eigenvectors().col(j)[i];

	for(int i=0;i<3;i++){
		eigenVal[i+nS] = es3.eigenvalues()[i];
	}

	//Select coefficient
	int imin=nS;
	double min=fabs(eigenVal[imin]);
	for(int i=nS;i<nS+3;i++){
		if(min > fabs(eigenVal[i])){
			imin=i;
			min=fabs(eigenVal[i]);
		}
	}

#ifdef CYLINDER_FITTING
	double dista = es3.eigenvalues()[(imin-nS+1)%3];
	double distb = es3.eigenvalues()[(imin-nS+2)%3];
	min *= (dista > distb) ? dista/distb : distb/dista;
#endif

#ifdef CENTER_SHIFT
	quadCenter=sumQuadDistributionShift.col(9).segment<3>(6)/sumQuadDistributionShift(9,9);;
#endif
	//quadScale=0;
	int mode = 0;
	Matrix3 C = Matrix3::Zero();
	//Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> es(M,N, Eigen::EigenvaluesOnly|Eigen::Ax_lBx);
/*誤差が2番目に小さい曲面*/
/*	if( (min> 0.001) && (area > 0.00001)  ){

		Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> es(M,N);
		eigenVal = es.eigenvalues();//lose the error of the plane
		for(int i=0;i<nS;i++){
			eigenVecs[i] = es.eigenvectors().col(i);
		}

		VectorXd I(nS);
		I << 0,1,2,3,4,5,6,7,8;
		int tmp;
		for(int j=0;j<nS-1;j++){
			for(int k=j+1;k<nS;k++){
				if(fabs(eigenVal[I[j]]) > fabs(eigenVal[I[k]])){
					tmp=I[j];
					I[j]=I[k];
					I[k]=tmp;
				}
			}
		}
		int i=I[1];
		if(min > fabs(eigenVal[i])){

			C(0,0)=eigenVecs[i][0];
			C(1,1)=eigenVecs[i][1];
			C(2,2)=eigenVecs[i][2];
			C(0,1)=eigenVecs[i][3]/2.0;
			C(1,2)=eigenVecs[i][4]/2.0;
			C(0,2)=eigenVecs[i][5]/2.0;
			C(1,0)=eigenVecs[i][3]/2.0;
			C(2,1)=eigenVecs[i][4]/2.0;
			C(2,0)=eigenVecs[i][5]/2.0;

			imin=i;
			min=fabs(eigenVal[i]);
			mode = 1;
		}
	}*/

	Matrix3 A = Matrix3::Zero();
	A(0,0)=Coefficient[0];
	A(1,1)=Coefficient[1];
	A(2,2)=Coefficient[2];
	A(0,1)=Coefficient[3]/2.0;
	A(1,2)=Coefficient[4]/2.0;
	A(0,2)=Coefficient[5]/2.0;
	A(1,0)=Coefficient[3]/2.0;
	A(2,1)=Coefficient[4]/2.0;
	A(2,0)=Coefficient[5]/2.0;
	//A = (A + A.transpose())/2.0;
	double k, m, r, s, vb, vv, bv, bb, vp, pv, pp, bp, nv, np, q1, q2, q3;
	Vector3d b, d, n, ghi ,p1, p2, v;

	// bin_1
	p1 << 0.000, -0.636, 0.629;
	p2 << 0.004, -0.629, 0.527;
	// blk-al-btl_1
	//p1 << 0.003, -0.636, 0.624;
	//p2 << 0.015, -0.631, 0.543;
	// blk-al-btl_2
	//p1 << 0.043, -0.644, 0.644;
	//p2 << 0.033, -0.640, 0.582;
	// orge-al-btl_1
	//p1 << -0.010, -0.636, 0.629;
	//p2 << 0.001, -0.628, 0.525;
	// blu-al-btl_1
	//p1 << -0.016, -0.634, 0.633;
	//p2 << -0.005, -0.628, 0.516;
	//p2 << -0.002, -0.606, 0.522;
	// blu-al-btl_2
	//p1 << 0.007, -0.639, 0.645;
	//p2 << 0.014, -0.639, 0.577;
	//p2 << 0.002, -0.636, 0.614;
	// wht-mug_1
	//p1 << 0.002, -0.625, 0.630;
	//p2 << 0.024, -0.620, 0.551;
	// red-mug_1
	//p1 << 0.010, -0.626, 0.628;
	//p2 << 0.024, -0.619, 0.554;
	// blwn-mug_1
	//p1 << 0.022, -0.637, 0.636;
	//p2 << 0.029, -0.629, 0.556;
	// blwn-mug_3
	//p1 << 0.024, -0.637, 0.640;
	//p2 << 0.029, -0.629, 0.556;
	// wht-cup1_1
	//p1 << 0.037, -0.646, 0.637;
	//p2 << 0.034, -0.636, 0.571;
	// wht-cup1_6
	//p1 << 0.042, -0.623, 0.642;
	//p2 << 0.040, -0.632, 0.578;
	// wht-cup2_3
	//p1 << 0.008, -0.630, 0.625;
	//p2 << 0.015, -0.620, 0.544;
	// wht-cup2_7
	//p1 << 0.052, -0.634, 0.641;
	//p2 << 0.043, -0.637, 0.584;
	//p1 << 0.042, -0.633, 0.641;
	//p2 << 0.040, -0.631, 0.631;

	v = p2-p1;
	ghi << Coefficient[6], Coefficient[7], Coefficient[8];
	n << eigenVecs[imin][6], eigenVecs[imin][7], eigenVecs[imin][8];

	double offset = 0;
	for(int i=0;i<9;i++){
		offset += - sumDistribution(i,9)*eigenVecs[imin][i];
	}
	offset /= sumDistribution(9,9);
	s = -offset;

	b = A.inverse()*ghi;
	b /= -2;
	r = b.transpose()*A*b - Coefficient[9];

	vb = v.transpose() * A * b;
	vv = v.transpose() * A * v;
	bb = b.transpose() * A * b;
	bv = b.transpose() * A * v;
	vp = v.transpose() * A * p1;
	pv = p1.transpose() * A * v;
	pp = p1.transpose() * A * p1;
	bp = b.transpose() * A * p1;
	nv = n.transpose() * v;
	np = n.transpose() * p1;

	if(mode == 0){
		q1 = nv * vv;
		//q2 = nv*(vp-pv+2*bv-2*vb) + 2*(np-d)*vv;
		q2 = 2*(np-s)*vv;
		q3 = 2*(np-s)*(vp-vb) - nv*(pp-2*bp+bb-r);

		m = (-q2+sqrt( pow(q2,2) - 4*q1*q3 )) / (2*q1);
		k = -(2*m*vv+2*vp-2*vb) / nv;
	}
/*	else{
		double t, ve, vCv, ee;
		Vector3d e;

		d = C.inverse()*n;
		d /= -2;
		t = d.transpose()*A*d - s;

		e = p1 - d;
		ve = v.transpose() * C * e;
		vCv = v.transpose() * C * v;
		ee = e.transpose() * C * e;

		q1 = ve*vv - vCv*(vp - vb);
		q2 = (ee-t)*vv - vCv*(pp-2*bp+bb-r);
		q3 = (ee-t)*(vp-vb) - ve*(pp-2*bp+bb-r);

		m = (-q2+sqrt( pow(q2,2) - 4*q1*q3 )) / (2*q1);
		k = -(m*vv+vp-vb) / (m*vCv+ve);
	}
*/
	//cout << k << endl;

	quadError += min;
	for(int i=0;i<9;i++){
		if(isnan(k)){
//			Coefficient[i] += eigenVecs[imin][i];
		}
		else
			Coefficient[i] += eigenVecs[imin][i]*k;
	}
	if(isnan(k)){
//		Coefficient[9] +=offset;
	}
	else
		Coefficient[9] +=offset*k;
	quadCoefficient = VectorXd::Zero(10);
	for(int i=0;i<10;i++){
		quadCoefficient[i] = Coefficient[i];
	}
#ifdef CENTER_SHIFT
	offset = 0;
	for(int i=0;i<9;i++){
		offset += - sumQuadDistributionShift(i,9)*eigenVecs[imin][i];
	}
	offset /= sumQuadDistributionShift(9,9);
	quadCoefficient[9]=offset;
#endif
	delete[] eigenVecs;
	return ;
}

/////////////////////////////////////////////////////
///   Calculate main axis of cluster;
/////////////////////////////////////////////////////

void ClusterData::calcMainAxis( cnoid::Vector3& v1,  cnoid::Vector3& v2){

	// plane approximation
	const int n3=3;
	dmatrix M2 (3,3);
	dmatrix N2 (3,3);

	for (int i = 6; i < 9; i++){
		for (int j = 6; j < 9; j++){
			M2(i-6,j-6)=sumQuadDistribution(i,j) - sumQuadDistribution(i,9)*sumQuadDistribution(9,j)/sumQuadDistribution(9,9);
			N2(i-6,j-6)=sumQuadError(i,j);
		}
	}

	Vector3 a[3];
	Vector3 ramda;


	Eigen::SelfAdjointEigenSolver<MatrixXd> es3(N2.inverse()*M2);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			a[i][j] = es3.eigenvectors().col(j)[i];

	for(int i=0;i<3;i++){
		ramda[i] = es3.eigenvalues()[i];
	}


	//固有値を小さい順に導出し，その順どおりに固有ベクトルを並べ替える
	for(int i=0 ; i<3 ; i++){
		int j=i%2;
		if(ramda[j] < ramda[j+1]){
			double temp=ramda[j];
			Vector3 tvec=a[j];

			ramda[j] = ramda[j+1];
			a[j] = a[j+1];
			ramda[j+1] = temp;
			a[j+1] = tvec;
		}
	}
	v1 = a[0];
	v2 = a[1];

	return ;
}


void ClusterImpl::copyParameter(ClusterData& c){

	area = c.area;

	sumCenter =  c.sumCenter;
	center = c.center;

	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			sumDistribute(j,k) = c.sumDistribute(j,k);
		}
	}
	for (int i = 0; i < 3; i++){
		eigenVector[i] = c.eigenVector[i];
	}
	eval = c.eval;

	//for quadrics
	sumQuadDistribution = c.sumQuadDistribution;
	sumQuadError = c.sumQuadError;
	quadError = c.quadError;
	quadCoefficient=c.quadCoefficient;
	planeCoefficient=c.planeCoefficient;
	quadDistribution = c.quadDistribution;

	quadCenter = c.quadCenter ;
	quadRadius = c.quadRadius ;
	quadRot = c.quadRot;
	quadScale = c.quadScale;

}

void ClusterImpl::mergeClusterForBinaryTree(ClusterImpl& c1,  ClusterImpl& c2){

	//mergeClusterParameter(c1,c2);
#ifdef CENTER_SHIFT
//	for(int i=0; i<c1.triangleList.size(); i++){
//		triangleList.push_back(c1.triangleList[i]);
//		c1.triangleList[i]->idCluster=id;
//	}
//	for(int i=0; i<c2.triangleList.size(); i++){
//		triangleList.push_back(c2.triangleList[i]);
//		c2.triangleList[i]->idCluster=id;
//	}
	for(int i=0;i<16;i++){
		sumQuadDistributionLow[i] = c1.sumQuadDistributionLow[i]+c2.sumQuadDistributionLow[i];
//		c1.sumQuadDistributionLow[i].resize(0,0);
//		c2.sumQuadDistributionLow[i].resize(0,0);
	}
	for(int i=0;i<4;i++){
		sumQuadErrorLow[i] = c1.sumQuadErrorLow[i] + c2.sumQuadErrorLow[i] ;
//		c1.sumQuadErrorLow[i].resize(0,0);
//		c2.sumQuadErrorLow[i].resize(0,0);
	}
#endif
	for(list<ClusterImpl*>::iterator it = c1.neighborClusters.begin();  it != c1.neighborClusters.end(); it++ ){
		if( *it == &c2) continue;
		neighborClusters.push_back(*it);
		list<ClusterImpl*>::iterator it2= (*it)->neighborClusters.begin();
		while (it2 != (*it)->neighborClusters.end() ){
			if( (*it2)==&c1){
				it2 = (*it)->neighborClusters.erase(it2);
			}else{
				it2++;
			}
		}
	}

	for(list<ClusterImpl*>::iterator it = c2.neighborClusters.begin();  it != c2.neighborClusters.end(); it++ ){
		if( *it == &c1) continue;
		neighborClusters.push_back(*it);
		list<ClusterImpl*>::iterator it2= (*it)->neighborClusters.begin();
		while (it2 != (*it)->neighborClusters.end() ){
			if( (*it2)==&c2){
				it2 = (*it)->neighborClusters.erase(it2);
			}else{
				it2++;
			}
		}
	}
	neighborClusters.sort();
	neighborClusters.unique();

	for(list<ClusterImpl*>::iterator it = neighborClusters.begin();  it != neighborClusters.end(); it++ ){
		(*it)->neighborClusters.push_back(this);
	}

	c1.triangleList.clear();
	c2.triangleList.clear();
	c1.neighborClusters.clear();
	c2.neighborClusters.clear();


	c1.parentNode=this;
	c2.parentNode=this;
	childNodes[0] = &c1;
	childNodes[1] = &c2;

	c1.isAliveNode=false;
	c2.isAliveNode=false;
	isAliveNode=true;

	return;
}


void ClusterData::calcQuadricSurfaceParameterOutput(){

	//for quadrics
	int nS=9;

	MatrixXd M (nS,nS);
	MatrixXd N (nS,nS);

	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)=sumQuadDistribution(i,j) - sumQuadDistribution(i,9)*sumQuadDistribution(9,j)/sumQuadDistribution(9,9);
			N(i,j)=sumQuadError(i,j);
		}
	}

	dvector* eigenVecs = new dvector[nS+3];
	//dvector eigenVecs[nS+3];
	dvector eigenVal = VectorXd::Zero(nS+3);

	Matrix3 M4;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			M4(i,j) = M(i,j);
		}
	}
	Eigen::SelfAdjointEigenSolver<MatrixXd> es3(M4);
	cout << es3.eigenvalues().transpose() << endl;
	cout << es3.eigenvectors().col(0).transpose() << endl;
	cout << es3.eigenvectors().col(1).transpose() << endl;
	cout << es3.eigenvectors().col(2).transpose() << endl;

	Eigen::GeneralizedSelfAdjointEigenSolver<MatrixXd> es(M,N);
	eigenVal = es.eigenvalues();
	for(int i=0;i<nS;i++){
		eigenVecs[i] = es.eigenvectors().col(i).normalized();
	}

	for(int i=0;i<nS;i++){
		double offset = 0;
		for(int j=0;j<9;j++){
			offset += - sumQuadDistribution(j,9)*eigenVecs[i][j]/sumQuadDistribution(9,9);
		}

		for(int j=0;j<nS;j++){
			if( fabs(eigenVecs[i][j]) < 1.0e-5) eigenVecs[i][j]=0;
		}
		cout << eigenVal[i] << " " << eigenVecs[i].transpose() << " " <<offset << endl;

		Matrix3 M3 = Matrix3::Zero();
		M3(0,0)=eigenVecs[i][0];
		M3(1,1)=eigenVecs[i][1];
		M3(2,2)=eigenVecs[i][2];
		M3(0,1)=eigenVecs[i][3]/2.0;
		M3(1,2)=eigenVecs[i][4]/2.0;
		M3(0,2)=eigenVecs[i][5]/2.0;
		M3(0,1)=eigenVecs[i][3]/2.0;
		M3(1,2)=eigenVecs[i][4]/2.0;
		M3(0,2)=eigenVecs[i][5]/2.0;
		Eigen::SelfAdjointEigenSolver<MatrixXd> M3a ( M3 );

		cout <<"base shape "<< M3a.eigenvalues().normalized().transpose() << endl ;


	}
	cout << "pos " <<  sumQuadDistribution(9,6)/sumQuadDistribution(9,9) << " " <<  sumQuadDistribution(9,7)/sumQuadDistribution(9,9) << " " <<  sumQuadDistribution(9,8)/sumQuadDistribution(9,9) << " " << endl << endl;
	cout <<"quadCenter "  << quadCenter.transpose() << endl;
	cout <<"Radius "  << 1.0/sqrt(quadRadius[0]/quadScale) << " "
					<< 1.0/sqrt(quadRadius[1]/quadScale) << " "
					<< 1.0/sqrt(quadRadius[2]/quadScale) << endl;

	cout << "Area " << area << endl;
	VectorXd temp =  quadCoefficient;
	cout << temp[0] << "*x*x+(" <<temp[1] << ")*y*y+(" <<temp[2] << ")*z*z+(" <<temp[3] << ")*x*y+(" <<temp[4] << ")*y*z+(" <<temp[5] << ")*z*x+(" <<temp[6] << ")*x+(" <<temp[7] << ")*y+("<<temp[8] << ")*z+(" <<temp[9] << ")";
	cout << endl;
	cout << endl;

	delete[] eigenVecs;
}

void ClusterImpl::writeData(std::ofstream & fout){

	Eigen::IOFormat oneline(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ");

	if(quadCoefficient.size() > 0){
		fout << "  - id:  " << id << endl;
		fout << "    quadExp:  \" "  <<  quadCoefficient.transpose().format(oneline) << " \" " << endl;
		fout << "    error:  " <<  quadError <<   endl;
		if(planeCoefficient.size() > 0){
			fout << "    planeExp:  \" "  <<  planeCoefficient.transpose().format(oneline) << " \" "<< endl;
//			fout << "  QuadDistribution:\n  " <<  quadDistribution << endl;
			fout << "    sumQuadDistribution: \"  " <<  sumQuadDistribution.format(oneline)  << " \" "<< endl;
			fout << "    sumQuadError: \"  " <<  sumQuadError.format(oneline) << " \" " << endl;
		}
	}
}

/**
 * @brief ClusterImpl::pointDistToBoundary
 * Can only be used when the point is inside the Cluster
 * usually follows lineIntersectToCluster
 * @param p
 */
double ClusterImpl::pointDistToBoundary(const Vector3 &p)
{
	// mindist should always be smaller than 1m
	double mindist = 1;
	int nbdry = boundaryList.size();
	for(int i=0; i<nbdry; i++) {
		int nverts = boundaryList[i].size();
		for(int j=0; j<nverts; j++) {
			int segstart = j;
			int segend = j+1;
			if(segend>=nverts) {
				segend = 0;
			}
			cnoid::Vector3d segstartpnt=boundaryList[i][segstart]->pos;
			cnoid::Vector3d segendpnt=boundaryList[i][segend]->pos;
			cnoid::Vector3d segvec = segendpnt-segstartpnt;
			cnoid::Vector3d pvec = p-segstartpnt;
			Vector3 segvec_e = segvec.normalized();
			cnoid::Vector3d projected_pvec = dot(pvec, segvec_e)*segvec_e;
			cnoid::Vector3d heightvec = pvec-projected_pvec;
			double dist_p_seg = heightvec.norm();
			if(dist_p_seg<mindist) {
				mindist = dist_p_seg;
			}
		}
	}

	return mindist;
}

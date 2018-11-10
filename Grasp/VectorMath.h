// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _VectorMath_H
#define _VectorMath_H

#ifndef constpi
#define constpi 3.141592
#endif

#include <vector>
#include <cnoid/EigenTypes>
#include <Eigen/Eigenvalues>
#include "exportdef.h"

namespace grasp{
#ifdef WIN32
    int EXCADE_API isnan(double x);
    int EXCADE_API isfinite(double x);
#else
	template<typename T>
	bool isnan(const T x) {
#ifdef CNOID_GE_16
		return std::isnan(x);
#else
		return ::isnan(x);
#endif
	}
#endif
    typedef cnoid::Matrix3 Matrix33;
    typedef cnoid::VectorXd dvector ;
    typedef cnoid::MatrixXd dmatrix ;

    cnoid::Matrix3 EXCADE_API sqew(const cnoid::Vector3& a);
    cnoid::Matrix3 EXCADE_API diag(double a, double b, double c);
    cnoid::Matrix3 EXCADE_API v3(const cnoid::Vector3& x,
                                 const cnoid::Vector3& y,
                                 const cnoid::Vector3& z);
    cnoid::Matrix3 EXCADE_API vr3(const cnoid::Vector3& x,
                                  const cnoid::Vector3& y,
                                  const cnoid::Vector3& z);
    cnoid::Matrix3 EXCADE_API m33(const cnoid::Vector3& x);
    cnoid::Matrix3 EXCADE_API d2v(const cnoid::MatrixXd& a);
    cnoid::MatrixXd EXCADE_API v2d(const cnoid::Matrix3& a);
    cnoid::Vector3 EXCADE_API col(const cnoid::Matrix3 &a, int n);
    cnoid::Vector3 EXCADE_API sort(const cnoid::Vector3 &a);
    cnoid::Vector3 EXCADE_API average(const std::vector<cnoid::Vector3> &a);
    double EXCADE_API average(const std::vector<double> &a);

    double EXCADE_API dbl(double a);
    double EXCADE_API dbl(cnoid::Vector3 &a);

    double EXCADE_API vmax(const std::vector<double> &a);
    int EXCADE_API argmax(const std::vector<double> &a);
    double EXCADE_API vmin(const std::vector<double> &a);
    int EXCADE_API argmin(const std::vector<double> &a);
    double EXCADE_API abs(const cnoid::Vector3& a);
    cnoid::Vector3 EXCADE_API unit(const cnoid::Vector3& a);

    double EXCADE_API det33(const cnoid::Matrix3& V);
    cnoid::Vector3 EXCADE_API cross( cnoid::Vector3 v1,cnoid::Vector3 v2);
    double EXCADE_API norm2( cnoid::Vector3 v);
    double EXCADE_API norm_2( cnoid::VectorXd v);
    cnoid::MatrixXd EXCADE_API inverse( cnoid::MatrixXd m);
    cnoid::Matrix3 EXCADE_API trans(const cnoid::Matrix3& m);
    cnoid::MatrixXd EXCADE_API trans(const cnoid::MatrixXd& m);
    double EXCADE_API min ( cnoid::Vector3 v );
    double EXCADE_API min ( double a, double b );
    double EXCADE_API dot( cnoid::Vector3 v1,cnoid::Vector3 v2);
    double EXCADE_API inner_prod( cnoid::VectorXd v1,cnoid::VectorXd v2);
    int EXCADE_API calcEigenVectors(const cnoid::MatrixXd &_a, cnoid::MatrixXd  &_evec, cnoid::VectorXd &_eval);
    int EXCADE_API calcPseudoInverse(const cnoid::MatrixXd &_a, cnoid::MatrixXd &_a_pseu, double _sv_ratio=1.0e-3);
    int EXCADE_API solveLinearEquation(const cnoid::MatrixXd &_a, const cnoid::VectorXd &_b, cnoid::VectorXd &_x, double _sv_ratio=1.0e-3);
    cnoid::Vector3 EXCADE_API omegaFromRot(const cnoid::Matrix3& r);
    cnoid::VectorXd EXCADE_API prod (const cnoid::MatrixXd &_a, const cnoid::VectorXd &_b);
    cnoid::MatrixXd EXCADE_API prod (const cnoid::MatrixXd &_a, const cnoid::MatrixXd &_b);

    template<class V> inline void setVector3(const cnoid::Vector3& v3, V& v, size_t top = 0){
        v[top++] = v3(0); v[top++] = v3(1); v[top] = v3(2);
    }

    template<class V> inline void setVector3(const cnoid::Vector3& v3, const V& v, size_t top = 0){
        v[top++] = v3(0); v[top++] = v3(1); v[top] = v3(2);
    }

    template<class V> inline void getVector3(cnoid::Vector3& v3, const V& v, size_t top = 0){
        v3(0) = v[top++]; v3(1) = v[top++]; v3(2) = v[top];
    }

    template<class M> inline void setVector3(const cnoid::Vector3& v3, M& m, size_t row, size_t col){
        m(row++, col) = v3(0); m(row++, col) = v3(1); m(row, col) = v3(2);
    }

    template<class M> inline void getVector3(cnoid::Vector3& v3, const M& m, size_t row, size_t col){
        v3(0) = m(row++, col);
        v3(1) = m(row++, col);
        v3(2) = m(row, col);
    }

    void EXCADE_API calcRodrigues(cnoid::Matrix3& out_R, const cnoid::Vector3& axis, double q);
    void EXCADE_API calcRotFromRpy(cnoid::Matrix3& out_R, double r, double p, double y);
    cnoid::Vector3 EXCADE_API rpyFromRot(const cnoid::Matrix3& m);
    cnoid::Matrix3 EXCADE_API rodrigues(const cnoid::Vector3& axis, double q);
    void EXCADE_API invRodrigues(cnoid::Vector3& out_axis,
                                 double& out_angle,
                                 cnoid::Matrix3 in_R);
    cnoid::Matrix3 EXCADE_API rotFromTwoVecs(cnoid::Vector3& vec1, cnoid::Vector3& vec2);
    cnoid::Matrix3 EXCADE_API rotFromRpy(const cnoid::Vector3& rpy);
    cnoid::Matrix3 EXCADE_API rotFromRpy(double r, double p, double y);
    double EXCADE_API det(const cnoid::MatrixXd &_a);

    double EXCADE_API distance(const cnoid::Vector3 & pt, const cnoid::VectorXd &a);
    cnoid::Vector3 EXCADE_API normalPoint(const cnoid::Vector3& pt, const cnoid::VectorXd& r);
    cnoid::Vector3 EXCADE_API normalPoint(const cnoid::Vector3& pt, const cnoid::Vector3& p, const cnoid::Vector3& e);
    cnoid::Vector3 EXCADE_API projection(const cnoid::Vector3& e, const cnoid::VectorXd& r);

    double EXCADE_API cos_th(const cnoid::Vector3& p0,
                             const cnoid::Vector3& p1,
                             const cnoid::Vector3& p2);
    double EXCADE_API angleOfTwoVecs(const cnoid::Vector3& v0,
                                     const cnoid::Vector3& v1);
    void EXCADE_API parallelPlane(const cnoid::VectorXd& r0, double l, cnoid::VectorXd& r1);
    bool EXCADE_API calcCommonLine(const cnoid::VectorXd& r0, const cnoid::VectorXd& r1, cnoid::Vector3& p, cnoid::Vector3& e);
    double EXCADE_API area(const cnoid::Vector3& p0, const cnoid::Vector3& p1, const cnoid::Vector3& p2);
    cnoid::Vector3 calcCOM(const std::vector<cnoid::Vector3>& p);
    void EXCADE_API sort_by(std::vector<cnoid::Vector3>& p, std::vector<double>& dist);
    void EXCADE_API sort_by(std::vector<double>& p, std::vector<double>& dist);
    void EXCADE_API sort_by(std::vector<int>& p, std::vector<double>& dist);
    bool EXCADE_API included(int a, const std::vector<int> b);
    int EXCADE_API argument(const std::vector<double>& a, double b);
    int EXCADE_API argument(const std::vector<int>& a, int n);
    double EXCADE_API minDistancePoints(const cnoid::Vector3& p1, const cnoid::Vector3& nn1, const cnoid::Vector3& p2, const cnoid::Vector3& nn2, cnoid::Vector3& q1, cnoid::Vector3& q2); //minimum distance points between two lines
    cnoid::Vector3 EXCADE_API intersectionPoint(const cnoid::Vector3& p1, const cnoid::Vector3& nn1, const cnoid::Vector3& planePt, const cnoid::Vector3& planeN);//intersection between line and plane
    bool EXCADE_API twoSegmentsIntersect(const cnoid::Vector3& P1,const cnoid::Vector3& P2,const cnoid::Vector3& P3,const cnoid::Vector3& P4,const cnoid::Vector3& n,cnoid::Vector3& Pout);

    double EXCADE_API cpoly4(double xa[4], double z);
    /// second degree equation
    int EXCADE_API sol2(double xa[3], double xz[2]);
    /// third degree equation
    int EXCADE_API sol3(double xa[4], double xz[3]);
    /// fourth degree equation
    int EXCADE_API sol4(double xa[5], double xz[4]);

    class Homogenous
    {
    public:
        cnoid::Vector3 p;
        cnoid::Matrix3 R;
    };
}

#endif

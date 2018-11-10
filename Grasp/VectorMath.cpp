/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <math.h>
#include <iostream>

#include "VectorMath.h"
#define ALM0 1e-10

using namespace std;
using namespace cnoid;

namespace grasp{

#ifdef WIN32
    int isnan(double x){return _isnan(x);}
    int isfinite(double x){return _finite(x);}
#endif

Matrix3 sqew(const Vector3& a){
  Matrix3 ret;

  ret(0,0)=0.0;      ret(0,1)=-a(2);    ret(0,2)=a(1);
  ret(1,0)=a(2);     ret(1,1)=0.0;      ret(1,2)=-a(0);
  ret(2,0)=-a(1);    ret(2,1)=a(0);     ret(2,2)=0.0;

  return ret;
}

Matrix3 diag(double a, double b, double c){
  Matrix3 ret;

  ret(0,0)=a;      ret(0,1)=0.0;    ret(0,2)=0.0;
  ret(1,0)=0.0;    ret(1,1)=b;      ret(1,2)=0.0;
  ret(2,0)=0.0;    ret(2,1)=0.0;    ret(2,2)=c;

  return ret;
}

Matrix3 v3(const Vector3& x, const Vector3& y, const Vector3& z){
  Matrix3 ret;

  ret(0,0)=x(0);    ret(0,1)=y(0);    ret(0,2)=z(0);
  ret(1,0)=x(1);    ret(1,1)=y(1);    ret(1,2)=z(1);
  ret(2,0)=x(2);    ret(2,1)=y(2);    ret(2,2)=z(2);

  return ret;
}

Matrix3 vr3(const Vector3& x, const Vector3& y, const Vector3& z){
  Matrix3 ret;

  ret(0,0)=x(0);    ret(0,1)=x(1);    ret(0,2)=x(2);
  ret(1,0)=y(0);    ret(1,1)=y(1);    ret(1,2)=y(2);
  ret(2,0)=z(0);    ret(2,1)=z(1);    ret(2,2)=z(2);

  return ret;
}


Matrix3 m33(const Vector3& x){

  Matrix3 ret;

  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
      ret(i,j) = x(i)*x(j);

  return ret;
}

Matrix3 d2v(const MatrixXd& a){
  Matrix3 ret;

  for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
      ret(i,j)=a(i,j);

  return ret;
}

MatrixXd v2d(const Matrix3& a){
  MatrixXd ret;

  for(int i=0; i<3; i++)
      for(int j=0; j<3; j++)
      ret(i,j)=a(i,j);

  return ret;
}

Vector3 col(const Matrix3 &a, int n)
{
  Vector3 ret;
  for(int i=0; i<3; i++)
      ret(i)=a(i,n);

  return ret;
}

Vector3 sort(const Vector3 &a)
{
  double b[3];
  for(int i=0; i<3; i++)
      b[i] = a[i];

  std::sort(b,b+3);

  Vector3 c;
  for(int i=0; i<3; i++)
      c[i] = b[i];

  return c;
}

cnoid::Vector3 average(const std::vector<cnoid::Vector3> &a)
{
  Vector3 sum = Vector3::Zero();

  for(unsigned int i=0; i<a.size(); i++)
    sum += a[i];

  sum /= (double)a.size();

  return sum;
}

double average(const std::vector<double> &a)
{
  double sum = 0.0;

  for(unsigned int i=0; i<a.size(); i++)
    sum += a[i];

  sum /= (double)a.size();

  return sum;
}

double dbl(double a){ return a*a;}
double dbl(Vector3 &a){ return (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);}


     Vector3 cross( cnoid::Vector3 v1,cnoid::Vector3 v2){
        return v1.cross(v2);
    }
     double norm2( cnoid::Vector3 v){
        return v.norm();
    }
     double norm_2( cnoid::VectorXd v){
        return v.norm();
    }
     MatrixXd inverse( cnoid::MatrixXd m){
         return m.inverse();
    }
     Matrix3 trans(const cnoid::Matrix3& m){
        return m.transpose();
    }
     MatrixXd trans(const cnoid::MatrixXd& m){
        return m.transpose();
    }
     double min ( cnoid::Vector3 v ){
        return v.minCoeff();
    }
     double min ( double a, double b ){
        return (a<b)? a : b;
    }


     double dot( cnoid::Vector3 v1,cnoid::Vector3 v2){
        return v1.dot(v2);
    }
     double inner_prod( cnoid::VectorXd v1,cnoid::VectorXd v2){
        return v1.dot(v2);
    }

     int calcEigenVectors(const cnoid::MatrixXd &_a, cnoid::MatrixXd  &_evec, cnoid::VectorXd &_eval)
    {
        assert( _a.rows() == _a.cols() );
        if( _a == MatrixXd::Zero(_a.rows(),_a.cols()) ){
            _eval = VectorXd::Zero(_a.rows());
            _evec = MatrixXd::Zero(_a.rows(),_a.cols());
        }else{
            Eigen::EigenSolver<cnoid::MatrixXd> es(_a);
            _eval = es.eigenvalues().real();
            _evec = es.eigenvectors().real();
        }
//		cout << "eigenvector" << endl<<_eval.transpose() << endl << _evec << endl;
        return 0;
    }
     int calcPseudoInverse(const cnoid::MatrixXd &_a, cnoid::MatrixXd &_a_pseu, double _sv_ratio)
    {
        Eigen::JacobiSVD<cnoid::MatrixXd> svd(_a, Eigen::ComputeThinU | Eigen::ComputeThinV);

        cnoid::VectorXd s = svd.singularValues();
        int i, j, k;

        double smin, smax=0.0;
        for (j = 0; j < s.size() ; j++) if (s[j] > smax) smax = s[j];
        smin = smax*_sv_ratio; 			// default _sv_ratio is 1.0e-3

        for (j = 0; j < s.size() ; j++){
            if (s[j] <= smin) s[j] = 0.0;
            else s[j] = 1.0/s[j];
        }
        // V * (S^(-1)*U^(T))
        _a_pseu = svd.matrixV() * s.asDiagonal() * svd.matrixU().transpose();

       return 0;
    }


     int solveLinearEquation(const cnoid::MatrixXd &_a, const cnoid::VectorXd &_b, cnoid::VectorXd &_x, double _sv_ratio)
    {
        if(_a.rows() == _a.cols() ){
             _x = _a.fullPivLu().solve(_b);
        }
        else{
            cnoid::MatrixXd _a_pseu;
            calcPseudoInverse(_a,_a_pseu,_sv_ratio);
            _x = _a_pseu* _b;
        }
        return 0;
    }
     Vector3 omegaFromRot(const cnoid::Matrix3& r)
    {
        using ::std::numeric_limits;

        double alpha = (r(0,0) + r(1,1) + r(2,2) - 1.0) / 2.0;

        if(fabs(alpha - 1.0) < 1.0e-6) {   //th=0,2PI;
            return cnoid::Vector3::Zero();

        } else {
					if (alpha > 1.0 || alpha < -1.0) {
						std::cerr << "[WARNING] : Input matrix of omegaFromRot fucntion is NOT an orthogonal matrix." << std::endl;
						std::cerr << "Input matrix:" << std::endl << r << std::endl;
					}
            double th = acos(alpha);
            double s = sin(th);

            if (s < numeric_limits<double>::epsilon()) {   //th=PI
                return cnoid::Vector3( sqrt((r(0,0)+1)*0.5)*th, sqrt((r(1,1)+1)*0.5)*th, sqrt((r(2,2)+1)*0.5)*th );
            }

            double k = -0.5 * th / s;

            return cnoid::Vector3( (r(1,2) - r(2,1)) * k,
                            (r(2,0) - r(0,2)) * k,
                            (r(0,1) - r(1,0)) * k );
        }
    }

    VectorXd prod (const cnoid::MatrixXd &_a, const cnoid::VectorXd &_b){
        return _a*_b;
    }
    MatrixXd prod (const cnoid::MatrixXd &_a, const cnoid::MatrixXd &_b){
        return _a*_b;
    }

    void calcRodrigues(cnoid::Matrix3& out_R, const cnoid::Vector3& axis, double q){
        const double sth = sin(q);
        const double vth = 1.0 - cos(q);

        double ax = axis(0);
        double ay = axis(1);
        double az = axis(2);

        const double axx = ax*ax*vth;
        const double ayy = ay*ay*vth;
        const double azz = az*az*vth;
        const double axy = ax*ay*vth;
        const double ayz = ay*az*vth;
        const double azx = az*ax*vth;

        ax *= sth;
        ay *= sth;
        az *= sth;

        out_R <<  1.0 - azz - ayy, -az + axy,       ay + azx,
            az + axy,        1.0 - azz - axx, -ax + ayz,
            -ay + azx,       ax + ayz,        1.0 - ayy - axx;
    }

     void calcRotFromRpy(cnoid::Matrix3& out_R, double r, double p, double y){
        const double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y);
        out_R(0,0)= cp*cy;
        out_R(0,1)= sr*sp*cy - cr*sy;
        out_R(0,2)= cr*sp*cy + sr*sy;
        out_R(1,0)= cp*sy;
        out_R(1,1)= sr*sp*sy + cr*cy;
        out_R(1,2)= cr*sp*sy - sr*cy;
        out_R(2,0)= -sp;
        out_R(2,1)= sr*cp;
        out_R(2,2)= cr*cp;
    }

     Vector3 rpyFromRot(const cnoid::Matrix3& m)
    {
        double roll, pitch, yaw;

        if ((fabs(m(0,0))<fabs(m(2,0))) && (fabs(m(1,0))<fabs(m(2,0)))) {
            // cos(p) is nearly = 0
            double sp = -m(2,0);
            if (sp < -1.0) {
                sp = -1;
            } else if (sp > 1.0) {
                sp = 1;
            }
            pitch = asin(sp); // -pi/2< p < pi/2

            roll = atan2(sp*m(0,1)+m(1,2),  // -cp*cp*sr*cy
                         sp*m(0,2)-m(1,1)); // -cp*cp*cr*cy

            if (m(0,0)>0.0) { // cy > 0
                (roll < 0.0) ? (roll += M_PI) : (roll -= M_PI);
            }
            double sr=sin(roll), cr=cos(roll);
            if (sp > 0.0) {
                yaw = atan2(sr*m(1,1)+cr*m(1,2), //sy*sp
                            sr*m(0,1)+cr*m(0,2));//cy*sp
            } else {
                yaw = atan2(-sr*m(1,1)-cr*m(1,2),
                            -sr*m(0,1)-cr*m(0,2));
            }
        } else {
            yaw = atan2(m(1,0), m(0,0));
            const double sa = sin(yaw);
            const double ca = cos(yaw);
            pitch = atan2(-m(2,0), ca*m(0,0)+sa*m(1,0));
            roll = atan2(sa*m(0,2)-ca*m(1,2), -sa*m(0,1)+ca*m(1,1));
        }
        return cnoid::Vector3(roll, pitch, yaw);
    }

    Matrix3 rodrigues(const cnoid::Vector3& axis, double q){
        cnoid::Matrix3 R;
        calcRodrigues(R, axis, q);
        return R;
    }

        Matrix3 rotFromTwoVecs(cnoid::Vector3& vec1, cnoid::Vector3& vec2){
            double q;
            cnoid::Vector3 axis;
            cnoid::Matrix3 R;

            if(vec1 == vec2) return cnoid::Matrix3::Identity(3,3);

            q = acos(dot(vec1, vec2)/(norm2(vec1)*norm2(vec2)));
            axis = vec1.cross(vec2);
            axis = axis/norm2(axis);
            calcRodrigues(R, axis, q);
        return R;
        }

    Matrix3 rotFromRpy(const cnoid::Vector3& rpy){
        cnoid::Matrix3 R;
        calcRotFromRpy(R, rpy[0], rpy[1], rpy[2]);
        return R;
    }

    Matrix3 rotFromRpy(double r, double p, double y){
        cnoid::Matrix3 R;
        calcRotFromRpy(R, r, p, y);
        return R;
    }

     double det(const cnoid::MatrixXd &_a)
    {
        assert( _a.rows() == _a.cols() );
        return _a.determinant();
        // simple determinant
    }


double vmax(const vector<double> &a){

  vector<double>::const_iterator I=a.begin();

  double x = *I;

  for(I=a.begin()+1; I!=a.end(); I++)
    if(*I > x)
      x = *I;

  return x;
}

int argmax(const vector<double> &a){

    if(a.size()==0){
        cout << "WARINIG: input size 0 in argmax" << endl;
        return -1;
    }

    int arg=0;
    double x=a[0];
    for(int i=1; i<(int)a.size(); i++)
        if(a[i]>x){
            x = a[i];
            arg = i;
        }

    return arg;
}

double vmin(const vector<double> &a){

  vector<double>::const_iterator I=a.begin();

  double x = *I;

  for(I=a.begin()+1; I!=a.end(); I++)
    if(*I < x)
      x = *I;

  return x;
}

int argmin(const vector<double> &a){

    if(a.size()==0){
        cout << "WARINIG: input size 0 in argmin" << endl;
        return -1;
    }

    int arg=0;
    double x=a[0];
    for(int i=1; i<(int)a.size(); i++)
        if(a[i]<x){
            x = a[i];
            arg = i;
        }

    return arg;
}

double abs(const Vector3& a)
{
  return sqrt(a(0)*a(0)+a(1)*a(1)+a(2)*a(2));
}

Vector3 unit(const Vector3& a)
{
  Vector3 b;

  if(isfinite(1.0/abs(a)))
    b= a/abs(a);
  else
    b << 0, 0, 0;

  return b;
}

double det33(const Matrix3& V)
{
  double d = V(0,0)*V(1,1)*V(2,2) + V(0,1)*V(1,2)*V(2,0) + V(1,0)*V(2,1)*V(0,2)
    -V(2,0)*V(1,1)*V(0,2) - V(1,0)*V(0,1)*V(2,2) - V(0,0)*V(2,1)*V(1,2);

  return d;
}

double distance(const Vector3 & pt, const VectorXd &r)
{
  //Distance between (pt) and (r(0) x + r(1) y + r(2) z + r(3) = 0)

  return (r(0)*pt(0)+r(1)*pt(1)+r(2)*pt(2)+r(3))/sqrt(r(0)*r(0)+r(1)*r(1)+r(2)*r(2));
}

Vector3 normalPoint(const Vector3& pt, const VectorXd& r)
{
  //Normal point of (pt) on the plane (r(0) x + r(1) y + r(2) z + r(3) = 0)

  Vector3 a(r(0), r(1), r(2));
  double b=r(3);

  return Vector3(pt - a*(dot(a,pt)+b)/dot(a,a));
}

Vector3 normalPoint(const Vector3& pt, const Vector3& p, const Vector3& e)
{
  //Normal point of (pt) on the line (p + t e)

  double t = dot(e, Vector3(pt-p))/dot(e,e);

  return Vector3(p + t*e);
}

Vector3 projection(const Vector3& e, const VectorXd& r)
{
  //Projection of the vector (e) onto (r(0) x + r(1) y + r(2) z + r(3) = 0)

  Vector3 n( r(0), r(1), r(2) );
  n=n/abs(n);

  Vector3 v = Vector3(e/abs(e));

  double s = dot(n, v);
  double t = sqrt(1 - s*s);

  return Vector3( (v-s*n)/t );
}

double cos_th(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  //Cosine theorem
  Vector3 p10 = Vector3(p1-p0);
  Vector3 p20 = Vector3(p2-p0);
  Vector3 p21 = Vector3(p2-p1);

  return (dot(p10, p10) + dot(p20,p20) - dot(p21, p21) ) / ( 2.0*abs(p10)*abs(p20) );
}

void parallelPlane(const VectorXd& r0, double l, VectorXd& r1)
{
  //Plane with distace (l) from (r0(0) x + r0(1) y + r0(2) z + r0(3) = 0)

  for(int i=0; i<3; i++)
      r1(i) = r0(i);

  r1(3) = r0(3) - l * sqrt(r0(0)*r0(0)+r0(1)*r0(1)+r0(2)*r0(2));

  return;
}

bool calcCommonLine(const VectorXd& r0, const VectorXd& r1, Vector3& p, Vector3& e)
{
  //Common line (p + t e) between two planes  (r0(0) x + r0(1) y + r0(2) z + r0(3) = 0) and (r1(0) x + r1(1) y + r1(2) z + r1(3) = 0)
  Vector3 a0(r0(0), r0(1), r0(2));
  Vector3 a1(r1(0), r1(1), r1(2));

  Vector3 e_tmp = cross(a0, a1);

  if(abs(e_tmp) < 1.0e-15)
     return false;

  e = e_tmp/abs(e_tmp);

  MatrixXd V(2,3);
  V(0,0) = r0(0);  V(0,1) = r0(1);  V(0,2) = r0(2);
  V(1,0) = r1(0);  V(1,1) = r1(1);  V(1,2) = r1(2);

  VectorXd d(2), x(3);
  d(0) = -r0(3);
  d(1) = -r1(3);

  solveLinearEquation(V, d, x);

  p << x(0), x(1), x(2);

  return true;

}

double area(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
  Vector3 alpha = normalPoint(p0, p1, Vector3(p2-p1));

  return 0.5*abs(Vector3(p0-alpha))*abs(Vector3(p2-p1));
}

Vector3 calcCOM(const vector<Vector3>& p)
{
  Vector3 a(0.0, 0.0, 0.0);

  for(vector<Vector3>::const_iterator I=p.begin(); I!=p.end(); I++)
      a += *I;

  return Vector3(a/(double)p.size());
}

void sort_by(vector<Vector3>& p, vector<double>& dist)
{

  if((int)p.size()<2)
      return;

  bool again;

  do{

      again = false;
      vector<Vector3>::iterator IP=p.begin()+1;
      vector<double>::iterator ID=dist.begin()+1;

      while( ID != dist.end() ) {

          if( *(ID-1) > *ID ){
          reverse(ID-1, ID+1);
          reverse(IP-1, IP+1);
          again = true;
      }
      IP++;
      ID++;
      }

  }while(again);

  return;

}

void sort_by(vector<double>& p, vector<double>& dist)
{

  if((int)p.size()<2)
      return;

  bool again;

  do{

      again = false;
      vector<double>::iterator IP=p.begin()+1;
      vector<double>::iterator ID=dist.begin()+1;

      while( ID != dist.end() ) {

          if( *(ID-1) > *ID ){
          reverse(ID-1, ID+1);
          reverse(IP-1, IP+1);
          again = true;
      }
      IP++;
      ID++;
      }

  }while(again);

  return;

}

void sort_by(vector<int>& p, vector<double>& dist)
{

  if((int)p.size()<2)
      return;

  bool again;

  do{

      again = false;
      vector<int>::iterator IP=p.begin()+1;
      vector<double>::iterator ID=dist.begin()+1;

      while( ID != dist.end() ) {

          if( *(ID-1) > *ID ){
          reverse(ID-1, ID+1);
          reverse(IP-1, IP+1);
          again = true;
      }
      IP++;
      ID++;
      }

  }while(again);

  return;

}

bool included(int a, const vector<int> b){

    for(unsigned int i=0; i<b.size(); i++)
        if(a==b[i]) return true;

    return false;
}

int argument(const vector<double> &a, double b){

  int arg=0;
  double x=a[0];
  for(int i=1; i<(int)a.size(); i++)
    if(fabs(a[i]-b) < fabs(x-b) ){
      x = a[i];
      arg = i;
    }

  return arg;
}

int argument(const vector<int>& a, int n){

  for(unsigned int i=0; i<a.size(); i++)
    if(a[i] == n)
      return i;

  return -1;
}

double minDistancePoints(const Vector3& p1, const Vector3& nn1, const Vector3& p2, const Vector3& nn2, Vector3& q1, Vector3& q2)
{
        Vector3 n1(nn1/norm2(nn1));
        Vector3 n2(nn2/norm2(nn2));


        double l1 = (dot(n1, p2-p1) + dot(n1,n2)*dot(n2,p1-p2))/(1-dot(n1,n2)*dot(n1,n2));
        double l2 = (dot(n1,n2)*dot(n1, p2-p1) + dot(n2,p1-p2))/(1-dot(n1,n2)*dot(n1,n2));

        q1 = p1 + l1*n1;
        q2 = p2 + l2*n2;

        return norm2(q1-q2);
}

Vector3 intersectionPoint(const Vector3& p1, const Vector3& nn1, const Vector3& planePt, const Vector3& planeN)
{

  double t = dot(planeN, Vector3(planePt-p1))/dot(nn1, planeN);

  if(isfinite(t))
    return Vector3(p1 + t*nn1);
  else
    return Vector3(0.0, 0.0, 0.0);
}

bool twoSegmentsIntersect(const Vector3& P1, const Vector3& P2, const Vector3& P3, const Vector3& P4, const Vector3& n, Vector3& Pout)
{
        double eps = 0.00001;

        Matrix3 A = v3(n, P2-P1, P3-P4);

        Vector3 x = A.inverse()*(P3-P1);

        Pout = P3 + x(2)*(P4-P3);

        if(x(1)>eps && x(1)<1-eps && x(2)>eps && x(2)<1-eps)
                return true;
        return false;
}

double cpoly4(double xa[4], double z) {
    return xa[0] + z*(xa[1] + z*(xa[2] + z*(xa[3] + z*xa[4])));
}


int sol2(double xa[3], double xz[2]) {
    if(xa[2]*xa[2] <= ALM0*ALM0){
        if(xa[1]*xa[1] <= ALM0*ALM0) return 0;
        xz[0] = xz[1] = -xa[0]/xa[1];
        return 1;
    }
    double D = xa[1] * xa[1] - 4.0 * xa[2] * xa[0];
//	cout << "D"<< D << " "<<xa[2] << " "<<xa[1] << " "<<xa[0] << endl;
    if (D < -ALM0*ALM0) return 0;
    if (D < 0) D = 0;
    double cx = sqrt(D);
    xz[0] = (-xa[1] + cx) / (2.0 * xa[2]);
    xz[1] = (-xa[1] - cx) / (2.0 * xa[2]);
    return 2;
}


int sol3(double xa[4], double xz[3]) {
    double p, q, b[3];

    //xa[3] x*x*x + xa[2] x*x + xa[1] x + xa[0]
    if(xa[3]*xa[3]<= ALM0*ALM0) return sol2(xa,xz);

    b[0] = xa[0] / xa[3];
    b[1] = xa[1] / xa[3];
    b[2] = xa[2] / xa[3];

    p = (3.0 * b[1] - b[2] * b[2]) / 9.0;
    q = (2.0 * b[2] * b[2] * b[2] - 9.0 * b[2] * b[1] + 27.0 * b[0]) / 27.0;

    double cxr = 0, cxi = 0;
    double D = (q * q + 4.0 * p * p * p);

    //cout << D<< endl;

    if ( D > 0) {
        cxr = sqrt(D);
    } else if (D < 0) {
        cxi = sqrt(-D);
    }
    //cx=csqrt(q*q+4.0*p*p*p);
    cxr = 0.5 * (-q + cxr);
    cxi = 0.5 * cxi;

    double rtt = pow(cxr * cxr + cxi * cxi, 1.0 / 6.0), th = atan2(cxi, cxr) / 3.0;

    double ct1r = rtt * cos(th);
    double ct1i = rtt * sin(th);
    //ct1=ccbrt(0.5*(-q+cx));

    if (D > 0) {
        xz[0] = ct1r - p / ct1r - b[2] / 3.0;
        xz[1] = 0;
        xz[2] = 0;
        return 1;
    }

    double ct2r = ct1r;
    xz[0] = ct1r + ct2r - b[2] / 3.0;
    xz[1] = -ct1r - sqrt(3.0) * ct1i - b[2] / 3.0;
    xz[2] = -ct1r + sqrt(3.0) * ct1i - b[2] / 3.0;
    return 3;

}

int sol4(double xa[5], double xz[4]) {
    double p, q, r;
    double b[4], cb[4], cz[3];
    std::complex<double> p3, q3;

    if(xa[4]*xa[4]<= ALM0*ALM0) return sol3(xa,xz);


    b[0] = xa[0] / xa[4];
    b[1] = xa[1] / xa[4];
    b[2] = xa[2] / xa[4];
    b[3] = xa[3] / xa[4];

    if ((b[1]*b[1]) <= ALM0*ALM0 && (b[3]*b[3]) <= ALM0*ALM0) {

        cb[0] = b[0];
        cb[1] = b[2];
        cb[2] = 1.0;
        if ( sol2(cb, cz) == 0) return 0;
        int cnt = 0;
        if (cz[0] >= 0) {
            xz[cnt++] = sqrt(cz[0]);
            xz[cnt++] = -sqrt(cz[0]);
        }
        if (cz[1] >= 0) {
            xz[cnt++] = sqrt(cz[1]);
            xz[cnt++] = -sqrt(cz[1]);
        }
        return cnt;
    }


    p = -3.0 * b[3] * b[3] / 8.0 + b[2];
    q = 0.5 * b[3] * (0.25 * b[3] * b[3] - b[2]) + b[1];
    r = 0.25 * b[3] * (-b[1] + 0.25 * b[3] * (b[2] - 3.0 * b[3] * b[3] / 16.0)) + b[0];

    cb[0] = -q * q / 64;
    cb[1] = (p * p - 4 * r) / 16;
    cb[2] = 0.5 * p;
    cb[3] = 1.0;
    int n = sol3(cb, cz);


    complex<double> sqt, ez[4];
    double err[3];
//    int si=0;
    double terr = 1e10;
    int tcnt = 0;
    double ezr[4];

    for (int ei = 0;ei < n;ei++) {
        sqt = sqrt(cz[ei]);
//	cout << "test3" << tcnt<<endl;

        if (norm(sqt) >= ALM0*ALM0) {
            p3 = sqrt(-sqt * sqt - 0.5 * p - 0.25 * q / sqt);
            q3 = sqrt(-sqt * sqt - 0.5 * p + 0.25 * q / sqt);

            ez[0] = sqt + p3 - 0.25 * b[3];
            ez[1] = sqt - p3 - 0.25 * b[3];
            ez[2] = -sqt + q3 - 0.25 * b[3];
            ez[3] = -sqt - q3 - 0.25 * b[3];
        } else ez[0] = ez[1] = ez[2] = ez[3] = -0.25 * b[3];
        int cnt = 0;

        for (int i = 0;i < 4;i++) {
//			cout << i << " " << real (ez[i] ) << " " << imag (ez[i]) << endl;
            if (fabs((ez[i]).imag()) > ALM0) continue;
            ezr[cnt++] = ez[i].real();
        }
        if(cnt == 0 ) continue;

        if ((b[3]*b[3]) >= ALM0*ALM0) {
            err[ei] = 0;
            for (int i = 0;i < cnt;i++) {
                err[ei] += std::pow(cpoly4(xa, ezr[i]), 2);
            }
        }

        if (terr >= err[ei]) {
            xz[0] = ezr[0];
            xz[1] = ezr[1];
            xz[2] = ezr[2];
            xz[3] = ezr[3];
            tcnt = cnt;
            terr = err[ei];
        }
    }
//	cout << terr << endl;
    return tcnt;
}

/**
 * @brief invRodrigues
 * find the rotation angle of a matrix, or namely the inverse rodriguez form
 * @param[out] out_axis
 * @param[out] out_angle
 * @param[in] in_R
 */
void invRodrigues(Vector3 &out_axis, double &out_angle, Matrix3 in_R)
{
    double r21r12 = in_R(2,1)-in_R(1,2);
    double r02r20 = in_R(0,2)-in_R(2,0);
    double r10r01 = in_R(1,0)-in_R(0,1);
    double denomi = sqrt(r21r12*r21r12+r02r20*r02r20+r10r01*r10r01);
    out_axis[0] = (in_R(2,1)-in_R(1,2))/denomi;
    out_axis[1] = (in_R(0,2)-in_R(2,0))/denomi;
    out_axis[2] = (in_R(1,0)-in_R(0,1))/denomi;
    out_angle = acos((in_R(0,0)+in_R(1,1)+in_R(2,2)-1)/2);
}

/**
 * @brief angleOfTwoVecs
 * compute the angle between two vectors
 * @param v0 the first vector
 * @param v1 the second vector
 * @return an angle in randian smaller than 180
 */\
double angleOfTwoVecs(const Vector3 &v0, const Vector3 &v1)
{
    cnoid::Vector3 p0(0, 0, 0);
    cnoid::Vector3 p1 = v0;
    cnoid::Vector3 p2 = v1;
    return acos(cos_th(p0, p1, p2));
}

}

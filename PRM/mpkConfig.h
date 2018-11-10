#ifndef MPK_CONFIG_H
#define MPK_CONFIG_H

#include <vector>
#include <math.h>
#include <assert.h>
#include <iostream>
//#ifdef WIN32
//#include <ostream>
//#endif
#include "exportdef.h"


using namespace std;

/**@memo Contains the parameters of a normalized configuration (a
   point in the c-space $[0,1]^d$).\\
   The operators {\bf ==,+,-,*,/} are defined for this class.
 */
class mpkConfig : public vector<double> {

public:

  ///@param dim number of parameters (dimension of c-space).
  mpkConfig(int dim = 0) : vector<double>(dim)
  {
    for ( int i=0; i<dim; i++ ) (*this)[i] = 0.0;
  };
  mpkConfig(const vector<double>& c) : vector<double>(c) {};
  mpkConfig& operator=(const mpkConfig& u);

  ///@memo Euclidean norm squared.
  double norm2() const;
  ///@memo Euclidean norm.
  double norm() const {return sqrt(norm2());};
  ///@memo Euclidean distance squared.
  double dist2(const mpkConfig& y) const;
  ///@memo Euclidean distance.
  double dist(const mpkConfig& y) const {return sqrt(dist2(y));};
  ///@memo Linf norm.
  double Linf() const;
  ///@memo Linf distance.
  double Linf_dist(const mpkConfig& y) const;
  ///@memo weighted Linf distance.
  double Linf_dist(const mpkConfig& y, const vector<double>& weights) const;

  ///@memo Set to {\bf t*q0 + (1-t)*q1}.
  void lin_interpol(double t, const mpkConfig& q0, const mpkConfig& q1);
  void lin_interpol(double t, const mpkConfig& q0, const mpkConfig& q1,
		    int from, int to);

  ///@memo Print to output stream.
  void print(ostream& out = cout) const;
};


inline mpkConfig& mpkConfig::operator=(const mpkConfig& u)
{
  clear();
  for(int i = 0; i < (int)u.size(); i++) push_back(u[i]);
  return *this;
}


inline double mpkConfig::norm2() const
{
  double sum = 0;
  for ( int i=0; i<(int)size(); i++ ) {
    double val = (*this)[i];
    sum += val*val;
  }
  return sum;
}

inline double mpkConfig::dist2(const mpkConfig& y) const
{
  double sum = 0;
  for ( int i=0; i<(int)size(); i++ ) {
    double diff = (*this)[i] - y[i];
    sum += diff*diff;
  }
  return sum;
}

inline double mpkConfig::Linf_dist(const mpkConfig& y) const
{
  double m = 0;
  for ( int i=0; i<(int)size(); i++ ) {
    double abs_val = fabs((*this)[i] - y[i]);
    if ( abs_val > m ) m = abs_val;
  }
  return m;
}

inline double mpkConfig::Linf_dist(const mpkConfig& y, const vector<double>& weights) const
{
  double m = 0;
  for ( int i=0; i<(int)size(); i++ ) {
    double abs_val = weights[i]*fabs((*this)[i] - y[i]);
    if ( abs_val > m ) m = abs_val;
  }
  return m;
}

inline double mpkConfig::Linf() const
{
  double m = 0;
  for ( int i=0; i<(int)size(); i++ ) {
    double abs_val = fabs((*this)[i]);
    if ( abs_val > m ) m = abs_val;
  }
  return m;
}

inline void mpkConfig::print(ostream& out) const
{
  for ( int i=0; i<(int)size(); i++ ) {
    out << (*this)[i] << " ";
  }
  out << endl;
}

inline ostream& operator<<(ostream& f, const mpkConfig& c)
{
  for ( int i=0; i<(int)c.size(); i++ ) {
    f << c[i];
    if ( i < (int)c.size()-1 ) f << " ";
  }
  return f;
}

inline void mpkConfig::lin_interpol(double t, const mpkConfig& q0,
				    const mpkConfig& q1)
{
  assert(q0.size() == q1.size() && q0.size() == size());
  for ( int i=0; i<(int)size(); i++ )
    (*this)[i] = (1-t) * q0[i] + t * q1[i];
}

inline void mpkConfig::lin_interpol(double t, const mpkConfig& q0,
				    const mpkConfig& q1,
				    int from, int to)
{
  assert(q0.size() == q1.size() && q0.size() == size());
  for ( int i=from; i<=to; i++ )
    (*this)[i] = (1-t) * q0[i] + t * q1[i];
}

/* operator ==
 */
inline bool operator==(const mpkConfig& u, const mpkConfig& v)
{
  assert(u.size() == v.size());
  for (int i=0; i<(int)u.size(); i++ ) if (u[i]!=v[i]) return false;
  return true;
}


/* operator +
 *    add two vectors
 */
inline mpkConfig operator+ (const mpkConfig& u, const mpkConfig& v)
{
  assert(u.size() == v.size());
  int i=u.size();
  mpkConfig w(i);
  while (i--) w[i] = u[i] + v[i];
  return w;
}

/* operator -
 *    subtract two vectors
 */
inline mpkConfig operator- (const mpkConfig& u, const mpkConfig& v)
{
  assert(u.size() == v.size());
  int i=u.size();
  mpkConfig w(i);
  while (i--) w[i] = u[i] - v[i];
  return w;
}

inline mpkConfig operator* (double a, const mpkConfig& v)
{
  int i=v.size();
  mpkConfig w(i); 
  while (i--) w[i] = a * v[i];
  return w;
}

/* operator /
 *    divide by a scalar
 */
inline mpkConfig operator/ (const mpkConfig& v, double a)
{
  return 1.0 / a * v;
}


/* operator /
 *    component-wise division
 */
inline mpkConfig operator/ (const mpkConfig& u, const mpkConfig& v)
{
  assert(u.size() == v.size());
  int i=u.size();
  mpkConfig w(i);
  while (i--) w[i] = u[i] / v[i];
  return w;
}

inline  mpkConfig operator- (const mpkConfig& v)
{
  int i=v.size();
  mpkConfig w(i);
  while (i--) w[i] = -v[i];
  return w;
}


#endif

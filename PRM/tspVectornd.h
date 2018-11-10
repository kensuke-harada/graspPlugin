#ifndef _VND_
#define _VND_

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string.h>
#include <vector>

#ifdef WIN32
#include <ostream>
#endif

#include "exportdef.h"


using namespace std;

//typedef int Bool;
typedef double Real;


class Vectornd{
 public:

  vector<Real> pt;
  int vdim;

  Vectornd(double *table[], int);
  Vectornd(int vdim=0);
  Vectornd(vector<double>& inp);

  // operations
  friend Vectornd operator+(Vectornd v1,Vectornd v2);
  friend Vectornd operator-(Vectornd v1,Vectornd v2);
  friend Vectornd operator-(Vectornd v2);
  friend Vectornd operator*(Real fac,Vectornd v);
  friend Vectornd operator*(Vectornd v,Real fac);
  friend Real operator*(Vectornd v1,Vectornd v2);
  friend Vectornd operator/(Vectornd v,Real fac);


  // relations
  Real dist(Vectornd v);
  Real cost(Vectornd v);

  // print
  void print(void);
  void print(char* strg);


  int index;

  double **local_table;
};

extern Vectornd nullvec;

#endif

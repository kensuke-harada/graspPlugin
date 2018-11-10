
#include "tspVectornd.h"


Vectornd nullvec;

Vectornd::Vectornd(double *table[], int dim_){
  // constructor for Vector
  local_table=table;
  pt.resize(dim_);
  vdim=dim_;
}

Vectornd::Vectornd(int dim_){
  // constructor for Vector
  pt.resize(dim_);
  vdim=dim_;
}

Vectornd::Vectornd(vector<double>& inp){
  // constructor for Vector
  pt=inp;
  vdim=inp.size();
}

Vectornd operator+(Vectornd v1,Vectornd v2){
  // + for Vectors
  Vectornd v(v1.vdim);
  for(int i=0; i<v1.vdim; i++) v.pt[i]=v1.pt[i]+v2.pt[i];

  return(v);
}

Vectornd operator-(Vectornd v1,Vectornd v2){
  // - for Vectors
  Vectornd v(v1.vdim);
  for(int i=0; i<v1.vdim; i++) v.pt[i]=v1.pt[i]-v2.pt[i];

  return(v);
}

Vectornd operator-(Vectornd v2){
  // - for Vectors
  Vectornd v(v2.vdim);
  for(int i=0; i<v2.vdim; i++) v.pt[i]=-v2.pt[i];

  return(v);
}

Vectornd operator*(Real fac,Vectornd v){
  // * for Real and Vector
  Vectornd r(v.vdim);
  for(int i=0; i<v.vdim; i++) v.pt[i]=fac*v.pt[i];

  return(r);
}

Vectornd operator*(Vectornd v,Real fac){
  // * for Real and Vector
  Vectornd r(v.vdim);
  for(int i=0; i<v.vdim; i++) v.pt[i]=fac*v.pt[i];;

  return(r);
}

Real operator*(Vectornd v1,Vectornd v2){
  // * for Vector and Vector
  Real r=0;
  for(int i=0; i<v1.vdim; i++)  r=r+v1.pt[i]*v2.pt[i];

  return(r);
}

Vectornd operator/(Vectornd v,Real fac){
  // / for Real and Vector
  Vectornd r(v.vdim);
  for(int i=0; i<v.vdim; i++) r.pt[i]=v.pt[i]/fac;

  return(r);
}

Real Vectornd::dist(Vectornd v){

  // modification
  if(local_table[index][v.index]>=0.0){
        return  local_table[index][v.index];
  }
  else{
    double dt = cost(v);
    local_table[index][v.index]=dt;
    local_table[v.index][index]=dt;
    return dt;
  }

}

Real Vectornd::cost(Vectornd v){
  double dt=0;
  for(int i=0;i<v.vdim; i++) dt=dt+(pt[i]-v.pt[i])*(pt[i]-v.pt[i]);
  dt=sqrt(dt);
  return dt;
}

void Vectornd::print(void){
  // print vector
  cout << "vectornd.C: print unimplemented"<< endl;
}

void Vectornd::print(char* strg){
  // print vector and string strg
  cout << "vectornd.C: print unimplemented"<< endl;
}

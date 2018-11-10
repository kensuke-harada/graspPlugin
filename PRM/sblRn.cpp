#ifndef WIN32
#include <ext/algorithm>
#endif
#include "sblRn.h"
#include "mpkConfig.h"

#include "mpk_rand.h"

sblRn::sblRn() : randSrc_()
{
}


sblRn::sblRn(long seed) : randSrc_(seed)
{
}

/* UniformSample
 *   generate configurations uniformly at random in [0,1]^n
 */

void
sblRn::UniformSample(mpkConfig *q)
{
  int dim = q->size();
  const double *tmp = randSrc_.get(dim);

  //memcpy(*q, tmp, q->size()*sizeof(double));   // a little hacky, but fast
  for(int i=0; i<q->size(); i++) (*q)[i] = tmp[i];
}


/* LocalBoxSample
 *   pick uniformly at random a configuraton in the neighborhood centered
 *   at q0 and with radius radius 
 *
 *     q0 : center of neighborhood
 * radius : size of the neighborhood 

 */

static double mpk_max(double a, double b){
  if(a>b) return a;
  else return b;
}

static double mpk_min(double a, double b){
  if(a<b) return a;
  else return b;
}

bool
sblRn::LocalBoxSample(const mpkConfig& q0, double radius, mpkConfig *q, const vector<int>& sampleDOF)
{
  mpkConfig& p =*q;

  int dim = p.size();
  //const double *tmp = randSrc_.get(dim);
  //mpkConfig nuevo(tmp);
  //nuevo.print();
  /*
  for (int i=0; i<dim; i++) {
    double nbhd[2];
    nbhd[0] = mpk_max( q0[i]-radius, 0.0 );
    nbhd[1] = mpk_min( q0[i]+radius, 1.0 );
    
    //p[i] = nbhd[0] + tmp[i]*(nbhd[1]-nbhd[0]);
    p[i] = nbhd[0] + mpk_drand()*(nbhd[1]-nbhd[0]);
   }
  */

  for (int i=0; i<dim; i++)
    p[i] = q0[i];
 
  for (unsigned int i=0; i<sampleDOF.size(); i++) {
    
    int dof = sampleDOF[i];

    double nbhd[2];
    nbhd[0] = mpk_max( q0[dof]-radius, 0.0 );
    nbhd[1] = mpk_min( q0[dof]+radius, 1.0 );
    
    //p[i] = nbhd[0] + tmp[i]*(nbhd[1]-nbhd[0]);
    p[dof] = nbhd[0] + mpk_drand()*(nbhd[1]-nbhd[0]);
   }
 
  return true;
}

/* LocalBallSample
 *   pick uniformly at random a configuraton 
 *   with distance smaller that radius to q0 
 */
bool
sblRn::LocalBallSample(const mpkConfig& q0, double radius, mpkConfig *q)
{
  // This function gets
  double sqr_eps = radius*radius;
  double d =0 ;
  mpkConfig& p =*q;
  int dim = q0.size();
  do{
    const double *tmp = randSrc_.get(dim);
  
    for (int i=0; i<dim; i++) {
      double nbhd[2];
      nbhd[0] = mpk_max( q0[i]-radius, 0.0 );
      nbhd[1] = mpk_min( q0[i]+radius, 1.0 );
      
      p[i] = nbhd[0] + tmp[i]*(nbhd[1]-nbhd[0]);

      d += (q0[i]-p[i])*(q0[i]-p[i]);
    } 

  }  while(sqr_eps > d);
  return true;
}




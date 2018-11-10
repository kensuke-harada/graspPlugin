#ifndef SBL_RN_H
#define SBL_RN_H

#include "sblRandVal.h"
#include "mpkConfig.h"

#include "exportdef.h"


using namespace std;

/**@memo Used by the sblPlanner class to obtain random samples in
    configuration space.
*/

class sblRn {
  
public:

  ///
  sblRn();

  ///
  sblRn(long seed);

  ///@memo Samples a random configuration uniformly from $[0,1]^d$.
  void UniformSample(mpkConfig *q);

  /**@memo Samples a random configuration uniformly in the
     intersection of the c-space $[0,1]^d$ with a box that has side
     lengths {\bf 2radius} and is centered at {\bf q0}.
  */
  bool LocalBoxSample(const mpkConfig& q0, double radius, mpkConfig *q, const vector<int>& sampleDOF);

  /**@memo Samples a random configuration uniformly from the
     intersection of the c-space $[0,1]^d$ and a ball ${\cal
     B}(q0,radius)$ centered at {\bf q0}.
  */
  bool LocalBallSample(const mpkConfig& q0, double radius, mpkConfig *q);

protected:
  sblRandVal randSrc_;
};


#endif

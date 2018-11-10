#ifndef MPK_SIMPLE_SEGMENT_CHECKER_H
#define MPK_SIMPLE_SEGMENT_CHECKER_H

#include "mpkConfigChecker.h"

#include "exportdef.h"


/**@memo Simple lazy segment checker that discretizes a segment up to
   a certain resolution $\varepsilon$ in c-space and checks the
   generated intermediate configurations for collisions using the
   {@link mpkConfigChecker mpkConfigChecker} class.

   @doc The checker can be embedded with other checkers in a priority
   queue for testing entire paths (sequences of segments).  To this
   end, the checker offers performing single iteration steps and a
   function that returns, after each iteration, the priority of
   further examining the segment.

   Each iteration step of the checker examines a number of
   2^curr_bisection_depth equally spaced points on the segment.  The
   points are chosen such that they fall between all the points that
   were checked before at previous iteration steps with smaller
   bisection depths.  The point for depth 0 is thus located at t=0.5,
   for depth 1 the points are located at t=0.25 and t=0.75, for depth
   2 at t=0.125, t=0.375, t=0.625 and t=0.875 and so on.  The time
   required by this function thus increases exponentially with each
   call.

   The priority of examining the segment is proportional to the
   (c-space) length of the untested subsegments between the
   intermediate tested points.
   
   @see mpkConfigChecker
 */
class mpkSimpleSegmentChecker {

public:

  enum cspace_metric {EUCLID, L_INFINITY};

  /**@doc
     @param ccheck simple point checker for testing intermediate points.
     @param q0 one endpoint of segment
     @param q1 the other endpoint (neither endpoint is tested
     explicitly by the segment checker)
     @param epsilon c-space resolution up to which the segment is
     bisected by additional intermediate points.  When the (c-space)
     distance between sampled intermediate points has fallen below
     {\bf epsilon}, the segment is considered collision-free.
     @param metric c-space metric for determining the length of the
     (sub-)segment(s); either {\bf EUCLID} or {\bf L_INFINITY} (max-norm).
   */
  mpkSimpleSegmentChecker(mpkConfigChecker* ccheck,
			  const mpkConfig* q0, const mpkConfig* q1,
			  double epsilon, cspace_metric metric=EUCLID);
  ~mpkSimpleSegmentChecker() {};

  /**@doc Returns the current priority of the segment which is
     proportional to the lengths of the untested subsegments between
     the points that have been sampled and tested so far.
   */
  double prio() {return dist_q0q1 / (1<<curr_bisection_depth);};

  /**@doc Performs a single iteration step of the checker. 

     @return {\bf true} if the status of the segment has not yet been
     determined.  Further calls are required to determine the status.
     In this case, calls to {\bf collision()} will return {\bf false}
     which means that no collision has been found yet and thus (so
     far) the segment is assumed to be free.

     @return {\bf false} when the status of the segment has been
     determined: resolution reached or collision found.  Further calls
     are not necessary.  The status can then be queried using the {\bf
     collision()} method.
  */
  enum collcheck_mode {COLLISION_TEST, TOLERANCE_TEST, ROUGH_CLEARANCE_TEST, DEBUG_TEST};
#ifdef DO_TOLERANCE_TEST
  bool iteration_step(collcheck_mode mode=TOLERANCE_TEST);
#else
  bool iteration_step(collcheck_mode mode=COLLISION_TEST);
#endif

  /**@doc Returns {\bf true} if it the segment has been found to be
     colliding and {\bf false} if the segment is assumed to be free.
     In the latter case, there is no guarantee that the segment is
     actually collision-free but the probability of a correct answer
     increases with decreasing resolution $\varepsilon$.
  */
  bool collision() {return coll_status;}
  
  double rough_clearance(){ return _rough_clearance; }
  
  void set_rough_clearance(double rc){ _rough_clearance=rc; }
  
  
  ///@doc Number of BV pair tests performed so far.
  int num_bv_tests;

  ///@doc Number of triangle pair tests performed so far.
  int num_tri_tests;

  // number of rigid body pairs checked for collision
  int num_obj_tests;

  ///@doc Number of points on segments
  int num_points_tested;

  /**@doc Normalized resolution up to which segment has been checked
     (initially 1 and then halved by each call to {\bf
     iteration_step()}.
   */
  double min_interval_sz;

  /**@doc Normalized parameter corresponding to collision point on
     segment (between 0 and 1).  Meaningless unless collision indicated
     by {\bf collision()}.
   */
  double coll_t;

private:

  mpkConfigChecker* ccheck;
  const mpkConfig* q0;
  const mpkConfig* q1;
  mpkConfig qtmp;
  double dist_q0q1;
  double epsilon;
  int curr_bisection_depth;
  bool coll_status;
  double _rough_clearance;

};

#endif

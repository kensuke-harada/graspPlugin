#include "mpkSimpleSegmentChecker.h"

#ifdef DEBUG_MODE
 #include "../Grasp/PlanBase.h"
 #include <cnoid/MessageView>
#endif

mpkSimpleSegmentChecker::
mpkSimpleSegmentChecker(mpkConfigChecker* ccheck,
		     const mpkConfig* q0, const mpkConfig* q1,
		     double epsilon, cspace_metric metric)
{
  this->ccheck = ccheck;
  this->q0 = q0;
  this->q1 = q1;
  this->epsilon = epsilon;
  curr_bisection_depth = 0;
  coll_status = false;
  _rough_clearance = 1.0e10;
  qtmp = mpkConfig(q0->size());
  if ( metric==EUCLID )
    dist_q0q1 = q0->dist(*q1);
  else
    dist_q0q1 = q0->Linf_dist(*q1);
  num_points_tested = 0;
  min_interval_sz = 1;
  num_tri_tests = 0;
  num_bv_tests = 0;
  num_obj_tests = 0;
}

// iteration_step()
//
// Performs an iteration step of the checker.  A single iteration step
// examines a number of 2^curr_bisection_depth equally spaced points
// on the segment.  The points are chosen such that they fall between
// all the points that were checked before at previous iteration steps
// with smaller bisection depths.  The point for depth 0 is thus
// located at t=0.5, for depth 1 the points are located at t=0.25 and
// t=0.75, for depth 2 at t=0.125, t=0.375, t=0.625 and t=0.875 and so
// on.
bool
mpkSimpleSegmentChecker::
iteration_step(collcheck_mode mode)
{
  // number of points to test at current bisection depth and
  // corresponding normalized step size
  int num_tests = 1 << curr_bisection_depth;
  double delta_t = 1.0 / num_tests;

  // first point is located at delta_t / 2
  double t = delta_t / 2;

  min_interval_sz /= 2;
	
	
	
  if(mode == ROUGH_CLEARANCE_TEST){
	mpkConfig qtmp1 = mpkConfig(q0->size());
	mpkConfig qtmp2 = mpkConfig(q0->size());
	qtmp1.lin_interpol(0,*q0,*q1);
	qtmp2.lin_interpol(1,*q0,*q1);
        double temp = ccheck->sweep_rough_clearance_fast(&qtmp1,&qtmp2,ccheck->model_id);
      _rough_clearance = (_rough_clearance < temp) ? _rough_clearance : temp; 
      return false;
  }
	

  // if desired resolution reached, return
  if ( delta_t*dist_q0q1 < epsilon ){
    if(mode == ROUGH_CLEARANCE_TEST){
      qtmp.lin_interpol(0,*q0,*q1);
      ccheck->sweep_rough_clearance(&qtmp,ccheck->model_id,true);
      for ( int i=0; i<num_tests; i++ ) {
        // set configuration
        qtmp.lin_interpol(t,*q0,*q1);
        // check for collision
        bool coll = false;
        double temp = ccheck->sweep_rough_clearance(&qtmp,ccheck->model_id,false);
	      
	if(_rough_clearance>temp){
		coll=true;
		 _rough_clearance = temp;
	}
        if ( coll ) {
          coll_status = true;
          coll_t = t;
          return false;
        }
        t += delta_t;
      }
      qtmp.lin_interpol(1,*q0,*q1);
      double temp = ccheck->sweep_rough_clearance(&qtmp,ccheck->model_id,false);
      _rough_clearance = (_rough_clearance < temp) ? _rough_clearance : temp; 
      return false;
    }
  }
 
  if ( delta_t*dist_q0q1 < epsilon){
	  return false;
  }
 
  for ( int i=0; i<num_tests; i++ ) {

    // set configuration
    qtmp.lin_interpol(t,*q0,*q1);

    // check for collision
    bool coll=false;
    if ( mode==COLLISION_TEST || mode == DEBUG_TEST ) coll = ccheck->collision(&qtmp,ccheck->model_id);
    else if(mode == ROUGH_CLEARANCE_TEST){
      double temp = ccheck->rough_clearance(&qtmp,ccheck->model_id,false);
      if(_rough_clearance>temp) coll=true;
      _rough_clearance = (_rough_clearance < temp) ? _rough_clearance : temp;
    }
    else // tolerance mode: verify separation greater than delta
      coll = (ccheck->clearance(&qtmp,ccheck->model_id,false) <= 0);
    num_points_tested++;
    num_bv_tests  += ccheck->num_bv_tests;
    num_tri_tests += ccheck->num_tri_tests;
    num_obj_tests += mpkConfigChecker::num_obj_tests;
    if ( coll ) {
      coll_status = true;
      coll_t = t;
      return false;
    }
#ifdef DEBUG_MODE
    if(mode ==DEBUG_TEST) {
	    if( delta_t*dist_q0q1 < 2*epsilon){
		    grasp::PlanBase::instance()->flush();
//		   cnoid::MessageView::instance()->cout() <<  (*q0)[q0->size()-1] << " " << (*q0)[q0->size()-2] <<  " " << (*q0)[q0->size()-3] << endl; 
	    }
    }
#endif        
    // advance
    t += delta_t;
  }

  ++curr_bisection_depth;

  return true; // means that more steps are necessary because neither
	       // resolution has been reached nor collision has been found
}


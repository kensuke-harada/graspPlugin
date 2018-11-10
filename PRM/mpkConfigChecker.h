//Original mpk modified by K.Harada @aist

#ifndef MPK_CONFIG_CHECKER_H
#define MPK_CONFIG_CHECKER_H


#include <queue>
#include "mpkConfig.h"
#include "mpkRobots.h"
#include "mpk_defs.h"
//#include "mpkCollPair.H"
//#include "mpkRobotCollection.H"
//#include "mpkCollDistAlgo.H"

#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/WorldItem>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/SceneBody>	/* modified by qtconv.rb 0th rule*/  
//#include <cnoid/SceneBodyManager>	/* modified by qtconv.rb 0th rule*/ 
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>  
#include <cnoid/OSGSceneBodyManager>  
#endif

#include "exportdef.h"

#include "../Grasp/GraspController.h"

///
class mpkConfigChecker {

public:

  ///
  //mpkConfigChecker(const vector<mpkCollPair>* test_pairs,
  //		   mpkRobotCollection* robots);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  mpkConfigChecker(cnoid::ColdetLinkPairPtr *test_pairs, int test_size, 
		   mpkRobots* robots);
#else
  mpkConfigChecker(grasp::ColdetLinkPairPtr *test_pairs, int test_size, 
		   mpkRobots* robots);
#endif
  
  /// tells if the configuration q is colliding
  bool collision(mpkConfig* q, mpk_idx model_id=0);

  /** returns lower bound on clearance (0 if colliding or separation <
      delta for some pair) */
  double clearance(mpkConfig* q, mpk_idx model_id=0,
		   bool store_dist=true);

  double rough_clearance(mpkConfig* q, bool initial=false, mpk_idx model_id=0);

  double sweep_rough_clearance(mpkConfig* q, mpk_idx model_id=0,
		   bool initial=false);
  double sweep_rough_clearance_fast(mpkConfig* q0, mpkConfig* q1, mpk_idx model_id=0);
  /** the index (in test_pairs) of the colliding (or too
      close) pair (undef if no collision). */
  mpk_idx collpair_idx() {return coll_idx;};

  static int num_tri_tests;
  static int num_bv_tests;
  static int num_obj_tests;

private:
  
  // general information
  //mpkRobotCollection* robots;
  //const vector<mpkCollPair>* test_pairs;
  grasp::PlanBase* gc;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  cnoid::ColdetLinkPairPtr *test_pairs;
#else
  grasp::ColdetLinkPairPtr *test_pairs;
#endif
  int test_size;
  mpkRobots* robots;

  // status corresponding to most recent operation
  mpk_idx model_id;
  const mpkConfig* q;
  vector<double> lbdist; // lower distance bounds (after clearance() only)
  double min_sep;  // minimum of lbdist (after clearance() only)
  mpk_idx coll_idx; // only valid if collision / too close

  //
  friend class mpkSimpleSegmentChecker;
  friend class mpkDiscrSegmentChecker;
  friend class mpkAdaptSegmentChecker;

};

#endif

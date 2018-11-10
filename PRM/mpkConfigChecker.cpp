// Original mpk modified by K.Harada @aist

#include "mpkConfigChecker.h"
#include "../Grasp/PlanBase.h"

using namespace cnoid;

int mpkConfigChecker::num_tri_tests;
int mpkConfigChecker::num_bv_tests;
int mpkConfigChecker::num_obj_tests;

mpkConfigChecker::
//mpkConfigChecker(const vector<mpkCollPair>* test_pairs,
//		  mpkRobotCollection* robots)
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
mpkConfigChecker(ColdetLinkPairPtr *test_pairs_, int test_size_, 
#else
mpkConfigChecker(grasp::ColdetLinkPairPtr *test_pairs_, int test_size_, 
#endif
		 mpkRobots* robots_)
{
  this->robots = robots_;
  this->test_pairs = test_pairs_;
  this->test_size = test_size_;
  this->q = 0;
  this->model_id = 0;
  gc = grasp::PlanBase::instance();
}

bool
mpkConfigChecker::
collision(mpkConfig* q, mpk_idx model_id)
{
  assert(model_id==0 || model_id==1);

  num_tri_tests = 0;
  num_bv_tests = 0;
  num_obj_tests = 0;
  this->model_id = model_id;
  this->q = 0;
  this->min_sep = -1; // disable: collision test does not determine min_sep

  robots->set_config(*q);
  robots->compute_forward_kinematics();
	
#ifdef ENABLE_SBL_MULTITHREAD
	gc->calcForwardKinematics(robots->robotbodys);
  if(gc->isColliding(robots->robotbodys)){
	  return true;
  }
#else
  gc->calcForwardKinematics();
  if(gc->isColliding()){
	  return true;
  }
#endif
  return false;

  for ( int i=0; i < test_size; i++ ) {

    num_obj_tests++;

    /*
    const mpkCollPair& curr_pair = (*test_pairs)[i]; 

    bool coll =
      mpkCollDistAlgo::Collision(curr_pair.o1.Tr->R, curr_pair.o1.Tr->T,
				 curr_pair.o1.pqp[model_id], 
				 curr_pair.o2.Tr->R, curr_pair.o2.Tr->T,
				 curr_pair.o2.pqp[model_id]);

    num_tri_tests += mpkCollDistAlgo::coll_res.num_tri_tests;
    num_bv_tests  += mpkCollDistAlgo::coll_res.num_bv_tests;
    */

//	ColdetLinkPairPtr cPair = linkObjPair[Ik];
//	ColdetModelPtr model = cPair->model(0);

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    test_pairs[i]->model(0)->setPosition(test_pairs[i]->link(0)->R(), test_pairs[i]->link(0)->p());
    test_pairs[i]->model(1)->setPosition(test_pairs[i]->link(1)->R(), test_pairs[i]->link(1)->p());
#else
    test_pairs[i]->model(0)->setPosition(test_pairs[i]->link(0)->T());
    test_pairs[i]->model(1)->setPosition(test_pairs[i]->link(1)->T());
#endif   

    bool coll = test_pairs[i]->checkCollision();

    if ( coll ) {
      coll_idx = i;
      return true;
    }

  }
  return false;
}

double
mpkConfigChecker::
clearance(mpkConfig* q, mpk_idx model_id, bool store_dist)
{
  assert(model_id==0 || model_id==1);

  num_tri_tests = 0;
  num_bv_tests = 0;
  num_obj_tests = 0;
  this->model_id = model_id;
  this->q = q;
  this->min_sep = DBL_MAX;
  if ( store_dist && (int)lbdist.size() != test_size )
    lbdist.resize(test_size);

  robots->set_config(*q);
  robots->compute_forward_kinematics();
#ifdef ENABLE_SBL_MULTITHREAD
	gc->calcForwardKinematics(robots->robotbodys);
  min_sep = gc->clearance(robots->robotbodys);
#else
  gc->calcForwardKinematics();
  min_sep = gc->clearance();
#endif
  //if(min_sep < 0.02) return 0;

  return min_sep;

  
  // before version is below
  for ( int i=0; i < test_size; i++ ) {

    num_obj_tests++;

    //const mpkCollPair& curr_pair = (*test_pairs)[i]; 

    double dist = 0;

    /*
    if ( dist <= 0 ) {
      dist =
	mpkCollDistAlgo::GreedyDistance(curr_pair.o1.Tr->R, curr_pair.o1.Tr->T,
					curr_pair.o1.pqp[model_id], 
					curr_pair.o2.Tr->R, curr_pair.o2.Tr->T,
					curr_pair.o2.pqp[model_id],
					curr_pair.delta);
      num_tri_tests += mpkCollDistAlgo::dist_res.num_tri_tests;
      num_bv_tests  += mpkCollDistAlgo::dist_res.num_bv_tests;
    }
    */

    double p1[3] = {0}, p2[3] = {0};
    int tid1, tid2;

//	ColdetLinkPairPtr cPair = linkObjPair[Ik];
//	ColdetModelPtr model = cPair->model(0);

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
    test_pairs[i]->model(0)->setPosition(test_pairs[i]->link(0)->R(), test_pairs[i]->link(0)->p());
    test_pairs[i]->model(1)->setPosition(test_pairs[i]->link(1)->R(), test_pairs[i]->link(1)->p());
#else
    test_pairs[i]->model(0)->setPosition(test_pairs[i]->link(0)->T());
    test_pairs[i]->model(1)->setPosition(test_pairs[i]->link(1)->T());
#endif       

    dist = test_pairs[i]->computeDistance(tid1, &p1[0], tid2, &p2[0]);
  
    if ( dist <= 0 ) {
      coll_idx = i;
      return 0;
    }
    if ( dist < min_sep )
      min_sep = dist;
    if ( store_dist )
      lbdist[i] = dist;

  }

  return min_sep;
}

double
mpkConfigChecker::
rough_clearance(mpkConfig* q, bool initial, mpk_idx model_id)
{
  assert(model_id==0 || model_id==1);

  robots->set_config(*q);
  robots->compute_forward_kinematics();
  gc->calcForwardKinematics();
  min_sep = gc->roughClearance(initial);

  return min_sep;
}

double
mpkConfigChecker::
sweep_rough_clearance(mpkConfig* q, mpk_idx model_id, bool initial)
{
  assert(model_id==0 || model_id==1);

  robots->set_config(*q);
  robots->compute_forward_kinematics();
  gc->calcForwardKinematics();
  min_sep = gc->sweepRoughClearance(initial);

  return min_sep;
}

double
mpkConfigChecker::
sweep_rough_clearance_fast(mpkConfig* q0, mpkConfig* q1, mpk_idx model_id)
{
  assert(model_id==0 || model_id==1);

  robots->set_config(*q0);
  robots->compute_forward_kinematics();
  gc->calcForwardKinematics();
  min_sep = gc->sweepRoughClearanceFast(true);
  robots->set_config(*q1);
  robots->compute_forward_kinematics();
  gc->calcForwardKinematics();
  min_sep = gc->sweepRoughClearanceFast(false);

  return min_sep;
}

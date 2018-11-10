// *************************************************************
// Visibility PRM Path Planner
// *************************************************************

#ifndef VPRM_PLANNER_H
#define VPRM_PLANNER_H

#ifdef WIN32
#include <sys/timeb.h>
#else
#include <sys/times.h>
#endif

#include <stdlib.h>
#include <queue>
#include <vector>
#include <list>
#include <fstream>

#include "sblMilestone.h"
#include "sblEdge.h"
#include "mpkRobots.h"

#ifdef ADAPT_COLLCHECKER
#include "mpkAdaptSegmentChecker.h"
#else
#include "mpkSimpleSegmentChecker.h"
#endif

#include <cnoid/Body>	/* modified by qtconv.rb 0th rule*/   
#include <cnoid/Link>	/* modified by qtconv.rb 0th rule*/   
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/BodyItem>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/WorldItem>	/* modified by qtconv.rb 0th rule*/  

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ModelNodeSet>	/* modified by qtconv.rb 0th rule*/  
#include <cnoid/ColdetLinkPair>	/* modified by qtconv.rb 0th rule*/  
#ifdef CNOID_ENABLE_OSG
  #include <cnoid/OSGSceneBody>
  #include <cnoid/OSGSceneBodyManager>
#endif
#else
#include "../Grasp/ColdetLinkPair.h"
//#include <cnoid/SceneBody>
//#include <cnoid/SceneBodyManager>
#endif

#include "exportdef.h"

typedef priority_queue<sblEdge *,vector<sblEdge *>, sblPrioritizeEdges> edgesQueue;


/**@memo This contains the implementation of the basic vprmPlanner.
   The current planner uses a simple segment checker (class
   {@link mpkSimpleSegmentChecker mpkSimpleSegmentChecker}) for the collision
   tests.

   @see mpkSimpleSegmentChecker
*/

class vprmPlanner {

public:

  /**@doc Takes a pointer {\bf robots} to the robot collection, a set
     of collision test pairs (see
     {@link mpkCollPairSet mpkCollPairSet})
     and a c-space resolution for the
     {@link mpkSimpleSegmentChecker mpkSimpleSegmentChecker}.
  */

  //vprmPlanner(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  vprmPlanner(mpkRobots *robots, cnoid::ColdetLinkPairPtr *test_pairs, int test_size, const std::vector<int>& SampleDOF
#else
  vprmPlanner(mpkRobots *robots, grasp::ColdetLinkPairPtr *test_pairs, int test_size, const std::vector<int>& SampleDOF
#endif
#ifndef ADAPT_COLLCHECKER
	     ,double epsilon=0.012
#endif
	     );

  virtual ~vprmPlanner() {
    delete point_checker;
  };

  /**@doc This function is used to invoke the planner.  {\bf rho} is
     the initial neighborhood size for sampling around a milestone and
     {\bf max_it} is the maximum number of iterations.  If no path is
     found within this number of iterations, false is returned. */
  bool Query( mpkConfig& q0, mpkConfig& q1, list<mpkConfig> &genPath,
	       double rho=0.15, int max_it=100000);

  bool constructRoadMaps(mpkConfig& qs, mpkConfig& qg, int max_it);

  /**@doc This function is used to construct network of roadmaps.
   * Here, Visible PRM is used.
   */

  bool Visible(mpkConfig& q1, mpkConfig& q2);

  void tryConnect(sblMilestone& q1, sblMilestone& q2);

  bool DFS(list<mpkConfig>& configPath);

#ifdef WIN32
  void printStats(_timeb start, _timeb end, list<mpkConfig> &genPath);
#else
  /**@doc Evaluates the planner time and outputs final Path (Unix) */
  void printStats(struct tms start, struct tms end, list<mpkConfig> &genPath);
#endif

  /**@doc Writes the final path to a file */
  void writePath(const list<mpkConfig>&, char *file_name);

protected:

  /* Function to sample a new sblMilestone around a current one */
  bool sample(const sblMilestone& q, sblMilestone& q_rand);

  /* Function to test whether a path is collision free or not */
  bool testPath(edgesQueue&);

  //
  bool testSegment(sblEdge &u);

  // For the management of the edges
  int getIndex(const sblMilestone& x0, const sblMilestone& x1);
  //
  void getsblEdges(sblMilestone& , sblMilestone& , edgesQueue& );

  /* Returns Path length */
  double pathLength(const list<mpkConfig>& tau);

  /* To report planner time */
#ifdef WIN32
  void report_times (_timeb start, _timeb end);
  void addTime(_timeb start, _timeb end);
#else
  void report_times (struct tms start, struct tms end);
  void addTime(struct tms start, struct tms end);
#endif

  bool alreadyConnected(int n1, int n2);
  void addConnectionList(int n1, int n2);
  void displayMilestone(sblMilestone& q);

  /* random number initializer flag */
  static bool initRand;

  int nodes_counter;

  /* To measure time */
#ifdef WIN32
  _timeb optstart, optend;
#else
  struct tms optstart, optend; 
#endif

  /* the set of all the nodes generated on both trees */

  //
  vector<sblMilestone>  allNodes;

  /* To store the bridge */
  sblEdge theBridge;

  /* To store the first path found */
  list<mpkConfig> freePath;

  /* Counter of total coll checks */
  int total_CC;

  /* Counter of coll checks when checking the path */
  int path_CC;

  /* To store the time required for Collision-Checking */
  double CC_time;

  mpkConfig coll_conf;

  //
  //vector<mpkCollPair> *test_pairs_;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  cnoid::ColdetLinkPairPtr *test_pairs_;
#else
  grasp::ColdetLinkPairPtr *test_pairs_;
#endif
  int test_size_;

  mpkConfigChecker *point_checker;

  vector<vector<int> > connectList;
  //

  /* resolution static bool initRand */
#ifndef ADAPT_COLLCHECKER
  double EPSILON;
#endif

  //mpkRobotCollection *robots_;
  mpkRobots *robots_;

  vector<int> sampleDOF;

};

#endif

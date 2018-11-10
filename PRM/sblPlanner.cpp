// **************************************************************
// SBL Path sblPlanner implementation.
// **************************************************************

#ifndef CLK_TCK
#define CLK_TCK CLOCKS_PER_SEC
#endif

#include <stdio.h>
#ifndef WIN32
//#include <algo.h>
#include <ext/algorithm>
#endif

#include "mpk_rand.h"
#include "sblMilestone.h"
#include "sblPlanner.h" 
#include "sblTree.h"


// Class variable initializations

// ------------------------------
// Constructor & Destructor
// ------------------------------

using namespace cnoid;

sblPlanner::
//sblPlanner(mpkRobotCollection* robots, vector<mpkCollPair> *test_pairs
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
sblPlanner(mpkRobots* robots, ColdetLinkPairPtr *test_pairs, int test_size, const vector<int>& SampleDOF
#else
sblPlanner(mpkRobots* robots, grasp::ColdetLinkPairPtr *test_pairs, int test_size, const vector<int>& SampleDOF
#endif
#ifndef ADAPT_COLLCHECKER
	   ,double epsilon
#endif
	   ) 
{ 
  test_pairs_ = test_pairs;
  test_size_ = test_size;
  robots_ = robots;

  sampleDOF = SampleDOF;

  nodes_counter = 0;
  freePath.clear();
  if(sampleDOF.size()>1){
    dof0_ = sampleDOF[0];
    dof1_ = sampleDOF[1];
  }else{
    dof0_ = 0;
    dof1_ = 1;
  }  
  total_CC = 0;
  path_CC = 0;
  path_rej  = 0;
  sampled_Mil = 0;
  CC_time  = 0;
  onBridge = 0;

  Ti.setDOFS(dof0_,dof1_);
  Tg.setDOFS(dof0_,dof1_);

  // Initialize the trees
  Ti.setGridSize(GRID_DIVS,GRID_DIVS); // Grid size for both trees
  Tg.setGridSize(GRID_DIVS,GRID_DIVS);
  
  point_checker = new mpkConfigChecker(test_pairs, test_size, robots);
#ifndef ADAPT_COLLCHECKER
  EPSILON = epsilon; // Epsilon is the resolution to which a segment is checked
#endif

}


// ----------------------------------------------------------------
// Query: receive the start and goal configurations (qs and qg) and
//        returns the path (genPath) if found. rho = neighborhood to
//        sample, max_it = max number of iterations allowed
// ----------------------------------------------------------------

bool
sblPlanner::
Query(mpkConfig& qs, mpkConfig& qg, list<mpkConfig> &genPath,
      double rho, int max_it)
{
	error_message_ = "";
  // check whether straight line connects

#ifdef ADAPT_COLLCHECKER
  mpkAdaptSegmentChecker seg_check(point_checker, &qs, &qg);
#else
  mpkSimpleSegmentChecker seg_check(point_checker, &qs, &qg, EPSILON);
#endif

  while (seg_check.iteration_step())
    ;

  if ( !seg_check.collision() ) {

    genPath.push_back(qs);
    genPath.push_back(qg);
    return true;
  }

  // if not -> plan
  allNodes.resize(max_it);

#ifdef WIN32
  _timeb start, end;
#else
  struct tms start, end; // to take time for planning
#endif

  if(! setRoot(qs,Ti)){
    cerr << "Initial configuration in collision!"<<endl;
		error_message_ = "Initial configuration in collision!";
    return false;
  }

  if(! setRoot(qg,Tg)){
    cerr << "Goal configuration in collision!"<<endl;
		error_message_ = "Goal configuration in collision!";
    return false;
  }
  
  if(sampleDOF.size() ==0){
	cerr << "Cannot find a linear motion without DOF" <<endl;
	error_message_ = "Cannot find a linear motion without DOF";
	  return false;
  }
  
  
  
  int iter = 2;
#ifdef WIN32
  _ftime(&start);
#else
  times(&start);
#endif

  // Algorithm PLANNER
  while(iter<max_it){

    sblMilestone &new_node = allNodes[nodes_counter];
    
#ifdef ADAPT_COLLCHECKER
    new_node.adapt_point_checker = new mpkConfigChecker(test_pairs_, test_size, robots_);
#endif

    expandTree(rho, new_node);

    if (ConnectTrees(IN_SAME_BIN, new_node))
    break;

    if( (iter % 50)==0){
      
      sblTree tmpTi;
      sblTree tmpTg;
      int dim = qs.size();
      do{

		if(sampleDOF.size()>1){
			int CDOF = sampleDOF.size();
			int tmp1=mpk_lrand()%CDOF;
			int tmp2=mpk_lrand()%CDOF;
	  
			dof0_ = sampleDOF[tmp1];
			dof1_ = sampleDOF[tmp2];
		}else{
			dof0_ = (mpk_lrand() % dim);
			dof1_ = (mpk_lrand() % dim);
		}

      }while(dof0_==dof1_);
      
      tmpTi.setDOFS(dof0_,dof1_);
      tmpTg.setDOFS(dof0_,dof1_);
      
      // Initialize the trees
      tmpTi.setGridSize(GRID_DIVS,GRID_DIVS); 
      tmpTg.setGridSize(GRID_DIVS,GRID_DIVS);
      
      // Copy the milestones
      Ti.rebuildTree(tmpTi, allNodes);
      Tg.rebuildTree(tmpTg, allNodes);
      Ti = tmpTi;
      Tg = tmpTg;
      
    }

    iter++;
    if ( (iter % 100) == 0 ) {
      //cout << " iter " << iter << endl;
    }
  };

  if(theBridge.seg_checker) {
    delete theBridge.seg_checker;
    theBridge.seg_checker = 0;
  }
  

#ifdef WIN32
  _ftime(&end);
#else
  times(&end);
#endif
  
  if(iter>=max_it){
    cerr << "Sorry, couldn't find a path" << endl;
		error_message_ = "Could not find a path";
    return false;
  }
  else {
    genPath.clear();
    printStats(start, end, genPath);
#ifdef DEBUG_MODE
	mpkConfigChecker check1(test_pairs_, test_size_, robots_);
	 list<mpkConfig>::iterator it;
	for ( it = genPath.begin(); *it != genPath.back(); ++it ){
	 list<mpkConfig>::iterator it2 = it;
		it2++;
		mpkSimpleSegmentChecker seg_check(&check1, &(*it), &(*it2), 0.005);
		while(seg_check.iteration_step(mpkSimpleSegmentChecker::DEBUG_TEST));
	}
 #endif
    return true;
  }
  
}

// -------------------------------------------------------
// setRoot: used to set  the root of the tree
// -------------------------------------------------------
bool
sblPlanner::
setRoot(mpkConfig& q, sblTree& T)
{    
  sblMilestone m(nodes_counter, q);
  total_CC++; // Update the counter of coll checks

#ifdef ADAPT_COLLCHECKER
  m.adapt_point_checker = new mpkConfigChecker(test_pairs_, test_size, robots_);
  if(m.adapt_point_checker->clearance(&q) <= 0){
#else
#ifdef DO_TOLERANCE_TEST
  if(point_checker->clearance(&q) <= 0){
#else
  if(point_checker->collision(&q)){
#endif
#endif

    cerr << "Configuration in collision!"<<endl;
    return false;
  }

  allNodes[nodes_counter] = m;

#ifdef ADAPT_COLLCHECKER
  m.adapt_point_checker = 0;
#endif

  T.addTosblBins(m);
  ++nodes_counter;

  return true;
}

// ----------------------------------------------------------------
// expandTree: it is the responsible of growing the trees
//             Basically, it selects a node from the tree 
//             to expand, and then samples in its neighborhood 
//             then add the new node to the tree             
// ----------------------------------------------------------------
void 
sblPlanner::
expandTree(double rho, sblMilestone& q_new)
{
  double rand_n = mpk_drand();

  if(rand_n>=0.5) // Expand each tree with prob = 0.5
    { 
      // Expand Tg
      // Choose a node from the tree to be expanded
      int selected = Tg.getNodeToExpand();
      
      // Get q_new by sampling on an rho-neighborhood of q_expand
      while(! sample(allNodes[selected], q_new, rho)){
	// sample couldn't find a free config, try again with
	// a different node to expand
	selected = Tg.getNodeToExpand(); 
      };
 
      // Add the q_new to the tree
      addsblMilestone(Tg, q_new,allNodes[selected]);
      tree_ = T_GOAL; // to signal that the expanded tree was Tg
    }
  else // Expand Ti
    {
      // Same process than in the if part...
      int selected = Ti.getNodeToExpand(); 

      while(! sample(allNodes[selected], q_new, rho)){
	selected = Ti.getNodeToExpand();
      };
      addsblMilestone(Ti, q_new, allNodes[selected]);
      tree_ = T_INIT;
    }
  return;
}
 
// ---------------------------------------------------------
// sample: It receives a milestone m and generate one new
//         milestone in the rho-neighborhood of m
// ---------------------------------------------------------
bool
sblPlanner::
sample(const sblMilestone& m, sblMilestone& new_node, double rho)
{
  bool isInCollision;
  mpkConfig conf(m.getConfig().size());
  int i=1;  double rho_local= rho;

  bool map;
  if(m.getConfig().size() > robots_->nJoints+5 ) map = true;
  else map = false;
	
	
  do{
    randConfig.LocalBoxSample(m.getConfig(),rho_local,&conf, sampleDOF);

    // reset those DOFs that should not be sampled
    int c=0;
    //for ( int k=0; k<robots_->num_robots(); k++ ) {
    //  const mpkBaseRobot* robk = robots_->rob[k];
    //  for ( int j=0; j<robk->num_params; j++, c++ ) {
    //if ( robk->param_opts[j].is_frozen || robk->param_opts[j].is_passive )
    //  conf[c] = m.getConfig()[c];
    //}
    //}
    for ( int j=0; j<robots_->nJoints; j++, c++ ) {
        if ( robots_->param_opts[j].is_frozen || robots_->param_opts[j].is_passive )
	    conf[c] = m.getConfig()[c];
    }
    if(map){
	for ( int j=robots_->nJoints; j< (robots_->nJoints+6); j++, c++ ) {
		if ( robots_->param_opts[j].is_frozen || robots_->param_opts[j].is_passive )
			conf[c] = m.getConfig()[c];
	}
    }

    new_node.setConfig(conf);

    total_CC++;
    sampled_Mil++;

#ifdef WIN32
    _ftime(&optstart);
#else
    times(&optstart);
#endif

#ifdef ADAPT_COLLCHECKER
    if(new_node.adapt_point_checker->clearance(&new_node.getConfig()) <= 0)
      isInCollision = 1;
#else    
#ifdef DO_TOLERANCE_TEST
    if(point_checker->clearance(&new_node.getConfig()) <= 0)
      isInCollision = 1;
#else
    if(point_checker->collision(&new_node.getConfig()))
      isInCollision = 1;
#endif
#endif

    else{
      isInCollision = 0;
    }

#ifdef WIN32
	_ftime(&optend);
#else
	times(&optend);
#endif

    addTime(optstart, optend);

    if(! isInCollision){
      return true;
    }

    // If the sampled config was in collision, reduce the size of rho
    rho_local = rho/(i);
    i++;
  }while(i<4);
  // it couldn't found a free config after 4 iters, then quit
  return false;
}


// ----------------------------------------------------------------
// addsblMilestone: Adds a milestone to the tree, and updates the bins
// ----------------------------------------------------------------

void
sblPlanner::
addsblMilestone(sblTree& T, sblMilestone& newNode, sblMilestone& expandedNode)
{
  newNode.setIndex(nodes_counter);
  int x = expandedNode.getIndex();
  allNodes[x].isFatherOf(newNode);
  allNodes[nodes_counter].edgeFromFather.seg_checker = 0;
  T.addTosblBins(newNode);
  ++nodes_counter;
}

// ----------------------------------------------------------------
// Function ConnectTrees:   It tries to establish connection
//                          between two nodes, one from each
//                          tree.
// ----------------------------------------------------------------

bool
sblPlanner::
ConnectTrees(int option,  const sblMilestone& m)
{ 
  int selNode;
  int a,b;

  // ConnectTrees try a connection between the new node and any other
  // node in the same bin on the other tree

  // Compute in which bin is the new node
  const mpkConfig& q = m.getConfig();
  double delta = Ti.getDelta();
  int cells = Ti.getDIVS(); 

  int selsblBin = int(floor(q[dof0_]/delta)+floor(q[dof1_]/delta)*cells);
  
  // Get a node on the same bin on the other tree
  if(tree_ == T_INIT)// if the expanded tree was Ti
    selNode = Tg.getClosestNode(m, allNodes);
  
  if(tree_ == T_GOAL)
    selNode = Ti.getClosestNode(m, allNodes);

  // If no node was selected, connectTrees has failed
  if(selNode == -1){
    if(tree_ == T_INIT){// if the expanded tree was Ti
      int bin = Tg.choosesblBin();
      selNode = Tg.getNodeFromsblBin(bin);
      double d = allNodes[m.getIndex()].distance(allNodes[selNode]);
      if(d >0.8)
	selNode = -1;
    }
    if(tree_ == T_GOAL){
      int bin = Ti.choosesblBin();
      selNode = Ti.getNodeFromsblBin(bin);
      double d = allNodes[m.getIndex()].distance(allNodes[selNode]);
      if(d >0.8)
	selNode = -1;
    }
  }
  
  if(selNode == -1)
    return false;
  
  // Otherwise, get the indexes for the two nodes that were
  // connected

  if(tree_ == T_INIT){
    a = m.getIndex();
    b = selNode;
  }
  if(tree_ == T_GOAL){// Must be T_GOAL
    a = selNode;
    b = m.getIndex();
  }

  // Constructs the bridge between the two trees
  theBridge.setFrom(a);
  theBridge.setTo(b);
  theBridge.setsblEdge(a, b, allNodes[a].distance(allNodes[b])); 
 
#ifdef ADAPT_COLLCHECKER
  theBridge.seg_checker =
    new mpkAdaptSegmentChecker(allNodes[a].adapt_point_checker,
			       allNodes[b].adapt_point_checker);
#else
  theBridge.seg_checker =
    new mpkSimpleSegmentChecker(point_checker,
				&allNodes[a].getConfig(),
				&allNodes[b].getConfig(),
				EPSILON);
#endif

  // Put the path in a Priority Queue sorted according to the 
  // length of the segments in decreasing order

  edgesQueue candidatePath;

  getsblEdges(allNodes[a], allNodes[b], candidatePath);

  return (testPath(candidatePath));
}

// ---------------------------------------------------------------------
// getsblEdges(): Once that all the segments have been marked as clear,
// this function extracts all the edges in such a way that creates the
// actual path
// ---------------------------------------------------------------------

void
sblPlanner::
getsblEdges(sblMilestone &node1, sblMilestone& node2, edgesQueue& pathQueue)
{
  freePath.clear();
  int from =  node1.getFather();
  sblEdge *tmpsblEdge = &node1.getsblEdge();
  
  if(from==-999){
    freePath.push_back(node1.getConfig());
  }
  else {
    freePath.push_back(node1.getConfig());

    //Extracts the segments on Ti
    do{
 
      if(tmpsblEdge->seg_checker==0){
	tmpsblEdge->seg_checker =

#ifdef ADAPT_COLLCHECKER
	  new mpkAdaptSegmentChecker(allNodes[tmpsblEdge->getFrom()].adapt_point_checker,
				     allNodes[tmpsblEdge->getTo()].adapt_point_checker);
#else
	  new mpkSimpleSegmentChecker(point_checker,
				      &allNodes[tmpsblEdge->getFrom()].getConfig(), 
				      &allNodes[tmpsblEdge->getTo()].getConfig(),
				      EPSILON);
#endif


      }
      
      pathQueue.push(tmpsblEdge);
      sblMilestone &next = allNodes[from];
      freePath.push_front(next.getConfig());
      tmpsblEdge = &next.getsblEdge();
      from = next.getFather();

    } while(from!=-999);
  }

  from = node2.getFather();
  tmpsblEdge =  &node2.getsblEdge();

  if(from==-999){
    freePath.push_back(node2.getConfig());
  }
  else {
    //Extracts the segments on Tg
    freePath.push_back(node2.getConfig());
    do{

      if(tmpsblEdge->seg_checker==0){
	tmpsblEdge->seg_checker =

#ifdef ADAPT_COLLCHECKER
	  new mpkAdaptSegmentChecker(allNodes[tmpsblEdge->getFrom()].adapt_point_checker,
				     allNodes[tmpsblEdge->getTo()].adapt_point_checker);
#else
	  new mpkSimpleSegmentChecker(point_checker,
				      &allNodes[tmpsblEdge->getFrom()].getConfig(), 
				      &allNodes[tmpsblEdge->getTo()].getConfig(),
				      EPSILON);
#endif
      }

      pathQueue.push(tmpsblEdge);
      sblMilestone &next = allNodes[from];
      freePath.push_back(next.getConfig());
      tmpsblEdge = &next.getsblEdge();
      from = next.getFather();

    } while(from !=-999);
  }
  pathQueue.push(&theBridge);

  return;
}


// -----------------------------------------------------------
// testPath():  Tests whether a path is clear or not at 
//              a certain resolution. It uses the partition
//              technique
// -----------------------------------------------------------

bool 
sblPlanner::
testPath(edgesQueue& tau)
{
  while(!tau.empty()){
    // Take the segment which is on the top of the queue
    // i.e. the one with longest distance between nodes
    sblEdge *u =  tau.top();
    tau.pop();

#ifdef  ADAPT_COLLCHECKER
    mpkAdaptSegmentChecker *lscu =  u->seg_checker;
#else
    mpkSimpleSegmentChecker *lscu =  u->seg_checker;
#endif

    bool checks_pending = lscu->iteration_step();

    if( !checks_pending && lscu->collision() ){
      delSegment(*u);
      return false;
    }
    
    // If segment wasn't in collision (at this resolution) but it 
    // cannot be marked as safe yet, re insert it on the queue
    if ( checks_pending )
      tau.push(u); 
  }
  return true;
}


// ---------------------------------------------------------------
// delSegment: When a collision is detected in one of the
//             edges, eliminate the edge from the tree
// ---------------------------------------------------------------

void
sblPlanner::
delSegment(sblEdge& inColl)
{
  int inTree= 0;
  
  // Get the milestones of the edge to be eliminated
  int from = inColl.getFrom();
  int to = inColl.getTo();
   
  delete inColl.seg_checker;
  inColl.seg_checker = 0;

  // If they coincide with the bridge, nothing to be done here
  if(from  == theBridge.getFrom()){
    return;
  }

  // Take node 'from' and go backwards until the root is reached
  // This is for finding out in which tree is the bad link 
  sblEdge *tmp = &allNodes[from].getsblEdge();
  
  while(tmp->getFrom()!=-999){
    tmp = &allNodes[tmp->getFrom()].getsblEdge();
  };

  // Each tree has as root a node with 'from' = -999 and 'to'= x
  // where x is an integer different for each tree. Ti = 0, Tg = 1 ...
  inTree = tmp->getTo(); 

  int a, b, c;
  sblEdge *theFatherOfb;

  if(inTree == T_INIT){ // The broken link is on Ti
    a = theBridge.getTo();
    b = theBridge.getFrom();
    theFatherOfb = &allNodes[b].getsblEdge();
    c = theFatherOfb->getFrom();
  }
  else{ // It is on Tg
    a = theBridge.getFrom();
    b = theBridge.getTo();
    theFatherOfb = &allNodes[b].getsblEdge();
    c = theFatherOfb->getFrom();
  }
  
  //This establishes the direction of the connection on the bridge
  allNodes[a].isFatherOf(allNodes[b]); // a is the father of b

#ifdef ADAPT_COLLCHECKER
  mpkAdaptSegmentChecker *tmp1, *tmp2;
#else
  mpkSimpleSegmentChecker *tmp1, *tmp2;
#endif

  tmp1 = allNodes[b].edgeFromFather.seg_checker;
  allNodes[b].edgeFromFather.seg_checker = theBridge.seg_checker;

  theBridge.seg_checker = 0;

  updatesblBins(inTree, allNodes[b]);
  
  while(b != to ){ 
    a = b;
    b = c;
    theFatherOfb = &allNodes[b].getsblEdge();
    c = theFatherOfb->getFrom();
    
    allNodes[a].isFatherOf(allNodes[b]); // a is father of b; earlier b was father of a
    allNodes[b].removeChild(allNodes[a]); // a is not any more a child of b
    
    tmp2 = allNodes[b].edgeFromFather.seg_checker;
    allNodes[b].edgeFromFather.seg_checker = tmp1;
    tmp1 = tmp2;

    updatesblBins(inTree, allNodes[b]);
  }
  allNodes[c].removeChild(allNodes[b]); // b is not any more a child of c
}



// -----------------------------------------------------------------
// updatesblBins: The purpose of this function is to update the bins of
//             trees Ti and Tg, deleting the milestone m from one of
//             them and adding it to the other one 
// -----------------------------------------------------------------

void 
sblPlanner::
updatesblBins(int del, sblMilestone& m)
{
  int sons = m.getNrSons();
  if(sons == 0){ // If the milestone doesn't have children
    if(del==T_INIT){ 
      // Delete the node from one bin and add it to the other
      Ti.delFromsblBins(m);
      Tg.addTosblBins(m);
    }
    else{ //Delete from T_GOAL
      Tg.delFromsblBins(m);
      Ti.addTosblBins(m);
    }
    
    return; // end with the recursion
  }

  else{ 
    // Otherwise, there are children. Do the same for
    // each one of them.
    int r;
    for(int k=0;k<sons;k++){
      r = m.getChild(k);
      updatesblBins(del, allNodes[r]);
    }
    // Once that all the children have been deleted,
    // delete also the root of this subtree
    delsblMilestone(del,m);
  }
}

/* --------------------------------------------------------------------
   This function is used as an auxiliary one to delete one node from 
   the tree, not taking care of the child. 
   -------------------------------------------------------------------- 
*/
void 
sblPlanner::
delsblMilestone(int del, sblMilestone& m)
{
  if(del==T_INIT){ 
    // Delete the node from one bin and add it to the other
    Ti.delFromsblBins(m);
    Tg.addTosblBins(m);
  }
  else{ //Delete from T_GOAL
    Tg.delFromsblBins(m);
    Ti.addTosblBins(m);
  }
  return;
}

/* -----------------------------------------------------------
   This function computes the total length of a path
   -----------------------------------------------------------
*/
double
sblPlanner::
pathLength(const list<mpkConfig>& tau)
{
  mpkConfig current, next;
  double dist=0.0;
  int pathSize =0;

  pathSize = tau.size();
  list<mpkConfig>::const_iterator pathIter = tau.begin();

  current = (*pathIter);
  for( int counter =0;counter <pathSize-1;counter++){
    next = (*(++pathIter));
    dist += current.dist(next);
    current = next;
  }
  return dist;
}


// **************************************************************
//                  Auxiliary I/O Functions
// **************************************************************

// --------------------------------------------------------------
// report_times: Print a report of time consumption
// --------------------------------------------------------------
#ifdef WIN32
void
sblPlanner::
report_times(_timeb start, _timeb end)
{
	int sec = end.time - start.time;
	double time = sec + (double)(end.millitm - start.millitm)/1000;
	// cerr << "Total time: " << utime + stime << "\n";
	cerr << "\n"<<time << "\t";
 }
#else
void
sblPlanner::
report_times(struct tms start, struct tms end)
{
  float ticks = CLK_TCK;
  float utime = (end.tms_utime - start.tms_utime) / ticks;
  float stime = (end.tms_stime - start.tms_stime) / ticks;
  // cerr << "Total time: " << utime + stime << "\n";
  cerr << "\n"<<utime + stime << "\t";
 }
#endif

// ------------------------------------------------------------------
// AddTime: To store the time required for collision checking
// ------------------------------------------------------------------
#ifdef WIN32
void
sblPlanner::
addTime(_timeb start, _timeb end)
{
	int sec = end.time - start.time;
	double time = sec + (double)(end.millitm - start.millitm)/1000;
	CC_time += time;
 }
#else
void
sblPlanner::
addTime(struct tms start, struct tms end){
  float ticks  = CLK_TCK;
  float utime = (end.tms_utime - start.tms_utime) / ticks;
  float stime = (end.tms_stime - start.tms_stime) / ticks;
  CC_time += (utime + stime); // time
}
#endif

// --------------------------------------------------------------
// printStats: Presents statistics of the trees and path
// --------------------------------------------------------------

#ifdef WIN32
void
sblPlanner::
printStats(_timeb start, _timeb end, list<mpkConfig> &genPath)
{
  //report_times(start, end);
  // checkPath(freePath);
  
  // This prints a line with:
  // time|Nr Mil|Mil on Path|Total CC|CC Path|sampled Mil|

  //cerr<<Ti.getSize()+Tg.getSize()<<"\t"<<freePath.size()<<"\t" <<total_CC<<"\t"<<path_CC<<"\t"<<sampled_Mil<<"\t"<<CC_time;

  // Copy the path to the parameter sent to the function
  
  list<mpkConfig>::iterator I;
  genPath.clear();
  for(I = freePath.begin(); I!=freePath.end(); I++)
    genPath.push_back((*I));
  
}
#else
void
sblPlanner::
printStats(struct tms start, struct tms end, list<mpkConfig> &genPath)
{
  //report_times(start, end);
  // checkPath(freePath);
  
  // This prints a line with:
  // time|Nr Mil|Mil on Path|Total CC|CC Path|sampled Mil|

  //cerr<<Ti.getSize()+Tg.getSize()<<"\t"<<freePath.size()<<"\t" <<total_CC<<"\t"<<path_CC<<"\t"<<sampled_Mil<<"\t"<<CC_time;

  // Copy the path to the parameter sent to the function
  
  list<mpkConfig>::iterator I;
  genPath.clear();
  for(I = freePath.begin(); I!=freePath.end(); I++)
    genPath.push_back((*I));
  
}
#endif

// --------------------------------------------------------------
// writePath: Sends the path to a file
// --------------------------------------------------------------

void 
sblPlanner::
writePath(const list<mpkConfig>& path, char* name)
{
  list<mpkConfig>::const_iterator it;
  FILE *fp;

  fp=fopen(name,"wt");

  if (fp==NULL) {
    cerr<<"Error in write_path when opening file"<<endl;
    return ;
  }

  for ( it = path.begin(); it != path.end(); it++) {
      const mpkConfig& q = (*it);
      int dim = q.size();
      for(int i = 0; i < dim; i++)       fprintf(fp, "%f ", q[i]);
      fprintf(fp, "\n");
  }
  fclose(fp);
}

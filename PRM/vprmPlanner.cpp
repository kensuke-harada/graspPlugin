// **************************************************************
// Visibility PRM Path Planner implementation.
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
#include "vprmPlanner.h"
#include "sblTree.h"


// Class variable initializations

// ------------------------------
// Constructor & Destructor
// ------------------------------

using namespace cnoid;

vprmPlanner::
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
vprmPlanner(mpkRobots* robots, ColdetLinkPairPtr *test_pairs, int test_size, const vector<int>& SampleDOF
#else
vprmPlanner(mpkRobots* robots, grasp::ColdetLinkPairPtr *test_pairs, int test_size, const vector<int>& SampleDOF
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
  CC_time  = 0;

  point_checker = new mpkConfigChecker(test_pairs, test_size, robots);
#ifndef ADAPT_COLLCHECKER
  EPSILON = epsilon; // Epsilon is the resolution to which a segment is checked
#endif

}

bool
vprmPlanner::
Query(mpkConfig& qs, mpkConfig& qg, list<mpkConfig> &genPath,
      double rho, int max_it)
{

	if(Visible(qs, qg)){
		genPath.push_back(qs);
		genPath.push_back(qg);
		return true;
	}

	if(!constructRoadMaps(qs, qg, max_it)) return false;

	if(!DFS(genPath)) return false;

	return true;
}

bool
vprmPlanner::
constructRoadMaps(mpkConfig& qs, mpkConfig& qg, int max_it){

	int dim = qs.size();

	connectList.clear();

	allNodes.resize(max_it);
	allNodes[0].setConfig(qs);
	allNodes[0].setIndex(0);
	allNodes[0].isGuard = true;

	int m=0;
	int max_m = 15;
	nodes_counter = 1;
	double eps = 1.0e-10;

	while(m<max_m-1){                                                   //cout << "m = " << m << "/" << max_it-1 << " nodes= " << nodes_counter << endl;

		double found = false;
	    sblMilestone &q = allNodes[nodes_counter];
	    q.setIndex(nodes_counter);

		while(!sample(allNodes[0], q))
			;

		sblMilestone &g_vis = allNodes[max_it-1];
		g_vis.setIndex(max_it-1);                                         //displayMilestone(g_vis);

		for(int n = 0; n<nodes_counter; n++){

			int nodes = (int)(nodes_counter*mpk_drand());
			sblMilestone &g = allNodes[nodes];
			g.setIndex(nodes);                                             //displayMilestone(g);

			if(g.isGuard){

				if(Visible(q.getConfig(), g.getConfig())){

					if(g_vis.getIndex() == max_it-1){
						g_vis = g;
					}
					else if(!alreadyConnected(g.getIndex(), g_vis.getIndex()) && (g.getIndex() != g_vis.getIndex())){

						q.isConnector = true;

						tryConnect(q, allNodes[g.getIndex()]);
						tryConnect(q, allNodes[g_vis.getIndex()]);
						addConnectionList(g.getIndex(), g_vis.getIndex());

						nodes_counter++;
						found = true;
						                                                      //displayMilestone(q);

					}
				}

				if(g_vis.getIndex() == max_it-1){

					q.isGuard = true;

					nodes_counter++;
					found = true;                                            //displayMilestone(q);
					m=0;
				}
				else
					m++;

				if(found)
					break;

			}
		}
	}

	allNodes[nodes_counter].setConfig(qg);
	allNodes[nodes_counter].setIndex(nodes_counter);

	//double maxDist = 1.0;

	for(int i=1; i<nodes_counter; i++){
		//if(allNodes[nodes_counter].getConfig().dist2(allNodes[i].getConfig()) > maxDist) continue;

		if(Visible(allNodes[nodes_counter].getConfig(), allNodes[i].getConfig())){    //cout << i << " and " << nodes_counter << " are connected " << endl;
			tryConnect(allNodes[i], allNodes[nodes_counter]);
			break;
		}

		if(i==nodes_counter-1){                                                       cout << "Goal cannot be connected." << endl;
			return false;
		}
	}

	nodes_counter++;

	for(int i=0; i<nodes_counter; i++)	displayMilestone(allNodes[i]);

	return true;

}

void vprmPlanner::
displayMilestone(sblMilestone& q){

	cout << "Milestone #" << q.getIndex();
	if(q.isGuard)
		cout << " is guard. ";
	else if(q.isConnector)
		cout << "is connector. ";
	cout << "It has " << q.getNrSons() << " children (";
	for(int j=0; j<q.getNrSons(); j++)
		cout << q.getChild(j) << " ";
	cout << ")" << endl;

}

void vprmPlanner::
tryConnect(sblMilestone& q1, sblMilestone& q2){

	q1.addChild(q2);
	q2.addChild(q1);
}

void vprmPlanner::
addConnectionList(int n1, int n2){

	vector<int> c;
	c.push_back(n1);
	c.push_back(n2);
	connectList.push_back(c);
}

bool vprmPlanner::
alreadyConnected(int n1, int n2)
{

	for(size_t i=0; i<connectList.size(); i++){
		if(connectList[i][0] == n1 && connectList[i][1] == n2) return true;

		if(connectList[i][0] == n2 && connectList[i][1] == n1) return true;
	}

	return false;
}

bool vprmPlanner::
Visible(mpkConfig& q1, mpkConfig& q2)
{

#ifdef ADAPT_COLLCHECKER
  mpkAdaptSegmentChecker seg_check(point_checker, &q1, &q2);
#else
  mpkSimpleSegmentChecker seg_check(point_checker, &q1, &q2, EPSILON);
#endif

  while (seg_check.iteration_step())
    ;

  if ( !seg_check.collision() )
    return true;

  return false;
}

bool vprmPlanner::DFS(list<mpkConfig>& configPath)
{
    for (unsigned int i=0; i<nodes_counter; i++)
        allNodes[i].visited = false;

    vector<sblMilestone> path;

    allNodes[0].visited = true;
    sblMilestone startNode = allNodes[0];
    sblMilestone goalNode  = allNodes[nodes_counter-1];

    path.push_back(startNode);
    while (path.size() > 0) {
        sblMilestone node = path.back();

        if (node.getIndex() == goalNode.getIndex()) {
            break;
        } else {
            int child=-1;
            for (int i=0; i<node.getNrSons(); i++) {
                if (!allNodes[node.getChild(i)].visited) {
                    child = node.getChild(i);
                    break;
                }
            }
            if (child == -1) {
                path.pop_back();
            } else {
                allNodes[child].visited = true;
                path.push_back(allNodes[child]);
            }
        }
    }



    for(unsigned int i=0; i<path.size(); i++)
    	configPath.push_back(path[i].getConfig());

    if(configPath.size()>1) return true;
    cout << "No path found." << endl;
    return false;
}


// ---------------------------------------------------------
// sample: It receives a milestone m and generate one new
//         milestone in the rho^neighborhood of m
// ---------------------------------------------------------
bool
vprmPlanner::
sample(const sblMilestone& org_node, sblMilestone& new_node)
{
	int dim = org_node.getConfig().size();
	int iterate=0;

	bool isInCollision;
	mpkConfig conf(dim);

	bool map;
	if(org_node.getConfig().size() > robots_->nJoints+5 ) map = true;
	else map = false;


	do{
		conf = org_node.getConfig();

		for (unsigned int i=0; i<sampleDOF.size(); i++) {

			int dof = sampleDOF[i];
			conf[dof] = mpk_drand();
		}

		// reset those DOFs that should not be sampled
		int c=0;
		for ( int j=0; j<robots_->nJoints; j++, c++ ) {
			if ( robots_->param_opts[j].is_frozen || robots_->param_opts[j].is_passive )
				conf[c] = org_node.getConfig()[c];
		}
		if(map){
			for ( int j=robots_->nJoints; j< (robots_->nJoints+6); j++, c++ ) {
				if ( robots_->param_opts[j].is_frozen || robots_->param_opts[j].is_passive )
					conf[c] = org_node.getConfig()[c];
			}
		}

		new_node.setConfig(conf);

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

		if(! isInCollision)
			return true;

		iterate++;
	}while(iterate<4);

    return false;
}




/* -----------------------------------------------------------
   This function computes the total length of a path
   -----------------------------------------------------------
*/
double
vprmPlanner::
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
vprmPlanner::
report_times(_timeb start, _timeb end)
{
	int sec = end.time - start.time;
	double time = sec + (double)(end.millitm - start.millitm)/1000;
	// cerr << "Total time: " << utime + stime << "\n";
	cerr << "\n"<<time << "\t";
 }
#else
void
vprmPlanner::
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
vprmPlanner::
addTime(_timeb start, _timeb end)
{
	int sec = end.time - start.time;
	double time = sec + (double)(end.millitm - start.millitm)/1000;
	CC_time += time;
 }
#else
void
vprmPlanner::
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
vprmPlanner::
printStats(_timeb start, _timeb end, list<mpkConfig> &genPath)
{
  list<mpkConfig>::iterator I;
  genPath.clear();
  for(I = freePath.begin(); I!=freePath.end(); I++)
    genPath.push_back((*I));

}
#else
void
vprmPlanner::
printStats(struct tms start, struct tms end, list<mpkConfig> &genPath)
{
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
vprmPlanner::
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

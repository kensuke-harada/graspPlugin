#include <list>

#include "mpk_rand.h"
#include "mpkPathSmoother.h"

using namespace cnoid;

mpkPathSmoother::
mpkPathSmoother(const vector<mpkConfig>& path,
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		  ColdetLinkPairPtr *test_pairs, int test_size,
#else
		  grasp::ColdetLinkPairPtr *test_pairs, int test_size,
#endif  
		  mpkRobots *robots,
		  double min_shortcut_len, double epsilon)
{
  len = 0;
  node* curr;
  node* prev = 0;

  // copy the path into internal list representation and compute its
  // length
  for ( int i=0; i < (int)path.size(); i++ ) {
    if ( i>0 ) len += path[i-1].dist(path[i]);
    curr = new node;
    if ( i==0 ) first = curr;
    curr->next = 0;
    curr->q = path[i];
    if ( prev ) prev->next = curr;
    prev = curr;
  }
  num_points = path.size();
  this->min_shortcut_len = min_shortcut_len;
  p1 = mpkConfig(path[0].size());
  p2 = mpkConfig(path[0].size());
  this->test_pairs = test_pairs;
  this->test_size = test_size;
  this->robots = robots;
  this->epsilon = epsilon;  // only used by simple segment checker
}

mpkPathSmoother::
~mpkPathSmoother()
{
  // delete internal list representation of path
  node* curr = first;
  while ( curr ) {
    node* tmp = curr;
    curr = curr->next;
    delete tmp;
  }
}

mpkPathSmoother::
node*
mpkPathSmoother::seg_node(int seg_idx)
{
  assert( seg_idx <= num_points-2 );
  node* curr = first;
  for ( int i=0; i<seg_idx; i++ ) {
    curr = curr->next;
  }
  assert(curr->next);
  return curr;
}

mpkPathSmoother::
node*
mpkPathSmoother::lin_interpol(double t, mpkConfig& q)
{
  assert( t >= 0.0 && t <= 1.0 );

  if ( !first->next || t==0.0 ) {
    q = first->q;
    return first;
  }
  else if ( t==1.0 ) {
    node* curr;
    for ( curr = first; curr->next && curr->next->next; curr = curr->next )
      ;
    if ( curr->next )
      q = curr->next->q;
    else
      q = curr->q;
    return curr;
  }
  
  double tcurr = 0;
  for ( node* curr = first; curr->next; curr = curr->next ) {

    double deltat = curr->q.dist(curr->next->q) / length();
    double tnext = tcurr + deltat;

    if ( tnext > t && deltat > 0 ) {
      q.lin_interpol((t-tcurr) / deltat, curr->q, curr->next->q);
      return curr;
    }

    tcurr = tnext;
  }

  return 0;
}

void
mpkPathSmoother::
replace_section(mpkPathSmoother::node* n1, mpkPathSmoother::node* n2,
		const mpkConfig& q1, const mpkConfig& q2)
{
  if ( n1==n2 ) {
    node* nnew1 = new node;
    node* nnew2 = new node;
    nnew2->next = n1->next;
    nnew1->next = nnew2;
    n1->next = nnew1;
    nnew2->q = q2;
    nnew1->q = q1;
    return;
  }
  assert(n1);
  assert(n2);
  assert(n1->next);
  assert(n2->next);
  // recompute length
  node* curr=n1;
  while ( 1 ) {
    len -= curr->q.dist(curr->next->q);
    if ( curr==n2 ) break;
    curr=curr->next;
  }
  // remove old section
  while ( 1 ) {
    node* del_node = n1->next;
    n1->next = del_node->next;
    delete del_node;
    num_points--;
    if (del_node == n2 ) break;
  }
  // insert three bridging segments between n1 and n1->next
  node* last = n1;
  if ( q1 != n1->q ) {
    node* new1 = new node;
    new1->q = q1;
    new1->next = n1->next;
    n1->next = new1;
    last = new1;
    num_points++;
  }
  if ( q2 != last->next->q ) {
    node* new2 = new node;
    new2->q = q2;
    new2->next = last->next;
    last->next = new2;
    last = new2;
    num_points++;
  }
  len += (n1->q.dist(q1)
    + q1.dist(q2)
    + q2.dist(last->next->q));
}

void
mpkPathSmoother::
get_path(vector<mpkConfig>& path)
{
  path.clear();
  // copy internal list representation into path
  for (node* curr = first; curr; curr=curr->next)
    path.push_back(curr->q);
}

void
mpkPathSmoother::
get_path(list<mpkConfig>& path)
{
  path.clear();
  // copy internal list representation into path
  for (node* curr = first; curr; curr=curr->next)
    path.push_back(curr->q);
}


void
mpkPathSmoother::
smoothe(int num_steps)
{
  mpkConfigChecker check1(test_pairs, test_size, robots);
#ifdef ADAPT_COLLCHECKER
  mpkConfigChecker check2(test_pairs, test_size, robots);
#endif

  double t1 = 0;
  
  int num_shortcuts = 0;
  int orig_num_segs = num_segs();
  double orig_len = length();
#ifdef ENABLE_SBL_MULTITHREAD
	grasp::RobotParallelizer parallel;
	parallel.initialize(true);

	int num_threads = parallel.getNumThreads();
	for ( int i=0; i<num_steps/num_threads+4; i++) {
		if ( num_segs() <= 1 ) break;

		std::vector<node*> n1(num_threads);
		std::vector<node*> n2(num_threads);
		std::vector<mpkConfig*> c1(num_threads);
		std::vector<mpkConfig*> c2(num_threads);
		std::vector<int> col(num_threads, 1);
		std::vector<double> shortcut_len(num_threads, 0);
		max_shortcut = 0;
		for ( int j = 0; j<num_threads; j++) {
			c1[j] = new mpkConfig(p1);
			c2[j] = new mpkConfig(p2);
			double r = mpk_drand() / (double)(num_threads) + (double)j/(double)num_threads;
			parallel.addTask(boost::bind(&mpkPathSmoother::smoothe_parallel, this, _1, &col[j], &n1[j], &n2[j], c1[j], c2[j], &shortcut_len[j], r));
		}
		parallel.doTasks();
		parallel.join();

		double max_len = -1;
		int target_id = -1;
		for (int j = 0; j<num_threads; j++) {
			if (col[j] == 1 || n1[j]==n2[j]) continue;
			if (max_len < shortcut_len[j]) {
				max_len = shortcut_len[j];
				target_id = j;
			}
		}

		if (target_id != -1) {
			replace_section(n1[target_id],n2[target_id],*c1[target_id],*c2[target_id]);
			p1 = *c1[target_id];
			p2 = *c2[target_id];
			num_shortcuts++;
		}

		for (int j = 0; j<num_threads; j++) {
			delete c1[j];
			delete c2[j];
			c1[j] = NULL;
			c2[j] = NULL;
		}
	}
#else
  for ( int i=0; i<num_steps; i++ ) {

    if ( num_segs() <= 1 ) break;

    double t = mpk_drand();
    double t1 = 0.5*t;
    double t2 = 0.5*(t+1.0);
 
    bool coll;
    node *n1;
    node *n2;
    do {

      n1 = lin_interpol(t1, p1);
      n2 = lin_interpol(t2, p2);
	    

#ifdef ADAPT_COLLCHECKER
      if ( check1.clearance(&p1) > 0 && check2.clearance(&p2) > 0 ) {
	mpkAdaptSegmentChecker seg_check(&check1,&check2);
#else
      // actually, p1 and p2 are on path and should be collision-free
      if ( !check1.collision(&p1) && !check1.collision(&p2) ) {
	mpkSimpleSegmentChecker seg_check(&check1, &p1, &p2, epsilon);
#endif
#ifdef TOLERANCE_SMOOTHER	
	while(seg_check.iteration_step(mpkSimpleSegmentChecker::TOLERANCE_TEST))
#else
	while(seg_check.iteration_step())
#endif
	  ;
	coll = seg_check.collision();
      }
      else coll = true;

			//modify:avoid infinite loop by Ohchi
      if ( coll ) {
				double tmp1 = t1;
				double tmp2 = t2;
				t1 = 0.5*(t1+t);
				t2 = 0.5*(t+t2);
				if(t1 == tmp1 && t2 == tmp2){
					t1 = t2;
				}
      }
	//		      if ( coll ) {
	//t1 = 0.5*(t1+t);
	//t2 = 0.5*(t+t2);
 //     }
	
   } while (coll && t2-t1 > min_shortcut_len);

   if ( !coll && n1!=n2 ) {
     replace_section(n1,n2,p1,p2);
     num_shortcuts++;
     //cerr << "shortcut ";
   }
   else{
     //cerr << "         ";
   }

   /*
   cerr << "step: " << i+1 << ", "
	<< "#segs: " << num_segs()
	<< " (";
   fprintf(stderr,"%.2f", double(num_segs())/orig_num_segs*100);
   cerr << "%), "
	<< "length: " << length()
	<< " (";
   fprintf(stderr,"%.2f",length()/orig_len*100);
   cerr << "%), ";
   cerr << "shortcuts: ";
   fprintf(stderr,"%.2f",100*double(num_shortcuts)/(i+1));
   cerr << "%";
   if ( !coll && n1!=n2 ) {
     cerr << ", shortcut interval [" << t1 << "," << t2 << "]";
   }
   cerr << endl;
   */
  }
#endif
    //Part added by Harada@aist
   for (node* curr = first; curr; curr=curr->next){
#ifdef ENABLE_SBL_MULTITHREAD
		 std::vector<int> col(20, 1);
		 std::vector<node*> target(20);
		 node* tmp_node = curr;
		 for(int i=0; i<20; i++){
			 if(!tmp_node->next || !tmp_node->next->next ) break;
			 target[i] = tmp_node->next->next;
			 tmp_node = tmp_node->next;
			 parallel.addTask(boost::bind(&mpkPathSmoother::smoothe_postproc_parallel, this, _1, &col[i], curr, target[i]));
		 }

		parallel.doTasks();
		parallel.join();

		int target_id;
		for (target_id = 0; target_id < 20; target_id++) {
			if (col[target_id] == 1) break;
		}
		if(target_id != 0) {
			curr->next = target[target_id-1];
		}

#else
     for(int i=0; i<20; i++){
       if(!curr->next || !curr->next->next ) break;
       
       bool coll;

#ifdef ADAPT_COLLCHECKER
      if ( check1.clearance(&curr->q) > 0 && check2.clearance(&curr->next->next->q) > 0 ) {
	mpkAdaptSegmentChecker seg_check(&check1,&check2);
#else
      if ( !check1.collision(&curr->q) && !check1.collision(&curr->next->next->q) ) {
	mpkSimpleSegmentChecker seg_check(&check1, &curr->q, &curr->next->next->q, epsilon);
#endif	
#ifdef TOLERANCE_SMOOTHER	
	while(seg_check.iteration_step(mpkSimpleSegmentChecker::TOLERANCE_TEST))
#else
	while(seg_check.iteration_step())
#endif
	  ;
	coll = seg_check.collision();
      }
      else coll = true;

      if(!coll) curr->next = curr->next->next;
      else break;

     }
#endif
  }
#ifdef DEBUG_MODE
   for (node* curr = first; curr; curr=curr->next){
     if(!curr->next ) break;
	mpkSimpleSegmentChecker seg_check(&check1, &curr->q, &curr->next->q, epsilon);
	while(seg_check.iteration_step(mpkSimpleSegmentChecker::DEBUG_TEST));
   }
#endif
  
}

void
mpkPathSmoother::
smoothe_rough_clearance(int num_steps)
{
	mpkConfigChecker check1(test_pairs, test_size, robots);

	double t1 = 0;

	int num_shortcuts = 0;
	int orig_num_segs = num_segs();
	double orig_len = length();
	for ( int i=0; i<num_steps; i++ ) {

		if ( num_segs() <= 1 ) break;

		double t = mpk_drand();
		double t1 = 0.5*t;
		double t2 = 0.5*(t+1.0);

//		bool coll;
		double rc;
		node *n1;
		node *n2;
		do {

			n1 = lin_interpol(t1, p1);
			n2 = lin_interpol(t2, p2);
			
			double rc1 = check1.rough_clearance(&p1,true) ;
			double rc2 = check1.rough_clearance(&p2) ;

			// actually, p1 and p2 are on path and should be collision-free
			if ( (rc1>0) && (rc2 >0) ) {
				mpkSimpleSegmentChecker seg_check(&check1, &p1, &p2, epsilon);
				seg_check.set_rough_clearance(rc2);
				while(seg_check.iteration_step(mpkSimpleSegmentChecker::ROUGH_CLEARANCE_TEST))
					;
				rc = seg_check.rough_clearance();
				if( (rc < rc1) && (rc < rc2) ) rc=0; 
			}
			else rc=0;

			//modify:avoid infinite loop by Ohchi
			if ( !rc ) {
				double tmp1 = t1;
				double tmp2 = t2;
				t1 = 0.5*(t1+t);
				t2 = 0.5*(t+t2);
				if(t1 == tmp1 && t2 == tmp2){
					t1 = t2;
				}
			}
		} while (!rc && t2-t1 > min_shortcut_len);

		if ( rc && n1!=n2 ) {
			replace_section(n1,n2,p1,p2);
			num_shortcuts++;
		//cerr << "shortcut ";
		}
		else{
		//cerr << "         ";
		}

	}
	//Part added by Harada@aist
	for (node* curr = first; curr; curr=curr->next){
		for(int i=0; i<20; i++){
			if(!curr->next || !curr->next->next ) break;

			double rc;
			double rc1 = check1.rough_clearance(&curr->q,true);
			double rc2 = check1.rough_clearance(&curr->next->next->q);

			if ( (rc1 >0) && (rc2 >0)) {
				mpkSimpleSegmentChecker seg_check(&check1, &curr->q, &curr->next->next->q, epsilon);
				seg_check.set_rough_clearance(rc2);
				while(seg_check.iteration_step(mpkSimpleSegmentChecker::ROUGH_CLEARANCE_TEST))
				;
				rc = seg_check.rough_clearance();
				if( (rc < rc1) && (rc < rc2) ) rc=0; 
			}
			else rc=0;

			if(rc) curr->next = curr->next->next;
			else break;

		}
	}
	//#define DEBUG_MODE
	#ifdef DEBUG_MODE
	for (node* curr = first; curr; curr=curr->next){
		static int cnt=0;
		cout << cnt++ << endl;
		//if(cnt !=2) continue;
		if(!curr->next ) break;
		mpkSimpleSegmentChecker seg_check(&check1, &curr->q, &curr->next->q, epsilon);
		while(seg_check.iteration_step(mpkSimpleSegmentChecker::DEBUG_TEST));
	}
	#endif

}


#ifdef ENABLE_SBL_MULTITHREAD
bool mpkPathSmoother::smoothe_parallel(grasp::RobotBodyPtr body, int* col, node** n1, node** n2, mpkConfig* c1, mpkConfig* c2, double* dist, double t) {
	mpkRobots robot = mpkRobots(body);
	mpkConfigChecker check1(test_pairs, test_size, &robot);
#ifdef ADAPT_COLLCHECKER
  mpkConfigChecker check2(test_pairs, test_size, &robot);
#endif

	double t1 = 0.5*t;
  double t2 = 0.5*(t+1.0);
 
	bool feasible = false;
	int num_count = 0;

	double est_shortcut = 0;

  do {
		num_count++;
    *n1 = lin_interpol(t1, *c1);
    *n2 = lin_interpol(t2, *c2);
	  
		// calculate shortcut length
		if(*n1 == *n2) {
			feasible = false;
			break;
		}else{
			node* n = (*n1)->next;
			est_shortcut = (*c1).dist(n->q);
			while (true) {
				if (n == (*n2)) break;
				est_shortcut += n->q.dist(n->next->q);
				n = n->next;
			}
			est_shortcut += (*n2)->q.dist(*c2);
			est_shortcut -= (*c1).dist(*c2);
			if (est_shortcut < get_max_shortcut()) {
				feasible = false;
				break;
			}
		}

		bool finish_flag = false;
#ifdef ADAPT_COLLCHECKER
      if ( check1.clearance(&p1) > 0 && check2.clearance(&p2) > 0 ) {
	mpkAdaptSegmentChecker seg_check(&check1,&check2);
#else
    // actually, p1 and p2 are on path and should be collision-free
    if ( !check1.collision(c1) && !check1.collision(c2) ) {
			mpkSimpleSegmentChecker seg_check(&check1, c1, c2, epsilon);
#endif
#ifdef TOLERANCE_SMOOTHER	
			while(seg_check.iteration_step(mpkSimpleSegmentChecker::TOLERANCE_TEST))
#else
			while(seg_check.iteration_step())
#endif
			{
				if (est_shortcut < get_max_shortcut()) {
					feasible = false;
					finish_flag = true;
					break;
				}
			};
			if (finish_flag) break;
			feasible = !(seg_check.collision());
		}
    else feasible = false;
		//modify:avoid infinite loop by Ohchi
    if ( !feasible ) {
			double tmp1 = t1;
			double tmp2 = t2;
			t1 = 0.5*(t1+t);
			t2 = 0.5*(t+t2);
			if(t1 == tmp1 && t2 == tmp2){
				t1 = t2;
			}
    }
	//		      if ( coll ) {
	//t1 = 0.5*(t1+t);
	//t2 = 0.5*(t+t2);
 //     }
	
   } while (!feasible && t2-t1 > min_shortcut_len);

	if(*n1 == *n2 || !feasible) {
		*dist = 0;
	}else{
		*dist = est_shortcut;
		update_max_shortcut(est_shortcut);
	}
	*col = feasible ? 0 : 1;
	return true;
}

double mpkPathSmoother::get_max_shortcut () {
	boost::shared_lock<boost::shared_mutex> read_lock(_mutex);
	return max_shortcut;
}

void mpkPathSmoother::update_max_shortcut (double shortcut) {
	boost::upgrade_lock<boost::shared_mutex> up_lock(_mutex);
	if (max_shortcut < shortcut) {
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(up_lock);
		max_shortcut = shortcut;
	}
}

bool mpkPathSmoother::smoothe_postproc_parallel(grasp::RobotBodyPtr body, int* col, node* curr, node* target) {
	mpkRobots robot = mpkRobots(body);
	mpkConfigChecker check1(test_pairs, test_size, &robot);
#ifdef ADAPT_COLLCHECKER
  mpkConfigChecker check2(test_pairs, test_size, &robot);
#endif

	bool coll;
#ifdef ADAPT_COLLCHECKER
      if ( check1.clearance(&curr->q) > 0 && check2.clearance(&curr->next->next->q) > 0 ) {
	mpkAdaptSegmentChecker seg_check(&check1,&check2);
#else
      if ( !check1.collision(&curr->q) && !check1.collision(&target->q) ) {
	mpkSimpleSegmentChecker seg_check(&check1, &curr->q, &target->q, epsilon);
#endif	
#ifdef TOLERANCE_SMOOTHER	
	while(seg_check.iteration_step(mpkSimpleSegmentChecker::TOLERANCE_TEST))
#else
	while(seg_check.iteration_step())
#endif
	  ;
	
	coll = seg_check.collision();
			}
			else{
				coll = true;
			}

	*col = coll ? 1 : 0;

	return !coll;
}

#endif

// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include "mpkInterface.h"
#include "ParamDialog.h"

#ifdef ENABLE_SBL_MULTITHREAD
#if !(EIGEN_VERSION_AT_LEAST(3,2,0))
#error EIGEN 3.2 or above is required for multithread
#endif
#include "../Grasp/Parallelizer.h"
#endif

using namespace cnoid;
using namespace grasp;

mpkInterface::mpkInterface(cnoid::BodyItemPtr rob, list<cnoid::BodyItemPtr> env) {
	robItem = rob;
	rob_body = RobotBodyPtr(new RobotBody(rob, false));
	init();
}

mpkInterface::mpkInterface(RobotBodyPtr robbody, list<cnoid::BodyItemPtr> env) {
	rob_body = robbody;
	robItem = rob_body->getRootBodyItem();
	init();
}

void mpkInterface::init()
{
	contactPair = NULL;
/*	
	//arm_path = rob->body()->getJointPath(base, tip);
	nPair = 0;
	for(unsigned int i=0; i<rob->body()->numJoints(); i++)
		for(unsigned int j=i+2; j<rob->body()->numJoints(); j++)
			nPair++;
	//tip_ = tip;
	//base_ = base;
	robItem = rob;

	contactPair = new ColdetLinkPairPtr[rob->body()->numJoints() * env.size() + nPair];
	int cnt=0;
	for(unsigned int j=0;j<rob->body()->numJoints();j++){
		list<cnoid::BodyItemPtr>::iterator it = env.begin();
		for(unsigned int k=0;k<env.size();k++){
			contactPair[cnt++] = new ColdetLinkPair(rob->body()->joint(j), (*it)->body()->link(0));
			contact.push_back(false);
			it++;
		}
	}

	for(unsigned int i=0; i<rob->body()->numJoints(); i++){
		for(unsigned int j=i+2; j<rob->body()->numJoints(); j++){
			contactPair[cnt++] = new ColdetLinkPair(rob->body()->joint(i), rob->body()->joint(j));
			contact.push_back(false);
		}
	}
*/
	//robots = new mpkRobots(tip, base, rob, env);

	// delta = 0.01;
	// epsilon = 0.05; //default =0.012(larger faster) hrp2demo 0.3
	rho = 0.15;
	// max_iter = 30000;
	max_animsteps = 100;
	step_size = 0.1;

	database_idx = -1;
	plan_idx = -1;
	planner_time = -999;

	// smoothe_steps = 50;

	PRMParams params;
	params.loadParams();
	delta = params.param.delta;
	epsilon = params.param.eps;
	max_iter = params.param.iteration_num;
	smoothe_steps = params.param.smoothestep;
	
	return;
}

mpkInterface::~mpkInterface()
{
	return;
}

bool mpkInterface::call_smoother(vector<VectorXd>& plan)
{
	int qdSize;
	bool map;
	if(plan[0].size() > rob_body->numJoints()+5 ){
		qdSize = plan[0].size();
		map = true;
	}
	else{
		qdSize = rob_body->numJoints();
		map = false;
	}

	mpkConfig qd(qdSize);
	VectorXd qd_(qdSize);

	mpkRobots robots(rob_body);

	vector<mpkConfig> sub_path;
	p2q(plan, sub_path);

#if ADAPT_COLLCHECKER
	mpkPathSmoother smoother(sub_path, contactPair, contact.size(),&robots,0);
#else
	mpkPathSmoother smoother(sub_path, contactPair, contact.size(),&robots,0, epsilon);
#endif
#ifdef ROUGH_CLEARANCE
	smoother.smoothe_rough_clearance(smoothe_steps);
#else
	smoother.smoothe(smoothe_steps);
#endif
	
	smoother.get_path(sub_path);

#ifdef WIN32
	int p_size = plan[0].size();
	plan.clear();
	q2p(sub_path,plan,p_size);
#else
	plan.clear();
	q2p(sub_path,plan);
#endif

	return true;

}

bool mpkInterface::call_planner(vector<VectorXd>& plan_, const vector<int>& SampleDOF)
{

	vector<mpkConfig> plan;

	error_message_ = "";

	//double time_start = GetTime();

	mpkRobots robots(rob_body);
	int qdSize;
	bool map;
	if(plan_[0].size() > rob_body->numJoints()+5 ){
		qdSize = plan_[0].size();
		map = true;
	}
	else{
		qdSize = rob_body->numJoints();
		map = false;
	}

	mpkConfig qd(qdSize);
	VectorXd qd_(qdSize);

	p2q(plan_, plan,true);

	vector<mpkConfig> tmp_plan;
	unsigned int start=0;
	while ( start < plan.size()-1 ) {
		
		//cout <<"pathplan " << start << endl;

		unsigned int goal;
		//for ( goal = start+1; goal < plan.size(); goal++ )
		//	;
		//if ( goal >= plan.size() ) goal = plan.size()-1;
		goal =start+1;
#ifdef ENABLE_SBL_MULTITHREAD
		list<mpkConfig> clist;
		bool ok = call_planner_parallel(SampleDOF, plan[start], plan[goal], clist);
#else
#if ADAPT_COLLCHECKER
		sblPlanner planner(&robots, contactPair, contact.size(), SampleDOF, epsilon);
#else
		sblPlanner planner(&robots, contactPair, contact.size(), SampleDOF);
#endif
		list<mpkConfig> clist;
		bool ok = planner.Query(plan[start], plan[goal], clist, rho, max_iter);
		if (!ok) {
			error_message_ = planner.error_message();
		}
#endif
		if ( !ok ) {
			cerr << "Failure" << endl;
			planner_time = -1;
			return false;
		}
		list<mpkConfig>::iterator it;
		for ( it = clist.begin(); it != clist.end(); ++it )
			tmp_plan.push_back(*it);
		tmp_plan.pop_back();

		start = goal;
	}
	tmp_plan.push_back(plan[plan.size()-1]);

#ifdef WIN32
	int p_size = plan_[0].size();
	plan_.clear();
	q2p(tmp_plan,plan_,p_size);
#else
	plan_.clear();
	q2p(tmp_plan,plan_);
#endif
	return true;
	//double delta_t = GetTime() - time_start;
	//cout << delta_t << " s" << endl;
	//planner_time = delta_t;
}

void mpkInterface::p2q(vector<cnoid::VectorXd>& p, vector<mpkConfig>& q, bool isPhaseModify){
	int size;
	bool map;
	if(p[0].size() > rob_body->numJoints()+5 ){
		size = p[0].size();
		map = true;
	}
	else{
		size = rob_body->numJoints();
		map = false;
	}

	mpkConfig qd(size);

	for (unsigned int j=0; j<p.size(); j++ ){

		for(unsigned int i=0; i<rob_body->numJoints(); i++){
            qd[i] = (p[j](i) - rob_body->joint(i)->q_lower())/(rob_body->joint(i)->q_upper() - rob_body->joint(i)->q_lower());
		}
		if(map){
			int top = rob_body->numJoints();
			for(unsigned int i=0; i<3; i++){
				qd[top+i] = (p[j](top+i) - PlanBase::instance()->llimitMap[i])/(PlanBase::instance()->ulimitMap[i] - PlanBase::instance()->llimitMap[i]);
			}
			top = top+3;
			for(unsigned int i=0; i<3; i++){
				double tmp = p[j](top+i);
				if( (j>0) && isPhaseModify){
					if( ( tmp - p[0](top+i) )> M_PI  ) tmp -= 2*M_PI;
					if( ( tmp - p[0](top+i) )< -M_PI  ) tmp += 2*M_PI;
				}
				qd[top+i] = (tmp  - (-7.0))/( 7.0 - (-7.0) );
			}
		}
		q.push_back(qd);
	}

}

void mpkInterface::q2p(vector<mpkConfig>& q, vector<cnoid::VectorXd>& p){
	int size;
	bool map;
	if(p[0].size() > rob_body->numJoints()+5 ){
		size = p[0].size();
		map = true;
	}
	else{
		size = rob_body->numJoints();
		map = false;
	}

	VectorXd qd_(size);

	for (unsigned int i=0; i<q.size(); i++ ){

		for(unsigned int j=0; j<rob_body->numJoints(); j++){
            qd_[j] =  rob_body->joint(j)->q_lower() + q[i][j]*(rob_body->joint(j)->q_upper() - rob_body->joint(j)->q_lower());
		}
		if(map){
			int top = rob_body->numJoints();
			for(unsigned int j=0; j<3; j++){
				qd_[top+j] = PlanBase::instance()->llimitMap[j] + q[i][top+j]*(PlanBase::instance()->ulimitMap[j] - PlanBase::instance()->llimitMap[j]);
			}
			top = top+3;
			for(unsigned int j=0; j<3; j++){
				qd_[top+j] = -7.0+ q[i][top+j]*( 7.0 - (-7.0) );
			}
		}
		p.push_back(qd_);
	}
}

void mpkInterface::q2p(vector<mpkConfig>& q, vector<cnoid::VectorXd>& p,int p_size){
	int size;
	bool map;

	if(p_size > rob_body->numJoints()+5 ){
		size = p_size;
		map = true;
	}
	else{
		size = rob_body->numJoints();
		map = false;
	}

	VectorXd qd_(size);

	for (unsigned int i=0; i<q.size(); i++ ){

		for(unsigned int j=0; j<rob_body->numJoints(); j++){
            qd_[j] =  rob_body->joint(j)->q_lower() + q[i][j]*(rob_body->joint(j)->q_upper() - rob_body->joint(j)->q_lower());
		}
		if(map){
			int top = rob_body->numJoints();
			for(unsigned int j=0; j<3; j++){
				qd_[top+j] = PlanBase::instance()->llimitMap[j] + q[i][top+j]*(PlanBase::instance()->ulimitMap[j] - PlanBase::instance()->llimitMap[j]);
			}
			top = top+3;
			for(unsigned int j=0; j<3; j++){
				qd_[top+j] = -7.0+ q[i][top+j]*( 7.0 - (-7.0) );
			}
		}
		p.push_back(qd_);
	}
}

const string& mpkInterface::error_message() const {
	return error_message_;
}

#ifdef ENABLE_SBL_MULTITHREAD

boost::mutex mtx;

bool mpkInterface::call_planner_parallel(const vector<int>& SampleDOF, mpkConfig& start, mpkConfig& end, list<mpkConfig>& clist) {
    Eigen::initParallel();
	grasp::RobotParallelizer parallel;
	parallel.initialize(true);

	int num_threads = parallel.getNumThreads();

	vector<int> ok(num_threads, 0);
	vector<list<mpkConfig> > clists(num_threads);
	for (int i = 0; i < num_threads; i++) {
		parallel.addTask(boost::bind(&mpkInterface::call_planner_parallel_sub, this, _1, SampleDOF, start, end, &clists[i], &ok[i], mpk_lrand()));
	}

	parallel.doTasks();
	parallel.join();

	double min_dist = DBL_MAX;
	bool ret = false;
	for (int i = 0; i < num_threads; i++) {
		if (ok[i] == 0) continue;
		
		list<mpkConfig>::iterator it;
		double dist = 0;
		for (it = clists[i].begin(); it != clists[i].end(); ++it ) {
			list<mpkConfig>::iterator next = it;
			if(++next == clists[i].end())break;
			dist += (*it).dist(*next);
		}
		if (min_dist > dist) {
			min_dist = dist;
			clist = clists[i];
		}
		ret = true;
	}
	return ret;
}

bool mpkInterface::call_planner_parallel_sub(grasp::RobotBodyPtr body, const vector<int>& SampleDOF, mpkConfig& start, mpkConfig& end, list<mpkConfig>* clist, int* ok, int r) {
	mpkRobots robots(body);
	mpk_initrand(r);
#if ADAPT_COLLCHECKER
		sblPlanner planner(&robots, contactPair, contact.size(), SampleDOF, epsilon);
#else
		sblPlanner planner(&robots, contactPair, contact.size(), SampleDOF);
#endif
		bool b_ok = planner.Query(start, end, *clist, rho, max_iter);
		*ok = b_ok ? 1 : 0;
		if (!b_ok) {
			boost::mutex::scoped_lock lock(mtx);
			error_message_ = planner.error_message();
		}
		return true;
}

#endif


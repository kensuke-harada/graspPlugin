// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef PATH_PLAN_INTERFACE_H
#define PATH_PLAN_INTERFACE_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <vector>
#include <string>
#include <queue>

#include <boost/filesystem.hpp>
#include <cnoid/MessageView>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/JointPath>
#include <cnoid/BodyItem>
#include <cnoid/WorldItem>
#include <cnoid/EigenTypes>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#include <cnoid/ModelNodeSet>
#include <cnoid/ColdetLinkPair>
#ifdef CNOID_ENABLE_OSG
  #include <cnoid/OSGSceneBody>  
  #include <cnoid/OSGSceneBodyManager>
#endif
#else
#include "../Grasp/ColdetLinkPair.h"
//#include <cnoid/SceneBody>
//#include <cnoid/SceneBodyManager>
#endif

#include "mpk_rand.h"
#include "mpk_defs.h"
#include "mpkRobots.h"
#include "sblPlanner.h"
#include "vprmPlanner.h"
#include "mpkPathSmoother.h"
#include "tspTour.h"
#include "exportdef.h"

#include "../Grasp/RobotBody.h"

//#include "GetTime.h"

#if ADAPT_COLLCHECKER
#define PLANNER "A-SBL"
#else
#define PLANNER "SBL"
#endif

#include "exportdef.h"

namespace grasp{

struct config {
	config() {};
	config(const mpkConfig& q, bool is_key=true)
	{this->q = q; this->is_key = is_key;};
	bool is_key;
	mpkConfig q;
};

class EXCADE_API mpkInterface
{

public :
//	static mpkInterface* instance();
	mpkInterface(cnoid::BodyItemPtr rob, list<cnoid::BodyItemPtr> env);
	mpkInterface(RobotBodyPtr rob, list<cnoid::BodyItemPtr> env);  /// for hand separated model
	~mpkInterface();

	vector<bool>contact;
	//cnoid::JointPathPtr arm_path;
	int nPair;

	//std::ostream& os;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	cnoid::ColdetLinkPairPtr *contactPair;
#else
	grasp::ColdetLinkPairPtr *contactPair;
#endif

	//cnoid::Link *tip_;
	//cnoid::Link *base_;
	cnoid::BodyItemPtr robItem;
	//cnoid::BodyItemPtr envItem;
	RobotBodyPtr rob_body;

	bool call_smoother(vector<cnoid::VectorXd>& plan);
	bool call_planner(vector<cnoid::VectorXd>& plan, const std::vector<int>& trajectoryPlanDOF);
#ifdef ENABLE_SBL_MULTITHREAD
	bool call_planner_parallel(const std::vector<int>& trajectoryPlanDOF, mpkConfig& start, mpkConfig& end, std::list<mpkConfig>& clist);
	bool call_planner_parallel_sub(grasp::RobotBodyPtr body, const std::vector<int>& trajectoryPlanDOF, mpkConfig& start, mpkConfig& end, std::list<mpkConfig>* clist, int* ok, int r);
#endif

	void p2q(std::vector<cnoid::VectorXd>& p, std::vector<mpkConfig>& q, bool isPhaseModify=false);
	void q2p(std::vector<mpkConfig>& q, std::vector<cnoid::VectorXd>& p);
	void q2p(std::vector<mpkConfig>& q, std::vector<cnoid::VectorXd>& p,int p_size);

	const string& error_message() const;

	double delta;
	double epsilon;
	double rho;
	int max_iter;
	int max_animsteps;
	double step_size;

	string conf_fname;
	vector<config> database;
	int database_idx;
	vector<config> plan;
	int plan_idx;
	double planner_time;

	int smoothe_steps;

	mpkConfig conf_buf;

 protected:
	void init();

	string error_message_;
};


}

#endif

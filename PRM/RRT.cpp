// -*- mode: c++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
#include "Roadmap.h"
#include "RoadmapNode.h"
#include "ConfigurationSpace.h"
#include "RRT.h"

using namespace PathEngine;

static bool debug=false;
//static bool debug=true;

#ifdef ENABLE_RRT_MULTITHREAD
#if !(EIGEN_VERSION_AT_LEAST(3,2,0))
#error EIGEN 3.2 or above is required for multithread
#endif
#endif

RRT::RRT(PathPlanner* plan) : Algorithm(plan)
{
    // set default properties
    properties_["max-trials"] = "10000";
    properties_["eps"] = "0.1";

    Tstart_ = Ta_ = roadmap_;
    Tgoal_  = Tb_ = RoadmapPtr(new Roadmap(planner_));

    extendFromStart_ = true;
    extendFromGoal_ = true;
}

RRT::~RRT() 
{
}

int RRT::extend(RoadmapPtr tree, Configuration& qRand, bool reverse, CollisionChecker* col_checker) {
    if (debug) std::cout << "RRT::extend("<< qRand << ", " << reverse << ")" 
                         << std::endl;

    RoadmapNodePtr minNode;
    double min;
    {
#ifdef ENABLE_RRT_MULTITHREAD
        boost::mutex::scoped_lock lock(_mutex);
#endif
    tree->findNearestNode(qRand, minNode, min);
    }

    if (debug) std::cout << "nearest : pos = (" << minNode->position() 
                         << "), d = " << min << std::endl;

    if (minNode != NULL) {
        Mobility* mobility = planner_->getMobility();

        if (min > eps_){
            Configuration qRandOrg = qRand;
            qRand = mobility->interpolate(minNode->position(), qRand, eps_/min);
            if (debug) std::cout << "qRand = (" << qRand << ")" << std::endl;
            if (mobility->distance(minNode->position(), qRand) > min){
                std::cout << "distance didn't decrease" << std::endl;
                std::cout << "qRandOrg : (" << qRandOrg << "), d = " << min
                          << std::endl;
                std::cout << "qRand    : (" << qRand << "), d = " 
                          << mobility->distance(minNode->position(), qRand)
                          << std::endl;
                getchar();
            }
        }

        if (reverse){
            if (mobility->isReachable(qRand, minNode->position(), true, col_checker)){
                RoadmapNodePtr newNode = RoadmapNodePtr(new RoadmapNode(qRand));
                {
#ifdef ENABLE_RRT_MULTITHREAD
                    boost::mutex::scoped_lock lock(_mutex);
#endif
                tree->addNode(newNode);
                tree->addEdge(newNode, minNode);
#ifdef ENABLE_RRT_MULTITHREAD
                if (tree == Tstart_) {
                    *lastadded_start_nid_ = Tstart_->nNodes() - 1;
                } else {
                    *lastadded_goal_nid_ = Tgoal_->nNodes() - 1;
                }
#endif
                }

                if (min <= eps_) {
                    if (debug) std::cout << "reached(" << qRand << ")"<< std::endl;
                    return Reached;
                }
                else {
                    if (debug) std::cout << "advanced(" << qRand << ")" << std::endl;
                    return Advanced;
                }
            } else {
                if (debug) std::cout << "trapped" << std::endl;
                return Trapped;
            }
        }else{
            if (mobility->isReachable(minNode->position(), qRand, true, col_checker, false)){
                RoadmapNodePtr newNode = RoadmapNodePtr(new RoadmapNode(qRand));
                {
#ifdef ENABLE_RRT_MULTITHREAD
                    boost::mutex::scoped_lock lock(_mutex);

#endif
                tree->addNode(newNode);
                tree->addEdge(minNode, newNode);
#ifdef ENABLE_RRT_MULTITHREAD
                if (tree == Tstart_) {
                    *lastadded_start_nid_ = Tstart_->nNodes() - 1;
                } else {
                    *lastadded_goal_nid_ = Tgoal_->nNodes() - 1;
                }
#endif
                }

                if (min <= eps_) {
                    if (debug) std::cout << "reached(" << qRand << ")"<< std::endl;
                    return Reached;
                }
                else {
                    if (debug) std::cout << "advanced(" << qRand << ")" << std::endl;
                    return Advanced;
                }
            } else {
                if (debug) std::cout << "trapped" << std::endl;
                return Trapped;
            }
        }
    }

    return Trapped;
}

int RRT::connect(RoadmapPtr tree,const Configuration &qNew, bool reverse, CollisionChecker* col_checker) {
    if (debug) std::cout << "RRT::connect(" << qNew << ")" << std::endl;

    int ret = Reached;
    Configuration q = qNew;

    do {
#ifdef ENABLE_RRT_MULTITHREAD
        {
            boost::mutex::scoped_lock lock(_mutex);
            if (is_succeed) return Trapped;
        }
#endif
        ret = extend(tree, q, reverse, col_checker);
        q = qNew;
    } while (ret == Advanced);
    return ret;
}

void RRT::extractPath() {
    extractPath(path_);
}

void RRT::extractPath(std::vector<Configuration>& o_path) {
    //std::cout << "RRT::path" << std::endl;
#ifdef ENABLE_RRT_MULTITHREAD
    RoadmapNodePtr startMidNode = Tstart_->node(laststart_nid);
    RoadmapNodePtr goalMidNode  = Tgoal_->node(lastgoal_nid);
#else
    RoadmapNodePtr startMidNode = Tstart_->lastAddedNode();
    RoadmapNodePtr goalMidNode  = Tgoal_ ->lastAddedNode();
#endif
    
    o_path.clear();
    if (!startMidNode || !goalMidNode) return;
    RoadmapNodePtr node;

    if (extendFromStart_){
        node = startMidNode;
        do {
            o_path.insert(o_path.begin(), node->position());
            node = node->parent(0);
        } while (node != NULL);
    }

    if (extendFromGoal_){
        node = goalMidNode;
        do {
            o_path.push_back(node->position());
            node = node->child(0);
        } while (node != NULL);
    }
#if 0
    startMidNode->children_.push_back(goalMidNode);
    goalMidNode->parent_ = startMidNode;
#endif
}

bool RRT::extendOneStep()
{
    Configuration qNew = planner_->getConfigurationSpace()->random();
    if (extendFromStart_ && extendFromGoal_){
        
        if (extend(Ta_, qNew, Tb_ == Tstart_) != Trapped) {
            if (connect(Tb_, qNew, Ta_ == Tstart_) == Reached) {
                return true;
            }
        }
        swapTrees();
    }else if (extendFromStart_ && !extendFromGoal_){
        if (extend(Tstart_, qNew) != Trapped) {
            Configuration p = goal_;
            int ret;
            do {
                ret = extend(Tstart_, p);
                p = goal_;
            }while (ret == Advanced);
            if (ret == Reached) return true;
        }
    }else if (!extendFromStart_ && extendFromGoal_){
        std::cout << "this case is not implemented" << std::endl;
        return false;
    }
    return false;
}

bool RRT::calcPath() 
{
    if (verbose_) std::cout << "RRT::calcPath" << std::endl;

    // 回数
    times_ = atoi(properties_["max-trials"].c_str());

    // eps
    eps_ = atof(properties_["eps"].c_str());

    if (verbose_){
        std::cout << "times:" << times_ << std::endl;
        std::cout << "eps:" << eps_ << std::endl;
    }

    RoadmapNodePtr startNode = RoadmapNodePtr(new RoadmapNode(start_));
    RoadmapNodePtr goalNode  = RoadmapNodePtr(new RoadmapNode(goal_));

    Tstart_->addNode(startNode);
    Tgoal_ ->addNode(goalNode);

    bool isSucceed = false;

    isSucceed = calcPathProc();
    
    Tgoal_->integrate(Tstart_);

    if (verbose_) {
        std::cout << std::endl << "fin.(calcPath), retval = " << isSucceed << std::endl;
    }
    return isSucceed;
}

bool RRT::calcPathProc() {
    bool isSucceed = false;
#ifdef ENABLE_RRT_MULTITHREAD
    Eigen::initParallel();
    extendFromStart_ = true;
    extendFromGoal_ = true;
    grasp::RobotParallelizer parallel;
    parallel.initialize(true);
    int num_threads = parallel.getNumThreads();
    times_ = times_ / num_threads + 1;
    is_succeed = false;
    for (int i = 0; i < num_threads; i++) {
        bool from_start = (i%2==1);
        parallel.addTask(boost::bind(&RRT::extendParallel, this, _1, from_start));
    }

    parallel.doTasks();
    parallel.join();
    isSucceed = is_succeed;
#else
    for (int i=0; i<times_; i++) {
        //if (!isRunning_) break;
        if (verbose_){
            printf("%5d/%5dtrials : %5d/%5dnodes\r", i+1, times_, Tstart_->nNodes(),Tgoal_->nNodes());
            fflush(stdout);
        }
        if (isSucceed = extendOneStep()) break;
    }
#endif
    extractPath();
    return isSucceed;
}

void RRT::swapTrees()
{
    RoadmapPtr tmp = Ta_;
    Ta_ = Tb_;
    Tb_ = tmp;
}

void RRT::setForwardTree(RoadmapPtr tree) { 
    Tstart_ = Ta_ = tree;
}

void RRT::setBackwardTree(RoadmapPtr tree) { 
    Tgoal_ = Tb_ = tree;
}
#ifdef ENABLE_RRT_MULTITHREAD
bool RRT::extendParallel(grasp::RobotBodyPtr body, bool reverse) {
    CollisionChecker col_checker(body);
    lastadded_start_nid_.reset(new int(0));
    lastadded_goal_nid_.reset(new int(0));

    for (int i=0; i<times_; i++) {
        {
            boost::mutex::scoped_lock lock(_mutex);
            if (is_succeed) return true;
        }
        Configuration qNew = planner_->getConfigurationSpace()->random();
         if (extend((reverse ? Tgoal_ : Tstart_), qNew, reverse, &col_checker) != Trapped) {
            if (connect((reverse ? Tstart_ : Tgoal_), qNew, !reverse, &col_checker) == Reached) {
                {
                    boost::mutex::scoped_lock lock(_mutex);
                    laststart_nid = *lastadded_start_nid_;
                    lastgoal_nid = *lastadded_goal_nid_;
                    is_succeed = true;
                }           
                return false;
            }
         }
    }

    return true;
}
#endif


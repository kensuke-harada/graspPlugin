#include "RRTStar.h"

using namespace PathEngine;

RRTStar::RRTStar(PathPlanner* plan) : RRT(plan) {
	properties_["radius"] = "0.2";
	properties_["number-path"] = "5";
	properties_["use-knn"] = "0";
	is_iterative_phase_ = false;
	best_cost_ = std::numeric_limits<double>::max();
	num_feasible_path_ = 0;
}

RRTStar::~RRTStar() {
}

int RRTStar::extend(RoadmapPtr tree, Configuration& qRand, bool reverse, CollisionChecker* col_checker) {
	double radius = radius_;
	// radius = std::max(pow((log10(tree->nNodes())/double(tree->nNodes())), 1/(double)(planner_->nDOF())) * radius, eps);
	RoadmapNodePtr minNode;
	double min;

	// find nearest node
	{
#ifdef ENABLE_RRT_MULTITHREAD
		boost::mutex::scoped_lock lock(_mutex);
#endif
	tree->findNearestNode(qRand, minNode, min);
	}
	if (minNode == NULL) {
		return Trapped;
	}

	// steer
	Mobility* mobility = planner_->getMobility();
	if (min > eps_) {
		Configuration qRandOrg = qRand;
		qRand = mobility->interpolate(minNode->position(), qRand, eps_/min);
	}

	// collision check
	bool is_collide;
	if (col_checker == NULL) {
		is_collide = planner_->checkCollision(qRand);
	} else {
		is_collide = col_checker->checkCollision(qRand);
	}
	if (is_collide) {
		return Trapped;
	}

	// find near nodes
	std::vector<RoadmapNodePtr> near_nodes;
	{
#ifdef ENABLE_RRT_MULTITHREAD
		boost::mutex::scoped_lock lock(_mutex);
#endif
		if (use_knn_) {
			double e = exp(1.0);
			int k = ceil((e + (e/(double)(planner_->nDOF()))) * log(tree->nNodes()+1));
			tree->findKNearNodes(qRand, k, near_nodes);
		} else {
			tree->findNearNodes(qRand, radius, near_nodes);
		}
	}

	if (near_nodes.empty()) {
		std::cerr << "error: there are no near nodes" << radius << ":" << min << std::endl;
		return Trapped;
	}

	// prune
	std::vector<RoadmapNodePtr> cand_nodes;
	if (is_iterative_phase_) {
		RoadmapNodePtr goal = (tree == Tstart_) ?  Tgoal_->node(0) : Tstart_->node(0);
		for (size_t i = 0; i < near_nodes.size(); i++) {
			if (near_nodes[i]->cost() + mobility->distance(near_nodes[i]->position(), qRand) +
					mobility->distance(qRand, goal->position()) < best_cost_) {
				cand_nodes.push_back(near_nodes[i]);
			}
		}
	} else {
		cand_nodes.insert(cand_nodes.end(), near_nodes.begin(), near_nodes.end());
	}
	if (cand_nodes.empty()) {
		return Trapped;
	}

	// sort by cost
	std::vector<Candidate> candidates;
	for (size_t i = 0; i < cand_nodes.size(); i++) {
		candidates.push_back(Candidate());
		candidates.back().node = cand_nodes[i];
		candidates.back().cost = cand_nodes[i]->cost() + mobility->distance(cand_nodes[i]->position(), qRand);
	}
	std::sort(candidates.begin(), candidates.end());

	// obtain min cost node
	RoadmapNodePtr minCostNode;
	double min_cost = -1;
	int min_nid;
	for (min_nid = 0; min_nid < candidates.size(); min_nid++) {
		if (mobility->isReachable(candidates[min_nid].node->position(), qRand, true, col_checker, false, false)) {
			minCostNode = candidates[min_nid].node;
			min_cost = candidates[min_nid].cost;
			break;
		}
	}
	if (min_cost < 0) {
		return Trapped;
	}

	// get nodes to be reconnected
	std::vector<RoadmapNodePtr> reconnect_nodes;
	for (size_t i = min_nid + 1; i < candidates.size(); i++) {
		RoadmapNodePtr target_node = candidates[i].node;
		if (target_node->cost() > min_cost + mobility->distance(target_node->position(), qRand)) {
			if (mobility->isReachable(target_node->position(), qRand, true, col_checker, false, false)) {
				reconnect_nodes.push_back(target_node);
			}
		}
	}

	// add new node
	{
#ifdef ENABLE_RRT_MULTITHREAD
		boost::mutex::scoped_lock lock(_mutex);
#endif
		if (!minCostNode->isValid()) return Trapped;
	RoadmapNodePtr newNode = RoadmapNodePtr(new RoadmapNode(qRand));
	newNode->setCost(min_cost);
	tree->addNode(newNode);
	if (reverse) {
		tree->addEdge(newNode, minCostNode);
	} else {
		tree->addEdge(minCostNode, newNode);
	}
#ifdef ENABLE_RRT_MULTITHREAD
	if (tree == Tstart_) {
		*lastadded_start_nid_ = Tstart_->nNodes() - 1;
	} else {
		*lastadded_goal_nid_ = Tgoal_->nNodes() - 1;
	}
#endif

	// rewrite tree
	for (size_t i = 0; i < reconnect_nodes.size(); i++) {
		if (reverse) {
			RoadmapNodePtr child_node = reconnect_nodes[i]->child(0);
			tree->addEdge(reconnect_nodes[i], newNode);
			tree->removeEdge(reconnect_nodes[i], child_node);
		} else {
			RoadmapNodePtr parent_node = reconnect_nodes[i]->parent(0);
			tree->addEdge(newNode, reconnect_nodes[i]);
			tree->removeEdge(parent_node, reconnect_nodes[i]);
		}
	}
	}

	if (min <= eps_) {
		return Reached;
	} else {
		return Advanced;
	}
	return Trapped;
}

bool RRTStar::calcPathProc() {
	radius_ = atof(properties_["radius"].c_str());
	num_path_ = atoi(properties_["number-path"].c_str());
	use_knn_ = (atoi(properties_["use-knn"].c_str()) != 0);

	bool isSucceed = false;
	num_feasible_path_ = 0;
	is_iterative_phase_ = false;
#ifdef ENABLE_RRT_MULTITHREAD
	Eigen::initParallel();
	extendFromStart_ = true;
	extendFromGoal_ = true;
	grasp::RobotParallelizer parallel;
	parallel.initialize(true);
	int num_threads = parallel.getNumThreads();
	times_ = times_ / num_threads + 1;
	is_succeed = false;
	has_feasible_path = false;
	for (int i = 0; i < num_threads; i++) {
		bool from_start = (i%2 == 1);
		parallel.addTask(boost::bind(&RRTStar::extendParallel, this, _1, from_start));
	}

	parallel.doTasks();
	parallel.join();
	isSucceed = has_feasible_path;
#else
	for (int i = 0; i < times_; i++) {
		//if (!isRunning_) break;
		if (verbose_) {
			printf("%5d/%5dtrials : %5d/%5dnodes\r", i+1, times_, Tstart_->nNodes(), Tgoal_->nNodes());
			fflush(stdout);
		}
		if (extendOneStep()) {
			isSucceed = true;
			extractPath();
			is_iterative_phase_ = true;
			updateBestCost();
			prune(Tstart_);
			prune(Tgoal_);
			num_feasible_path_++;
		}
		if (num_feasible_path_ >= num_path_) break;
	}
#endif
	if (isSucceed) {
		path_.clear();
		path_.insert(path_.end(), best_path_.begin(), best_path_.end());
	}
	return isSucceed;
}

void RRTStar::updateBestCost() {
	double cost = 0;
	Mobility* mobility = planner_->getMobility();
	for (size_t i = 1; i < path_.size(); i++) {
		cost += mobility->distance(path_[i-1], path_[i]);
	}

	if (cost < best_cost_) {
		best_cost_ = cost;
		best_path_.clear();
		best_path_.insert(best_path_.end(), path_.begin(), path_.end());
	}
}

void RRTStar::prune(RoadmapPtr tree) {
	Mobility* mobility = planner_->getMobility();
	RoadmapNodePtr dest_node = (tree == Tstart_) ? Tgoal_->node(0) : Tstart_->node(0);
	for (size_t i = 0; i < tree->nNodes(); i++) {
		RoadmapNodePtr target_node = tree->node(i);
		if (target_node->cost() + mobility->distance(target_node->position(), dest_node->position()) > best_cost_) {
			target_node->setInvalid();
		}
	}
}

#ifdef ENABLE_RRT_MULTITHREAD
bool RRTStar::extendParallel(grasp::RobotBodyPtr body, bool reverse) {
	CollisionChecker col_checker(body);
	lastadded_start_nid_.reset(new int(0));
	lastadded_goal_nid_.reset(new int(0));

	for (int i = 0; i < times_; i++) {
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

					extractPath();
					is_iterative_phase_ = true;
					updateBestCost();
					prune(Tstart_);
					prune(Tgoal_);
					num_feasible_path_++;
					has_feasible_path= true;
					if (num_feasible_path_ >= num_path_) {
						is_succeed = true;
						return false;
					}
				}
			}
		}
	}

	return true;
}
#endif


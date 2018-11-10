#include "PathSearcher.h"

#include <math.h>

double _drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
}

void _initrand(){
#ifdef WIN32
	  srand((unsigned)time(NULL));
#else
	  srand48(time(0));
#endif
}

using namespace std;
using namespace cnoid;
using namespace grasp;

/**
* Get shortest route through each point in @c regions  by using simulated annealing.
* @param[in] regions   vector of GraspableRegion
* @param[in] start_p   start point
* @param[out] path     route(region id)
* @param[out] coord_id route(grid coords)
*/
void PathSearcher::getPath(const vector<GraspableRegion>& regions, const Vector3& start_p, vector<int>& path, vector<Eigen::Vector2i>& coords_id) {
	_initrand();
	std::vector<int> tmp_path;
	std::vector<int> point_id, best_point;
	for (size_t i = 0; i < regions.size(); i++) {
		tmp_path.push_back(i);
		point_id.push_back(0);
	}

	const double init_temp = 1.0;
	const double finish_temp = 0.00001;
	const double alpha = 0.8;
	
	double cur_dist = calcDist(regions, start_p, tmp_path, point_id);
	double min_dist = cur_dist;
	path = tmp_path;
	best_point = point_id;

	for (double t = init_temp; t > finish_temp; t *= alpha) {
		int count = 0;
		tmp_path = path;
		point_id = best_point;
		for (int i = 0; i < 1000; i++) {
			vector<int> new_path = tmp_path;
			vector<int> new_point = point_id;
			move(regions, new_path, new_point);
			double new_dist = calcDist(regions, start_p, new_path, new_point);
			if (new_dist < cur_dist || _drand() <= pow(M_E, -(new_dist - cur_dist) / t)) {
				cur_dist = new_dist;
				tmp_path = new_path;
				point_id = new_point;
				if (new_dist < min_dist) {
					count++;
					min_dist = new_dist;
					path = new_path;
					best_point = point_id;
				}
			}
		}
		if (count == 0) break;
	}

	for (size_t i = 0; i < path.size(); i++) {
		coords_id.push_back(regions[path[i]].getCoordID(best_point[i]));
	}
}

/**
* Calculate path distance.
* @param[in] regions  vector of GraspableRegion
* @param[in] start_p  start point
* @param[in] path     route(region id)
* @param[in] point_id point id
* @return distance
*/
double PathSearcher::calcDist(const vector<GraspableRegion>& regions, const Vector3& start_p, const vector<int>& path, const vector<int>& point_id) const {
	Vector3 cur_point = start_p;
	double dist = 0;
	for (size_t i = 0; i < path.size(); i++) {
		Vector3 next_point = regions[path[i]].getCoord(point_id[i]);
		dist += (cur_point - next_point).norm();
		cur_point = next_point;
	}
	dist += (cur_point - start_p).norm();
	return dist;
}

/**
* Move neighbouring state.
* @param[in]     regions  vector of GraspableRegion
* @param[in,out] path     list of region id
* @param[in,out] point_id list of point id
*/
void PathSearcher::move(const vector<GraspableRegion>& regions, vector<int>& path, vector<int>& point_id) const {
	if (_drand() < 0.25 && regions.size() > 1) {
		// swap elements in path
		int r_id1 = static_cast<int>(floor(_drand() * regions.size()));
		if (r_id1 == regions.size()) r_id1 = regions.size()-1;
		int r_id2;
		do {
			r_id2 = static_cast<int>(floor(_drand() * regions.size()));
			if (r_id2 == regions.size()) r_id2 = regions.size()-1;
		} while (r_id1 == r_id2);
		int tmp = path[r_id1];
		path[r_id1] = path[r_id2];
		path[r_id2] = tmp;
		tmp = point_id[r_id1];
		point_id[r_id1] = point_id[r_id2];
		point_id[r_id2] = tmp;
	} else {
		// change point id in point_id of r_id
		int r_id = static_cast<int>(floor(_drand() * regions.size()));
		if (r_id == regions.size()) r_id = regions.size()-1;
		int p_size =  regions[path[r_id]].size();

		int p_id = static_cast<int>(floor(_drand() * p_size));
		if (p_id == p_size) p_id = p_size-1;
		point_id[r_id] = p_id;
	}
}

/**
* Decide sets of objects that will be picked at same place by solving set covering problem.
* param[in] regions vector of GraspableRegion
* param[out] sets   sets of objects id
*/
void SetCoveringSolver::solve(const vector<GraspableRegion>& regions, vector<vector<int> >& sets) {
	makeSets(regions);
	doSolve(sets);
}

/**
* Make candidate sets. 
* param[in] regions vector of GraspableRegion
*/
void SetCoveringSolver::makeSets(const vector<GraspableRegion>& regions) {
	vector<boost::dynamic_bitset<> > pre_sets;
	vector<GraspableRegion> pre_regions;
	int r_size = regions.size();

	// make sets which have one member
	for (int i = 0; i < r_size; i++) {
		boost::dynamic_bitset<> set(r_size);
		set.set(i);
		pre_sets.push_back(set);
		pre_regions.push_back(regions[i]);
		set_list.push_back(set);
	}

	// make sets which have more than 2 members
	while (!pre_sets.empty()) {
		vector<boost::dynamic_bitset<> > target_sets = pre_sets;
		vector<GraspableRegion> target_regions = pre_regions;
		pre_sets.clear();
		pre_regions.clear();

		for (size_t i = 0; i < target_sets.size(); i++) {
			for (size_t j = i + 1; j < target_sets.size(); j++) {
				if((target_sets[i] & target_sets[j]).count() != target_sets[i].count() - 1) {
					continue;
				}

				GraspableRegion prod_region = target_regions[i] & target_regions[j];
				if (!prod_region.empty()) {
					boost::dynamic_bitset<> sum_set = target_sets[i] | target_sets[j];
					pre_sets.push_back(sum_set);
					pre_regions.push_back(prod_region);
					set_list.push_back(sum_set);
				}
			}
		}
	}
}

/**
* solve set covering problem.
* param[out] sets   sets of objects id
*/
void SetCoveringSolver::doSolve(vector<vector<int> >& sets) {
	pattern.resize(set_list.size());

	// try in ascending order of cost (count of move)
	do {
		updatePattern();
	} while(!isFeasiblePattern());

	// convert
	for (boost::dynamic_bitset<>::size_type i = pattern.find_first();
		i != pattern.npos; i = pattern.find_next(i)) {
			vector<int> s;
			for (boost::dynamic_bitset<>::size_type j = set_list[i].find_first();
				j != set_list[i].npos; j = set_list[i].find_next(j)) {
					s.push_back(j);
				}
			sets.push_back(s);
	}
}

/**
* check whether the solution(pattern) is feasible. 
*/
bool SetCoveringSolver::isFeasiblePattern() {
	boost::dynamic_bitset<> sum(set_list[0].size());
	for (boost::dynamic_bitset<>::size_type i = pattern.find_first();
		i != pattern.npos; i = pattern.find_next(i)) {
			sum |= set_list[i];
	}
	return (sum.count() == sum.size());
}

/**
* udpate the solution(pattern). 
*/
void SetCoveringSolver::updatePattern() {
	int t_count = t_pos.size();

	if (t_count == 0) {
		pattern.set(0);
		t_pos.push_back(0);
		return;
	}

	if (t_pos[0] == pattern.size() - t_count) {
		for (int i = 0; i < t_count; i++) {
			pattern.reset(t_pos[i]);
			t_pos[i] = i;
			pattern.set(i);
		}
		t_pos.push_back(t_count);
		pattern.set(t_count);
		return;
	}

	int target = t_count - 1;
	while ((t_pos[target] - target == pattern.size() - t_count) && (target > 0)) {
		target--;
	}

	pattern.reset(t_pos[target]);
	t_pos[target]++;
	pattern.set(t_pos[target]);
	if(target != t_count - 1) {
		int target_l = target + 1;
		for (int i = target + 1; i < t_count; i++) {
			pattern.reset(t_pos[i]);
			t_pos[i] = t_pos[i-1]+1;
			pattern.set(t_pos[i]);
		}
	}
}

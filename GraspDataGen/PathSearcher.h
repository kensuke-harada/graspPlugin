#ifndef _GRASPDATAGEN_PATHSEARCHER_H_
#define _GRASPDATAGEN_PATHSEARCHER_H_

#include <vector>

#include <boost/dynamic_bitset.hpp>

#include <cnoid/EigenTypes>

#include "GraspableRegion.h"

namespace grasp {
	class PathSearcher {
	public:
		void getPath(const std::vector<GraspableRegion>& regions, const cnoid::Vector3& start_p, std::vector<int>& path, std::vector<Eigen::Vector2i>& coords_id);
	
	private:
		void move(const std::vector<GraspableRegion>& regions, std::vector<int>& path, std::vector<int>& point_id) const;
		double calcDist(const std::vector<GraspableRegion>& regions, const cnoid::Vector3& start_p, const std::vector<int>& path, const std::vector<int>& point_id) const;
	};

	class SetCoveringSolver {
	public:
		void solve(const std::vector<GraspableRegion>& regions, std::vector<std::vector<int> >& sets);
	private:
		std::vector<boost::dynamic_bitset<> > set_list;
		boost::dynamic_bitset<> pattern;
		std::vector<int> t_pos;

		void makeSets(const std::vector<GraspableRegion>& regions);
		void doSolve(std::vector<std::vector<int> >& sets);
		bool isFeasiblePattern();
		void updatePattern();
	};

}

#endif

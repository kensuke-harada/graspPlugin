#ifndef _PRM_RRTSTAR_H_
#define _PRM_RRTSTAR_H_

#include <limits>
#include <vector>

#include "RRT.h"

namespace PathEngine {

	class RRTStar
		: public RRT {
	public:
		explicit RRTStar(PathPlanner* planner);
		~RRTStar();

		int extend(RoadmapPtr tree, Configuration& qRnad, bool reverse = false, CollisionChecker* col_checker = NULL);

		bool calcPathProc();

	protected:
		bool is_iterative_phase_;
		double best_cost_;
		int num_feasible_path_;
		bool has_feasible_path;
		std::vector<Configuration> best_path_;
		double radius_;
		int num_path_;
		bool use_knn_;

		class Candidate {
		public:
			RoadmapNodePtr node;
			double cost;

			bool operator <(const Candidate& r) const {
				return cost < r.cost;
			}
		};

		void updateBestCost();
		void prune(RoadmapPtr tree);
#ifdef ENABLE_RRT_MULTITHREAD
		bool extendParallel(grasp::RobotBodyPtr body, bool reverse);
#endif
	};
};

#endif  // _PRM_RRSTART_H_


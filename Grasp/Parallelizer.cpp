#include "Parallelizer.h"

#include "PlanBase.h"
#include "RobotBodyColChecker.h"
#include "ColdetModelGetter.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

Parallelizer::Parallelizer() {
#if EIGEN_VERSION_AT_LEAST(3,2,0)
	Eigen::initParallel();
#endif
}

Parallelizer::~Parallelizer() {

}

void Parallelizer::doTasks() {
	thr_grp = new boost::thread_group();

	for (int i = 0; i < num_threads; i++) {
		thr_grp->create_thread(boost::bind(&Parallelizer::run, this));
	}
}

void Parallelizer::run() {
	boost::function0<void> f;
	while(take(f)){
		f();
	}
}

boost::mutex RobotParallelizer::g_mutex;

RobotParallelizer::RobotParallelizer() {

}

RobotParallelizer::~RobotParallelizer() {
	clearColChecker();
	ColdetModelGetter::update();
}

void RobotParallelizer::initialize(bool use_collision_check, int size) {
	setNumThreads(size);
	clearQueue();
	is_finish = false;
	bodies.clear();
	clearColChecker();
	for (int i = 0; i < num_threads; i++) {
		boost::mutex::scoped_lock lock(g_mutex);
		PlanBase::instance()->col_checkers.push_back(new RobotBodyColChecker());
		bodies.push_back(PlanBase::instance()->col_checkers.back()->getRobotBody());
	}
}

void RobotParallelizer::doTasks() {
	thr_grp = new boost::thread_group();

	for (int i = 0; i < num_threads; i++) {
		thr_grp->create_thread(boost::bind(&RobotParallelizer::run, this, bodies[i]));
	}
}

void RobotParallelizer::run(RobotBodyPtr body) {
	boost::function<bool (RobotBodyPtr)> f;
	while(take(f)) {
		if(!f(body)) is_finish = true;
	}
}

void RobotParallelizer::clearColChecker() {
	for (size_t i = 0; i < PlanBase::instance()->col_checkers.size(); i++) {
		if (PlanBase::instance()->col_checkers[i] != NULL) {
				delete PlanBase::instance()->col_checkers[i];
				PlanBase::instance()->col_checkers[i] = NULL;
		}
	}
	PlanBase::instance()->col_checkers.clear();
}

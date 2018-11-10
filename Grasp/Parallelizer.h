#ifndef _GRASP_PARALLELIZER_H_
#define _GRASP_PARALLELIZER_H_

#include <vector>
#include <queue>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <cnoid/Body>

#include "Arm.h"
#include "GraspPluginManager.h"
#include "RobotBody.h"

#include "exportdef.h"

namespace grasp {
	template <class F = boost::function0<void> >
	class AbstractParallelizer {
	public:
		AbstractParallelizer() : thr_grp(NULL), num_threads(0), is_finish(false){
			
		}

		virtual ~AbstractParallelizer() {
			if (thr_grp != NULL) {
				delete thr_grp;
			}
		}

		virtual void initialize(int size = 0) {
			setNumThreads(size);
			clearQueue();
			is_finish = false;
		}

		virtual void addTask(const F& f) {
			f_que.push(f);
		}

		virtual void doTasks() = 0;

		void join() {
			if (thr_grp == NULL) return;
			thr_grp->join_all();
			delete thr_grp;
			thr_grp = NULL;
			clearQueue();
			is_finish = false;
		}

		int getNumThreads() const {return num_threads;}
	protected:
		boost::thread_group* thr_grp;
		boost::mutex _mutex;
		std::queue<F> f_que;

		int num_threads;
		bool is_finish;

		virtual bool take(F& f) {
			boost::mutex::scoped_lock lock(_mutex);

			if(is_finish) return false;
			if(f_que.empty()) return false;
			f = f_que.front();
			f_que.pop();
			return true;
		}

		void setNumThreads(int size) {
			if (size == 0) {
				num_threads = boost::thread::hardware_concurrency();
			} else {
				num_threads = size;
			}
		}

		void clearQueue() {
			while (!f_que.empty()) f_que.pop();
		}

	};

	class EXCADE_API Parallelizer : public AbstractParallelizer<boost::function0<void> > {
	public:
		Parallelizer();
		virtual ~Parallelizer();
	
		void doTasks();
	private:
		void run();
	};

	class EXCADE_API RobotParallelizer : public AbstractParallelizer<boost::function<bool (RobotBodyPtr)> > {
	public:
		RobotParallelizer();
		virtual ~RobotParallelizer();

		void initialize(bool use_collision_check = true, int size = 0);
		void doTasks();
		static void clearColChecker();

		static boost::mutex g_mutex;
	private:
		std::vector<RobotBodyPtr> bodies;
		void run(RobotBodyPtr body);
	};
}

#endif /* _GRASP_PARALLELIZER_H_ */

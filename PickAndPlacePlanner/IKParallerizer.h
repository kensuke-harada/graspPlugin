#ifndef _PICKANDPLACEPLANNER_IKPARALLERIZER_H_
#define _PICKANDPLACEPLANNER_IKPARALLERIZER_H_

#include <vector>
#include <queue>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <cnoid/Body>

#include "../Grasp/Arm.h"
#include "../Grasp/GraspPluginManager.h"
#include "../Grasp/PlanBase.h"

namespace grasp{

	class IKParallerizer {
	public:
		IKParallerizer() {
			max_numthreads = 0;
			thr_grp = NULL;
		}
		virtual ~IKParallerizer() {
			if (thr_grp != NULL) {
				delete thr_grp;
			}
			clearArms();
		}

		struct Task {
			cnoid::Vector3 p;
			cnoid::Matrix3 R;
			double phi;
			bool is_fix_waist;
			bool has_sol;
			cnoid::VectorXd sol_q;
		};

		void initialize(const cnoid::BodyPtr body, const ArmPtr arm, const GrasplotEntry getGrasplotFunc, int size = 0) {
			clearArms();
			bodies.clear();
			if (size == 0) {
				max_numthreads = IKParallerizer::getNumThreads();
			} else {
				max_numthreads = size;
			}
			for (int i = 0; i < max_numthreads; i++) {
				bodies.push_back(body->clone());
				ArmPtr tmp_arm;
				if (getGrasplotFunc != NULL) {
					tmp_arm = (Arm *)(*getGrasplotFunc)(bodies.back(), bodies.back()->link(arm->arm_path->baseLink()->name()), bodies.back()->link(arm->palm->name()));
				} else {
					tmp_arm =  new Arm(bodies.back(), bodies.back()->link(arm->arm_path->baseLink()->name()), bodies.back()->link(arm->palm->name()));
				}
				tmp_arm->multithread_mode = true;
				tmp_arm->armStandardPose = arm->armStandardPose;
				tmp_arm->interLinks = PlanBase::instance()->interLinkList;
				tmp_arm->setPairs(tmp_arm->interLinks);
				arms.push_back(tmp_arm);
			}
		}

		void addTasks(const std::vector<cnoid::Vector3>& p, const std::vector<cnoid::Matrix3>& R) {
			int id = que.size();
			for (size_t i = 0; i < p.size(); i++) {
				Task task;
				task.p = p[i];
				task.R = R[i];
				task.is_fix_waist = false;
				task.phi = 0;
				tasks.push_back(task);
				que.push(id++);
			}
		}

		void addTask(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi) {
			int id = que.size();
			Task task;
			task.p = p;
			task.R = R;
			task.is_fix_waist = true;
			task.phi = phi;
			tasks.push_back(task);
			que.push(id);
		}

		void getResults(std::vector<bool>& has_sol, std::vector<cnoid::VectorXd>& sol_q) {
			for (size_t i = 0; i < tasks.size(); i++) {
				has_sol.push_back(tasks[i].has_sol);
				sol_q.push_back(tasks[i].sol_q);
			}
		}

		void clearTasks() {
			tasks.clear();
		}

		void solveIKs() {
			solveIKs(max_numthreads);
		}

		void solveIKs(int size) {
			if (size <= 0) {
				return;
			}

			int num_threads = (size > max_numthreads) ?  max_numthreads : size;

			thr_grp = new boost::thread_group();
			for (int i = 0; i < num_threads; i++) {
				thr_grp->create_thread(boost::bind(&IKParallerizer::run, this, arms[i]));
			}
		}

		void join() {
			if (thr_grp == NULL) {
				return;
			}
			thr_grp->join_all();
			delete thr_grp;
			thr_grp = NULL;
		}

		static int getNumThreads() {
			return boost::thread::hardware_concurrency();
		}

	private:
		boost::thread_group* thr_grp;
		boost::mutex _mutex;

		std::queue<int> que;

		std::vector<cnoid::BodyPtr> bodies;
		std::vector<ArmPtr> arms;
		std::vector<Task> tasks;
		int max_numthreads;

		bool take(int& id) {
			boost::mutex::scoped_lock lock(_mutex);

			if (que.empty()) return false;

			id = que.front();
			que.pop();
			return true;
		}

		void run(ArmPtr arm) {
			int id; 
			while (take(id)) {
				solve_ik(arm, id);
			}
		}

		void solve_ik(ArmPtr arm, int id) {
			if(tasks[id].is_fix_waist) {
				tasks[id].has_sol = arm->IK_arm(tasks[id].p, tasks[id].R, tasks[id].phi);
			} else {
				tasks[id].has_sol = arm->IK_arm(tasks[id].p, tasks[id].R);
			}

			tasks[id].sol_q = cnoid::VectorXd::Zero(arm->arm_path->numJoints());
			if(tasks[id].has_sol){
				for (int n = 0; n < arm->arm_path->numJoints(); n++)
					tasks[id].sol_q[n] = arm->arm_path->joint(n)->q();
			}
		}

		void clearArms() {
			for (size_t i = 0; i < arms.size(); i++) {
				delete arms[i];
				arms[i] = NULL;
			}
			arms.clear();
		}
	};
}

#endif /* _PICKANDPLACEPLANNER_IKPARALLERIZER_H_ */

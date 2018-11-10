#include <math.h>

#include <boost/lexical_cast.hpp>

#include "../Grasp/PlanBase.h"
#include "../Grasp/VectorMath.h"

#include "ConfigurationSpace.h"
#include "RRTPathPlanner.h"
#include "PRM.h"
#include "RRT.h"
#include "RRTRobot.h"
#include "ParamDialog.h"

using namespace PathEngine;

bool PathPlanner::checkCollision (const Configuration &pos) {
	return col_checker_->checkCollision(pos);
}

bool PathPlanner::checkCollision()
 {
	return col_checker_->checkCollision();
}

bool PathPlanner::checkCollision(const std::vector<Configuration> &path, bool docheckstart, bool docheckend) {
	return col_checker_->checkCollision(path, docheckstart, docheckend);
}

void PathPlanner::init(grasp::RobotBodyPtr body, const std::vector<int>& sampleDoF) {
	if(col_checker_ == NULL) col_checker_ = new CollisionChecker(body);
	if(mobility_ == NULL) mobility_  = new RRTRobot(this);
	for (size_t i = 0; i < sampleDoF.size(); i++) {
		cspace_.setDoF(sampleDoF[i], true);
	}
	for (int i = 0; i < body->numJoints(); i++) {
		cspace_.bounds(i, body->joint(i)->q_lower(), body->joint(i)->q_upper());
		cspace_.weight(i) = 1.0/(body->joint(i)->q_upper() - body->joint(i)->q_lower());
	}
	for (int i = 0; i < 3; i++) {
		cspace_.bounds(body->numJoints() + i, grasp::PlanBase::instance()->llimitMap[i], grasp::PlanBase::instance()->ulimitMap[i]);
		cspace_.weight(body->numJoints() + i) =  1.0/(grasp::PlanBase::instance()->ulimitMap[i] - grasp::PlanBase::instance()->llimitMap[i]);
	}
	for (int i = 0; i < 3; i++) {
		cspace_.bounds(body->numJoints() + 3 + i, -7.0, 7.0);
		cspace_.weight(body->numJoints() + 3 + i) =  1.0/14.0;
	}
	dofsize = sampleDoF.size();
}

bool PathPlanner::calcPath(std::vector<cnoid::VectorXd>& path) {
	Configuration c_start(path[0].size());
	Configuration c_goal(path[1].size());

	for (size_t i = 0; i < path[0].size(); i++) {
		c_start[i] = path[0][i];
		c_goal[i] = path[1][i];
		cspace_.setInitValue(i, path[0][i]);
	}

	setParameters();

	error_message_ = "";

	algorithm_->setStartConfiguration(c_start);
	algorithm_->setGoalConfiguration(c_goal);
	if (!algorithm_->preparePlanning()){
        std::cout << "preparePlanning() failed" << std::endl;
				error_message_ = algorithm_->error_message();
        return false;
    }
    if (algorithm_->tryDirectConnection()){
        path_ = algorithm_->getPath();
				convertPath(path);
        std::cout << "connected directly" << std::endl;
        return true;
    }
    std::cout << "failed direct connection" << std::endl;

    if (algorithm_->calcPath()){
        path_ = algorithm_->getPath();
				convertPath(path);
        return true;
    }
		std::cout << "failed connection" << std::endl;
		error_message_ = "Could not find a path";
    return false;
}

void PathPlanner::convertPath(std::vector<cnoid::VectorXd>& path) {
	path.clear();
	for (size_t i = 0; i < path_.size(); i++) {
		cnoid::VectorXd tmp(path_[i].size());
		for (size_t j = 0; j< path_[i].size(); j++) {
			tmp[j] = path_[i][j];
		}
		path.push_back(tmp);
	}
}

bool CollisionChecker::checkCollision (const Configuration &pos) {
	int nJoints = body_->numJoints();
	for (int i = 0; i < nJoints; i++) {
		body_->joint(i)->q() = pos[i];
	}
	body_->link(0)->p() = cnoid::Vector3(pos[nJoints], pos[nJoints+1], pos[nJoints+2]);
	body_->link(0)->R() = grasp::rotFromRpy(pos[nJoints+3], pos[nJoints+4], pos[nJoints+5]);

    // 干渉チェック
    bool ret = checkCollision();

    return ret;
}

bool CollisionChecker::checkCollision()
 {
	 grasp::PlanBase::instance()->calcForwardKinematics(body_);
	  if (grasp::PlanBase::instance()->isColliding(body_)) {
		 return true;
	 }
	 return false;
}

bool CollisionChecker::checkCollision(const std::vector<Configuration> &path, bool docheckstart, bool docheckend) {
    unsigned int checked = 0;
    unsigned int div = 2;

    std::vector<bool> isVisited;
    for (unsigned int i=0; i<path.size(); i++) {
        isVisited.push_back(false);
    }

		if (!docheckend) {
			isVisited.back() = true;
		}
 
    while (checked < (path.size()-1) || path.size()/div > 0) {
        int step = path.size()/div;
        for (unsigned int i=step; i<path.size(); i+=step) {
            if (!isVisited[i]) {
                checked++;
                if (checkCollision(path[i])) {
                    return true;
                }
                isVisited[i] = true;
            }
        }
        div++;
    }
    if (checked != path.size()-1) {
        std::cout << "checkCollision() : there are unchecked configurations."
                  << " path.size() = " << path.size() << ", checked = " 
                  << checked << std::endl;
    }
		if (docheckstart){
			return checkCollision(path[0]);
		}
		return false;
}

void PathPlanner::setParameters() {
	if (algorithm_ == NULL) return;
	grasp::PRMParams params;
	params.loadParams();
	std::string value;
	value = boost::lexical_cast<std::string>(params.param.eps);
	algorithm_->setProperty("interpolation-distance", value);
	algorithm_->setProperty("eps", value);
	value = boost::lexical_cast<std::string>(params.param.iteration_num);
	algorithm_->setProperty("max-trials", value);
	value = boost::lexical_cast<std::string>(params.param.radius);
	algorithm_->setProperty("radius", value);
	value = boost::lexical_cast<std::string>(params.param.num_path);
	algorithm_->setProperty("number-path", value);
	if (params.param.nnmethod == grasp::PRMParams::NN_KNN) {
		algorithm_->setProperty("use-knn", "1");
	} else {
		algorithm_->setProperty("use-knn", "0");
	}
}

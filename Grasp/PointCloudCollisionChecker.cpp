#include "PointCloudCollisionChecker.h"

#include "ColdetModelGetter.h"
#include "PointCloudEnv.h"

using namespace grasp;

namespace {
	void getAABB(const cnoid::ColdetModelPtr& c, cnoid::Link* link, cnoid::Vector3& min_p, cnoid::Vector3& max_p, double tol) {
		// corner of bb
		std::vector<cnoid::Vector3> bbdata;
		c->getBoundingBoxData(0, bbdata);
		std::vector<cnoid::Vector3> v(8);
		const cnoid::Vector3& center = bbdata[0];
		double v_x = bbdata[1](0);
		double v_y = bbdata[1](1);
		double v_z = bbdata[1](2);
		v[0] = center + cnoid::Vector3( v_x,	v_y,	v_z);
		v[1] = center + cnoid::Vector3( v_x,	v_y, -v_z);
		v[2] = center + cnoid::Vector3( v_x, -v_y,	v_z);
		v[3] = center + cnoid::Vector3( v_x, -v_y, -v_z);
		v[4] = center + cnoid::Vector3(-v_x,	v_y,	v_z);
		v[5] = center + cnoid::Vector3(-v_x,	v_y, -v_z);
		v[6] = center + cnoid::Vector3(-v_x, -v_y,	v_z);
		v[7] = center + cnoid::Vector3(-v_x, -v_y, -v_z);

		min_p.setConstant(std::numeric_limits<double>::max());
		max_p.setConstant(-std::numeric_limits<double>::max());

		for (int j = 0; j < 8; j++) {
			cnoid::Vector3 rv;
			rv = link->R() * v[j] + link->p();
			max_p = max_p.cwiseMax(rv);
			min_p = min_p.cwiseMin(rv);
		}

		max_p = max_p.array() + tol;
		min_p = min_p.array() - tol;
	}

	bool isOverlap(const cnoid::Vector3& min_p, const cnoid::Vector3& max_p, const PointCloudEnv* pc_env) {
		cnoid::Vector3 env_min(pc_env->min_x, pc_env->min_y, pc_env->min_z);
		cnoid::Vector3 env_max(pc_env->max_x, pc_env->max_y, pc_env->max_z);

		bool is_ovelap = !((max_p.array() < env_min.array()).any() ||
											 (env_max.array() < min_p.array()).any());
		return is_ovelap;
	}
}

bool PointCloudCollisionChecker::isCollidingPointCloud(const std::vector<cnoid::Vector3>& p,
																											 const cnoid::BodyItemPtr& item,
																											 double tol) {
	for (int i = 0; i < item->body()->numLinks(); i++) {
		if (ColdetModelGetter::get(item->body()->link(i))->checkCollisionWithPointCloud(p, tol)) {
			return true;
		}
	}
	return false;
}

bool PointCloudCollisionChecker::isCollidingPointCloud(const PointCloudEnv* pc_env,
																											 const cnoid::BodyItemPtr& item,
																											 double tol) {
	return PointCloudCollisionChecker::isCollidingPointCloud(pc_env, item->body(), tol);
}

bool PointCloudCollisionChecker::isCollidingPointCloud(const PointCloudEnv* pc_env,
																											 const cnoid::BodyPtr& body,
																											 double tol) {
	for (int i = 0; i < body->numLinks(); i++) {
		if (PointCloudCollisionChecker::isCollidingPointCloudSub(pc_env, body->link(i), tol)) {
			return true;
		}
	}
	return false;
}

bool PointCloudCollisionChecker::isCollidingPointCloudSub(const PointCloudEnv* pc_env,
																													cnoid::Link* link,
																													double tol) {
	cnoid::ColdetModelPtr c = ColdetModelGetter::get(link);

	cnoid::Vector3 min_p, max_p;
	getAABB(c, link, min_p, max_p, tol);

	if (!isOverlap(min_p, max_p, pc_env)) return false;

	std::vector<cnoid::Vector3> target_p;
	for (int j = 0; j < pc_env->p().size(); j++) {
		const cnoid::Vector3& p = pc_env->p()[j];
		if ((p.array() < max_p.array()).all() &&
				(p.array() > min_p.array()).all()) {
			target_p.push_back(p);
		}
	}

	if (target_p.empty()) return false;
	if (c->checkCollisionWithPointCloud(target_p, tol)) {
		return true;
	}
	return false;
}

bool PointCloudCollisionChecker::getColPointCloud(std::vector<cnoid::Vector3>& colpoints,
																									const PointCloudEnv* pc_env,
																									const cnoid::BodyItemPtr& item,
																									double tol) {
	for (int i = 0; i < item->body()->numLinks(); i++) {
		if (PointCloudCollisionChecker::getColPointCloudSub(colpoints, pc_env, item->body()->link(i), tol)) {
			return true;
		}
	}
	return false;
}

bool PointCloudCollisionChecker::getColPointCloudSub(std::vector<cnoid::Vector3>& colpoints,
																										 const PointCloudEnv* pc_env,
																										 cnoid::Link* link,
																										 double tol) {
	bool ret = false;
	cnoid::ColdetModelPtr c = ColdetModelGetter::get(link);

	cnoid::Vector3 min_p, max_p;
	getAABB(c, link, min_p, max_p, tol);

	if (!isOverlap(min_p, max_p, pc_env)) return false;

	std::vector<cnoid::Vector3> target_p(1);
	for (int j = 0; j < pc_env->p().size(); j++) {
		const cnoid::Vector3& p = pc_env->p()[j];
		if ((p.array() < max_p.array()).all() &&
				(p.array() > min_p.array()).all()) {
			target_p[0] = p;
			if (c->checkCollisionWithPointCloud(target_p, tol)) {
				colpoints.push_back(p);
				ret = true;
			}
		}
	}
	return ret;
}

bool PointCloudCollisionChecker::isCollidingPointCloudAllCheck(const PointCloudEnv* pc_env,
																															 const cnoid::BodyItemPtr& item,
																															 std::vector<std::string>& collink,
																															 double tol) {
	bool ret = false;
	collink.clear();

	for (int i = 0; i < item->body()->numLinks(); i++) {
		cnoid::ColdetModelPtr c = ColdetModelGetter::get(item->body()->link(i));

		cnoid::Vector3 min_p, max_p;
		getAABB(c, item->body()->link(i), min_p, max_p, tol);

		if (!isOverlap(min_p, max_p, pc_env)) continue;

		std::vector<cnoid::Vector3> target_p;
		for (int j = 0; j < pc_env->p().size(); j++) {
			const cnoid::Vector3& p = pc_env->p()[j];
			if ((p.array() < max_p.array()).all() &&
					(p.array() > min_p.array()).all()) {
				target_p.push_back(p);
			}
		}

		if (target_p.empty()) continue;
		if (c->checkCollisionWithPointCloud(target_p, tol)) {
			collink.push_back(item->body()->link(i)->name());
			ret = true;
		}
	}
	return ret;
}


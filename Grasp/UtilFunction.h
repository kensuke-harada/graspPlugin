#ifndef _GRASP_UTILFUNCTION_H_
#define _GRASP_UTILFUNCTION_H_

#include <cnoid/ColdetModel>
#include <cnoid/ColdetModelPair>
#include <cnoid/JointPath>

#ifdef CNIOD_GE_16
#include <memory>
#else
#include <boost/make_shared.hpp>
#endif


namespace grasp {
	inline cnoid::ColdetModelPtr createColdetModel() {
#ifdef CNOID_GE_16
		return std::make_shared<cnoid::ColdetModel>();
#else
#ifdef CNOID_10_11_12_13
		return new cnoid::ColdetModel();
#else
		return boost::make_shared<cnoid::ColdetModel>();
#endif
#endif
	}

	inline cnoid::ColdetModelPairPtr createColdetModelPair(const cnoid::ColdetModelPtr& model1, const cnoid::ColdetModelPtr& model2) {
#ifdef CNOID_GE_16
		return std::make_shared<cnoid::ColdetModelPair>(model1, model2);
#else
		return boost::make_shared<cnoid::ColdetModelPair>(model1, model2);
#endif
	}

	inline cnoid::JointPathPtr createJointPath(cnoid::Link* link1, cnoid::Link* link2) {
#ifdef CNOID_GE_16
		return std::make_shared<cnoid::JointPath>(link1, link2);
#else
		return boost::make_shared<cnoid::JointPath>(link1, link2);
#endif
	}
}

#endif /* _GRASP_UTILFUNCTION_H_ */

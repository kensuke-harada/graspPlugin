/**
 * @file   ClassifierFactory.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACEPLANNER_CLASSIFIERFACTORY_H_
#define _PICKANDPLACEPLANNER_CLASSIFIERFACTORY_H_

#include <string>

#include "SVMClassifier.h"
#include "RandomForestClassifier.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

namespace pt = boost::property_tree;
using boost::optional;

class ClassifierFactory {
 public:
	AbstractClassifier* create(const std::string& param_path) {
		pt::ptree property;
		pt::read_ini(param_path.c_str(), property);

		optional<std::string> tmp_str;
		std::string method_name;

		tmp_str = property.get_optional<std::string>("General.method");
		if (tmp_str) {
			method_name = tmp_str.get();
		} else {
			// error
			return NULL;
		}

		return create(param_path, method_name);
	}

	AbstractClassifier* create(const std::string& param_path, const std::string& method_name) {
		AbstractClassifier* classifier;

		if (method_name == "SVM") {
			classifier = new SVMClassifier();
		} else if (method_name == "RandomForest") {
			classifier = new RandomForestClassifier();
		} else {
			// error
			return NULL;
		}
		classifier->loadParameter(param_path);
		return classifier;
	}
};

#endif


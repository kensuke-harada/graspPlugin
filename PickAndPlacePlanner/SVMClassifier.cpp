/**
 * @file   SVMClassifier.cpp
 * @author Akira Ohchi
*/

#include "SVMClassifier.h"

#include <iostream>
#include <cmath>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
namespace pt = boost::property_tree;
using boost::optional;

namespace {
	void print_null(const char *s) {}
}

SVMClassifier::SVMClassifier() :
	AbstractClassifier("SVM"),
	model_(NULL),
	prob_(NULL) {
}

SVMClassifier::~SVMClassifier() {
	if (model_ != NULL) svm_free_and_destroy_model(&model_);
	if (prob_ != NULL) clearProblem();
}

bool SVMClassifier::train(const Samples& data) {
	if (model_ != NULL) svm_free_and_destroy_model(&model_);

	if (do_gridsearch_) gridSearch(data);

	int p_count, n_count;
	if (!countSamples(data, &p_count, &n_count)) {
		std::cerr << "Only one class in data" << std::endl;
		return false;
	}
	double w = static_cast<double>(p_count) / n_count;

	// setting problem
	settingProblem(data);

	svm_parameter param;
	param.svm_type = C_SVC;
	param.kernel_type = (is_linear_) ? LINEAR : RBF;
	param.degree = 3;
	param.gamma = gamma_;  //  1/num_features
	param.coef0 = 0;
	param.nu = 0.5;
	param.cache_size = 100;
	param.C = cost_;
	param.eps = 1e-3;
	param.p = 0.1;
	param.shrinking = 1;
	param.probability = 1;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;
	// param.nr_weight = 1;
	// param.weight_label = new int[1];
	// param.weight = new double[1];
	// param.weight_label[0] = -1;
	// param.weight[0] = w;
	// std::cout << w << std::endl;

	svm_set_print_string_function(print_null);

	model_ = svm_train(prob_, &param);

	// delete[] param.weight;
	// delete[] param.weight_label;

	return true;
}

void SVMClassifier::gridSearch(const Samples& data) {
	double best_score = 0;
	double best_c = 0;
	double best_g = 0;
	const int n_fold = 5;

	double c = pow(2, 15);
	for (int i = 0; i < 10; i++) {
		double g = pow(2, -13);
		for (int j = 0; j < 9; j++) {
			double score = crossValidation(data, n_fold, c, g);
			score *= 0.5;
			if (score >= best_score) {
				best_score = score;
				best_c = c;
				best_g = g;
			}
			g *= 4;
			if (is_linear_) break;
		}
		c /= 4;
	}
	cost_ = best_c;
	gamma_ = best_g;
}

double SVMClassifier::crossValidation(const Samples& data, int n_fold, double cost, double gamma) {
	// setting problem
	settingProblem(data);

	double* target = new double[prob_->l];

	svm_parameter param;
	param.svm_type = C_SVC;
	param.kernel_type = (is_linear_) ? LINEAR : RBF;
	param.degree = 3;
	param.gamma = (is_linear_) ? 0 : gamma;  //  1/num_features
	param.coef0 = 0;
	param.nu = 0.5;
	param.cache_size = 100;
	param.C = cost;
	param.eps = 1e-3;
	param.p = 0.1;
	param.shrinking = 1;
	param.probability = 0;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;

	svm_set_print_string_function(print_null);

	svm_cross_validation(prob_, &param, n_fold, target);

	int tp, fp, tn, fn;
	tp = fp = tn = fn = 0;
	for (int i = 0; i < prob_->l; i++) {
		if (prob_->y[i] == 1) {
			if (target[i] == prob_->y[i]) {
				tp++;
			} else {
				fn++;
			}
		} else {
			if (target[i] == prob_->y[i]) {
				tn++;
			} else {
				fp++;
			}
		}
	}

	double pos_rec  = ((double)tp/(double)(tp + fn));
	double neg_rec	= ((double)tn/(double)(tn + fp));

	delete[] target;

	// return (pos_rec + neg_rec);
	return ((double)(tp + tn) / (tp + fn + tn + fp));
}

bool SVMClassifier::countSamples(const Samples& data, int* p_count, int* n_count) const {
	*p_count = 0;
	*n_count = 0;
	for (size_t i = 0; i < data.size(); i++) {
		if (data[i].label == 1) (*p_count)++;
		if (data[i].label == -1) (*n_count)++;
	}
	if (p_count == 0 || n_count == 0) {
		return false;
	}
	return true;
}

void SVMClassifier::settingProblem(const Samples& data) {
	clearProblem();

	prob_ = new svm_problem;

	prob_->l = data.size();
	prob_->y = new double[prob_->l];
	for (size_t i = 0; i < data.size(); i++) {
		prob_->y[i] = data[i].label;
	}
	prob_->x = new svm_node*[prob_->l];
	for (int i = 0; i < prob_->l; i++) {
		prob_->x[i] = new svm_node[feature_size_+1];
	}
	for (size_t i = 0; i < prob_->l; i++) {
		for (size_t j = 0; j < feature_size_; j++) {
			prob_->x[i][j].index = j + 1;
			prob_->x[i][j].value = data[i].feature[j];
		}
		prob_->x[i][feature_size_].index = -1;
	}
}

void SVMClassifier::clearProblem() {
	if (prob_ == NULL) return;

	for (int i = 0; i < prob_->l; i++) {
		delete[]  prob_->x[i];
	}
	delete[] prob_->x;
	delete[] prob_->y;

	delete prob_;
	prob_ = NULL;
}

bool SVMClassifier::loadParameter(const std::string& filename) {
	loadGeneralParameter(filename);

	pt::ptree property;
	pt::read_ini(filename.c_str(), property);

	optional<int> tmp_int;
	optional<double> tmp_double;

	tmp_int = property.get_optional<int>("SVM.linear");
	if (tmp_int) is_linear_ = (tmp_int.get() > 0);

	tmp_double = property.get_optional<double>("SVM.cost");
	if (tmp_double) cost_ = tmp_double.get();

	tmp_double = property.get_optional<double>("SVM.gamma");
	if (tmp_double) gamma_ = tmp_double.get();

	tmp_int = property.get_optional<int>("SVM.gridSearch");
	if (tmp_int) do_gridsearch_ = (tmp_int.get() > 0);

	return true;
}

bool SVMClassifier::predict(const Feature& feature, double* probability) const {
	svm_node* x = new svm_node[feature_size_+1];
	for (size_t i = 0; i < feature_size_; i++) {
		x[i].index = i + 1;
		x[i].value = feature[i];
	}
	x[feature_size_].index = -1;

	double predict_label = svm_predict_probability(model_, x, probability);

	delete[] x;
	return (predict_label > 0);
}

bool SVMClassifier::saveModel(const std::string& filename) const {
	if (model_ == NULL) return false;
	svm_save_model(filename.c_str(), model_);
	return true;
}

bool SVMClassifier::loadModel(const std::string& filename) {
	if (!fs::exists(fs::path(filename))) return false;
	if (model_ != NULL) svm_free_and_destroy_model(&model_);
	model_ = svm_load_model(filename.c_str());
	return true;
}

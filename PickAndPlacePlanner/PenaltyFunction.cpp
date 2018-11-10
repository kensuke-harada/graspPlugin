/**
 * @file   PenaltyFunction.cpp
 * @author Akira Ohchi
*/

#include "PenaltyFunction.h"

#include <iostream>
#include <string>

#include <cnoid/ExecutablePath>

#include "ClassifierFactory.h"

namespace {
	std::string getPluginDirPath() {
		return cnoid::executableTopDirectory() + "/extplugin/graspPlugin/PickAndPlacePlanner/";
	}

	std::string getPenaltyTrainDataFilePath(const std::string& hand_name, const std::string& obj_name, const std::string& method_name) {
		return getPluginDirPath() + "/learning_data/" + hand_name + "_" + obj_name + "_" + method_name + "_training_data.txt";
	}

	std::string getPenaltyModelFilePath(const std::string& hand_name, const std::string& obj_name, const std::string& method_name) {
		return getPluginDirPath() + "/learning_data/" + hand_name + "_" + obj_name + "_" + method_name + "_model.txt";
	}

	std::string getPenaltyParamFilePath() {
		return getPluginDirPath() + "/classify_param.ini";
	}
}

PenaltyCalculator::PenaltyCalculator() {
	init();
}

PenaltyCalculator::~PenaltyCalculator() {
	if (classifier_ != NULL) delete classifier_;
}

double PenaltyCalculator::getScore(const std::vector<double>& v) const {
	if (v.size() != classifier_->getFeatureSize()) {
		// error
		return -1;
	}

	double score;
	classifier_->predict(v, &score);
	return (score - 0.5);
}

double PenaltyCalculator::getDefaultScore(const std::vector<double>& v) const {
	if (v.size() != 2) {
		// error
		return -1;
	}
	double score = 100;
	score -= (v[0] + v[1]);
	return score;
}

bool PenaltyCalculator::hasEnoughData() const {
	return has_enough_data_;
}

bool PenaltyCalculator::useHistogramFeature() const {
	return use_hist_;
}

const HistogramParameter* PenaltyCalculator::getHistogramParameter() const {
	return classifier_->getHistogramParameter();
}

void PenaltyCalculator::update(const std::string& hand_name, const std::string& obj_name) {
	if (!loadModel(hand_name, obj_name)) {
		has_enough_data_ = false;
		return;
	}
	int required_amount = classifier_->getRequiredDataAmount();
	AbstractClassifier::Samples data;
	std::string filepath = getPenaltyTrainDataFilePath(hand_name, obj_name, classifier_->getMethodName());
	classifier_->loadTrainData(filepath, data);
	int p_count, n_count;
	p_count = n_count = 0;
	for (size_t i = 0; i < data.size(); i++) {
		if (data[i].label > 0) {
			p_count++;
		} else {
			n_count++;
		}
	}
	has_enough_data_ = (p_count >= required_amount && n_count >= required_amount);
}

bool PenaltyCalculator::loadModel(const std::string& hand_name, const std::string& obj_name) {
	std::string filepath = getPenaltyModelFilePath(hand_name, obj_name, classifier_->getMethodName());
	return classifier_->loadModel(filepath);
}

void PenaltyCalculator::init() {
	has_enough_data_ = false;

	ClassifierFactory factory;
	classifier_ = factory.create(getPenaltyParamFilePath());

	use_hist_ = classifier_->getHistogramParameter()->use;
}

PenaltyFunctionUpdater* PenaltyFunctionUpdater::instance() {
	static PenaltyFunctionUpdater* instance = new PenaltyFunctionUpdater();
	return instance;
}

PenaltyFunctionUpdater::PenaltyFunctionUpdater() {
	ClassifierFactory factory;
	classifier_ = factory.create(getPenaltyParamFilePath());
}

PenaltyFunctionUpdater::~PenaltyFunctionUpdater() {
	if (classifier_ != NULL) delete classifier_;
}

void PenaltyFunctionUpdater::addData(const std::vector<double>& v) {
	tmp_data_ = v;
}

bool PenaltyFunctionUpdater::registerData(bool label) {
	if (tmp_data_.empty()) return false;

	std::string filepath = getPenaltyTrainDataFilePath(hand_name_, obj_name_, classifier_->getMethodName());
	bool ret = classifier_->appendTrainData(filepath, label, tmp_data_);

	tmp_data_.clear();

	return ret;
}

void PenaltyFunctionUpdater::setHandName(const std::string& hand_name) {
	hand_name_ = hand_name;
}

void PenaltyFunctionUpdater::setObjectName(const std::string &obj_name) {
	obj_name_ = obj_name;
}

bool PenaltyFunctionUpdater::update() {
	int required_amount = classifier_->getRequiredDataAmount();
	AbstractClassifier::Samples data;
	std::string filepath = getPenaltyTrainDataFilePath(hand_name_, obj_name_, classifier_->getMethodName());
	classifier_->loadTrainData(filepath, data);
	int p_count, n_count;
	p_count = n_count = 0;
	for (size_t i = 0; i < data.size(); i++) {
		if (data[i].label > 0) {
			p_count++;
		} else {
			n_count++;
		}
	}
	if (p_count < required_amount || n_count < required_amount) return false;
	if (classifier_->getMethodName() == "SVM") {
		classifier_->upsampling(data);
	}
	bool ret = classifier_->train(data);
	if (ret) classifier_->saveModel(getPenaltyModelFilePath(hand_name_, obj_name_, classifier_->getMethodName()));
	return ret;
}

bool PenaltyFunctionUpdater::isExistModelFile(const std::string& hand_name, const std::string& obj_name) const {
	if (classifier_ == NULL) return false;
	std::string filepath = getPenaltyModelFilePath(hand_name, obj_name, classifier_->getMethodName());
	return QFile::exists(filepath.c_str());
}

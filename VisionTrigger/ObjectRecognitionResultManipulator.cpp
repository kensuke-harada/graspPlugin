#include "ObjectRecognitionResultManipulator.h"

#include <iostream>

using namespace grasp;

ObjectRecognitionResultManipulator::ObjectRecognitionResultManipulator() :
	recog_start_flag_(false),
	finish_flag_(false),
	obj_id_(-1) {
}

ObjectRecognitionResultManipulator* ObjectRecognitionResultManipulator::instance() {
	static ObjectRecognitionResultManipulator* instance = new ObjectRecognitionResultManipulator();
	return instance;
}

void ObjectRecognitionResultManipulator::startRecognition() {
	boost::mutex::scoped_lock lock(mtx_);
	finish_flag_ = false;
	while(!results_.empty()) results_.pop();
	recog_start_flag_ = true;
}

void ObjectRecognitionResultManipulator::changeStartFlag(bool val) {
	boost::mutex::scoped_lock lock(mtx_);
	recog_start_flag_ = val;
}

bool ObjectRecognitionResultManipulator::getStartFlag() {
	boost::mutex::scoped_lock lock(mtx_);
	return recog_start_flag_;
}

void ObjectRecognitionResultManipulator::setObjectID(int id) {
	obj_id_ = id;
}

int ObjectRecognitionResultManipulator::getObjectID() const {
	return obj_id_;
}

void ObjectRecognitionResultManipulator::pushResult(double* res, int len) {
	boost::mutex::scoped_lock lock(mtx_);

	ConvertData::RecognitionResultExt result;
	bool suc = ConvertData::dblSeqToResult(res, len, result);
	if (result.is_finish) finish_flag_ = true;

	if (result.valid) results_.push(result);
}

bool ObjectRecognitionResultManipulator::isFinish() const {
	return finish_flag_;
}

bool ObjectRecognitionResultManipulator::popResult(ConvertData::RecognitionResultExt& res) {
	boost::mutex::scoped_lock lock(mtx_);

	if (results_.empty()) return false;

	res = results_.front();
	results_.pop();

	return true;
}

void ObjectRecognitionResultManipulator::setRecogParams(const ConvertData::RecognitionParam& param) {
	params_ = param;
}

void ObjectRecognitionResultManipulator::getRecogParamsDblVec(std::vector<double>& param_vec) const {
	ConvertData::recognitionParamTodblVec(params_, param_vec);
}

void ObjectRecognitionResultManipulator::setEnvResult(double* res, int len) {
	bool suc = ConvertData::dblSeqToEnvResult(res, len, env_res_);
	if (!suc) std::cerr << "ERROR: setEnvResults failed!" << std::endl;
}

void ObjectRecognitionResultManipulator::getEnvResult(ConvertData::RecognitionEnvResult& res) const {
	res = env_res_;
}


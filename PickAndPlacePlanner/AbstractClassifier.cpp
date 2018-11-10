/**
 * @file   AbstractClassifier.cpp
 * @author Akira Ohchi
*/

#include "AbstractClassifier.h"

#include <iostream>
#include <fstream>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

namespace pt = boost::property_tree;
using boost::optional;

namespace {
	class DistBetweenSamples {
	public:
		double dist;
		int from;
		int to;
		static bool sortDist(const DistBetweenSamples& l, const DistBetweenSamples& r) {return (l.dist < r.dist);}
	};
}

AbstractClassifier::AbstractClassifier(const std::string& method_name) :
	method_name_(method_name) {
}

AbstractClassifier::~AbstractClassifier() {
}

std::string AbstractClassifier::getMethodName() const {
	return method_name_;
}

int AbstractClassifier::getFeatureSize() const {
	return feature_size_;
}

const HistogramParameter* AbstractClassifier::getHistogramParameter() const {
	return &hist_param_;
}

int AbstractClassifier::getRequiredDataAmount() const {
	return required_data_amount_;
}

bool AbstractClassifier::appendTrainData(const std::string& file_path, const Sample& sample) const {
	return appendTrainData(file_path, sample.label, sample.feature);
}

bool AbstractClassifier::appendTrainData(const std::string& file_path, bool label, const Feature &feature) const {
	if (feature.size() != feature_size_) {
		std::cerr << "wrong feature size!" << std::endl;
		return false;
	}
	std::ofstream fout(file_path.c_str(), std::ios::app);
	fout << (label ? "1" : "-1");
	for (int i = 0; i < feature_size_; i++) {
		fout << " " << feature[i];
	}
	fout << std::endl;
	return true;
}

bool AbstractClassifier::loadTrainData(const std::string& filepath, Samples &data) const {
	data.clear();

	std::ifstream fin;
	fin.open(filepath.c_str());
	if(!fin) return false;

	std::string buf;
	while (getline(fin, buf)) {
		boost::trim(buf);
		if (buf.empty()) continue;
		std::vector<std::string> split_data;
		boost::split(split_data, buf, boost::is_space(), boost::algorithm::token_compress_on);

		if (split_data.empty()) continue;
		if (split_data[0].at(0) == '#') continue;

		data.push_back(Sample());

		data.back().label = boost::lexical_cast<int>(split_data[0]);
		data.back().feature.resize(feature_size_);
		std::vector<double> tmp(feature_size_);

		for (int i = 0; i < feature_size_; i++) {
			data.back().feature[i] = boost::lexical_cast<double>(split_data[i+1]);
		}
	}
	return true;
}

bool AbstractClassifier::upsampling(Samples& data) {
	int p_count = 0;
	int n_count = 0;
	std::vector<int> p_indices, n_indices;

	for (int i = 0; i < data.size(); i++) {
		if (data[i].label > 0) {
			p_count++;
			p_indices.push_back(i);
		} else {
			n_count++;
			n_indices.push_back(i);
		}
	}

	if (p_count == 0 || n_count == 0) return false;

	if (p_count > n_count) {
		upsamplingProc(data, p_count - n_count, n_indices);
	} else {
		upsamplingProc(data, n_count - p_count, p_indices);
	}
	return true;
}

void AbstractClassifier::upsamplingProc(Samples& data, int amount, const std::vector<int>& indices) {
	int sample_size = indices.size();

	std::vector<DistBetweenSamples> rel(sample_size * (sample_size - 1) / 2);

	int idx = 0;
	for (int i = 0; i < sample_size; i++) {
		int f_idx = indices[i];
		for (int j = i + 1 ; j < sample_size; j++) {
			int t_idx = indices[j];
			rel[idx].from = f_idx;
			rel[idx].to = t_idx;
			rel[idx].dist = 0;
			for (int k = 0; k < feature_size_; k++) {
				rel[idx].dist += (data[f_idx].feature[k] - data[t_idx].feature[k]) * (data[f_idx].feature[k] - data[t_idx].feature[k]);
			}
			idx++;
		}
	}

	sort(rel.begin(), rel.end(), DistBetweenSamples::sortDist);

	for (int i = 0; i < amount; i++) {
		if (i >= rel.size()) break;
		Sample s;
		data.push_back(s);
		data.back().label = data[indices[0]].label;
		data.back().feature.clear();
		data.back().feature.resize(feature_size_);
		for (int j = 0; j < feature_size_; j++) {
			data.back().feature[j] = (data[rel[i].from].feature[j] + data[rel[i].to].feature[j]) / 2.0;
		}
	}
}

void AbstractClassifier::loadGeneralParameter(const std::string& filename) {
	pt::ptree property;
	pt::read_ini(filename.c_str(), property);

	optional<int> tmp_int;
	optional<double> tmp_double;

	tmp_int = property.get_optional<int>("General.featureSize");
	if (tmp_int) feature_size_ = tmp_int.get();

	tmp_int = property.get_optional<int>("General.useHistogram");
	if (tmp_int) hist_param_.use = (tmp_int.get() > 0);

	tmp_int = property.get_optional<int>("General.hDim");
	if (tmp_int) hist_param_.h_dim = tmp_int.get();

	tmp_int = property.get_optional<int>("General.dDim");
	if (tmp_int) hist_param_.d_dim = tmp_int.get();

	tmp_double = property.get_optional<double>("General.hStep");
	if (tmp_double) hist_param_.h_step = tmp_double.get();

	tmp_double = property.get_optional<double>("General.dStep");
	if (tmp_double) hist_param_.d_step = tmp_double.get();

	tmp_int = property.get_optional<int>("General.requiredDataSize");
	if (tmp_int) required_data_amount_ = tmp_int.get();
}


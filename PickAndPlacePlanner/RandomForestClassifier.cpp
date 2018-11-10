/**
 * @file   RandomForestClassifier.cpp
 * @author Akira Ohchi
*/

#include "RandomForestClassifier.h"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <limits>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
namespace pt = boost::property_tree;
using boost::optional;

RandomForestClassifier::RandomForestClassifier() :
	AbstractClassifier("RandomForest"),
	num_tree_(3),
	max_depth_(5),
	num_feature_trial_(50),
	num_threshold_trial_(5),
	num_feature_(1),
	data_per_tree_(.5f) {
}

RandomForestClassifier::~RandomForestClassifier() {
	clearTrees();
}

bool RandomForestClassifier::train(const Samples& data) {
	srand(time(NULL));

	double pw, nw;
	if (!calcWeight(data, &pw, &nw)) {
		std::cerr << "Only one class in data" << std::endl;
		return false;
	}

	clearTrees();

	for (int t = 0; t < num_tree_; t++) {
		trees.push_back(new RfTree());

		std::vector<int> indices;
		sampling(data.size(), indices);

		trees.back()->build(data, indices,
												max_depth_, num_feature_trial_, num_threshold_trial_, num_feature_,
												pw, nw);
	}
	return true;
}

void RandomForestClassifier::clearTrees() {
	for (size_t t = 0; t < trees.size(); t++) {
		delete trees[t];
	}
	trees.clear();
}

bool RandomForestClassifier::calcWeight(const Samples& data, double* pw, double* nw) const {
	int p_count = 0;
	int n_count = 0;
	for (size_t i = 0; i < data.size(); i++) {
		if (data[i].label == 1) p_count++;
		if (data[i].label == -1) n_count++;
	}
	if (p_count == 0 || n_count == 0) {
		return false;
	}
	*pw = 1.0f / p_count;
	*nw = 1.0f / n_count;
	return true;
}

void RandomForestClassifier::sampling(int size, std::vector<int>& indices) const {
	indices.clear();
	int subset_size = static_cast<int>(ceil(size * data_per_tree_));
	indices.resize(subset_size);
	for (int i = 0; i < subset_size; i++) {
		indices[i] = rand() % size;
	}
}

bool RandomForestClassifier::loadParameter(const std::string& filename) {
	loadGeneralParameter(filename);

	pt::ptree property;
	pt::read_ini(filename.c_str(), property);

	optional<int> tmp_int;
	optional<double> tmp_double;

	tmp_int = property.get_optional<int>("RandomForest.featureSize");
	if (tmp_int) feature_size_ = tmp_int.get();

	tmp_int = property.get_optional<int>("RandomForest.numTree");
	if (tmp_int) num_tree_ = tmp_int.get();

	tmp_int = property.get_optional<int>("RandomForest.maxDepth");
	if (tmp_int) max_depth_ = tmp_int.get();

	tmp_int = property.get_optional<int>("RandomForest.featureTrial");
	if (tmp_int) num_feature_trial_ = tmp_int.get();

	tmp_int = property.get_optional<int>("RandomForest.thresholdTrial");
	if (tmp_int) num_threshold_trial_ = tmp_int.get();

	tmp_int = property.get_optional<int>("RandomForest.numFeature");
	if (tmp_int) num_feature_ = tmp_int.get();

	tmp_double = property.get_optional<double>("RandomForest.dataPerTree");
	if (tmp_double) data_per_tree_ = tmp_double.get();

	return true;
}

bool RandomForestClassifier::predict(const Feature &feature, double *probability) const {
	double p_prob = 0;
	double n_prob = 0;
	for (size_t i = 0; i < trees.size(); i++) {
		double p, n;
		trees[i]->getProbability(feature, &p, &n);
		p_prob += p;
		n_prob += n;
	}
	double total = p_prob + n_prob;

	*probability = (p_prob / total);

	return (p_prob > n_prob);
}

bool RandomForestClassifier::saveModel(const std::string& filename) const {
	std::ofstream fout(filename.c_str());
	fout << trees.size() << std::endl;
	for (size_t i = 0; i < trees.size(); i++) {
		trees[i]->save(fout);
	}
	return true;
}

bool RandomForestClassifier::loadModel(const std::string& filename) {
	clearTrees();
	if (!fs::exists(fs::path(filename))) return false;
	std::ifstream fin(filename.c_str());
	int tree_size;
	fin >> tree_size;
	for (int i = 0; i < tree_size; i++) {
		RfTree* tree = new RfTree();
		tree->setNumUsingFeature(num_feature_);
		tree->load(fin);
		trees.push_back(tree);
	}
	return true;
}

RfNode::RfNode() :
	rchild(NULL), lchild(NULL), is_leaf(false),
	p_prob(0.0), n_prob(0.0) {
}

RfNode::~RfNode() {
	if (rchild != NULL) {
		delete rchild;
	}
	if (lchild!= NULL) {
		delete lchild;
	}
}

RfTree::RfTree() :
	root(NULL) {
}

RfTree::~RfTree() {
	delete root;
}

void RfTree::build(const Samples& data, const std::vector<int>& indices,
									 int max_depth, int feature_trial, int threshold_trial, int num_using_feature,
									 double pw, double nw) {
	if (root != NULL) delete root;

	max_depth_ = max_depth;
	feature_trial_ = feature_trial;
	threshold_trial_ = threshold_trial;
	num_using_feature_ = num_using_feature;
	pw_ = pw;
	nw_ = nw;

	root = addNode(data, indices, 0);

	setProbability(data);
}

RfNode* RfTree::addNode(const Samples& data, const std::vector<int>& indices, int depth) {
	std::vector<double> feature_min;
	std::vector<double> feature_max;

	computeFeatureMinMax(data, indices, feature_min, feature_max);

	double best_gini = std::numeric_limits<double>::max();
	std::vector<int> best_f_id(num_using_feature_);
	std::vector<double> best_th(num_using_feature_);
	// int best_f_id = 0;
	// double best_th = 0;
	std::vector<int> best_lindices;
	std::vector<int> best_rindices;

	std::vector<int> f_id(num_using_feature_);
	std::vector<double> th(num_using_feature_);
	for (int i = 0; i < feature_trial_; i++) {
		for (int n = 0; n < num_using_feature_; n++) {
			f_id[n] = rand() % data[0].feature.size();
		}
		// int f_id = rand() % data[0].feature.size();
		for (int j = 0; j < threshold_trial_; j++) {
			for (int n = 0; n < num_using_feature_; n++) {
				th[n] = feature_min[f_id[n]] + static_cast<double>(rand())/RAND_MAX * (feature_max[f_id[n]] - feature_min[f_id[n]]);
			}
			// double th = feature_min[f_id] + static_cast<double>(rand())/RAND_MAX * (feature_max[f_id] - feature_min[f_id]);n

			std::vector<int> lindices;
			std::vector<int> rindices;

			double gini = computeGini(data, indices, f_id, th, lindices, rindices);

			if (best_gini > gini) {
				best_gini = gini;
				best_f_id = f_id;
				best_th = th;
				best_lindices = lindices;
				best_rindices = rindices;
			}
		}
	}

	RfNode* node = new RfNode();
	node->feature_idx = best_f_id;
	node->threshold = best_th;

	if (isReachLeaf(data, best_lindices, best_rindices) || max_depth_ < depth + 2 || best_lindices.empty() || best_rindices.empty()) {
		node->is_leaf = true;
	} else {
		node->is_leaf = false;
		node->lchild = addNode(data, best_lindices, depth+1);
		node->rchild = addNode(data, best_rindices, depth+1);
	}

	return node;
}

void RfTree::computeFeatureMinMax(const Samples& data, const std::vector<int>& indices,
															std::vector<double>& min, std::vector<double>& max) const {
	int f_size = data[0].feature.size();
	min.clear();
	max.clear();
	min.resize(f_size, std::numeric_limits<double>::max());
	max.resize(f_size, -std::numeric_limits<double>::max());

	for (int i = 0; i < indices.size(); i++) {
		for (int j = 0; j < f_size; j++) {
			if (min[j] > data[indices[i]].feature[j]) {
				min[j] = data[indices[i]].feature[j];
			}
			if (max[j] < data[indices[i]].feature[j]) {
				max[j] = data[indices[i]].feature[j];
			}
		}
	}
}

double RfTree::computeGini(const Samples& data, const std::vector<int>& indices,
													 const std::vector<int>& feature_id, const std::vector<double>& threshold,
													 std::vector<int>& lindices, std::vector<int>& rindices) const {
	lindices.clear();
	rindices.clear();
	int lp, ln, rp, rn;
	lp = ln = rp = rn = 0;
	for (size_t i = 0; i < indices.size(); i++) {
		// if (data[indices[i]].feature[feature_id] < threshold) {
		if (split(data[indices[i]].feature, feature_id, threshold)) {
			lindices.push_back(indices[i]);
			if(data[indices[i]].label == 1) {
				lp++;
			} else {
				ln++;
			}
		} else {
			rindices.push_back(indices[i]);
			if(data[indices[i]].label == 1) {
				rp++;
			} else {
				rn++;
			}
		}
	}

	double w_lp = pw_ * lp;
	double w_ln = nw_ * ln;
	double w_rp = pw_ * rp;
	double w_rn = nw_ * rn;

	double ltotal = w_lp + w_ln;
	double rtotal = w_rp + w_rn;

	double lgini = lindices.empty() ? 0 : ltotal * (1 - (((w_lp * w_lp) + (w_ln * w_ln)) / (ltotal * ltotal)));
	double rgini = rindices.empty() ? 0 : rtotal * (1 - (((w_rp * w_rp) + (w_rn * w_rn)) / (rtotal * rtotal)));

	return (lgini + rgini);
}

bool RfTree::isReachLeaf(const Samples& data, const std::vector<int>& lindices, const std::vector<int>& rindices) const {
	for (size_t i = 1; i < lindices.size(); i++) {
		if (data[lindices[0]].label != data[lindices[i]].label) return false;
	}
	for (size_t i = 1; i < rindices.size(); i++) {
		if (data[rindices[0]].label != data[rindices[i]].label) return false;
	}
	return true;
}

void RfTree::setProbability(const Samples& data) {
	for (size_t i = 0; i < data.size(); i++) {
		vote(data[i], root);
	}
	normalizedProbability(root);
}

void RfTree::vote(const Sample& sample, RfNode* node) {
	if (node == NULL) return;
	if (node->is_leaf) {
		if (sample.label == 1) {
			node->p_prob += pw_;
		} else {
			node->n_prob += nw_;
		}
	} else {
		// if (sample.feature[node->feature_idx] < node->threshold) {
		if (split(sample.feature, node->feature_idx, node->threshold)) {
			vote(sample, node->lchild);
		} else {
			vote(sample, node->rchild);
		}
	}
}

void RfTree::normalizedProbability(RfNode* node) {
	if (node == NULL) return;
	if (node->is_leaf) {
		double total = node->p_prob + node->n_prob;
		if (total == 0) return;
		node->p_prob /= total;
		node->n_prob /= total;
	} else {
		normalizedProbability(node->lchild);
		normalizedProbability(node->rchild);
	}
}

void RfTree::getProbability(const Feature& feature, double* p_prob, double* n_prob) const {
	RfNode* leaf_node;
	leaf_node = getLeafNode(feature, root);
	if (leaf_node == NULL) return;
	*p_prob = leaf_node->p_prob;
	*n_prob = leaf_node->n_prob;
}

RfNode* RfTree::getLeafNode(const Feature& feature, RfNode* node) const {
	if (node == NULL) return NULL;
	if (node->is_leaf) return node;

	// if (feature[node->feature_idx] < node->threshold) {
	if (split(feature, node->feature_idx, node->threshold)) {
		return getLeafNode(feature, node->lchild);
	} else {
		return getLeafNode(feature, node->rchild);
	}
	return NULL;
}

void RfTree::save(std::ofstream& fout) {
	if (root == NULL) return;
	saveNode(fout, root);
}

void RfTree::load(std::ifstream& fin) {
	if (root != NULL) delete root;
	root = loadNode(fin);
}

void RfTree::setNumUsingFeature(int size) {
	num_using_feature_ = size;
}

void RfTree::saveNode(std::ofstream& fout, RfNode* node) {
	if (node == NULL) return;
	fout << ((node->is_leaf) ? 1 : 0) << " ";
	for (int i = 0; i < num_using_feature_; i++) {
		fout <<  node->feature_idx[i] << " " <<  node->threshold[i] << " ";
	}
	fout << node->p_prob << " " <<  node->n_prob << std::endl;
	saveNode(fout, node->lchild);
	saveNode(fout, node->rchild);
}

RfNode* RfTree::loadNode(std::ifstream& fin) {
	RfNode* node = new RfNode();
	int leaf;
	fin >> leaf;
	node->is_leaf = (leaf == 1);
	node->feature_idx.resize(num_using_feature_);
	node->threshold.resize(num_using_feature_);
	for (int i = 0; i < num_using_feature_; i++) {
		fin >> node->feature_idx[i] >> node->threshold[i];
	}
	fin >>	node->p_prob >> node->n_prob;
	if (!node->is_leaf) {
		node->lchild = loadNode(fin);
		node->rchild = loadNode(fin);
	}
	return node;
}


bool RfTree::split(const Feature& data, const std::vector<int>& feature_id, const std::vector<double>& threshold) const {
	for (size_t i = 0; i < feature_id.size(); i++) {
		if (data[feature_id[i]] > threshold[i]) {
			return false;
		}
	}
	return true;
}

/**
 * @file   RandomForestClassifier.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACEPLANNER_RANDOMFORESTCLASSIFIER_H_
#define _PICKANDPLACEPLANNER_RANDOMFORESTCLASSIFIER_H_

#include <string>
#include <vector>

#include "AbstractClassifier.h"

class RfTree;

class RandomForestClassifier :
public AbstractClassifier {
 public:
	RandomForestClassifier();
	virtual ~RandomForestClassifier();

	typedef AbstractClassifier::Feature Feature;
	typedef AbstractClassifier::Samples Samples;

	bool train(const Samples& data);
	bool loadParameter(const std::string& filename);
	bool predict(const Feature& feature, double* probability) const;
	bool saveModel(const std::string& filename) const;
	bool loadModel(const std::string& filename);

 private:
	int num_tree_;
	int max_depth_;
	int num_feature_trial_;
	int num_threshold_trial_;
	int num_feature_;
	double data_per_tree_;

	std::vector<RfTree*> trees;

	void clearTrees();
	bool calcWeight(const Samples& data, double* pw, double* nw) const;
	void sampling(int size, std::vector<int>& indices) const;
};

class RfNode {
 public:
	RfNode();
	virtual ~RfNode();

	std::vector<int> feature_idx;
	std::vector<double> threshold;
	RfNode* rchild;
	RfNode* lchild;
	bool is_leaf;
	double p_prob;
	double n_prob;
};

class RfTree {
 public:
	RfTree();
	virtual ~RfTree();

	typedef AbstractClassifier::Samples Samples;
	typedef AbstractClassifier::Sample Sample;
	typedef AbstractClassifier::Feature Feature;

	void build(const Samples& data, const std::vector<int>& indices,
						 int max_depth, int featrue_trial, int threshold_trial, int num_using_feature,
						 double pw, double nw);
	void getProbability(const Feature& feature, double* p_prob, double* n_prob) const;

	void save(std::ofstream& fout);
	void load(std::ifstream& fin);

	void setNumUsingFeature(int size);
	
 private:
	RfNode* root;

	int max_depth_;
	int feature_trial_;
	int threshold_trial_;
	int num_using_feature_;
	double pw_;
	double nw_;

	RfNode* addNode(const Samples& data, const std::vector<int>& indices, int depth);
	void computeFeatureMinMax(const Samples& data, const std::vector<int>& indices,
												std::vector<double>& min, std::vector<double>& max) const;
	double computeGini(const Samples& data, const std::vector<int>& indices,
										 const std::vector<int>& feature_id, const std::vector<double>& threshold,
										 std::vector<int>& lindices, std::vector<int>& rindices) const;
	bool isReachLeaf(const Samples& data, const std::vector<int>& lindices, const std::vector<int>& rindices) const;

	void setProbability(const Samples& data);
	void vote(const Sample& sample, RfNode* node);
	void normalizedProbability(RfNode* node);

	RfNode* getLeafNode(const Feature& feature, RfNode* node) const;

	void saveNode(std::ofstream& fout, RfNode* node);
	RfNode* loadNode(std::ifstream& fin);
	bool split(const Feature& data, const std::vector<int>& feature_id, const std::vector<double>& threshold) const;
};

#endif

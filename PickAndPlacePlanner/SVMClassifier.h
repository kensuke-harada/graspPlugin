/**
 * @file   SVMClassifier.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACEPLANNER_SVMCLASSIFIER_H_
#define _PICKANDPLACEPLANNER_SVMCLASSIFIER_H_

#include <string>

#include "AbstractClassifier.h"

#include "libsvm/svm.h"

class SVMClassifier :
public AbstractClassifier {
 public:
	SVMClassifier();
	virtual ~SVMClassifier();

	typedef AbstractClassifier::Feature Feature;
	typedef AbstractClassifier::Samples Samples;

	bool train(const Samples& data);
	bool loadParameter(const std::string& filename);
	bool predict(const Feature& feature, double* probability) const;
	bool saveModel(const std::string& filename) const;
	bool loadModel(const std::string& filename);

	void gridSearch(const Samples& data);
	double crossValidation(const Samples& data, int n_fold, double cost, double gamma);

 private:
	svm_model* model_;
	svm_problem* prob_;

	bool is_linear_;
	double cost_;
	double gamma_;
	bool do_gridsearch_;

	void settingProblem(const Samples& data);
	void clearProblem();
	bool countSamples(const Samples& data, int* p_count, int* n_count) const;
};

#endif

/**
 * @file   AbstractClassifier.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACEPLANNER_ABSTRACTCLASSIFIER_H_
#define _PICKANDPLACEPLANNER_ABSTRACTCLASSIFIER_H_

#include <vector>
#include <string>

class HistogramParameter {
 public:
	bool use;
	int h_dim;
	int d_dim;
	double h_step;
	double d_step;
};

class AbstractClassifier {
 public:
	explicit AbstractClassifier(const std::string& method_name);
	virtual ~AbstractClassifier();

	typedef std::vector<double> Feature;
	class Sample {
	public:
		int label;
		Feature feature;
	};
	typedef std::vector<Sample> Samples;

	std::string getMethodName() const;
	int getFeatureSize() const;
	const HistogramParameter* getHistogramParameter() const;
	int getRequiredDataAmount() const;

	virtual bool train(const Samples& data) = 0;
	virtual bool loadParameter(const std::string& filename) = 0;
	virtual bool predict(const Feature& feature, double* probability) const = 0;
	virtual bool saveModel(const std::string& filename) const = 0;
	virtual bool loadModel(const std::string& filename) = 0;

	bool appendTrainData(const std::string& filepath, const Sample& sample) const;
	bool appendTrainData(const std::string& filepath, bool label, const Feature& feature) const;
	bool loadTrainData(const std::string& filepath, Samples& data) const;

	bool upsampling(Samples& data);

 protected:
	std::string method_name_;
	int feature_size_;
	int required_data_amount_;
	HistogramParameter hist_param_;

	void loadGeneralParameter(const std::string& filename);

 private:
	void upsamplingProc(Samples& data, int amount, const std::vector<int>& indices);
};

#endif

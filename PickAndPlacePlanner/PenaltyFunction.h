/**
 * @file   PenaltyFunction.h
 * @author Akira Ohchi
*/

#ifndef _PICKANDPLACEPLANNER_PENALTYFUNCTION_H_
#define _PICKANDPLACEPLANNER_PENALTYFUNCTION_H_

#include <vector>
#include <string>

#include <boost/utility.hpp>

#include <QFile>

class HistogramParameter;
class AbstractClassifier;

class PenaltyCalculator :
boost::noncopyable {
 public:
	PenaltyCalculator();
	virtual ~PenaltyCalculator();

	double getScore(const std::vector<double>& v) const;
	double getDefaultScore(const std::vector<double>& v) const;

	bool hasEnoughData() const;
	bool useHistogramFeature() const;
	const HistogramParameter* getHistogramParameter() const;

	void update(const std::string& hand_name, const std::string& obj_name);

	bool loadModel(const std::string& hand_name, const std::string& obj_name);
 private:
	AbstractClassifier* classifier_;
	bool use_hist_;
	bool has_enough_data_;

	void init();
};

class PenaltyFunctionUpdater :
boost::noncopyable {
 public:
	static PenaltyFunctionUpdater* instance();
	virtual ~PenaltyFunctionUpdater();

	void addData(const std::vector<double>& v);
	bool registerData(bool label);

	void setHandName(const std::string& hand_name);
	void setObjectName(const std::string& obj_name);

	bool update();

	bool isExistModelFile(const std::string& hand_name, const std::string& obj_name) const;

 private:
	PenaltyFunctionUpdater();

	AbstractClassifier* classifier_;

	std::string hand_name_;
	std::string obj_name_;

	std::vector<double> tmp_data_;
};

#endif /* _PICKANDPLACEPLANNER_PENALTYFUNCTION_H_ */

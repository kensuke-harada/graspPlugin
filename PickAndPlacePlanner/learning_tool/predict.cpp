#include <iostream>
#include <fstream>
#include <algorithm>

#include "../ClassifierFactory.h"


class RocData {
public:
	double deci;
	int label;
	static bool sortDeci(const RocData& l, const RocData& r) {return (l.deci > r.deci);}
};


int main(int argc, char **argv) {

	if (argc < 2) return -1;

	ClassifierFactory factory;
	AbstractClassifier* classifier = factory.create("param.ini");

	AbstractClassifier::Samples data;

	if (classifier->getMethodName() == "SVM") {
		std::cout << "loadSVM" << std::endl;
		classifier->loadModel("SVM.model.txt");
	} else {
		std::cout << "loadRF" << std::endl;
		classifier->loadModel("RF.model.txt");
	}

	classifier->loadTrainData(argv[1], data);

	std::vector<RocData> rocdata(data.size());

	int  tp, fp, tn, fn;
	tp = fp = tn = fn = 0;
	std::cout << "predict:" << std::endl;
	for (int i = 0; i< data.size(); i++) {
		double prob;
		bool label = classifier->predict(data[i].feature, &prob);
		std::cout << " " <<  ((label) ? 1 : -1) << " : " << data[i].label << " : " << prob << std::endl;
		if (label && (data[i].label == 1)) tp++;
		if (label && (data[i].label != 1)) fp++;
		if ((!label) && (data[i].label == 1)) fn++;
		if ((!label) && (data[i].label != 1)) tn++;
		//rocdata[i].deci = (label ? prob : -prob);
		rocdata[i].deci = prob;
		rocdata[i].label = data[i].label;
	}
	std::cout << "TP:" << tp << " FP:" << fp << " TN:" << tn << " FN:" << fn << std::endl;
	double pre_p = (float)(tp)/(tp+fp);
	double pre_n = (float)(tn)/(tn+fn);
	double rec_p = (float)(tp)/(tp+fn);
	double rec_n = (float)(tn)/(tn+fp);
	std::cout << "Precision: p:" << pre_p << " n:" << pre_n << std::endl;
	std::cout << "Recall:    p:" << rec_p << " n:" << rec_n << std::endl;
	std::cout << "F1:        p:" << (2 * pre_p * rec_p)/(pre_p + rec_p) << " n:" << (2 * pre_n * rec_n)/(pre_n + rec_n) << std::endl;
	std::cout << "Accuracy: " << (float)(tp + tn) / (tp + tn + fn+ fp) * 100 << std::endl;

	std::sort(rocdata.begin(), rocdata.end(), RocData::sortDeci);
	std::ofstream fout("roc.data");
	int t_count = 0;
	int f_count = 0;
	double prev = -2;
	double prev_x = 0;
	double auc = 0;
	for (int i = 0; i < rocdata.size(); i++) {
		if (rocdata[i].label > 0) {
			t_count++;
		} else {
			f_count++;
		}
		double x = (double)(f_count) / (tn+fp);
		double y = (double)(t_count) / (tp+fn);
		if (prev != rocdata[i].deci) {
			fout << x << " " << y << std::endl;
		}
		if (prev_x != x) {
			auc += (x - prev_x) * y;
		}
		prev = rocdata[i].deci;
		prev_x = x;
	}
	std::cout << "AUC: " << auc << std::endl;
	return 0;
}

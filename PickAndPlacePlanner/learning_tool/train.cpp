#include <iostream>

#include "../ClassifierFactory.h"

int main(int argc, char **argv) {

	if (argc < 2) return -1;

	ClassifierFactory factory;

	AbstractClassifier* classifier = factory.create("param.ini");

	AbstractClassifier::Samples data;

	classifier->loadTrainData(argv[1], data);

	if (classifier->getMethodName() == "SVM") {
		classifier->upsampling(data);
	}

	classifier->train(data);

	if (classifier->getMethodName() == "SVM") {
		classifier->saveModel("SVM.model.txt");
	} else {
		classifier->saveModel("RF.model.txt");
	}

	return 0;
}

#include "GraspDatabaseManipulator.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

bool GraspDatabaseManipulator::readFile(const string& filepath) {
  ifstream ifs;
  ifs.open(filepath.c_str());
	
	records.clear();

	int id = 0;

	string line;
	while (getline(ifs, line)) {
		if (line.empty()) continue;

		stringstream buf(line);
		vector<double> val;
		double tmp;
		while (buf >> tmp) {
			val.push_back(tmp);
		}

		if (val.size() < 13) return false;

		GraspPosture pose;
		pose.id = id++;
		pose.q = val[0];
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				pose.R(i, j) = val[i*3+j+1];
			}
		}
		for (int i = 0; i < 3; i++) {
			pose.p(i) = val[10+i];
		}
		pose.finger_q = VectorXd::Zero(val.size() - 13);
		for (size_t i = 13; i < val.size(); i++) {
			pose.finger_q(i - 13) = val[i];
		}
		records.push_back(pose);
	}

	ifs.close();
	return true;
}

bool GraspDatabaseManipulator::writeFile(const string& filepath) {
	ofstream fout(filepath.c_str());

	for (size_t i = 0; i < records.size(); i++) {
		fout << records[i].q << " ";
		 for (int m = 0; m < 3; m++) {
			 for (int n = 0; n < 3; n++) {
				 fout << records[i].R(m, n) << " ";
			 }
		 }
		 for (int m = 0; m < 3; m++) {
			 fout << records[i].p(m) << " ";
		 }
		 for (int m = 0;m < records[i].finger_q.size();m++){
			 fout << records[i].finger_q[m] << " ";
		 }
		 fout << endl;
	}

	fout.close();
	return true;
}
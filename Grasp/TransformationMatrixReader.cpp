#include "TransformationMatrixReader.h"

#include <cstdio>
#include <fstream>

using namespace grasp;

bool TransMatReader::getTransMatrix(const std::string& filepath,
																		cnoid::Matrix3& R,
																		cnoid::Vector3& p) {
	bool ret = true;
	cnoid::Vector3 t1, t2, t3;
	cnoid::Matrix3 R1, R2, R3;

	t1 = t2 = t3 = cnoid::Vector3::Zero();
	R1 = R2 = R3 = cnoid::Matrix3::Identity();

	char line[1024];

	FILE *ifp0 = NULL;
	ifp0 = fopen(filepath.c_str(), "rb");

	if (ifp0 != NULL) {
		if (!fgets(line, 256, ifp0)) {
			printf("result mat: Broken format1 \n");
			ret = false;
		}

		int j = 0;
		while (1) {
			while (fgets(line, 256, ifp0) != NULL) {
				if (line[0] != '#' && line[0] != ' ' &&
					 line[0] != 0x0D && line[0] != 0x0A && line[0] != '\t') {
					break;
				}
			}

			if (line[0] != '4') {
				printf("result mat: Broken format1 \n");
				ret = false;
			}

			cnoid::Vector3 P0;
			cnoid::Matrix3 R0;

			int i = 0;
			while (fgets(line, 256, ifp0) != NULL) {
				if (i < 3 && sscanf(line, "%lf%lf%lf%lf", &R0(i, 0),
												 &R0(i, 1), &R0(i, 2), &P0(i)) != 4) {
					printf("result mat: Broken format2 %d \n", i);
					fclose(ifp0);
					return false;
				}
				i++;
				if (i >= 4) {
					break;
				}
			}
			if (j == 0) {R1 = R0; t1 = P0;}
			if (j == 1) {R2 = R0; t2 = P0;}
			if (j == 2) {R3 = R0; t3 = P0;}
			if (++j == 3) break;
		}
		fclose(ifp0);
	} else {
		printf("cannot open file %s\n", filepath.c_str());
		ret = false;
	}

	p = t3 + R3 * (t2 + R2 * t1);
	R = R3 * R2 * R1;
	return ret;
}

bool TransMatReader::writeTransMatrix(const std::string& filepath, const cnoid::Matrix4f& T) {
	std::ofstream fout(filepath.c_str());
	if (!fout) return false;

	cnoid::Matrix4f I = cnoid::Matrix4f::Identity();

	fout.setf(std::ios::fixed | std::ios::left);
	fout << "#" << std::endl;
	fout << "4 4" << std::endl;
	fout << T << std::endl;

	fout << "#" << std::endl;
	fout << "4 4" << std::endl;
	fout << I << std::endl;

	fout << "#" << std::endl;
	fout << "4 4" << std::endl;
	fout << I;

	return true;
}


#include <fstream>
#include <string>
#include <iostream>

#include <math.h>

#include <algorithm>
#include <list>
#include <vector>

#include <time.h>
#include <sys/resource.h>

#include <hrpUtil/Tvmet3d.h>
#include  <hrpUtil/uBlasCommonTypes.h>

#define deg2Rad(x)   ((x)*(3.141596)/(180.0))
#define m_pi    (3.141596)

using namespace std;
using namespace hrp;

int main(int argc, char *argv[]){
	
	double qtmp;
	Vector3 rpp;
	Matrix33 rpr;
	
	if(argc < 3) {
		cout << "Error: MirrorTransfome inputFolder outputFolder" << endl;
	}
	
	string obj_data   = string(argv[1]) + "/obj_list.txt";
	ifstream fin_obj(obj_data.c_str());
	string line2;

	string pos_data;
	string tmp_obj;

	if(!fin_obj){
		cout << "obj_list.txt not found" << endl;
	}
		
	while(fin_obj){
		getline(fin_obj,line2);
		stringstream li2;
		li2 << line2;

		li2 >> qtmp;
		if(!li2.good()) break;
		li2 >> tmp_obj;

		pos_data = string(argv[1]) + "/" + tmp_obj;
		ifstream fin_pos(pos_data.c_str());
		if(!fin_pos){
			cout << "file not found " << pos_data << endl;
			continue;
		}

		
		ofstream fout_pos(string(string(argv[2]) + "/" + tmp_obj).c_str());
		if(!fout_pos){
			cout << "Cannot access: " << string(string(argv[2]) + "/" + tmp_obj) << endl;
			continue;
		}

		string line;
		vector<double> fpos, fpos_temp, fpos_cand;
		while(fin_pos){
			getline(fin_pos,line);
			stringstream li;
			li << line;

			li >> qtmp;
			if(!li.good()) break;
			for(int i=0;i<3;i++){
				for(int j=0;j<3;j++){
					li >> rpr(i,j);
				}
			}
			for(int i=0;i<3;i++){
				li >> rpp(i);
			}
			Vector3 mirror(trans(rpr)*rpp);
			mirror[1] = - mirror[1];
			rpp = rpr*mirror;
			
			fout_pos << qtmp << " ";
			for(int i=0;i<3;i++){
				for(int j=0;j<3;j++){
					fout_pos << rpr(i,j) << " ";
				}
			}
			for(int i=0;i<3;i++){
				fout_pos << rpp(i) << " ";
			}
			
			while(! li.eof()){
				double tmp;
				li >> tmp;
				if(li.good()) fout_pos << tmp << " ";
			}
			fout_pos << endl;
		}
	}
	string command;
	command = "cp " + string( argv[1] ) + "/obj_list.txt " + string( argv[2] ) + "/obj_list.txt";
	if( system(command.c_str()) ) cout << "obj_list copy error" << endl;

	return true;
	
}
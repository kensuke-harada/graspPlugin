#include "ObjectPosReader.h"

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

ObjectPosReader::ObjectPosReader() : isMulti(false) {

}

ObjectPosReader::~ObjectPosReader() {

}

bool ObjectPosReader::doReadFromFile(string dataPath, string calibFile, int num) {

		char line[1024];

		PlanBase* tc = PlanBase::instance();

		Matrix3 Rm;
		Vector3 Pm;
		FILE *ifp=NULL;

		ifp = fopen(dataPath.c_str(),"rb");

		if(ifp==NULL){
				printf("No data.mat\n");
				return false;
		}

		if(! fgets(line,256,ifp) ){
			printf("result mat: Broken format1 \n");
		}

		int count=0;
		while(1){
				while(fgets(line,256,ifp) != NULL)
						if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;

				if(line[0] != '4'){
						printf("result mat: Broken format1 \n");
						return false;
				}

				int i=0;
				while(fgets(line,256,ifp) != NULL){
						if(i<3 && sscanf(line,"%lf%lf%lf%lf",&Rm(i,0),&Rm(i,1),&Rm(i,2),&Pm(i)) != 4){
								printf("result mat: Broken format2 %d \n",i);
								return false;
						}
						i++;
						if(i >= 4) break;
				}
				if (i<3) return false;

				Pm /= 1000.0; //model m -> mm

				if(++count > num ) break;
		}

		setTransMatrix(calibFile);

		Vector3 vu = t3 + R3*(t2 + R2*(t1 + R1*Pm));
		Matrix3 Ru = R3*R2*R1*Rm;

		tc->setObjPos(vu, Ru);

		tc->flush();

		std::cout << "Position/orientation of object was set to:" << tc->objVisPos().transpose() << endl;cout << tc->objVisRot() << endl;

		return true;
}

void ObjectPosReader::setTransMatrix(string calibFile)
{
	char line[1024];
//	string calibFile = "extplugin/graspPlugin/PCL/calibtools/calibmat.txt";

	FILE *ifp0=NULL;
	ifp0 = fopen(calibFile.c_str(),"rb");

	t1 = t2 = t3 = Vector3::Zero();
	R1 = R2 = R3 = Matrix3::Identity();

	if(ifp0 != NULL){
			if(! fgets(line,256,ifp0) )
					printf("result mat: Broken format1 \n");

			int j=0;
			while(1){

				while(fgets(line,256,ifp0) != NULL)
						if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;

				if(line[0] != '4')
						printf("result mat: Broken format1 \n");

				Vector3 P0 = Vector3::Zero();
				Matrix3 R0 = Matrix3::Identity();

				int i=0;
				while(fgets(line,256,ifp0) != NULL){
						if(i<3 && sscanf(line,"%lf%lf%lf%lf",&R0(i,0),&R0(i,1),&R0(i,2),&P0(i)) != 4){
								printf("result mat: Broken format2 %d \n",i);
								fclose(ifp0);
								return;
						}
						i++;
						if(i >= 4) break;
				}
				if(j==0){R1 = R0; t1 = P0;}
				if(j==1){R2 = R0; t2 = P0;}
				if(j==2){R3 = R0; t3 = P0;}
				if(++j == 3) break;;
			}
			fclose(ifp0);
	}
}


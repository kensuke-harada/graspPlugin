// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include "ZmapInterface.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

ZmapInterface::ZmapInterface(){
    return;
}

void ZmapInterface::doReadFromFile() {

		char line[1024];
		Matrix3 R0;
		R0=Matrix3::Identity();
		Vector3 P0(0,0,0);

		PlanBase* tc = PlanBase::instance();
		string calibFile = "extplugin/graspPlugin/ObjectPlacePlanner/data/Tzmap.mat";

		FILE *ifp0=NULL;
		ifp0 = fopen(calibFile.c_str(),"rb");

		if(ifp0 != NULL){
				if(! fgets(line,256,ifp0) ){
						printf("result mat: Broken format1 \n");
				}

				while(fgets(line,256,ifp0) != NULL){
						if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;
				}

				if(line[0] != '4'){
						printf("result mat: Broken format1 \n");
				}

				int i=0;
				while(fgets(line,256,ifp0) != NULL){
						if(sscanf(line,"%lf%lf%lf%lf",&R0(i,0),&R0(i,1),&R0(i,2),&P0(i)) != 4){
								printf("result mat: Broken format2 %d \n",i);
								return;
						}
						i++;
						if(i >= 3) break;
				}

				for(int i=0;i<3;i++){
						P0(i) /= 1000.0; //model m -> mm
				}
		}

		int er;
		er=system("echo \"/tmp/data.rng\" > /tmp/KinectTrigger");
		er=system("cp /tmp/data.rng .");
		er=system("range_move -i data.rng -o data_R.rng -m extplugin/graspPlugin/ObjectPlacePlanner/zmap/tmat-20120626113056.d");
		er=system("extplugin/graspPlugin/ObjectPlacePlanner/zmap/make_zmap -i data_R.rng");

		//char line[1024];
		Matrix3 Rm;
		Vector3 Pm;
		FILE *ifp=NULL;

		ifp = fopen("extplugin/graspPlugin/ObjectPlacePlanner/data/data_cap.mat","rb");

		if(ifp==NULL){
				printf("No data.mat\n");
				return;
		}

		if(! fgets(line,256,ifp) ){
			printf("result mat: Broken format1 \n");
		}

		//while(fgets(line,256,ifp) != NULL){
		//		if(line[0] != '#' && line[0]!=' ' && line[0]!=0x0D && line[0]!=0x0A && line[0]!='\t') break;
		//}

		int cnt=0;
		while(fgets(line,256,ifp) != NULL){
			cout << line << endl;
			Matrix3 Rm;
			Vector3 Pm;
			if(line[0] == '4'){
				int i=0;
				while(fgets(line,256,ifp) != NULL){
					cout << line << endl;
					stringstream sline;
					sline << line;
					sline >> Rm(i,0);
					sline >> Rm(i,1);
					sline >> Rm(i,2);
					sline >> Pm(i);
					i++;
					if(i >= 3) break;
				}
				for(int i=0;i<3;i++){
					Pm(i) /= 1000.0; //model m -> mm
				}
				cout << "cap pos " << Vector3(P0+R0*Pm).transpose() << endl;
				ItemPtr temp= tc->targetObject->bodyItemObject->duplicateAll();
				tc->targetObject->bodyItemObject->parentItem()->addChildItem( temp);
				BodyItem* btemp = (BodyItem*)(temp.get());
                btemp->body()->link(0)->p() = P0+R0*Pm;
                btemp->body()->link(0)->R() = R0*Rm;
				ItemTreeView::mainInstance()->checkItem(btemp,true);
				cnt++;
			}
		}

		return;
}


// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "PlacePlanner.h"


#define m_pi 3.141592

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace grasp::PickAndPlacePlanner;

//#define DEBUG2

PlacePlanner::PlacePlanner()  : 	os (MessageView::mainInstance()->cout() ) {

		clusterIncludeTest = false;
		stabilityTest = false;
		collisionTest = false;
		colTestMargin = 0.001;
		pf = NULL;
		cp = NULL;
}

PlacePlanner::~PlacePlanner() {
}

PlacePlanner* PlacePlanner::instance() {
		static PlacePlanner* instance = new PlacePlanner();
		return instance;
}

bool PlacePlanner::calcPutPos(Vector3& pressPos, const Matrix3& pressOri, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put, bool useClusterCenter){
		string filename = CHOREONOID_TOP_PATH + "extplugin/graspPlugin/ObjectPlacePlanner/data/data_put.mat";
		struct stat st;

		if(stat(filename.c_str(), &st)==0){
				cout << "File data_put.mat exists. Reading target position from this file. " << endl;
				Po_put.clear();
				Ro_put.clear();
				return readPutPos(Po_put, Ro_put);
		}

		if (cp != NULL) delete cp;
		if (pf != NULL) delete pf;
		cp = new CollisionPair();
		pf = new ParameterFileData();

		pf->readEnvClusterParameters();
		cp->setCollisionObj();

#ifdef DEBUG2
		ofstream fout("ofs.m", ios::out | ios::app);
		for(unsigned int i=0; i<pf->envClusters.size(); i++){

				fout << "%Cluster #" << pf->envClusters[i].id << endl;
				fout << "pt=[" << pressPos.transpose() << "];" << endl;
				fout << "cv" << i << "=[";
				for(size_t j=0; j<pf->envClusters[i].convexhullPoints.size(); j++)
						fout << pf->envClusters[i].convexhullPoints[j].transpose() << ";" << endl;
				fout << "];" << endl;
		}
#endif
		int ide=-1;
		for(unsigned int i=0; i<pf->envClusters.size(); i++){
				if(includeClusterBoundary(pressPos, pf->envClusters[i])){
						ide = i;
						break;
				}
		}

		vector<int> idList;
		for(unsigned int i=0; i<pf->objEnvClusters.size(); i++)
				if(pf->objEnvClusters[i].isPuttingCluster)
						idList.push_back(i);

#ifdef EXPO_DEMO
		if(pf->envClusters.size() != 4){
				os << "Error may cause! Remove parameter files except for W4.prm: " << pf->envClusters.size() << endl;
                Po_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p());
				Ro_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude());
				return;
		}

		if(PlanBase::instance()->targetObject->bodyItemObject->name()=="W0"){

				int putPosIndex_old = putPos.Index;
#if 1
				if(putPosIndex_old%4 != 2)
						putPos.Index = 1;
#endif
				if(putPos.IndexSet[0]==0 && putPos.IndexSet[1]==0 && putPos.IndexSet[2]==0 && putPos.IndexSet[3]==0 ){
						for(int i=0; i<4; i++)
								putPos.IndexSet[i]=1;
						os<< "Put Pos database is cleared" << endl;
				}

				do{
						putPos.Index++;
				}while(putPos.IndexSet[putPos.Index%4]==0 );


                Vector3 P = PlanBase::instance()->bodyItemRobot()->body()->link(0)->attitude().transpose()*(PlanBase::instance()->objVisPos() - PlanBase::instance()->bodyItemRobot()->body()->link(0)->p());

				if(P(1) <0.06 && putPos.IndexSet[2]==1 && putPosIndex_old%4 != 2)
						putPos.Index = 2;

				if(putPos.assigned > -1){
						putPos.Index = putPos.assigned;
						putPos.assigned=-2;
				}

				ide = putPos.Index%4;

				putPos.IndexSet[ide] = 0;

				pressPos = pf->envClusters[ide].bbCenter;

		}
		else{
                Po_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p());
				Ro_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude());
				return;
		}
#endif

		Po_put.clear();
		Ro_put.clear();

		if( ide==-1 ){
				cout << "Cannot put object: obj cluster size=" << idList.size() << ", env cluster #=" << ide << endl;
                Po_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p());
				Ro_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude());
				return false;
		}
		else if( idList.size()==0 ){
				cout << "Entering bounding box put mode: obj cluster size=" << idList.size() << ", env cluster #=" << ide << endl;
				calcObjPosFaceBoundingBox(pressPos,pf->envClusters[ide],Po_put,Ro_put);
		}
		else{
				for(unsigned int i=0; i<idList.size(); i++){

						int ido = idList[i];

#ifdef EDGE_FACE_CONTACT
						calcObjPosEdgeFace(pressPos, pf->envClusters[ide], pf->objEnvClusters[ido], Po_put, Ro_put);
#else
						calcObjPosFaceFace(pressPos, pf->envClusters[ide], pf->objEnvClusters[ido], Po_put, Ro_put, useClusterCenter);
#endif
				}
		}

		return true;
}


bool PlacePlanner::readPutPos(vector<Vector3>& Po_put, vector<Matrix3>& Ro_put){

	char line[1024];
	Matrix3 R0;
	R0=Matrix3::Identity();
	Vector3 P0(0,0,0);
	bool ret = false;

	PlanBase* tc = PlanBase::instance();
	string calibFile = CHOREONOID_TOP_PATH + "extplugin/graspPlugin/ObjectPlacePlanner/data/Tzmap.mat";

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
							fclose(ifp0);
							return false;
					}
					i++;
					if(i >= 3) break;
			}

			for(int i=0;i<3;i++){
					P0(i) /= 1000.0; //model m -> mm
			}
			ret = true;
	}
	fclose(ifp0);

	Matrix3 Rm;
	Vector3 Pm;
	FILE *ifp=NULL;

	ifp = fopen(string(CHOREONOID_TOP_PATH + "extplugin/graspPlugin/ObjectPlacePlanner/data/data_put.mat").c_str(),"rb");

	if(ifp==NULL){
			printf("No data.mat\n");
			return ret;
	}

	if(! fgets(line,256,ifp) ){
		printf("result mat: Broken format1 \n");
	}

	while(fgets(line,256,ifp) != NULL){
			if(line[0] == '#' || line[0]==' ' || line[0]==0x0D || line[0]==0x0A || line[0]=='\t') continue;
			if(line[0] != '4'){
					printf("result mat: Broken format1 \n");
					fclose(ifp);
					return false;
			}
			int i=0;

			while(fgets(line,256,ifp) != NULL){
					if(sscanf(line,"%lf%lf%lf%lf",&Rm(i,0),&Rm(i,1),&Rm(i,2),&Pm(i)) != 4){
							printf("result mat: Broken format2 %d \n",i);
							fclose(ifp);
							return false;
					}
					i++;
					if(i >= 3){
							if (fgets(line,256,ifp) == NULL) break;
							break;
					}
			}

			for(int i=0;i<3;i++){
					Pm(i) /= 1000.0; //model m -> mm
			}

			Po_put.push_back(P0+R0*Pm);
			Ro_put.push_back(R0*Rm);
	}
	fclose(ifp);
	return true;

}

bool PlacePlanner::writePutPos(const vector<Vector3>& Po_put, const vector<Matrix3>& Ro_put, bool is_append) {
	char line[1024];
	Matrix3 R0;
	R0=Matrix3::Identity();
	Vector3 P0(0,0,0);

	string calibFile = CHOREONOID_TOP_PATH + "extplugin/graspPlugin/ObjectPlacePlanner/data/Tzmap.mat";

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
							return false;
					}
					i++;
					if(i >= 3) break;
			}

			for(int i=0;i<3;i++){
					P0(i) /= 1000.0; //model m -> mm
			}

			fclose(ifp0);
	} else {
		printf ("cannot open Tzmap.mat\n");
	}

	string dataFile =  CHOREONOID_TOP_PATH + "extplugin/graspPlugin/ObjectPlacePlanner/data/data_put.mat";

	string open_mode = is_append ? "a+b" : "w+b";

	FILE *ifp=NULL;
	ifp = fopen(dataFile.c_str(), open_mode.c_str());

	if (ifp == NULL) {
		printf("cannot open data_put.mat\n");
		return false;
	}

	for (size_t i = 0; i < Po_put.size(); i++) {
		Vector3 Po = R0.transpose() * (Po_put[i] - P0) * 1000;
		Matrix3 Ro = R0.transpose() * Ro_put[i];
		fputs("\n4 4\n", ifp);
		for (int j = 0; j < 3; j++) {
			char outputline[1024] = {'\0'};
			sprintf(outputline, "%lf %lf %lf %lf\n", Ro(j,0), Ro(j,1), Ro(j,2), Po(j));
			fputs(outputline, ifp);
		}
		fputs("0.0 0.0 0.0 1.0\n", ifp);
	}

	fclose(ifp);

	return true;
}


void PlacePlanner::calcIntermediatePutPos(vector<Vector3>& Po_put, vector<Matrix3>& Ro_put, vector<Vector3>& Po_tmp, vector<Matrix3>& Ro_tmp){//Heuristic rule (to replace)


		Vector3 Po = PlanBase::instance()->objVisPos();
		Matrix3 Ro = PlanBase::instance()->objVisRot();

		Vector3 P_tmp = (Po + Po_put[0])*0.5;

		//target point on environment
		int ide=-1;
		for(unsigned int i=0; i<pf->envClusters.size(); i++)
				if(includeClusterBoundary(P_tmp, pf->envClusters[i])){
						ide = i;
						break;
				}

		//target point on object
		vector<int> idList;
		for(unsigned int i=0; i<pf->objEnvClusters.size(); i++)
				if(pf->objEnvClusters[i].isPuttingCluster)
						idList.push_back(i);

		if(idList.size()==0){
				cout << "Set is_putting_cluster parameter in PRM file" << endl;
				return;
		}

		Vector3 envEdge = sort( pf->envClusters[ide].bbEdge );
		Vector3 P_t = pf->envClusters[ide].bbCenter + 0.5*pf->envClusters[ide].normal*pf->envClusters[ide].bbEdge(2);
		Vector3 P_ec = intersectionPoint(Vector3(0.5*(Po + Po_put[0])), pf->envClusters[ide].normal, P_t, pf->envClusters[ide].normal);

		Po_tmp.clear();
		Ro_tmp.clear();

		for(unsigned int i=0; i<idList.size(); i++){

				int ido = idList[i];
				Vector3 objEdge = sort( pf->objEnvClusters[ido].bbEdge );

				if(objEdge(1) < envEdge(1) && objEdge(2) < envEdge(2) ){ //FaceFace
						calcObjPosFaceFace(P_ec, pf->envClusters[ide], pf->objEnvClusters[ido], Po_tmp, Ro_tmp);
						//cout << "Intermediate object " << Po_tmp[i].transpose() << endl; cout << Ro_tmp[i] << endl;
				}
		}

		return;
}

void PlacePlanner::calcObjPosFaceBoundingBox(const Vector3& pos, ClusterParameter& env, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put)
{
	cout << "FaceBoundingBox" << endl;

	TargetObject* trgt_object = PlanBase::instance()->targetObject;
	Vector3 edge,center,com;
	Matrix3 Rot;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	PlanBase::instance()->calcBoundingBox(trgt_object->bodyItemObject->body()->link(0)->coldetModel(),edge,center,com,Rot);
#else
	PlanBase::instance()->calcBoundingBox(trgt_object->bodyItemObject->body()->link(0)->collisionShape(),edge,center,com,Rot);
#endif
    Vector3 Po_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p();
	Matrix3 Ro_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude();

	Vector3 envEdge=env.bbEdge;


	Matrix3 E_e = v3(env.tangent, cross(env.normal, env.tangent), env.normal);

	vector<Vector3> Po;
	vector<Matrix3> Ro;

	vector<rotatedBBox> rbb;

	vector <Matrix3> R;

	Vector3 x = Vector3(1,0,0);
	Vector3 y = Vector3(0,1,0);
	Vector3 z = Vector3(0,0,1);

	for(int i=0;i<6;i++) rbb.push_back(rotatedBBox());

	rbb[0].R = v3(x,y,z);    rbb[0].perpendicular = 0.5*edge(2); rbb[0].base_area = edge(0)*edge(1);
	rbb[1].R = v3(-y,-x,-z); rbb[1].perpendicular = 0.5*edge(2); rbb[1].base_area = edge(0)*edge(1);
	rbb[2].R = v3(z,x,y);    rbb[2].perpendicular = 0.5*edge(0); rbb[2].base_area = edge(1)*edge(2);
	rbb[3].R = v3(-z,-y,-x); rbb[3].perpendicular = 0.5*edge(0); rbb[3].base_area = edge(1)*edge(2);
	rbb[4].R = v3(y,z,x);    rbb[4].perpendicular = 0.5*edge(1); rbb[4].base_area = edge(2)*edge(0);
	rbb[5].R = v3(-x,-z,-y); rbb[5].perpendicular = 0.5*edge(1); rbb[5].base_area = edge(2)*edge(0);

	std::sort(rbb.begin(),rbb.end(),rotatedBBox::Greater);

	for(int i=0;i<6;i++){
		Matrix3 tmp_R = rbb[i].R;
		for(int j=0;j<4;j++){
			tmp_R = v3(-y,x,z) * tmp_R;
			Ro.push_back(E_e*tmp_R*Rot.transpose());
			Po.push_back(pos - E_e*(Vector3(0,0,-rbb[i].perpendicular) + tmp_R*Rot.transpose()* center));
		}
	}

	for(size_t i=0; i<Po.size(); i++){
		bool inc=true, stb=true, col=false;

		if(collisionTest)
			col = objEnvCollision(Po[i]+ colTestMargin*env.normal, Ro[i]);
		if(col) os << "collision" << endl;
		if(col) continue;

		Po_put.push_back(Po[i]);
		Ro_put.push_back(Ro[i]);
	}

	setObjectPose(Po_cur, Ro_cur);
}

void PlacePlanner::calcObjPosFaceFace(const Vector3& pos, ClusterParameter& env, ClusterParameter& obj, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put, bool useClusterCenter)
{
		cout << "FaceFace" << endl;

        Vector3 Po_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p();
		Matrix3 Ro_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude();

		bool convex=false;
		if(env.Convexity==ClusterData::CONVEX   && obj.Convexity==ClusterData::CONCAVE) convex=true;
		if(obj.Convexity==ClusterData::CONVEX   && env.Convexity==ClusterData::CONCAVE) convex=true;
		if(obj.Convexity==ClusterData::TABLELEG && env.Convexity==ClusterData::CONCAVE) convex=true;

		Vector3 envEdge=env.bbEdge;
		Vector3 objEdge=obj.bbEdge;

		Matrix3 E_e = v3(env.tangent, cross(env.normal, env.tangent), env.normal);
		vector<Vector3> Po;
		vector<Matrix3> Ro, oE_o;

		Vector3 P_ec = pos;
		if(((envEdge(0) < 2*objEdge(0) && envEdge(1) < 2*objEdge(1)) || (envEdge(0) < 2*objEdge(1) && envEdge(1) < 2*objEdge(0))) && useClusterCenter)
				P_ec = env.bbCenter + 0.5*env.normal*envEdge(2);

#define BOUNDINGBOX_PUT
#ifdef BOUNDINGBOX_PUT
		if(convex && (((envEdge[0] < envEdge[1]) && (objEdge[0] < objEdge[1])) || ((envEdge[0] >= envEdge[1]) && (objEdge[0] >= objEdge[1])))){
				oE_o.push_back( v3( obj.tangent, -unit(cross(obj.normal, obj.tangent)), -obj.normal) );
				oE_o.push_back( v3(-obj.tangent,  unit(cross(obj.normal, obj.tangent)), -obj.normal) );
		}
		else if(convex && (((envEdge[0] < envEdge[1]) && (objEdge[0] >= objEdge[1])) || ((envEdge[0] >= envEdge[1]) && (objEdge[0] < objEdge[1])))){
				oE_o.push_back( v3( unit(cross(obj.normal, obj.tangent)),  obj.tangent, -obj.normal) );
				oE_o.push_back( v3(-unit(cross(obj.normal, obj.tangent)), -obj.tangent, -obj.normal) );
		}
		else {
			oE_o.push_back( v3( unit(cross(obj.normal, obj.tangent)),  obj.tangent, -obj.normal) );
			oE_o.push_back( v3( obj.tangent, -unit(cross(obj.normal, obj.tangent)), -obj.normal) );
			oE_o.push_back( v3(-unit(cross(obj.normal, obj.tangent)), -obj.tangent, -obj.normal) );
			oE_o.push_back( v3(-obj.tangent,  unit(cross(obj.normal, obj.tangent)), -obj.normal) );
		}

		for(size_t i=0; i<oE_o.size(); i++){
				Ro.push_back( E_e*oE_o[i].transpose() );
				Po.push_back( P_ec - Ro[i]*(obj.bbCenter + 0.5*obj.normal*objEdge(2)) );
		}
#else
		double d_angle = m_pi/4.0;
		oE_o.push_back( v3( unit(cross(obj.normal, obj.tangent)),  obj.tangent, -obj.normal) );
		Matrix3 R_base = E_e*oE_o[0].transpose();
		for(double angle=0.0; angle<2*m_pi; angle+= d_angle){
			Ro.push_back( rodrigues(env.normal, angle)*R_base );
			Po.push_back( P_ec - Ro.back()*(obj.bbCenter + 0.5*obj.normal*objEdge(2)) );
		}
#endif

		for(size_t i=0; i<Po.size(); i++){
				bool inc=true, stb=true, col=false;

				if(clusterIncludeTest && convex)
					inc = includeClusterBoundary(obj, Po[i], Ro[i], env, Vector3::Zero(), Matrix3::Identity());
				if(!inc) continue;

				if(stabilityTest)
					stb = isGravityStable(obj, Po[i], Ro[i], env, Vector3::Zero(), Matrix3::Identity());
				if(!stb) continue;

				if(collisionTest)
					col = objEnvCollision(Po[i]+ colTestMargin*env.normal, Ro[i]);
				if(col) continue;

#ifdef DEBUG2
				os << "inclusionTest:" << inc <<"/"<<convex << ", stabilityTest:" << stb << ", collisionTest " << !col << ", tolerance " << PlanBase::instance()->tolerance << endl;
#endif

				Po_put.push_back(Po[i]);
				Ro_put.push_back(Ro[i]);
		}

		setObjectPose(Po_cur, Ro_cur);
}


bool PlacePlanner::searchPutPos(Vector3& pressPos, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put) {

	cp = new CollisionPair();
	pf = new ParameterFileData();

	pf->readEnvClusterParameters();
	cp->setCollisionObj();

	int ide=-1;
	for(unsigned int i=0; i<pf->envClusters.size(); i++){
		if(includeClusterBoundary(pressPos, pf->envClusters[i])){
			ide = i;
			break;
		}
	}

	vector<int> idList;

	Po_put.clear();
	Ro_put.clear();

	int grid_row = 9;
	int max_candidate = 1;

	for(unsigned int i=0; i<pf->objEnvClusters.size(); i++)
		if(pf->objEnvClusters[i].isPuttingCluster)
			idList.push_back(i);

		if( idList.size()==0 ){
				cout << "Entering bounding box put mode: obj cluster size=" << idList.size() << ", env cluster #=" << ide << endl;
				searchObjPosFaceBoundingBox(pressPos,pf->envClusters[ide],Po_put,Ro_put, grid_row, max_candidate);
		}
		else{
				for(unsigned int i=0; i<idList.size(); i++){

						int ido = idList[i];

#ifdef EDGE_FACE_CONTACT
						cout << "no implementation" << endl;
#else
						searchObjPosFaceFace(pf->envClusters[ide], pf->objEnvClusters[ido], Po_put, Ro_put, grid_row, max_candidate);
#endif
				}
		}
		writePutPos(Po_put, Ro_put);
		delete cp;
		delete pf;
		return true;
}

void PlacePlanner::searchObjPosFaceBoundingBox(const Vector3& pos, ClusterParameter& env, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put, int grid_row, int maxnum_candidate) {
	cout << "FaceBoundingBox" << endl;

	TargetObject* trgt_object = PlanBase::instance()->targetObject;
	Vector3 edge,center,com;
	Matrix3 Rot;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
	PlanBase::instance()->calcBoundingBox(trgt_object->bodyItemObject->body()->link(0)->coldetModel(),edge,center,com,Rot);
#else
	PlanBase::instance()->calcBoundingBox(trgt_object->bodyItemObject->body()->link(0)->collisionShape(),edge,center,com,Rot);
#endif
    Vector3 Po_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p();
	Matrix3 Ro_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude();

	Vector3 envEdge=env.bbEdge;


	Matrix3 E_e = v3(env.tangent, cross(env.normal, env.tangent), env.normal);

	vector<Vector3> Po;
	vector<Matrix3> Ro;

	vector<rotatedBBox> rbb;

	vector <Matrix3> R;

	Vector3 x = Vector3(1,0,0);
	Vector3 y = Vector3(0,1,0);
	Vector3 z = Vector3(0,0,1);

	for(int i=0;i<6;i++) rbb.push_back(rotatedBBox());

	rbb[0].R = v3(x,y,z);    rbb[0].perpendicular = 0.5*edge(2); rbb[0].base_area = edge(0)*edge(1);
	rbb[1].R = v3(-y,-x,-z); rbb[1].perpendicular = 0.5*edge(2); rbb[1].base_area = edge(0)*edge(1);
	rbb[2].R = v3(z,x,y);    rbb[2].perpendicular = 0.5*edge(0); rbb[2].base_area = edge(1)*edge(2);
	rbb[3].R = v3(-z,-y,-x); rbb[3].perpendicular = 0.5*edge(0); rbb[3].base_area = edge(1)*edge(2);
	rbb[4].R = v3(y,z,x);    rbb[4].perpendicular = 0.5*edge(1); rbb[4].base_area = edge(2)*edge(0);
	rbb[5].R = v3(-x,-z,-y); rbb[5].perpendicular = 0.5*edge(1); rbb[5].base_area = edge(2)*edge(0);

	std::sort(rbb.begin(),rbb.end(),rotatedBBox::Greater);

	vector<int> m, n;
	obtainSpiralSearchPath(grid_row, m, n);

	for(int i=0;i<6;i++){
		double d_angle = m_pi/4.0;
		vector<Matrix3> Ro_tmp;
		for (double angle=0.0; angle<2*m_pi; angle+= d_angle){
			Ro_tmp.push_back( E_e*rodrigues(z, angle) * rbb[i].R*Rot.transpose() );
		}
		for (size_t l = 0; l < Ro_tmp.size(); l++) {
			int candidate_num = 0;
			for (size_t j = 0; j < m.size(); j++) {
				Vector3 P_e = env.bbCenter + 0.5*env.normal*envEdge(2) + 0.5*env.tangent*envEdge(0)*m[j]/double(grid_row/2) + 0.5*unit(cross(env.normal, env.tangent))*envEdge(1)*n[j]/double(grid_row/2);
				Vector3 Po_tmp = P_e + E_e*Vector3(0,0,rbb[i].perpendicular) - Ro_tmp[l] * center;

				bool inc=true, col=false;

				if(clusterIncludeTest){
					Vector3 p_tmp = Po_tmp;
					inc = includeClusterBoundary(p_tmp, env);
				}
				if(!inc) continue;

				if(collisionTest)
					col = objEnvCollision(Po_tmp+ colTestMargin*env.normal, Ro_tmp[l]);
				if(col) continue;

				Ro_put.push_back(Ro_tmp[l]);
				Po_put.push_back(Po_tmp);
				candidate_num++;
				if (maxnum_candidate <= candidate_num) break;
			}
		}
	}

	setObjectPose(Po_cur, Ro_cur);
}

void PlacePlanner::searchObjPosFaceFace(ClusterParameter& env, ClusterParameter& obj, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put, int grid_row, int maxnum_candidate) {
	Vector3 Po_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p();
	Matrix3 Ro_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude();
	bool convex=false;
	if(env.Convexity==ClusterData::CONVEX   && obj.Convexity==ClusterData::CONCAVE) convex=true;
	if(obj.Convexity==ClusterData::CONVEX   && env.Convexity==ClusterData::CONCAVE) convex=true;
	if(obj.Convexity==ClusterData::TABLELEG && env.Convexity==ClusterData::CONCAVE) convex=true;

	Vector3 envEdge=env.bbEdge;
	Vector3 objEdge=obj.bbEdge;

	Matrix3 E_e = v3(env.tangent, cross(env.normal, env.tangent), env.normal);
	vector<Matrix3> Ro, oE_o;

	Vector3 P_ec = env.bbCenter + 0.5*env.normal*envEdge(2);

	vector<int> m, n;
	obtainSpiralSearchPath(grid_row, m, n);

	double d_angle = m_pi/4.0;
	oE_o.push_back( v3( unit(cross(obj.normal, obj.tangent)),  obj.tangent, -obj.normal) );
	Matrix3 R_base = E_e*oE_o[0].transpose();
	for (double angle=0.0; angle<2*m_pi; angle+= d_angle){
		Ro.push_back( rodrigues(env.normal, angle)*R_base );
	}

	for (size_t i = 0; i < Ro.size(); i++) {
		for (size_t j = 0; j < m.size(); j++) {
			int candidate_num = 0;
			Vector3 P_e = env.bbCenter + 0.5*env.normal*envEdge(2) + 0.5*env.tangent*envEdge(0)*m[j]/double(grid_row/2) + 0.5*unit(cross(env.normal, env.tangent))*envEdge(1)*n[j]/double(grid_row/2);
			Matrix3 Ro_tmp = Ro[i];
			Vector3 Po_tmp = P_e - Ro_tmp * (obj.bbCenter + 0.5*obj.normal*objEdge(2));

			bool inc=true, stb=true, col=false;

			if(clusterIncludeTest && convex)
				inc = includeClusterBoundary(obj, Po_tmp, Ro_tmp, env, Vector3::Zero(), Matrix3::Identity());
			if(!inc) continue;

			if(stabilityTest)
				stb = isGravityStable(obj, Po_tmp, Ro_tmp, env, Vector3::Zero(), Matrix3::Identity());
			if(!stb) continue;

			if(collisionTest)
				col = objEnvCollision(Po_tmp+ colTestMargin*env.normal, Ro_tmp);
			if(col) continue;

			Po_put.push_back(Po_tmp);
			Ro_put.push_back(Ro_tmp);
			candidate_num++;
			if (maxnum_candidate <= candidate_num) break;
		}
	}

	setObjectPose(Po_cur, Ro_cur);
}

bool PlacePlanner::objEnvCollision(const Vector3& P, const Matrix3& R){

		setObjectPose(P,R);

		bool ret = cp->isColliding();

		return ret;
}

void PlacePlanner::setObjectPose(const Vector3& P, const Matrix3& R){

        PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p() = P;
		PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->setAttitude( R);
		PlanBase::instance()->targetObject->bodyItemObject->body()->calcForwardKinematics();
		PlanBase::instance()->targetObject->bodyItemObject->notifyKinematicStateChange();

#ifdef DEBUG2
		PlanBase::instance()->flush();
		usleep(10000000);
#endif
}

void PlacePlanner::calcObjPosEdgeFace(const Vector3& pos, ClusterParameter& env, ClusterParameter& obj, vector<Vector3>& Po_put, vector<Matrix3>& Ro_put)
{
		cout << "EdgeFace" << endl;

        Vector3 Po_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p();
		Matrix3 Ro_cur = PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude();

		Po_put.clear();
		Ro_put.clear();

		calcObjPosFaceFace(pos, env, obj, Po_put, Ro_put);

		vector<Matrix3> Ro_;
		for(unsigned int i=0; i<Ro_put.size(); i++)
				Ro_.push_back(Ro_put[i]);

		Po_put.clear();
		Ro_put.clear();

		int numPoints = obj.convexhullPoints.size();
		vector<double> edgeLength;
		vector<int> o;

		for(int j=0; j<numPoints; j++){
				edgeLength.push_back( -norm2(obj.convexhullPoints[(j+1)%numPoints] - obj.convexhullPoints[j]) );
				o.push_back(j);
		}

		sort_by(o, edgeLength);

		for(int j=0; j<numPoints; j++){

				if(fabs(edgeLength[j]) > sqrt(env.bbEdge[0]*env.bbEdge[0] + env.bbEdge[1]*env.bbEdge[1])) continue;

				int idb = o[j];

				Vector3 r = unit(obj.convexhullPoints[(idb+1)%obj.convexhullPoints.size()] - obj.convexhullPoints[idb]);
				Vector3 oPc = (obj.convexhullPoints[(idb+1)%obj.convexhullPoints.size()] + obj.convexhullPoints[idb])/2.0;

				bool hasAnswer = false;
				int den = 1;
				for(int i=1; i<5; i++){
						den *= 2;

						for(int num=1; num<den; num+=2){

								double phi = -M_PI/2.0 + M_PI*num/(double)den;
								Matrix3 Ro = rodrigues(r, phi)*Ro_[0] ;



								if( ! objEnvCollision(env.bbCenter - Ro*oPc + 0.0003*env.normal, Ro)){
										//*
										double high=phi, low=0.0;
										bool collision = false;
										while(fabs(low-high) > 0.01 || collision){
												double mid = (low+high)/2.0;

												Ro = rodrigues(r, mid)*Ro_[0] ;

												if( objEnvCollision(env.bbCenter - Ro*oPc + 0.0003*env.normal, Ro)){
														low = mid;
														collision = true;
												}
												else{
														high = mid;
														collision = false;
												}
										}
										//*/
										hasAnswer = true;
										break;
								}
						}
						if(hasAnswer)
								break;
				}

				if(hasAnswer){
                        Po_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->p());
						Ro_put.push_back(PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->attitude());
				}

				setObjectPose(Po_cur, Ro_cur);
		}

}

bool PlacePlanner::lineIntersectToCluster(const Vector3& p, const Vector3& n, Vector3& p_out, const vector<Vector3>& Pset, bool ccw){

		int size = Pset.size();

		if(size<3)
			return false;

#define CROSSING_NUMBER_ALGO
#ifdef CROSSING_NUMBER_ALGO

		Vector3 x(1,0,0);
		Vector3 y = unit(cross(n,x));
		x = unit(cross(y,n));

		Vector3 p_(dot(p,x), dot(p,y), 0);
		vector<Vector3> Pset_;
		for(int i=0; i<size; i++)
				Pset_.push_back(Vector3(dot(Pset[i],x), dot(Pset[i],y), 0));

		int cn = 0;
		for(int i=0; i<size; i++){
				Vector3 ini = Pset_[i];
				Vector3 fin = Pset_[(i+1)%size];

				if(dot(Vector3(0,1,0), Vector3(fin-ini)) == 0 && fin(1)==p_(1))
						cout << "Error in line IntersectToCluster " << endl; //To be added

				Vector3 po = ini + (p_(1) - ini(1))/(fin(1)-ini(1))*(fin - ini);

				if(fin(1)>ini(1)){
						if(p_(1) >= ini(1) && p_(1) < fin(1)  && Vector3(po - p_)(0) >= 0 )
								cn++;
				}
				else{
						if(p_(1) >= fin(1) && p_(1) < ini(1) && Vector3(po - p)(0) >= 0)
								cn++;
				}
		}

		p_out = p + dot(Vector3(Pset[0]-p), n)*n;

		if(cn%2==1)return true;
		else       return false;

#else
		Vector3 r0, p0, p1, p2, pn;
		Vector3 x = cross(Vector3(0,1,0), n);

		vector<double> dist;
		for(size_t i=0; i<Pset.size(); i++){
				r0 = p + dot(n,(Pset[i]-p))*n;
				p1 = Pset[i];
				p2 = Pset[(i+1)%Pset.size()];
				pn = normalPoint(r0, p1, unit(p2-p1));
				double t = dot(pn-p1, x)/dot(p2-p1, x);
				if(t>=0 && t<=1)
						dist.push_back(norm2(r0-pn));
				else
						dist.push_back(1.0e10);
		}
		int id = argmin(dist);
		double d = vmin(dist);

		dist.clear();
		for(size_t i=0; i<Pset.size(); i++){
				r0 = p + dot(n,(Pset[i]-p))*n;
				dist.push_back(norm2(r0 - Pset[i]));
		}
		if(vmin(dist) < d)
				id = argmin(dist);

		r0 = p + dot(n,(Pset[id]-p))*n;
		p0 = Pset[(Pset.size() + id -1 )%Pset.size()];
		p1 = Pset[id];
		p2 = Pset[(id+1)%Pset.size()];
		p_out = r0;

		double dir=-1.0;
		if(ccw) dir=1.0;
		double s1 = dot(cross(p2-p1, r0-p1),n)*dir;
		double s2 = dot(cross(p1-p0, r0-p0),n)*dir;

		if(s1>=0 && s2>=0) return true;
		else if(s1<0 && s2<0) return false;
		else{
				Vector3 r1 = p0 + 0.8*(r0-p0);
				Vector3 r2 = p2 + 0.8*(r0-p2);
				double s3 = dot(cross(p2-p1, r1-p1),n)*dir;
				double s4 = dot(cross(p1-p0, r2-p0),n)*dir;

				if(s3>=0 || s4>=0) return false;
				else return true;
		}
#endif
}

int PlacePlanner::calcOuterBoundary(vector<vector<Vector3> >& boundary){

		vector<double> maxEdge;
		for(size_t i=0; i<boundary.size(); i++){
				Vector3 av = average(boundary[i]);

				vector<double> edge;
				for(unsigned int j=0; j<boundary[i].size(); j++)
						edge.push_back(norm2(boundary[i][j]-av));

				maxEdge.push_back(vmax(edge));
		}

		return argmax(maxEdge);
}

void PlacePlanner::calcCommonArea(const vector<Vector3>& s1, const vector<Vector3>& s2, list<Vector3>& boundary){

		Vector3 z( 0,0,1 );
		Vector3 Pout;

		boundary.clear();

		for(size_t i=0; i<s2.size(); i++)
				if(lineIntersectToCluster(s2[i], z, Pout, s1, true))
						boundary.push_back(s2[i]);

		for(size_t i=0; i<s1.size(); i++)
				if(lineIntersectToCluster(s1[i], z, Pout, s2, true))
						boundary.push_back(s1[i]);

		for(size_t i=0; i<s1.size(); i++){
				for(size_t j=0; j<s2.size(); j++){

						if(twoSegmentsIntersect(s1[i], s1[(i+1)%s1.size()], s2[j], s2[(j+1)%s2.size()], z, Pout))
								boundary.push_back(Pout);
				}
		}
}

void PlacePlanner::excludeHole(const vector<Vector3>& hole, const vector<Vector3>& outer, list<Vector3>& boundary){

		Vector3 z( 0,0,1 );
		Vector3 Pout;

		vector<Vector3> p;
		for(list<Vector3>::iterator jt=boundary.begin(); jt!=boundary.end(); ++jt)
				p.push_back(*jt);

		boundary.clear();

		vector<Vector3> h;
		for(size_t i=0; i<hole.size(); i++)
				h.push_back(hole[hole.size()-i-1]);

		for(size_t i=0; i<p.size(); i++)
				if( ! lineIntersectToCluster(p[i], z, Pout, h, true))
						boundary.push_back(p[i]);

		for(size_t i=0; i<outer.size(); i++){
				for(size_t j=0; j<h.size(); j++){

						if(twoSegmentsIntersect(outer[i], outer[(i+1)%p.size()], h[j], h[(j+1)%h.size()], z, Pout))
								boundary.push_back(Pout);
				}
		}
}

void PlacePlanner::calcPlanarConvexHull(list<Vector3>& boundary, vector<Vector3>& convexhull){

		vector<double> pt, pt_out;
		for(list<Vector3>::iterator jt=boundary.begin(); jt!=boundary.end(); ++jt){
				pt.push_back((*jt)(0));
				pt.push_back((*jt)(1));
		}

		ConvexAnalysis ca;
		ca.calcConvexHull(2, pt, pt_out, false);

		for(size_t i=0; i<pt_out.size()/2; i++)
				convexhull.push_back(Vector3(pt_out[2*i], pt_out[2*i+1], 0));


		Vector3 cog = average(convexhull);

		vector<double> angle;
		for(size_t k=0; k<convexhull.size(); k++){

				Vector3 p1 = convexhull[k];

				double cosq = dot(Vector3(1,0,0), p1-cog)/norm2(p1-cog);
				double sinq = dot(Vector3(0,1,0), p1-cog)/norm2(p1-cog);
				angle.push_back(atan2(sinq, cosq) );
		}
		sort_by(convexhull, angle);

}

void writePoints(const vector<vector<Vector3> >& p1, const vector<vector<Vector3> >& p2, const vector<Vector3>& p3, Vector3 p4)
{
	ofstream fout;
	fout.open("stabilitytest.m");

	for(size_t i=0; i<p1.size(); i++){
		fout << "p1" << i << "=[" << endl;
		for(size_t j=0; j<p1[i].size(); j++)
			fout << p1[i][j].transpose() << endl;
		fout << "];" << endl;
		fout << "[m,n] = size(p1" << i << ");" << endl;
		fout << "p1" << i << "(m+1,:)= p1" << i << "(1,:);" << endl;
	}
	for(size_t i=0; i<p2.size(); i++){
		fout << "p2" << i << "=[" << endl;
		for(size_t j=0; j<p2[i].size(); j++)
			fout << p2[i][j].transpose() << endl;
		fout << "];" << endl;
		fout << "[m,n] = size(p2" << i << ");" << endl;
		fout << "p2" << i << "(m+1,:)= p2" << i << "(1,:);" << endl;
	}
	fout << "p3=[" << endl;
	for(size_t j=0; j<p3.size(); j++)
		fout << p3[j].transpose() << endl;
	fout << "];" << endl;
	fout << "[m,n] = size(p3);" << endl;
	fout << "p3(m+1,:)= p3(1,:);" << endl;
	fout << "p4=[" << endl;
	fout << p4.transpose() << endl;
	fout << "];" << endl;

	fout << "plot(";
	for(size_t i=0; i<p1.size(); i++)
		fout << "p1" << i << "(:,1), p1" << i << "(:,2), ";

	for(size_t i=0; i<p2.size(); i++)
		fout << "p2" << i << "(:,1), p2" << i << "(:,2), ";

	fout << "p3(:,1), p3(:,2), p4(:,1), p4(:,2),\"*\")" << endl;
}

bool PlacePlanner::isGravityStable(ClusterParameter& obj, const Vector3& Po, const Matrix3& Ro, ClusterParameter& env, const Vector3& Pe, const Matrix3& Re)
{
        Vector3 Pcm = Po + Ro * PlanBase::instance()->targetObject->bodyItemObject->body()->link(0)->c();
		Vector3 Porg = Pe + Re*env.bbCenter;

		if(dot(Pcm-Porg, Vector3(0,0,1)) < 0) {
				cout << "CoM is below contact point " << endl;
				return true;
		}

		vector<vector<Vector3> > objBoundary;
		if(obj.boundaryPointList.size()>0){
				for(size_t i=0; i<obj.boundaryPointList.size(); i++){
						vector<Vector3> b;
						for(size_t j=0; j<obj.boundaryPointList[i].size(); j++)
								b.push_back(Po + Ro*obj.boundaryPointList[i][obj.boundaryPointList[i].size()-1 - j] );
						objBoundary.push_back(b);
				}
		}
		else{
				vector<Vector3> b;
				for(size_t j=0; j<obj.convexhullPoints.size(); j++)
						b.push_back(Po + Ro*obj.convexhullPoints[obj.convexhullPoints.size()-1 - j] );
				objBoundary.push_back(b);
		}

		vector<vector<Vector3> > envBoundary;
		if(env.boundaryPointList.size()>0){
				for(size_t i=0; i<env.boundaryPointList.size(); i++){
						vector<Vector3> b;
						for(size_t j=0; j<env.boundaryPointList[i].size(); j++)
								b.push_back(Pe + Re*env.boundaryPointList[i][j] );
						envBoundary.push_back(b);
				}
		}
		else{
				vector<Vector3> b;
				for(size_t j=0; j<env.convexhullPoints.size(); j++)
						b.push_back(Pe + Re*env.convexhullPoints[j] );
				envBoundary.push_back(b);
		}

		int objArg = calcOuterBoundary(objBoundary);
		int envArg = calcOuterBoundary(envBoundary);

		list<Vector3> boundary;
		Vector3 z( 0,0,1 );
		Vector3 Pout;

		calcCommonArea(objBoundary[objArg], envBoundary[envArg], boundary);

		/*
		for(list<Vector3>::iterator it=boundary.begin(); it != boundary.end() ; it++)
				cout << (*it).transpose() << endl;
		*/

		for(size_t i=0; i<envBoundary.size(); i++){

				if((int)i==envArg) continue;

				excludeHole(envBoundary[i], objBoundary[objArg], boundary);
		}

		for(size_t i=0; i<objBoundary.size(); i++){

				if((int)i==objArg) continue;

				excludeHole(objBoundary[i], envBoundary[envArg], boundary);
		}

		vector<Vector3> convexhull;
		calcPlanarConvexHull(boundary, convexhull);


#ifdef DEBUG2
		writePoints(objBoundary, envBoundary, convexhull, Pcm);
#endif

		if(lineIntersectToCluster(Pcm, Vector3(0,0,1), Pout, convexhull, true)){
				os << "Stability test passed." << endl;
				return true;
		}
		else{
				os << "Stability test failed." << endl;
				return false;
		}
}

bool PlacePlanner::includeClusterBoundary(ClusterParameter& c1, const cnoid::Vector3& p1, const cnoid::Matrix3& R1,ClusterParameter& c2, const cnoid::Vector3& p2, const cnoid::Matrix3& R2){


		vector<Vector3> b1, b2;
		if(c1.boundaryPointList.size()>0){
				int o1 = calcOuterBoundary(c1.boundaryPointList);
				for(unsigned int i=0; i<c1.boundaryPointList[o1].size(); i++)
						b1.push_back(p1 + R1*c1.boundaryPointList[o1][i] );
		}
		else
				for(unsigned int i=0; i<c1.convexhullPoints.size(); i++)
						b1.push_back(p1 + R1*c1.convexhullPoints[i] );

		if(c2.boundaryPointList.size()>0){
				int o2 = calcOuterBoundary(c2.boundaryPointList);
				for(unsigned int i=0; i<c2.boundaryPointList[o2].size(); i++)
						b2.push_back(p2 + R2*c2.boundaryPointList[o2][i] );
		}
		else
				for(unsigned int i=0; i<c2.convexhullPoints.size(); i++)
						b2.push_back(p2 + R2*c2.convexhullPoints[i] );


		Vector3 p_out;
		bool incl=true;
		for(size_t i=0; i<b1.size(); i++)
				if(!lineIntersectToCluster(b1[i], Vector3(R1*c1.normal), p_out, b2, true)){
						incl = false;
						break;
				}

		if(!incl){
				incl=true;
				for(size_t i=0; i<b2.size(); i++)
						if(!lineIntersectToCluster(b2[i], Vector3(R2*c2.normal), p_out, b1, true)){
								incl = false;
								break;
						}
		}

		if(incl)	os << "Inclusion test passed." << endl;
		else		os << "Inclusion test failer." << endl;

		return incl;
}

bool PlacePlanner::includeClusterBoundary(Vector3& pos, ClusterParameter& c1){

		double eps = 0.0001;
		Vector3 x = c1.tangent;
		Vector3 y = cross(c1.normal, c1.tangent);
		Vector3 z = c1.normal;
		Vector3 e = c1.bbEdge;
		Vector3 c = c1.bbCenter;

		if(dot(pos,x)<dot(c,x)-e(0)-eps || dot(pos,x)>dot(c,x)+e(0)+eps ) return false;
		if(dot(pos,y)<dot(c,y)-e(1)-eps || dot(pos,y)>dot(c,y)+e(1)+eps ) return false;
		//if(dot(pos,z)<dot(c,z)-e(2)-eps || dot(pos,z)>dot(c,z)+e(2)+eps ) return false;

		Vector3 p_out;

		if(c1.boundaryPointList.size()>0){
				int o1 = calcOuterBoundary(c1.boundaryPointList);

				if(!lineIntersectToCluster(pos, z, p_out, c1.boundaryPointList[o1], true))
						return false;
		}
		else
				if(!lineIntersectToCluster(pos, z, p_out, c1.convexhullPoints, true))
						return false;

		pos = p_out;

		return true;
}

void PlacePlanner::obtainSpiralSearchPath(int row, vector<int>& x, vector<int>& y) const {
	x.clear();
	y.clear();

	int cur_x = 0;
	int cur_y = 0;

	int state = 0;

	int cur_move = 0;
	int max_move = 1;

	while (cur_move != row) {
		x.push_back(cur_x);
		y.push_back(cur_y);

		if (max_move % 2 == 1) {
			if (state == 0) {
				cur_x++;
				cur_move++;
				if (cur_move == max_move) {
					state = 1;
					cur_move = 0;
				}
			} else {
				cur_y++;
				cur_move++;
				if (cur_move == max_move) {
					state = 0;
					max_move++;
					cur_move = 0;
				}
			}
		} else {
			if (state == 0) {
				cur_x--;
				cur_move++;
				if (cur_move == max_move) {
					state = 1;
					cur_move = 0;
				}
			} else {
				cur_y--;
				cur_move++;
				if (cur_move == max_move) {
					state = 0;
					max_move++;
					cur_move = 0;
				}
			}
		}
	}
}

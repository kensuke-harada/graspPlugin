// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-
/**
  c) Kensuke Harada (AIST)
 */

#include "ClusterParameter.h"

//#define DEBUG
#include <cnoid/ExecutablePath>
#define PLUGIN_PATH cnoid::executableTopDirectory() + string("/")

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#define NODE_TYPE type()
#else
#define NODE_TYPE nodeType()
#endif

using namespace std;
using namespace cnoid;
using namespace grasp;
//using namespace grasp::PickAndPlacePlanner;

ParameterFileData* ParameterFileData::instance() {
		static ParameterFileData* instance = new ParameterFileData();
		return instance;
}

void ParameterFileData::calcIDPair(int intention)
{
		idPairs.clear();

		for(unsigned int id0=0; id0<objClusters.size(); id0++){

				if(objClusters[id0].intention != intention) continue;

				for(unsigned int j=0; j<objClusters[id0].idPair.size(); j++){

						int id1=-1;
						for(unsigned int k=0; k<objClusters.size(); k++){

								if(objClusters[k].id != objClusters[id0].idPair[j]) continue;
								if(objClusters[k].intention != objClusters[id0].intention) continue;
								id1=k;
								break;
						}

						if(id1 == -1) continue;

						vector<int> id;
						id.push_back(id0);
						id.push_back(id1);
						idPairs.push_back(id);
				}
		}
}

void ParameterFileData::readObjClusterParameters()
{
		graspPostureCandidate.clear();
		for(int i=0; i<8; i++)
				graspPostureCandidate.push_back(i);

		struct stat st;

		string objectpath = PLUGIN_PATH + "extplugin/graspPlugin/PickAndPlacePlanner/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + ".prm";

		objClusters.clear();
		if(stat(objectpath.c_str(), &st)==0)
				readPRMFile(objectpath.c_str(), objClusters);
		else
				readObjYamlFile(PlanBase::instance()->targetObject->bodyItemObject, objClusters);

		string gripperpath = PLUGIN_PATH + "extplugin/graspPlugin/PickAndPlacePlanner/PRM/" + PlanBase::instance()->bodyItemRobot()->name() + ".prm";

		handClusters.clear();
		if(stat(gripperpath.c_str(), &st)==0)
				readPRMFile(gripperpath.c_str(), handClusters);
		else
				readObjYamlFile(PlanBase::instance()->bodyItemRobot(), handClusters);

}

void ParameterFileData::readEnvClusterParameters()
{
		struct stat st;

		string objenvpath = PLUGIN_PATH + "extplugin/graspPlugin/PickAndPlacePlanner/PRM/" + PlanBase::instance()->targetObject->bodyItemObject->name() + "env.prm";

		objEnvClusters.clear();
		if(stat(objenvpath.c_str(), &st)==0)
				readPRMFile(objenvpath.c_str(), objEnvClusters);
		else
				readEnvYamlFile(PlanBase::instance()->targetObject->bodyItemObject, objEnvClusters);

		envClusters.clear();
		int i=0;
		for(list<BodyItemPtr>::iterator it = PlanBase::instance()->bodyItemEnv.begin(); it != PlanBase::instance()->bodyItemEnv.end(); ++it){

				string envpath = PLUGIN_PATH + "extplugin/graspPlugin/PickAndPlacePlanner/PRM/" + (*it)->body()->name() + ".prm";

				if(stat(envpath.c_str(), &st)==0)
						readPRMFile(envpath.c_str(), envClusters);
				else
						readEnvYamlFile(*it, envClusters);

				for(unsigned int j=i; j<envClusters.size(); j++){
						envClusters[j].normal   = (*it)->body()->link(0)->attitude()*envClusters[j].normal;
						envClusters[j].tangent  = (*it)->body()->link(0)->attitude()*envClusters[j].tangent;
                        envClusters[j].bbCenter = (*it)->body()->link(0)->p() + (*it)->body()->link(0)->attitude()*envClusters[j].bbCenter;
						for(unsigned int k=0; k<envClusters[j].convexhullPoints.size(); k++)
                                envClusters[j].convexhullPoints[k] = (*it)->body()->link(0)->p() + (*it)->body()->link(0)->attitude()*envClusters[j].convexhullPoints[k];
						for(unsigned int k=0; k<envClusters[j].boundaryPointList.size(); k++)
								for(unsigned int l=0; l<envClusters[j].boundaryPointList[k].size(); l++)
                                        envClusters[j].boundaryPointList[k][l] = (*it)->body()->link(0)->p() + (*it)->body()->link(0)->attitude()*envClusters[j].boundaryPointList[k][l];
				}
				i = envClusters.size();
		}
}

void ClusterParameter::deleteCluster(){

		controlPoints.clear();
		approachVec.clear();
		convexhullPoints.clear();
		boundaryPointList.clear();
		boundaryIndex.clear();
		idPair.clear();

		normal <<0,0,0;
		tangent <<0,0,0;
		area = 0.0;

		isPuttingCluster = false;
}

void ClusterParameter::printCluster(){

		cout << "id " << id << endl;
		cout << "idPair ";
		for(unsigned int i=0; i<idPair.size(); i++)
				cout << idPair[i] << " ";
		cout << endl;
		cout << "Control Points: " << endl;
		for(unsigned int i=0; i<controlPoints.size(); i++)
				cout << controlPoints[i].transpose() << endl;
		cout << "Approach Vectors: " << endl;
		for(unsigned int i=0; i<approachVec.size(); i++)
				cout << approachVec[i].transpose() << endl;
		cout << "BBedge ";
		cout << bbEdge.transpose() << endl;
}

void ParameterFileData::readPRMFile(const char fname[], vector<ClusterParameter>& clusterSet)
{
		ifstream fp;
		double x;
		Vector3 p;
		char Jname[256];

		bool write=false;
		ClusterParameter cluster;

		cluster.isPuttingCluster = false;

		doInclusionTest = false;
		doStabilityTest = false;
		doCollisionTest = false;
		colTestMargin = 0.001;

		fp.open(fname);

		while(GetString(fp,Jname)){
				//cout << "Jname:" << Jname << endl;
				if(strcmp(Jname,"Plane") == 0 || strcmp(Jname,"Gripper") == 0){

						while( GetVRMLdouble(fp,x) )
								;

						if(write)
							clusterSet.push_back(cluster);
						cluster.deleteCluster();

						cluster.id = (int)x;
						write = true;
				}
				else if(strcmp(Jname,"id") == 0){

						while( GetVRMLdouble(fp,x) )
								cluster.id = (int)x;
				}
				else if(strcmp(Jname,"pair") == 0){

						while( GetVRMLdouble(fp,x) )
								cluster.idPair.push_back((int)x);
				}
				else if(strcmp(Jname,"area") == 0){

						while( GetVRMLdouble(fp,x) )
								cluster.area = x;
				}
				else if(strcmp(Jname,"intention") == 0){

						while( GetVRMLdouble(fp,x) )
								cluster.intention = (int)x;
				}
				else if(strcmp(Jname, "outer_normal") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.normal(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "tangent_vector") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.tangent(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "approach_vector") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) cluster.approachVec.push_back(p);
								i++;
						}
				}
				else if(strcmp(Jname, "control_points") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) cluster.controlPoints.push_back(p);
								i++;
						}
				}
				else if(strcmp(Jname, "convexhull") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) cluster.convexhullPoints.push_back(p);
								i++;
						}
				}
				else if(strcmp(Jname, "boundary") == 0){

						int i=0;
						vector<Vector3> points;
						while( GetVRMLdouble(fp,x) ){
								p(i%3) = x;
								if(i%3 ==2) points.push_back(p);
								i++;
						}
						cluster.boundaryPointList.push_back(points);

				}
				else if(strcmp(Jname, "bounding_box_edge") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.bbEdge(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "bounding_box_center") == 0){

						int i=0;
						while( GetVRMLdouble(fp,x) ){
								cluster.bbCenter(i%3) = x;
								i++;
						}
				}
				else if(strcmp(Jname, "grasp_posture") == 0){
						graspPostureCandidate.clear();
						while( GetVRMLdouble(fp,x) )
								graspPostureCandidate.push_back((int)x);
				}
				else if(strcmp(Jname, "gain_values") == 0){
						gainParameter.clear();
						while( GetVRMLdouble(fp,x) )
								gainParameter.push_back(x);
						if(gainParameter.size()>5){
							appLength = gainParameter[2];
							appHeight = gainParameter[3];
							liftHeight = gainParameter[4];
							releaseHeight = gainParameter[5];
						}
				}
				else if(strcmp(Jname, "object_place_test") == 0){
						vector<int> p;
						while( GetVRMLdouble(fp,x) )
								p.push_back((int)x);
						if(p.size()>2){
							if(p[0]==1) doInclusionTest = true;
							if(p[1]==1) doStabilityTest = true;
							if(p[2]==1) doCollisionTest = true;
						}
				}
				else if(strcmp(Jname, "approach_length") == 0){
						while( GetVRMLdouble(fp,x) )
							appLength = x;
				}
				else if(strcmp(Jname, "approach_height") == 0){
						while( GetVRMLdouble(fp,x) )
							appHeight = x;
				}
				else if(strcmp(Jname, "lift_height") == 0){
						while( GetVRMLdouble(fp,x) )
							liftHeight = x;
				}
				else if(strcmp(Jname, "release_height") == 0){
						while( GetVRMLdouble(fp,x) )
							releaseHeight = x;
				}
				else if(strcmp(Jname, "collision_test_margin") == 0){
						while( GetVRMLdouble(fp,x) )
							colTestMargin = x;
				}
				else if(strcmp(Jname, "motion_time") == 0){
						motionTime.clear();
						while( GetVRMLdouble(fp,x) )
								motionTime.push_back(x);
				}
				else if(strcmp(Jname, "is_putting_cluster") == 0){
						while( GetVRMLdouble(fp,x) )
								if((int)x==1) cluster.isPuttingCluster=true;
				}
		}

		clusterSet.push_back(cluster);

		fp.close();

		return;
}

void ParameterFileData::readObjYamlFile(BodyItemPtr item, vector<ClusterParameter>& clusterSet)
{

	ClusterParameter cluster;
	Vector3 p;
	cluster.deleteCluster();
	cluster.isPuttingCluster = false;

	if( item->body()->info()->find("Cluster")->NODE_TYPE == YAML_SEQUENCE){
			const Listing& glist = *(*item->body()->info())["Cluster"].toListing();
			for(int i=0;i<glist.size();i++){

					const Mapping& gSettings = *glist[i].toMapping();
					if ( gSettings.isValid() && !gSettings.empty()) {

							ClusterParameter cluster;
							cluster.isPuttingCluster = false;

							if( gSettings.find("id")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["id"].toListing();
									cluster.id = list[0].toInt();
							}

							if( gSettings.find("pair")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["pair"].toListing();
									for(int i=0;i<list.size();i++){
											cluster.idPair.push_back(list[i].toInt());
									}
							}

							if( gSettings.find("area")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["area"].toListing();
									cluster.area = list[0].toDouble();
							}

							if( gSettings.find("intention")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["intention"].toListing();
									cluster.intention = list[0].toInt();
							}

							if( gSettings.find("outer_normal")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["outer_normal"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.normal(i) = list[i].toDouble();
							}

							if( gSettings.find("tangent_vector")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["tangent_vector"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.tangent(i) = list[i].toDouble();
							}

							if( gSettings.find("approach_vector")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["approach_vector"].toListing();
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													cluster.approachVec.push_back(p);
									}
							}

							if( gSettings.find("control_points")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["control_points"].toListing();
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													cluster.controlPoints.push_back(p);
									}
							}

							if( gSettings.find("bounding_box_edge")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["bounding_box_edge"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.bbEdge(i) = list[i].toDouble();
							}

							if( gSettings.find("bounding_box_center")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["bounding_box_center"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.bbCenter(i) = list[i].toDouble();
							}

							clusterSet.push_back(cluster);
					}
			}
	}

	if( item->body()->info()->find("PickAndPlacePlannerParameter")->NODE_TYPE == YAML_SEQUENCE){
			const Listing& glist = *(*item->body()->info())["PickAndPlacePlannerParameter"].toListing();
			for(int i=0;i<glist.size();i++){

					const Mapping& gSettings = *glist[i].toMapping();
					if ( gSettings.isValid() && !gSettings.empty()) {

							if( gSettings.find("grasp_posture")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["grasp_posture"].toListing();
									for(int i=0;i<list.size();i++)
											graspPostureCandidate.push_back( list[i].toInt() );
							}

							if( gSettings.find("gain_values")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["gain_values"].toListing();
									for(int i=0;i<list.size();i++)
											gainParameter.push_back( list[i].toDouble() );
							}

							if( gSettings.find("motion_time")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["motion_time"].toListing();
									for(int i=0;i<list.size();i++)
											motionTime.push_back( list[i].toDouble() );
							}
					}
			}
	}



	return;
}


void ParameterFileData::readEnvYamlFile(BodyItemPtr item, vector<ClusterParameter>& clusterSet)
{

	ClusterParameter cluster;
	Vector3 p;

	cluster.deleteCluster();
	cluster.isPuttingCluster = false;

	if( item->body()->info()->find("EnvCluster")->NODE_TYPE == YAML_SEQUENCE){
			const Listing& glist = *(*item->body()->info())["EnvCluster"].toListing();
			for(int i=0;i<glist.size();i++){

					const Mapping& gSettings = *glist[i].toMapping();
					if ( gSettings.isValid() && !gSettings.empty()) {

							ClusterParameter cluster;
							cluster.isPuttingCluster = false;



							if( gSettings.find("id")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["id"].toListing();
									cluster.id = list[0].toInt();
							}

							if( gSettings.find("area")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["area"].toListing();
									cluster.area = list[0].toDouble();
							}

							if( gSettings.find("outer_normal")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["outer_normal"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.normal(i) = list[i].toDouble();
							}

							if( gSettings.find("tangent_vector")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["tangent_vector"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.tangent(i) = list[i].toDouble();
							}


							if( gSettings.find("convexhull")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["convexhull"].toListing();
									for(int i=0;i<list.size();i++){
											p(i%3) = list[i].toDouble();
											if(i%3==2)
													cluster.convexhullPoints.push_back(p);
									}
							}

							if( gSettings.find("boundaryList")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["boundaryList"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.boundaryIndex.push_back(list[i].toInt());
							}

							if( gSettings.find("boundary")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["boundary"].toListing();
									if(cluster.boundaryIndex.size()>0){
											int k=0;
											for(size_t i=0; i<cluster.boundaryIndex.size(); i++){
													vector<Vector3> pList;
													for(int j=0; j<cluster.boundaryIndex[i]*3; j++){
														p(k%3) = list[k].toDouble();
														if(k%3==2) pList.push_back(p);
														k++;
													}
													cluster.boundaryPointList.push_back(pList);
											}
									}
									else{
											vector<Vector3> pList;
											for(size_t k=0; k<list.size(); k++){
												p(k%3) = list[k].toDouble();
												if(k%3==2) pList.push_back(p);
											}
											cluster.boundaryPointList.push_back(pList);
									}
							}

							if( gSettings.find("convexity")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["convexity"].toListing();
									cluster.Convexity = list[0].toInt();
							}

							if( gSettings.find("bounding_box_edge")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["bounding_box_edge"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.bbEdge(i) = list[i].toDouble();
							}

							if( gSettings.find("bounding_box_center")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["bounding_box_center"].toListing();
									for(int i=0;i<list.size();i++)
											cluster.bbCenter(i) = list[i].toDouble();
							}

							if( gSettings.find("is_putting_cluster")->NODE_TYPE == YAML_SEQUENCE ){
									const Listing& list = *gSettings["is_putting_cluster"].toListing();
									cluster.isPuttingCluster = false;
									if( list[0].toInt() == 1)
											cluster.isPuttingCluster = true;
							}
							clusterSet.push_back(cluster);
					}
			}
	}

	return;
}





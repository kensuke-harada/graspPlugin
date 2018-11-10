// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include "AssemblyPlan.h"

#include <iostream>
#include <cnoid/MessageView>	/* modified by qtconv.rb 0th rule*/  

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
//#define LOG

void AssemblyPlan::calcPlaneClusters(ObjectShape& object)
{
		object.generateClusterSeeds(1);
		object.mergeClustersFromSeed(true);
}


bool areSame(const vector<int>& s, const vector<int>& t){

	if(s.size() != t.size() )
		return false;
	
	for(unsigned int i=0; i<s.size(); i++){
		bool inc=false;
		for(unsigned int j=0; j<t.size(); j++){
			if(s[i]==t[j]){
				inc=true;
				break;
			}
		}
		if(!inc)
			return false;
	}
	
	return true;
}

void AssemblyPlan::showContactPointLocus()
{
		double sampleTime = 0.001;
		
		readResult_camera();

		PlanBase* tc = PlanBase::instance();

		int sphere_offset = 0;

#ifdef LOG
               ofstream fout;
		fout.open("log.txt");
#endif

		for(unsigned int cur_time=0; cur_time<tme2[0].back(); cur_time++){
				sphere_offset = 0;

				BodyItemPtr sphereItem;
				/*
#ifdef LOG
				fout << cur_time << " ";
#endif
				*/
				for(int obj=0; obj<NUM_OBJ; obj++){

						BodyItemPtr bodyItem=tc->targetObject->bodyItemObject;

						while(bodyItem != NULL){
								if((obj==0 && bodyItem->name()=="Frontcase") || (obj==1 && bodyItem->name()=="RearcaseA") || 
								   (obj==2 && bodyItem->name()=="RearcaseB1") || (obj==3 && bodyItem->name()=="RearcaseB2") )
										break;

							bodyItem=(BodyItem*)bodyItem->nextItem();	
						}

						int idObj=0;
						for(unsigned int i=0; i<objectNames.size(); i++){
								if(objectNames[i] == bodyItem->name()){
										idObj = i;
										break;
								}
						}

						sphereItem=tc->targetObject->bodyItemObject;
						
						while(sphereItem != NULL){
								if( sphereItem->name()=="sphere" )
										break;
								
								sphereItem=(BodyItem*)sphereItem->nextItem();	
						}

						for(int i=0; i<sphere_offset; i++)
								if(sphereItem->nextItem() != NULL)
										sphereItem=(BodyItem*)sphereItem->nextItem();


						idList2[obj].clear();
						
						for(unsigned int c=0; c<px2[obj].size(); c++){

								if(cur_time==tme2[obj][c]){
										cout << "\r" << cur_time << flush;

										Vector3 pc(px2[obj][c], py2[obj][c], pz2[obj][c]);
										/*
#ifdef LOG
										Vector3 p0 = bodyItem->body()->link(0)->p + bodyItem->body()->link(0)->attitude()*pc;
										fout << obj << "(" << p0(0) << "," << p0(1) << "," << p0(2) << ")";
#endif
										*/
                                        sphereItem->body()->link(0)->p() = bodyItem->body()->link(0)->p() + bodyItem->body()->link(0)->attitude()*pc;
										sphereItem->body()->link(0)->attitude() = bodyItem->body()->link(0)->attitude();

										ItemTreeView::mainInstance()->checkItem(sphereItem, true);
										sphereItem->notifyKinematicStateChange();

										if(sphereItem->nextItem() != NULL)
												sphereItem=(BodyItem*)sphereItem->nextItem();
										else{
												ItemPtr temp= sphereItem->duplicateAll();
 												tc->targetObject->bodyItemObject->parentItem()->addChildItem( temp);
												sphereItem = (BodyItem*)(temp.get());

										}

										sphere_offset++;

										int id0 = calcClusterId(tme2[obj][c], px2[obj][c], py2[obj][c], pz2[obj][c], *object2[idObj]);
										idList2[obj].push_back(id0);
								}
						}
						Vector3 finishPosition(0.000298421, -0.699061, 0.0303471);
						static bool prt=true;
						if(objPosition.size()>cur_time){
								if(norm2(objPosition[cur_time] - finishPosition)<0.001 && prt){
										cout << "Finish: " << cur_time << endl;
										prt=false;
								}
						}

				}
						
				while(sphereItem != NULL){

                        sphereItem->body()->link(0)->p() << 0.5,0.5,0.5;
						sphereItem->body()->link(0)->attitude() << 1,0,0,0,1,0,0,0,1;
						ItemTreeView::mainInstance()->checkItem(sphereItem, true);
						sphereItem->notifyKinematicStateChange();
						sphereItem=(BodyItem*)sphereItem->nextItem();	
				}
				MessageView::mainInstance()->flush();

				bool same_ = true;
				for(int obj=0; obj<NUM_OBJ; obj++)
						if( !areSame(idList2_old[obj], idList2[obj]) )
								same_ = false;
				
				if(!same_){
						for(int obj=0; obj<NUM_OBJ; obj++)
								idListSet2[obj].push_back(idList2[obj]);
						timeList.push_back(cur_time*sampleTime);
				}

				for(int obj=0; obj<NUM_OBJ; obj++){
						idList2_old[obj].clear();
						
						for(unsigned int c=0; c<idList2[obj].size(); c++)
								idList2_old[obj].push_back(idList2[obj][c]);
				}
				/*
#ifdef LOG
				fout << endl;
				BodyItemPtr bodyItem=tc->targetObject->bodyItemObject;
				while(bodyItem != NULL){
						if(bodyItem->name() == "sphere")
								fout << "(" << bodyItem->body()->link(0)->p(0) << "," << bodyItem->body()->link(0)->p(1) << "," << bodyItem->body()->link(0)->p(2) << ")";
						bodyItem=(BodyItem*)bodyItem->nextItem();	
				}
				fout << endl;
#endif
				*/
		}

		return;
}

void AssemblyPlan::readResult_camera(){

		cout << "Enter file number: ";
		string filenum;
		cin >> filenum;

		string filename_pa10 = "extplugin/graspPlugin/GeometryHandler/project/resultPA10_" + filenum + ".dat";
		string filename_cam  = "extplugin/graspPlugin/GeometryHandler/project/resultCAM_"  + filenum + ".dat";
		string filename_obj  = "extplugin/graspPlugin/GeometryHandler/project/resultObjPos_"  + filenum + ".dat";

		ifstream fp_pa10, fp_cam, fp_obj;
		fp_pa10.open(filename_pa10.c_str());
		fp_cam.open(filename_cam.c_str());
		fp_obj.open(filename_obj.c_str());


		if(!fp_pa10 || !fp_cam){
				cout << "Error file does not exist: " << filename_pa10 << " " << filename_cam <<  endl;
				return;
		}

		double dummy;
		int skip=16, numb, loop=0;

		while(!fp_pa10.eof()){

				for(int i=0; i<skip; i++)
						fp_pa10 >> dummy;

				fp_pa10 >> numb;
				
				for(int i=0; i<numb; i++){

						tme2[0].push_back(loop);
						fp_pa10 >> dummy;	px2[0].push_back(dummy);
						fp_pa10 >> dummy;	py2[0].push_back(dummy);
						fp_pa10 >> dummy;	pz2[0].push_back(dummy);
						fp_pa10 >> dummy;	fx2[0].push_back(dummy);
						fp_pa10 >> dummy;	fy2[0].push_back(dummy);
						fp_pa10 >> dummy;	fz2[0].push_back(dummy);
				}
				loop++;
		}		

		loop=0;
		skip=3;

		while(!fp_cam.eof()){

				for(int i=0; i<skip; i++)
						fp_cam >> dummy;

				for(int j=1; j<4; j++){
						
						fp_cam >> numb;
						
						for(int i=0; i<numb; i++){

								tme2[j].push_back(loop);
								fp_cam >> dummy;	px2[j].push_back(dummy);
								fp_cam >> dummy;	py2[j].push_back(dummy);
								fp_cam >> dummy;	pz2[j].push_back(dummy);
								fp_cam >> dummy;	fx2[j].push_back(dummy);
								fp_cam >> dummy;	fy2[j].push_back(dummy);
								fp_cam >> dummy;	fz2[j].push_back(dummy);
						}
				}
				loop++;
		}		

		if(fp_obj)
			while(!fp_obj.eof()){

				for(int i=0; i<1; i++)
						fp_obj >> dummy;

				Vector3 pos;
				for(int j=0; j<3; j++)
						fp_obj >> pos(j);

				for(int i=0; i<9; i++)
						fp_obj >> dummy;

				objPosition.push_back(pos);
			}		

		return;
}

int AssemblyPlan::calcClusterId(double time, double x, double y, double z, ObjectShape& object)
{
		Vector3 p(x,y,z);

		for(int j=0;j<object.nTriangles;j++){
				if(includedIn(object.triangles[j], p)>0)
						return object.triangles[j].idCluster;
		}

		vector<double> dist;
		for(int j=0;j<object.nTriangles;j++){

				Vector3 cog(object.triangles[j].ver[0]->pos + object.triangles[j].ver[1]->pos + object.triangles[j].ver[2]->pos );
				cog /= 3.0;
				
				dist.push_back(norm2(p-cog));
		}

		return object.triangles[argmin(dist)].idCluster;
}

double AssemblyPlan::includedIn(Triangle& t, const Vector3 &p)
{

		double eps = min( min( norm2(t.ver[0]->pos -t.ver[1]->pos),  norm2(t.ver[1]->pos -t.ver[2]->pos) ), norm2(t.ver[2]->pos -t.ver[0]->pos) );
 		eps = max(eps, 0.001);
		double distance = (norm2(t.ver[0]->pos -t.ver[1]->pos) + norm2(t.ver[1]->pos -t.ver[2]->pos) + norm2(t.ver[2]->pos -t.ver[0]->pos) )*3.0;

		for(int i=0; i<3; i++){
				
				Vector3 q1, q2;
				if(minDistancePoints(t.ver[i%3]->pos, Vector3(p - t.ver[i%3]->pos), t.ver[(i+1)%3]->pos, Vector3(t.ver[(i+2)%3]->pos - t.ver[(i+1)%3]->pos), q1, q2) > eps)
						return -1.0;

				double d0 = norm2(q1 - t.ver[i%3]->pos) - norm2(p - t.ver[i%3]->pos);
				
				if( d0 < 0 )
					    return -1.0;

				if(d0 < distance)
						distance = d0; 
		}

		return distance;
}

int AssemblyPlan::checkId(vector<vector<int> > idListSet[],vector<vector<int> > idListNode[], int tm, int nObj)
{
	for(int k=0; k<idListNode[0].size(); k++){
		
		bool include2 = true;
		for(int obj=0; obj<nObj; obj++)
			if(!areSame(idListSet[obj][tm], idListNode[obj][k]))
				include2 = false;
		
		if(include2)
			return k;
	}
	
	return -1;
}

int AssemblyPlan::edgeArrayNum(int i, int j, int N)
{
	if(i<j)
		return ( i*(2*N-i+1)/2 + j-i+1);
	else
		return ( i*(2*N-j+1)/2 + i-j+1);
}

void writeNodeInfo(vector<vector<int> > idListNode[], int nObj)
{
        ofstream fout;
		fout.open("idList.txt");

		for(unsigned int row=0; row<idListNode[0].size(); row++){
			for(int obj=0; obj<nObj; obj++){
				for(unsigned int pt=0; pt<idListNode[obj][row].size(); pt++)
					fout << idListNode[obj][row][pt] << " ";
				fout << "/ ";
			}
			fout << endl;
		}

		return;
}

//void AssemblyPlan::generateGraph(vector<vector<int> > idListSet[], vector<double>& timeList, vector<ObjectShape*> &object, int nObj)
void AssemblyPlan::generateGraph()
{
	int nObj = NUM_OBJ;

	vector<vector<int> >* idListNode = new vector<vector<int> >[nObj];
	//vector<vector<int> > idListNode[nObj];
	vector<double> timeListNode;
    
	int num_vertices = 1000; //object[0].clusters.size();

	const int num_edges = num_vertices*(num_vertices-1)/2;

	vector<int> edges;
	for(unsigned int t=1; t<idListSet2[0].size(); t++){

		int pt=idListNode[0].size();

		bool newVert = false;
		int id[2]={-1, -1};
		for(int k=0; k<2; k++){
			id[k] = checkId(idListSet2, idListNode, t+k-1, nObj);
			
			if(id[k] == -1){
				newVert = true;
				for(int j=0; j<nObj; j++)
					idListNode[j].push_back(idListSet2[j][t+k-1]);
				timeListNode.push_back(timeList[t+k-1]);
				id[k] = pt++;
			}
		}				
		
		if(pt >= num_vertices){
			cout << " Too many vertices " << endl;
			delete[] idListNode;
			return;
		}

		if(newVert){
			//cout << "(" << id[0] << "," << id[1] << "),"; 
			int h = edgeArrayNum(id[0], id[1], pt);
			edges.push_back(id[0]);
			edges.push_back(id[1]);
		}
	}

	//Draw graph using Graphviz
	//ofstream fout;
	//fout.open("graphviz.dot");
	//write_graphviz(fout, g);

	//Draw graph using Pajek
	ofstream fout;
	fout.open("pajek.net");
	fout << "*Vertices " << idListNode[0].size() << endl;
	/*
	for(unsigned int t=0; t<idListNode[0].size(); t++){
		fout << t+1 << " \" "<< t+1 << "::";
		fout << " 0:";
		for(unsigned int c=0; c<idListNode[0][t].size(); c++)
			fout << idListNode[0][t][c] << " ";
		fout << " 1:";
		for(unsigned int c=0; c<idListNode[1][t].size(); c++)
			fout << idListNode[1][t][c] << " ";
		fout << " 2:";
		for(unsigned int c=0; c<idListNode[2][t].size(); c++)
			fout << idListNode[2][t][c] << " ";
		fout << " 3:";
		for(unsigned int c=0; c<idListNode[3][t].size(); c++)
			fout << idListNode[3][t][c] << " ";
		fout << "\" " << endl;
	}
	*/
	for(unsigned int t=0; t<idListNode[0].size(); t++){
		fout << t+1 << " \" "<< t+1 << "::" << timeListNode[t] << "\" " << endl;
	}




	fout << "*Edges" << endl;
	for(unsigned int i=0; i<edges.size()/2; i++)
		fout << edges[2*i]+1 << " " << edges[2*i+1]+1 << endl;
	


	writeNodeInfo(idListNode,nObj);
	delete[] idListNode;
}

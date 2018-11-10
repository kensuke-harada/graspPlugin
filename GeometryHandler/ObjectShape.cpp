
#include "ObjectShape.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include <list>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <Eigen/Eigenvalues>

#include "../Grasp/PlanBase.h"
#include "../Grasp/VectorMath.h"
//#define THREAD

using namespace std;
using namespace cnoid;
using namespace grasp;


ObjectShape::ObjectShape(vector<double> vertex, vector<int> crd){

	nVerticies = vertex.size()/3;
	nTriangles = crd.size()/4;

	verticies = new VertexLink[nVerticies];
	triangles = new Triangle[nTriangles];

	for(int i=0;i<nVerticies;i++){
		verticies[i].pos[0] = vertex[3*i+0];
		verticies[i].pos[1] = vertex[3*i+1];
		verticies[i].pos[2] = vertex[3*i+2];
		verticies[i].id=i;
		verticies[i].check=-1;
	}
	for(int i=0;i<nTriangles;i++){
		triangles[i].ver[0] = &verticies[crd[4*i+0]];
		triangles[i].ver[1] = &verticies[crd[4*i+1]];
		triangles[i].ver[2] = &verticies[crd[4*i+2]];
		triangles[i].id=i;
		triangles[i].idCluster = -1;
	}
	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1]->pos - triangles[i].ver[0]->pos);
		Vector3 e2 (triangles[i].ver[2]->pos - triangles[i].ver[0]->pos);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
		triangles[i].normal =  cross(e1,e2)/ norm2 ( cross(e1,e2) );
	}

	clusterNodes=NULL;
	quadVerticies=NULL;
	thickness = 0.005;
	alpha = 1.0;

}

ObjectShape::ObjectShape(ColdetModelPtr c){
	nVerticies = c->getNumVertices();
	nTriangles = c->getNumTriangles();

	verticies = new VertexLink[nVerticies];
	triangles = new Triangle[nTriangles];

	float tx, ty, tz;
	for(int i=0;i<nVerticies;i++){
		c->getVertex(i, tx, ty, tz);
		verticies[i].pos[0] = tx;
		verticies[i].pos[1] = ty;
		verticies[i].pos[2] = tz;
		verticies[i].id=i;
		verticies[i].check=-1;
	}
	int t1, t2, t3;
	for(int i=0;i<nTriangles;i++){
		c->getTriangle(i, t1, t2, t3);
		triangles[i].ver[0] = &verticies[t1];
		triangles[i].ver[1] = &verticies[t2];
		triangles[i].ver[2] = &verticies[t3];
		triangles[i].id=i;
		triangles[i].idCluster = -1;
	}
	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1]->pos - triangles[i].ver[0]->pos);
		Vector3 e2 (triangles[i].ver[2]->pos - triangles[i].ver[0]->pos);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
		triangles[i].normal =  cross(e1,e2)/ norm2 ( cross(e1,e2) );
	}

	clusterNodes=NULL;
	quadVerticies=NULL;
	thickness = 0.005;
	alpha = 1.0;
}


ObjectShape::ObjectShape(ObjectShape* orig){

	nVerticies = orig->nVerticies;
	nTriangles = orig->nTriangles;

	verticies = new VertexLink[nVerticies];
	triangles = new Triangle[nTriangles];

	for(int i=0;i<nVerticies;i++){
		verticies[i].pos = orig->verticies[i].pos;
		verticies[i].id=i;
		verticies[i].check=-1;
	}
	for(int i=0;i<nTriangles;i++){
		triangles[i].ver[0] = &verticies[orig->triangles[i].ver[0]->id];
		triangles[i].ver[1] = &verticies[orig->triangles[i].ver[1]->id];
		triangles[i].ver[2] = &verticies[orig->triangles[i].ver[2]->id];
		triangles[i].id=i;
		triangles[i].idCluster = -1;
	}
	for(int i=0;i<nTriangles;i++){
		Vector3 e1 (triangles[i].ver[1]->pos - triangles[i].ver[0]->pos);
		Vector3 e2 (triangles[i].ver[2]->pos - triangles[i].ver[0]->pos);
		triangles[i].area = norm2 ( cross(e1,e2) ) /2.0;
		triangles[i].normal =  cross(e1,e2)/ norm2 ( cross(e1,e2) );
	}
	clusterNodes=NULL;
	quadVerticies=NULL;
	thickness = 0.005;
	alpha = 1.0;

}


ObjectShape::~ObjectShape(){

	delete [] verticies;
	delete [] triangles;

	clusters.clear();
	std::vector<ClusterImpl>().swap (clusters);
	for(int i=0;i<clusterPairHeap.size();i++){
		delete clusterPairHeap[i];
	}
	clusterPairHeap.clear();
	std::vector<ClusterPair*>().swap (clusterPairHeap);
	if(clusterNodes){
		delete [] clusterNodes;
	}
}


void ObjectShape::makeNbr(){

	vector < Triangle* >* tl;

	for (int i=0; i< nTriangles;i++){
		triangles[i].ver[0]->tlist.push_back(&triangles[i]);
		triangles[i].ver[1]->tlist.push_back(&triangles[i]);
		triangles[i].ver[2]->tlist.push_back(&triangles[i]);
	}
	for(int i=0;i< nTriangles;i++){
		for(int j=0;j<3;j++){
			int flag = 0;
			tl = &triangles[i].ver[j]->tlist;
			for(unsigned int k=0; k< tl->size();k++){
				Triangle* t = (*tl)[k];
				for(int l=0;l<3;l++){
					if( triangles[i].ver[j] == t->ver[l] && & triangles[i] != t){
#ifdef HEURISTICS_FOR_PCL
						if( triangles[i].ver[(j+1)%3] == t->ver[(l+2)%3] || triangles[i].ver[(j+1)%3] == t->ver[(l+1)%3])
#else
						if( triangles[i].ver[(j+1)%3] == t->ver[(l+2)%3])
#endif
						{
							triangles[i].nbr[j] = t;
							flag++;
						}
					}
				}
			}
			if(flag != 1){
				triangles[i].nbr[j] = NULL;

#ifdef OUTPUT_COMMENT
				cout << "TRIANGLE NEIGHBOR ERROR " <<flag <<" "  << i << " "<< j << " ver " << triangles[i].ver[j]->id << " "<< triangles[i].ver[(j+1)%3]->id  << endl;
#endif
			}
		}
	}
#ifdef OUTPUT_COMMENT
	cout << "calcurate triangle neighbor" << endl;
#endif
}

void ObjectShape::transform(Matrix33 R, Vector3 t){
	Vector3 sum;
	sum.setZero();

	for(int i=0;i<nVerticies;i++){
		verticies[i].pos = R*verticies[i].pos + t;
//		sum+= verticies[i].pos;
	}
	sum /= (double)nVerticies;
	cout << "geometry" << endl;
	cout << sum[0] << " "<< sum[1] << " "<< sum[2] << endl;
//	for(int i=0;i<nVerticies;i++){
//		verticies[i].pos[0] *= 3;
//		cout << verticies[i].pos[0] << " "<< verticies[i].pos[1] << " "<< verticies[i].pos[2] << endl;
//	}
	cout << "geometry end" << endl;
}

void ObjectShape::generateClusterSeeds(int size){

	cout << "triangle size " << nTriangles << endl;

	unsigned int idCluster=0, idTriangle=0;

	do{

		ClusterImpl cluster(idCluster);
		cluster.addTriangleToCluster(triangles[idTriangle]);
		cluster.addTriangleToNeighborsList(triangles[idTriangle]);
		cluster.calcClusterNormal();
		cluster.calcClusterBoundary();
		cluster.calcClusterBoundingBox();

		for(int i=1; i<size; i++){
			vector<double> normalList;
			vector<int> idList;
			Triangle *tn;
			//for(unsigned int j=0; j<3; j++){
			for(unsigned int j=0; j<cluster.neighborList.size(); j++){

				//tn = triangles[idTriangle].nbr[j];
				tn = cluster.neighborList[j];

				if(tn == NULL) continue;
				if(tn->idCluster != -1) continue;
				if(cluster.clusterTriangleDistance(*tn) > thickness) continue;

				normalList.push_back( dot(triangles[idTriangle].normal, tn->normal) );

				idList.push_back(j);
			}

			if(normalList.size()>0){

				sort_by(idList, normalList);

				//tn = triangles[idTriangle].nbr[idList.back()];
				tn = cluster.neighborList[idList.back()];

				cluster.addTriangleToCluster(*tn);
				cluster.addTriangleToNeighborsList(*tn);
				cluster.calcClusterNormal();
				cluster.calcClusterBoundary();
				cluster.calcClusterBoundingBox();
			}
		}

		cluster.closed = false;
		clusters.push_back(cluster);
		parentList.push_back(idCluster);

		idCluster++;
		idTriangle++;

		if(idTriangle < nTriangles){
			while(triangles[idTriangle].idCluster != -1 && idTriangle < nTriangles){
				idTriangle++;
				if(idTriangle >= nTriangles) break;
			}
		}

	}while(idTriangle < nTriangles);

	cout << "cluster seeds generated" << endl;

	return;
}

void ObjectShape::mergeClustersFromSeed(bool PlaneMode)
{

	unsigned int cnt=0;
	closed = false;

	while(!closed){

		if(!clusters[parentList[cnt]].closed)
			mergeClusters2(parentList[cnt], clusters[parentList[cnt]].normal, PlaneMode);

		cnt++;
		if(parentList.size() <= cnt)
			cnt = 0;

		closed = true;
		for(unsigned int i=0; i<parentList.size(); i++)
			if(clusters[parentList[i]].closed == false)
				closed = false;
	}

	return;
}

void ObjectShape::mergeClustersFromSeed(const Vector3& normal, const Vector3& position)
{
#ifdef DEBUG2
	ofstream fout;
	fout.open("res.txt", std::ios::out | std::ios::app);
	fout << "Merge Clusters from Seed" << normal.transpose() << endl;
#endif

	vector<double> nList;
	vector<int> o;

	for(unsigned int i=0; i<parentList.size(); i++){
		nList.push_back(dot(clusters[parentList[i]].normal, normal) - 0.01*norm2(clusters[parentList[i]].center - position)); //Check!
		o.push_back(parentList[i]);
	}

	int id0 = o[argmax(nList)], id1; //o.size()-1;

	while(!clusters[id0].closed)
		mergeClusters2(id0, normal);

	clusters[id0].calcControlPoints(0.0);

	vector<Vector3> controlPoint;
	for(unsigned int i=0; i< clusters[id0].controlPoints.size(); i++)
		controlPoint.push_back(clusters[id0].controlPoints[i]);

#ifdef DEBUG2
	fout << "ID0 " << id0 << "/" << clusters[id0].closed << ", CP size " << clusters[id0].controlPoints.size() << ", Area " << clusters[id0].area << endl;
	fout << "BBedge " << clusters[id0].bbedge.transpose() << ", Normal " << clusters[id0].normal.transpose() << endl;
#endif

	Vector3 normalId0 = clusters[id0].normal;
	int closed = false;

	for(int cnt=0; cnt<5; cnt++){

		nList.clear();
		o.clear();
		for(unsigned int i=0; i<parentList.size(); i++){
			if(!clusters[parentList[i]].closed && dot(clusters[parentList[i]].normal, normal)<-0.95){
				nList.push_back(dot(clusters[parentList[i]].normal, normal));
				o.push_back(parentList[i]);
			}
		}
		sort_by(o, nList);

		bool incl=false;
		for(unsigned int j=0; j<o.size(); j++){
			id1=o[j];

			for(unsigned int i=0; i< controlPoint.size(); i++){

				Vector3 pt;
				if( clusters[id1].lineIntersectToCluster(controlPoint[i], normalId0, pt) ){
					incl=true;
					break;
				}
			}

			if(incl)
				break;
		}

		if(incl){
			while(!clusters[id1].closed)
				mergeClusters2(id1, Vector3(-normalId0));
			clusters[id1].calcControlPoints(0.0);
		}

#ifdef DEBUG2
		fout << "Counter cluster found: " << incl << ", ID1 " << id1 << "/" << clusters[id1].closed << endl;
		fout << "CP size " << clusters[id1].controlPoints.size() << ", Normal " << clusters[id1].normal.transpose() << endl;
#endif
	}

	for(unsigned int i=0; i<parentList.size(); i++){
		id0 = parentList[i];
		if(clusters[id0].closed && dot(clusters[id0].normal, normalId0)>0.95)
			break;
	}

	for(unsigned int i=0; i<parentList.size(); i++){
		id1 = parentList[i];
		if(clusters[id1].closed && dot(clusters[id1].normal, normalId0)<-0.95){
			clusters[id0].idPair.push_back(clusters[id1].id);
			clusters[id1].idPair.push_back(clusters[id0].id);
		}
		else if(id0!=id1 && clusters[id1].closed)
			clusters[id1].closed = false;
	}

	if(clusters[id0].idPair.size()==0)
		clusters[id0].closed = false;

	return;
}

void thread_process( ClusterPair** cpList, ClusterImpl *c1, ClusterImpl *c2){
	*cpList = new ClusterPair( *c1, *c2 );
//	cpList->push_back (  new ClusterPair( *c1, *c2 ) );
}


bool ObjectShape::mergeClusters2(int id, const Vector3& normal, bool PlaneMode){

#ifdef DEBUG2
	ofstream fout;
	fout.open("res.txt", std::ios::out | std::ios::app);
	fout << "Merge Clusters2 id=" << id << " normal=" << normal.transpose() << endl;
#endif

	vector<double> normalList;
	vector<int>    numberList;
	for(unsigned int i=0; i<clusters[id].neighborList.size(); i++){
		if(  clusters[id].neighborList[i]->idCluster == -1) continue;
		if(  included(clusters[id].neighborList[i]->idCluster, numberList) ) continue;
		//normalList.push_back( dot(clusters[id].normal, clusters[clusters[id].neighborList[i]->idCluster].normal) );
		normalList.push_back( dot(normal, clusters[clusters[id].neighborList[i]->idCluster].normal) );
		numberList.push_back(clusters[id].neighborList[i]->idCluster);
	}

	sort_by(numberList, normalList);

	if(normalList.size()>0){
		for(unsigned int i=0; i<normalList.size(); i++){

			int id2 = numberList[normalList.size()-i-1];
			Triangle *t;

			bool withinThreshold = true;
			for(unsigned int j=0; j<clusters[id2].triangleList.size(); j++){

				t = clusters[id2].triangleList[j];
#ifdef DEBUG2
				fout << "id2=" << id2 << ", normalList: " << i << "/" << normalList.size() << ", triList: " << j << "/" << clusters[id2].triangleList.size() << ",d=" << clusters[id].clusterTriangleDistance(*t) << endl;
#endif
				if(!PlaneMode && clusters[id].clusterTriangleDistance(*t) > thickness ){
					withinThreshold = false;
					break;
				}
			}
			if(PlaneMode && dot(clusters[id].normal,clusters[id2].normal) < 0.99)
				withinThreshold = false;


			if(withinThreshold){

				ClusterImpl cluster(id);

				cluster.calcTriangleList(clusters[id], clusters[id2]);
				cluster.calcNeighborList(clusters[id], clusters[id2]);
				cluster.calcClusterNormal();
				cluster.calcClusterBoundary();
				cluster.calcClusterBoundingBox();

				clusters[id] = cluster;

				vector<int>::iterator it = parentList.begin();
				while(it!=parentList.end()){
					if((*it)==id2)
						it = parentList.erase(it);
					else
						++it;
				}
#ifdef DEBUG2
				fout << "merging: " <<  id << "(" << clusters[id].triangleList.size() << ") - " <<id2 << "(" << clusters[id2].triangleList.size() << "), P=" << parentList.size() << endl;
#endif
				return true;
			}
		}
#ifdef DEBUG2
		fout << "closing " << id << endl;
#endif
		clusters[id].closed = true;
	}
	else clusters[id].closed = true;

	return false;
}


void ObjectShape::initialClustersFromOneFace(){

	clusterNodes = new ClusterImpl[nTriangles*2];
	nClusterNodes=0;

	for(int i=0;i<nTriangles*2;i++){
		clusterNodes[i].id = i;
	}

	cout << "initial clusterNode" << endl;
	for(int i=0;i<nTriangles;i++){
		clusterNodes[i].addTriangleToCluster(triangles[i]);
		clusterNodes[i].calcQuadricAreaParameter();
#ifdef CENTER_SHIFT
		clusterNodes[i].calcQuadricAreaParameterLow();
#endif

		if(nTriangles>100)  if(  ( i % (nTriangles /100) ) == 0 ) cout << "\r" << (i / (nTriangles /100)) << "% " << flush;
	}
	nClusterNodes=nTriangles;

	cout <<endl << "initial cluster pair heap" << endl;

#ifdef THREAD
	ClusterPair** cpLink = new ClusterPair*[nTriangles*3];
	//ClusterPair* cpLink[nTriangles*3];
	int cnt=0;
	boost::thread_group* thr_grp = new boost::thread_group();
	for(int i=0;i<nTriangles;i++){
		int idc = triangles[i].idCluster;
		for(int j=0;j<3;j++){
			if(triangles[i].nbr[j]==NULL) continue;
			clusterNodes[idc].neighborClusters.push_back(&clusterNodes[triangles[i].nbr[j]->idCluster]);
			if(idc < triangles[idc].nbr[j]->idCluster) thr_grp->create_thread( boost::bind(&thread_process,  &cpLink[cnt++] , &clusterNodes[idc], &clusterNodes[triangles[i].nbr[j]->idCluster]));
			if(  cnt%16 ==0 ){
				thr_grp->join_all();
				delete thr_grp;
				thr_grp=  new boost::thread_group();
			}
		}
		if(nTriangles>100)  if(  (i % (nTriangles /100)) == 0 ) cout << "\r" << (i / (nTriangles /100)) << "% " << flush;
	}
	thr_grp->join_all();
	delete thr_grp;

	for(int i=0;i<cnt;i++){
		clusterPairHeap.push_back( cpLink[i] );
//		push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
	}
	delete[] cpLink;
#else
	for(int i=0;i<nTriangles;i++){
		if(triangles[i].idCluster==-1) continue;
		int idc = triangles[i].idCluster;
		for(int j=0;j<3;j++){
			if(triangles[i].nbr[j]==NULL) continue;
			if(triangles[i].nbr[j]->idCluster == -1) continue;
			clusterNodes[idc].neighborClusters.push_back(&clusterNodes[triangles[i].nbr[j]->idCluster]);
			if(idc < triangles[idc].nbr[j]->idCluster) clusterPairHeap.push_back( new ClusterPair( clusterNodes[idc], clusterNodes[triangles[i].nbr[j]->idCluster] ) );
		}
		if(nTriangles>100)  if(  (i % (nTriangles /100)) == 0 ) cout << "\r" << (i / (nTriangles /100)) << "% " << flush;
	}
#endif

	make_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );


	cout << endl <<"the number of cluster pair "<< clusterPairHeap.size() << endl;
	for(int i=0; i < clusterPairHeap.size(); i++){
//		clusterPairHeap[i].id= i;
		clusterPairHeap[i]->id= i;
//		clusterPairHeap[i].error = clusterPairHeap[i].clusters[0]->area + clusterPairHeap[i].clusters[1]->area;
	}
}


void ObjectShape::calcClusterBinaryTreeStep(){

	static int cnt0 = nTriangles*0.50;
	int cnt = 0;

	while(  clusterPairHeap.size() > 0 ){
		ClusterPair& cp = *clusterPairHeap.front();
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){

			ClusterImpl& cluster =  clusterNodes[nClusterNodes++];
			cluster.copyParameter(cp);
			cluster.mergeClusterForBinaryTree(*cp.clusters[0],*cp.clusters[1]);


			//pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), compClusterHeap ); clusterPairHeap.pop_back();
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){

				clusterPairHeap.push_back( new ClusterPair( cluster, *(*it) ) );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			}

			if(nTriangles>100)  if( ((nClusterNodes-nTriangles) % (nTriangles /100) )== 0 ) cout << "\r" << int ( (nClusterNodes-nTriangles) / nTriangles * 100 ) << "% " << flush;
			cluster.calcQuadricSurfaceParameterOutput();
			cout << endl;
			//cluster.outputClusterQuadParameter();
			//break;
//			if(cnt++ > cnt0 ){
			//cout << cluster.id <<" "<<cluster.quadError << endl;
			//cout <<cluster.quadCoefficient.transpose() << endl;
//			cout << cluster.sumQuadDistribution << endl;
//			cout << cluster.sumQuadError << endl;
			cnt0 = 0.50*cnt0-1;
				break;
//			}
		}
		else{
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();
		}
	}
	for(int i=0;i< nTriangles; i++){
		ClusterImpl* c0= &clusterNodes[triangles[i].idCluster];
		while(! c0->isAliveNode) c0 = c0->parentNode;
		triangles[i].idCluster = c0->id;
	}

	calcVertexPositionOnQuadSurface();
	return;
}


void ObjectShape::calcClusterBinaryTree(const int nNodes){

	cout << "Clustering Start"<<endl;

//	while( ( nClusterNodes < (nTriangles*2 - 5) ) && (clusterPairHeap.size() > 0) ){
	while(  clusterPairHeap.size() > 0 ){
//		ClusterPair& cp = * clusterPairHeap.front();
		ClusterPair& cp = * clusterPairHeap.front();
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
			if( (cp.error > 0.01) && (nNodes==0)) break;
			if(nNodes >= (nTriangles*2 - nClusterNodes)) break;
			//cout << cp.quadError << cp.quadCoefficient << endl;

//			cout << sumError << endl;

			ClusterImpl& cluster =  clusterNodes[nClusterNodes++];
			cluster.copyParameter(cp);
			cluster.mergeClusterForBinaryTree(*cp.clusters[0],*cp.clusters[1]);

			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();

#ifdef THREAD
			ClusterPair** cpLink = new ClusterPair*[cluster.neighborClusters.size()];
			//ClusterPair* cpLink[cluster.neighborClusters.size()];
			int cnt=0;
			boost::thread_group* thr_grp = new boost::thread_group();
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				thr_grp->create_thread( boost::bind(&thread_process,  &cpLink[cnt++] , &cluster,(*it)));
//				if( (cnt%16)==0 ){
//					thr_grp->join_all();
//					delete thr_grp;
//					thr_grp = new boost::thread_group();
//				}
			}
			thr_grp->join_all();
			delete thr_grp;

			for(int i=0;i<cnt;i++){
				clusterPairHeap.push_back( cpLink[i] );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
//				delete cpLink[i];
			}
			delete[] cpLink;
#else
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				clusterPairHeap.push_back( new ClusterPair( cluster, *(*it) ) );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			}
#endif
			if(nTriangles>100)  if( ((nClusterNodes-nTriangles) % (nTriangles/100) )== 0 ) cout << "\r" << int ( (nClusterNodes-nTriangles) / (nTriangles/100) ) << "% " << flush;
		}
		else{
			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();
		}
	}
	cout << endl<< "Clustering Finish "<<endl;
	for(int i=0;i< nTriangles; i++){
		if(triangles[i].idCluster==-1) continue;
		ClusterImpl* c0= &clusterNodes[triangles[i].idCluster];
		while(! c0->isAliveNode) c0 = c0->parentNode;
		triangles[i].idCluster = c0->id;
	}

	for (int i=0;i< clusterPairHeap.size(); i++ ){
		ClusterPair& cp = *clusterPairHeap[i];
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
#ifdef OUTPUT_COMMENT
			cout <<cp.clusters[0]->id << " " <<cp.clusters[1]->id << " " <<cp.quadError << cp.quadCoefficient << endl;
			//cp.outputClusterQuadParameter();
#endif
		}
	}
	cout << endl ;


	ofstream fout("test.yaml");

	for(int i=0;i<nClusterNodes;i++){
		if(clusterNodes[i].isAliveNode){
#ifdef OUTPUT_COMMENT
			cout << clusterNodes[i].id <<" " << clusterNodes[i].neighborClusters.size() <<" "<<clusterNodes[i].quadError << " "<< clusterNodes[i].quadCoefficient.transpose() << endl;
			clusterNodes[i].calcQuadricSurfaceParameterOutput();
			cout << endl;
#endif
//			if(clusterNodes[i].area > 0.01)
			clusterNodes[i].writeData(fout);
//			cout << endl;
			//cout << clusterNodes[i].sumQuadError << endl;
			//clusterNodes[i].calcQuadricSurfaceParameter();
			//cout << clusterNodes[i].sumQuadDistribution << endl << endl;
//			clusterNodes[i].outputClusterQuadParameter();
		}
	}

	calcVertexPositionOnQuadSurface();

	return;
}

void ObjectShape::calcClusterBinaryTreeFV(const int nNodes){

	cout << "Clustering Start"<<endl;

//	while( ( nClusterNodes < (nTriangles*2 - 5) ) && (clusterPairHeap.size() > 0) ){
	while(  clusterPairHeap.size() > 0 ){
//		ClusterPair& cp = * clusterPairHeap.front();
		ClusterPair& cp = * clusterPairHeap.front();
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
			if( (cp.error > 0.01) && (nNodes==0)) break;
			if(nNodes >= (nTriangles*2 - nClusterNodes)) break;
			//cout << cp.quadError << cp.quadCoefficient << endl;

//			cout << sumError << endl;

			ClusterImpl& cluster =  clusterNodes[nClusterNodes++];
			cluster.copyParameter(cp);
			cluster.mergeClusterForBinaryTree(*cp.clusters[0],*cp.clusters[1]);

			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();
			// std::cout << "check1" << '\n';

#ifdef THREAD
			ClusterPair** cpLink = new ClusterPair*[cluster.neighborClusters.size()];
			//ClusterPair* cpLink[cluster.neighborClusters.size()];
			int cnt=0;
			boost::thread_group* thr_grp = new boost::thread_group();
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				thr_grp->create_thread( boost::bind(&thread_process,  &cpLink[cnt++] , &cluster,(*it)));
//				if( (cnt%16)==0 ){
//					thr_grp->join_all();
//					delete thr_grp;
//					thr_grp = new boost::thread_group();
//				}
			}
			thr_grp->join_all();
			delete thr_grp;

			for(int i=0;i<cnt;i++){
				clusterPairHeap.push_back( cpLink[i] );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
//				delete cpLink[i];
			}
			delete[] cpLink;
#else
			// std::cout << "check3" << '\n';
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				clusterPairHeap.push_back( new ClusterPair( cluster, *(*it) ) );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			}
#endif
			// std::cout << "check4" << '\n';
			if(nTriangles>100)  if( ((nClusterNodes-nTriangles) % (nTriangles/100) )== 0 ) cout << "\r" << int ( (nClusterNodes-nTriangles) / (nTriangles/100) ) << "% " << flush;
		}
		else{
			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();
		}
	}
	cout << endl<< "Clustering Finish "<<endl;
	for(int i=0;i< nTriangles; i++){
		if(triangles[i].idCluster==-1) continue;
		ClusterImpl* c0= &clusterNodes[triangles[i].idCluster];
		while(! c0->isAliveNode) c0 = c0->parentNode;
		triangles[i].idCluster = c0->id;
	}

	for (int i=0;i< clusterPairHeap.size(); i++ ){
		ClusterPair& cp = *clusterPairHeap[i];
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
#ifdef OUTPUT_COMMENT
			cout <<cp.clusters[0]->id << " " <<cp.clusters[1]->id << " " <<cp.quadError << cp.quadCoefficient << endl;
			//cp.outputClusterQuadParameter();
#endif
		}
	}
	cout << endl ;
  ClusterClass();
	ofstream fout("test.yaml");

	for(int i=0;i<nClusterNodes;i++){
		if(clusterNodes[i].isAliveNode){
#ifdef OUTPUT_COMMENT
			cout << clusterNodes[i].id <<" " << clusterNodes[i].neighborClusters.size() <<" "<<clusterNodes[i].quadError << " "<< clusterNodes[i].quadCoefficient.transpose() << endl;
			clusterNodes[i].calcQuadricSurfaceParameterOutput();
			cout << endl;
#endif
//			if(clusterNodes[i].area > 0.01)
			clusterNodes[i].writeData(fout);
//			cout << endl;
			//cout << clusterNodes[i].sumQuadError << endl;
			//clusterNodes[i].calcQuadricSurfaceParameter();
			//cout << clusterNodes[i].sumQuadDistribution << endl << endl;
//			clusterNodes[i].outputClusterQuadParameter();
		}
	}

	calcVertexPositionOnQuadSurface();

	return;
}
void ObjectShape::ClusterClass(){

  std::cout << "nClusterNodes " <<nClusterNodes<<'\n';
  std::cout << "nTriangles " <<nTriangles<<'\n';
	int *clusterid;
  int num_cluster=0;
  // ClusterImpl& cluster =  clusterNodes[object->nClusterNodes++];
  for(int i=0;i<nClusterNodes;i++){
  	if(clusterNodes[i].isAliveNode && clusterNodes[i].quadCoefficient.size()>0 && clusterNodes[i].quadError!=0){
			 num_cluster++;
 		}
 	}
  std::cout << "num_cluster=" <<num_cluster<<'\n';
  std::cout << '\n';

	int num=0;
	clusterid= new int [num_cluster];
	for(int i=0;i<nClusterNodes;i++){
  	if(clusterNodes[i].isAliveNode && clusterNodes[i].quadCoefficient.size()>0 &&  clusterNodes[i].quadError!=0){
			 clusterid[num]=clusterNodes[i].id;
			 num++;
 		}
 	}
	for (int i = 0; i < num_cluster; i++) {
		std::cout << "clusterid["<<i<<"]=" <<clusterid[i]<< '\n';
	}
	std::cout << '\n';
	for (int i = 0; i < num_cluster; i++) {
		std::cout << "id : " << clusterid[i]<< '\n';
		ShapeJudge(clusterid[i]);
	}
	std::cout << '\n';

int cnt[num_cluster];
MatrixXd sum=MatrixXd::Zero(num_cluster,3);
MatrixXd average=MatrixXd::Zero(num_cluster,3);
 // vector<vector< Matrix3 > >  convarianceR;			//共分散行列
vector< vector< vector<double> > > convarianceR;
convarianceR=vector< vector <vector<double> > >(num_cluster, vector< vector<double> >(3, vector<double>(3, 0)));

/*
for (int k = 0; k < num_cluster; k++) {
	for(int i=0 ; i<clusterNodes[clusterid[k]].triangleList.size() ; i++){
		for(int j=0 ; j<3 ; j++){
			std::cout << "clusterNodes[clusterid[k]].triangleList[i]->ver[j]->pos" <<clusterNodes[clusterid[k]].triangleList[i]->ver[j]->pos<< '\n';
			}
	}
}*/
 for (int k = 0; k < num_cluster; k++) {
	 	cnt[k]=0;
	for(int j=0;j<nTriangles;j++){
		Triangle& t= triangles[j];
      if (clusterid[k]==t.idCluster) {
				for(int i=0 ; i<3 ; i++){
					cnt[k]++;
					for (int l = 0; l < 3; l++) {
					sum(k,l)+=t.ver[i]->pos[l];
					}
      	}
			}
	}
}
for (int k = 0; k < num_cluster; k++) {
	  std::cout << "nPointCloud["<<k<<"]=" <<cnt[k]<< '\n';
}
std::cout << '\n';

for (int k = 0; k < num_cluster; k++) {
		for(int l=0 ; l<3 ; l++){
			// std::cout << "sum("<<k<<"," <<l<<")="<<sum(k,l) <<'\n';
			average(k,l)=sum(k,l)/cnt[k];
			std::cout << "average("<<k<<"," <<l<<")="<<average(k,l) <<'\n';
			}
			std::cout << '\n';
	}
	for (int k = 0; k < num_cluster; k++) {
	 for(int j=0;j<nTriangles;j++){
		 Triangle& t= triangles[j];
			 if (clusterid[k]==t.idCluster) {
				 for(int i=0 ; i<3 ; i++){
					      convarianceR[k][0][1]+= (average(k,0)-t.ver[i]->pos[0])*(average(k,1)-t.ver[i]->pos[1])/cnt[k];
					      convarianceR[k][1][2]+= (average(k,1)-t.ver[i]->pos[1])*(average(k,2)-t.ver[i]->pos[2])/cnt[k];
					      convarianceR[k][0][2]+= (average(k,2)-t.ver[i]->pos[2])*(average(k,0)-t.ver[i]->pos[0])/cnt[k];
					 for (int l = 0; l < 3; l++) {
					     convarianceR[k][l][l]+= (average(k,l)-t.ver[i]->pos[l])*(average(k,l)-t.ver[i]->pos[l])/cnt[k];
					 }
				 }
			 }
	 }
	}
	for (int k = 0; k < num_cluster; k++) {
			convarianceR[k][1][0]=convarianceR[k][0][1];
			convarianceR[k][2][1]=convarianceR[k][1][2];
			convarianceR[k][2][0]=convarianceR[k][0][2];
  }
/*
  for (int k = 0; k < num_cluster; k++) {
		for(int j=0 ; j<3 ; j++){
 			for(int i=0 ; i<3 ; i++){
 			std::cout << "convarianceR["<<k<<"]["<<j<<"]["<<i<<"]=" << convarianceR[k][j][i]<<'\n';
			}
		}
		std::cout<< '\n';
	}
*/
  vector< vector< vector<double> > > evec_3d;
	evec_3d=vector< vector <vector<double> > >(num_cluster, vector< vector<double> >(3, vector<double>(3, 0)));
  dmatrix eval_3d = MatrixXd::Zero(num_cluster,3);

	dmatrix evec_3dtem = MatrixXd::Zero(3,3);
	dvector eval_3dtem(3);
  Matrix3 convarianceRtem;
  int info;

	for (int k = 0; k < num_cluster; k++) {
		for(int j=0 ; j<3 ; j++){
 			for(int i=0 ; i<3 ; i++){
 			convarianceRtem(j,i)=convarianceR[k][j][i];
			}
		}
		info = calcEigenVectors(convarianceRtem, evec_3dtem, eval_3dtem); //行列"covarianceR"の固有ベクトル"evec"と固有値"evel"を求める関数
		for(int j=0 ; j<3 ; j++){
			for(int i=0 ; i<3 ; i++){
				evec_3d[k][j][i]=evec_3dtem(j,i);
			}
		}
		for(int j=0 ; j<3 ; j++){
			eval_3d(k,j)=eval_3dtem[j];
		}
	}
/*
	for (int k = 0; k < num_cluster; k++) {
		for(int j=0 ; j<3 ; j++){
 			for(int i=0 ; i<3 ; i++){
 			std::cout << "evec_3d["<<k<<"]["<<j<<"]["<<i<<"]=" << evec_3d[k][j][i]<<'\n';
			}
		}
		std::cout<< '\n';
	}
	*/
	/*
	for (int k = 0; k < num_cluster; k++) {
		for(int j=0 ; j<3 ; j++){
			std::cout << "eval_3d["<<k<<"]["<<j<<"]=" << eval_3d(k,j)<<'\n';
		}
		std::cout<< '\n';
	}
 */

double tmp=0.0;
for (int k = 0; k < num_cluster; k++) {
		for (int j=0; j<3; ++j) {
		    for (int i=j+1; i<3; ++i) {
		      if (eval_3d(k,j) > eval_3d(k,i)) {
		        tmp =  eval_3d(k,j);
		        eval_3d(k,j) = eval_3d(k,i);
		        eval_3d(k,i) = tmp;
		      }
				}
			}
}
/*
std::cout << "sort" << '\n';
for (int k = 0; k < num_cluster; k++) {
	for(int j=0 ; j<3 ; j++){
		std::cout << "eval_3d["<<k<<"]["<<j<<"]=" << eval_3d(k,j)<<'\n';
	}
	std::cout<< '\n';
}
*/

	std::cout << "1. pet" << '\n';
	std::cout << "2. cup" << '\n';
	std::cout << "please select number" << '\n';
	int namenum;
	std::cin >> namenum;
	std::cout << '\n';
	double eval_3dcom0[3]={0.0,0.0,0.0};
	double eval_3dcom1[3]={0.0,0.0,0.0};
	double min;

	int capid=0;
	int handleid=0;
	int bodyid=0;

switch (namenum) {
	case 1:
	 	std::cout << "select pet" << '\n';
	 	//cap
		eval_3dcom0[0]=5.1937e-06;
		eval_3dcom0[1]=5.29968e-05;
		eval_3dcom0[2]=6.20382e-05;
		//handle
	  eval_3dcom1[0]=5.31185e-05;
		eval_3dcom1[1]=3.25692e-04;
		eval_3dcom1[2]=2.15658e-03;
  //  double eval_3dcom0[3]={1.87762e-06,4.18766e-05,8.19298e-05};//cap
	//  double eval_3dcom1[3]={3.10082e-05,2.58955e-04,2.13184e-03};//handle

	min=100.0;
	for (int k = 0; k < num_cluster; k++) {
		if (1.0 <= eval_3d(k,0)/1.0e-06 && eval_3d(k,0)/1.0e-06 < 10.0)
		if(1.0 <= eval_3d(k,1)/1.0e-05 && eval_3d(k,1)/1.0e-05 < 10.0)
		if(1.0 <= eval_3d(k,2)/1.0e-05 && eval_3d(k,2)/1.0e-05 < 10.0) {
			if (fabs((eval_3dcom0[0]-eval_3d(k,0))/1.0e-06)+fabs((eval_3dcom0[1]-eval_3d(k,1))/1.0e-05)+fabs((eval_3dcom0[2]-eval_3d(k,2))/1.0e-05) < min) {
				min = fabs((eval_3dcom0[0]-eval_3d(k,0))/1.0e-06)+fabs((eval_3dcom0[1]-eval_3d(k,1))/1.0e-05)+fabs((eval_3dcom0[2]-eval_3d(k,2))/1.0e-05);
				capid = clusterid[k];
				petid[0] = capid;
			}
		}
	}

	std::cout << '\n';
	min=100.0;
	for (int k = 0; k < num_cluster; k++) {
		if (1.0 <= eval_3d(k,0)/1.0e-05 && eval_3d(k,0)/1.0e-05 < 10.0)
		if(1.0 <= eval_3d(k,1)/1.0e-04 && eval_3d(k,1)/1.0e-04 < 10.0)
		if(1.0 <= eval_3d(k,2)/1.0e-03 && eval_3d(k,2)/1.0e-03 < 10.0) {
			if (fabs((eval_3dcom0[0]-eval_3d(k,0))/1.0e-05)+fabs((eval_3dcom0[1]-eval_3d(k,1))/1.0e-04)+fabs((eval_3dcom0[2]-eval_3d(k,2))/1.0e-03) < min) {
				min = fabs((eval_3dcom0[0]-eval_3d(k,0))/1.0e-05)+fabs((eval_3dcom0[1]-eval_3d(k,1))/1.0e-04)+fabs((eval_3dcom0[2]-eval_3d(k,2))/1.0e-03);
				handleid = clusterid[k];
				petid[1] = handleid;
			}
		}
	}

	std::cout << "capid = " << capid <<'\n';
	ShapeJudge(capid);

	std::cout << "handleid = " << handleid <<'\n';
  ShapeJudge(handleid);
  break;

	case 2:
		std::cout << "select cup" << '\n';
		//body
		eval_3dcom0[0]=5.39474e-05;
		eval_3dcom0[1]=2.95669e-04;
		eval_3dcom0[2]=5.13543e-04;
		min=100.0;
		for (int k = 0; k < num_cluster; k++) {
			if (1.0 <= eval_3d(k,0)/1.0e-05 && eval_3d(k,0)/1.0e-05 < 10.0)
			if(1.0 <= eval_3d(k,1)/1.0e-04 && eval_3d(k,1)/1.0e-04 < 10.0)
			if(1.0 <= eval_3d(k,2)/1.0e-04 && eval_3d(k,2)/1.0e-04 < 10.0) {
				if (fabs((eval_3dcom0[0]-eval_3d(k,0))/1.0e-05)+fabs((eval_3dcom0[1]-eval_3d(k,1))/1.0e-04)+fabs((eval_3dcom0[2]-eval_3d(k,2))/1.0e-04) < min) {
					min = fabs((eval_3dcom0[0]-eval_3d(k,0))/1.0e-05)+fabs((eval_3dcom0[1]-eval_3d(k,1))/1.0e-04)+fabs((eval_3dcom0[2]-eval_3d(k,2))/1.0e-04);
					bodyid = clusterid[k];
					cupid = bodyid;
				}
			}
		}
		std::cout << "bodyid = " << bodyid <<'\n';
		ShapeJudge(bodyid);
		break;
}
	std::cout << "=color=" << '\n';
	std::cout << "#petbottle" << '\n';
	std::cout << "cap : Red" << '\n';
	std::cout << "handle : Green" << '\n';
	std::cout << "#cup" << '\n';
	std::cout << "body : Blue" << '\n';
		delete[] clusterid;

}

void ObjectShape::ShapeJudge(int id) {

	//shape judge
	VectorXd co;
	std::string co_shape;

	VectorXd co_gap=VectorXd::Zero(3);
	VectorXd co_rad=VectorXd::Zero(3);
	MatrixXd co_evec=MatrixXd::Zero(3,3);

	bool allzero=true;
	for(int k=0 ; k<6 ; k++){
		if( fabs(clusterNodes[id].quadCoefficient[k]) > 1.0e-10)	allzero = false;	//最初の6項全てが"1.0e-15"より小さければ"allzero"は"true"
	}

	if( allzero == true )	co_shape = "plane";
	else{
		dmatrix M3 = MatrixXd::Zero(3,3);
		M3 << clusterNodes[id].quadCoefficient[0], clusterNodes[id].quadCoefficient[3]/2, clusterNodes[id].quadCoefficient[5]/2, clusterNodes[id].quadCoefficient[3]/2, clusterNodes[id].quadCoefficient[1], clusterNodes[id].quadCoefficient[4]/2, clusterNodes[id].quadCoefficient[5]/2, clusterNodes[id].quadCoefficient[4]/2, clusterNodes[id].quadCoefficient[2];

		dmatrix evec = MatrixXd::Zero(3,3);
		dvector eval(3);

		double info = calcEigenVectors(M3, evec, eval); //行列"M3"の固有ベクトル"evec"と固有値"evel"を求める関数

		std::cout << "evec[x]" << '\n';
		for (int i = 0; i < 3; i++) {
			std::cout <<evec(i,0) << '\n';
		}

		std::cout << "evec[y]" << '\n';
		for (int i = 0; i < 3; i++) {
			std::cout <<evec(i,1) << '\n';
		}

		std::cout << "evec[z]" << '\n';
		for (int i = 0; i < 3; i++) {
			std::cout <<evec(i,2) << '\n';
		}



		if(evec.determinant() < 0) evec = -evec;

		/*回転行列を"co_evec"に書き込み*/
		for(int i=0 ; i<3 ; i++)
			for(int j=0 ; j<3 ; j++) co_evec(i,j) = evec(i,j);

		double k;
		Vector3 ghj;
		dvector eval_r(3);
		Vector3 gap;
		Vector3 gap2;

		k=clusterNodes[id].quadCoefficient[9];
		ghj << clusterNodes[id].quadCoefficient[6], clusterNodes[id].quadCoefficient[7], clusterNodes[id].quadCoefficient[8];
		ghj=ghj.transpose()*evec;

		for(int i=0 ; i<3 ; i++)
			if(eval[i]!=0) k += -ghj[i]*ghj[i]/(4*eval[i]);
			else k += 0;

		for(int i=0 ; i<3 ; i++) eval_r[i] = -eval[i]/k;

		/*co_gapへの書き込み*/
		for(int i=0 ; i<3 ; i++) co_gap[i] = -1*(ghj[i] / (2*eval[i]));
		co_gap = evec * co_gap;	//元の座標系に戻す

	//	cout << "二次曲面の式は" << endl;
	//	cout << evel[0] << "x^2+" << evel[1] << "y^2+" << evel[2] << "z^2+" << LinerFunction[0] << "x+" << LinerFunction[1] << "y+" << LinerFunction[2] << "z=" << r << endl;

	//	for(int i=0 ; i<3 ; i++) cout << "eval[" << i << "]:" << eval[i] << endl;
	//	cout << "k=" << k << endl;
	//	for(int i=0 ; i<3 ; i++) cout << "eval_r[" << i << "]:" << eval_r[i] << endl;


	//////////////////////２次曲面の判別/////////////////////

		if(k == 0){
	//		cout << "この2次曲面は楕円錐面である" << endl;
		}

		else{

			//////////////////////////////////////////////////
			///   eval_rの中で最も大きい値eval_maxを導出する   ///
			//////////////////////////////////////////////////
			double eval_max=-1;

			for(int i=0 ; i<3 ; i++)
				if( fabs(eval_r[i]) > eval_max ) eval_max = fabs(eval_r[i]);

			if(eval_max==0) eval_max=1;


			////////////////////////////////////////////////////////////////////
			///   eval_maxで各eval_rを割った絶対値が0に近い値か否かで形状を判別する   ///
			////////////////////////////////////////////////////////////////////
	//		for(int i=0 ; i<3 ; i++) eval[i] = fabs(eval_r[i])/eval_max;
			/////////////////////////////////////////////////////////////////
			///   eval_rを１で正規化し,その絶対値が0に近い値か否かで形状を判別する   ///
			/////////////////////////////////////////////////////////////////
			eval = eval_r/norm2(eval_r);

	//			cout << endl;
	//			cout << "eval:" << eval.transpose() << endl;
	//			cout << "eval_r:" << eval_r.transpose() << endl;

			///////////////////////
			///   Aがほぼ０の場合   ///
			///////////////////////
			if( fabs(eval[0])<0.01 ){
				/////////////////////////////////////
				///   BかCもほぼ０の場合 → 二次曲線   ///
				/////////////////////////////////////
				if( fabs(eval[1])<0.01 ||  fabs(eval[2])<0.01 ) co_shape = "quadric line";

				///////////////////////////////
				///   B,Cが正の場合 → 楕円柱   ///
				///////////////////////////////
				else if( eval_r[1]>0 && eval_r[2]>0){
					co_shape = "cylindrical surface_x";
					co_rad[0] = sqrt(1/eval_r[1]);
					co_rad[1] = sqrt(1/eval_r[2]);
					co_rad[2] = 0;
				}

				//////////////////////////////////////////////////////////
				///   上記のいずれでもない場合 → 交差二平面or虚の楕円柱   ///
				/////////////////////////////////////////////////////////
				else co_shape = "NUM";
			}


			///////////////////////
			///   Bがほぼ０の場合   ///
			///////////////////////
			else if( fabs(eval[1])<0.01 ){
				//////////////////////////////////
				///   Cもほぼ０の場合 → 二次曲線   ///
				//////////////////////////////////
				if( fabs(eval[2])<0.01 )co_shape = "quadric line";

				///////////////////////////////
				///   A,Cが正の場合 → 楕円柱   ///
				///////////////////////////////
				else if( eval_r[0]>0 && eval_r[2]>0){
					co_shape = "cylindrical surface_y";
					co_rad[0] = sqrt(1/eval_r[0]);
					co_rad[1] = sqrt(1/eval_r[2]);
					co_rad[2] = 0;
				}

				//////////////////////////////////////////////////////////
				///   上記のいずれでもない場合 → 交差二平面or虚の楕円柱   ///
				/////////////////////////////////////////////////////////
				else co_shape = "NUM";
			}


			///////////////////////
			///   Cがほぼ０の場合   ///
			///////////////////////
			else if( fabs(eval[2])<0.01 ){
				///////////////////////////////
				///   A,Bが正の場合 → 楕円柱   ///
				///////////////////////////////
				if( eval_r[0]>0 && eval_r[1]>0){
					co_shape = "cylindrical surface_z";
					co_rad[0] = sqrt(1/eval_r[0]);
					co_rad[1] = sqrt(1/eval_r[1]);
					co_rad[2] = 0;
				}

				//////////////////////////////////////////////////////////
				///   上記のいずれでもない場合 → 交差二平面or虚の楕円柱   ///
				/////////////////////////////////////////////////////////
				else co_shape = "NUM";
			}


			/////////////////////
			///   Aが正の場合   ///
			////////////////////
			else if( eval_r[0]>0 ){
				/////////////////////
				///   Bが正の場合   ///
				/////////////////////
				if(eval_r[1]>0){
					//////////////////////////////
					///   Cが正の場合 → 楕円体   ///
					/////////////////////////////
					if(eval_r[2]>0){
						int check=0;
						for(int i=0 ; i<3 ; i++){
	//						cout << "ratio : " << (sqrt(1/eval_r[i])/sqrt(1/eval_max)) << endl;
							if( (sqrt(1/eval_r[i])/sqrt(1/eval_max) )>10 ) check=1;
						}

						if(check==0){
							co_shape = "ellipsoid";
							for(int i=0 ; i<3 ; i++) co_rad[i] = sqrt(1/eval_r[i]);
						}
						else co_shape = "Bigger ellipsoid";
					}

					//////////////////////////////////
					///   Cが負の場合 → 一葉双曲面   ///
					/////////////////////////////////
					else{
						co_shape = "OneHyperboloid_z";
						co_rad[0] = sqrt(1/eval_r[0]);
						co_rad[1] = sqrt(1/eval_r[1]);
						co_rad[2] = 0;
					}
				}

				/////////////////////
				///   Bが負の場合   ///
				/////////////////////
				else{
					//////////////////////////////////
					///   Cが正の場合 → 一葉双曲面   ///
					/////////////////////////////////
					if(eval_r[2]>0){
						co_shape = "OneHyperboloid_y";
						co_rad[0] = sqrt(1/eval_r[0]);
						co_rad[1] = 0;
						co_rad[2] = sqrt(1/eval_r[2]);
					}

					////////////////////////////////
					///   Cが負の場合 → 二葉双曲面   ///
					///////////////////////////////
					else co_shape = "TwoHyperboloid_x";
				}
			}

			////////////////////
			///   Aが負の場合   ///
			///////////////////
			else{
				/////////////////////
				///   Bが正の場合   ///
				/////////////////////
				if(eval_r[1]>0){
					////////////////////////////////
					///   Cが正の場合 → 一葉双曲面   ///
					///////////////////////////////
					if(eval_r[2]>0){
						co_shape = "OneHyperboloid_x";
						co_rad[0] = 0;
						co_rad[1] = sqrt(1/eval_r[1]);
						co_rad[2] = sqrt(1/eval_r[2]);
					}

					////////////////////////////////
					///   Cが負の場合 → 二葉双曲面   ///
					////////////////////////////////
					else co_shape = "TwoHyperboloid_y";
				}

				////////////////////
				///   Bが負の場合   ///
				////////////////////
				else{
					////////////////////////////////
					///   Cが正の場合 → 二葉双曲面   ///
					////////////////////////////////
					if(eval_r[2]>0) co_shape = "TwoHyperboloid_z";

					///////////////////////////
					///   Cが負の場合 → 不明   ///
					//////////////////////////
					else co_shape = "NUM";
				}
			}
		}
	}

		cout << "gap : " << co_gap.transpose() << endl;
		cout << "rad : " << co_rad.transpose() << endl;
	//	cout << "evec" << co_evec << endl;
	//	cout << "eval_r:" << endl << eval_r << endl;
		cout << "shape : " << co_shape << endl << endl;


}


void ObjectShape::LimitedcalcClusterBinaryTree(const int nNodes){

	cout << "Clustering Start"<<endl;

//	while( ( nClusterNodes < (nTriangles*2 - 5) ) && (clusterPairHeap.size() > 0) ){
	while(  clusterPairHeap.size() > 0 ){
//		ClusterPair& cp = * clusterPairHeap.front();
		ClusterPair& cp = * clusterPairHeap.front();
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
			if( (cp.error > 0.04) && (nNodes==0)) break;
			if(nNodes >= (nTriangles*2 - nClusterNodes)) break;
			//cout << cp.quadError << cp.quadCoefficient << endl;

//			cout << sumError << endl;

			ClusterImpl& cluster =  clusterNodes[nClusterNodes++];
			cluster.copyParameter(cp);
			cluster.mergeClusterForBinaryTree(*cp.clusters[0],*cp.clusters[1]);

			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();

#ifdef THREAD
			ClusterPair** cpLink = new ClusterPair*[cluster.neighborClusters.size()];
			//ClusterPair* cpLink[cluster.neighborClusters.size()];
			int cnt=0;
			boost::thread_group* thr_grp = new boost::thread_group();
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				thr_grp->create_thread( boost::bind(&thread_process,  &cpLink[cnt++] , &cluster,(*it)));
//				if( (cnt%16)==0 ){
//					thr_grp->join_all();
//					delete thr_grp;
//					thr_grp = new boost::thread_group();
//				}
			}
			thr_grp->join_all();
			delete thr_grp;

			for(int i=0;i<cnt;i++){
				clusterPairHeap.push_back( cpLink[i] );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
//				delete cpLink[i];
			}
			delete[] cpLink;
#else
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				clusterPairHeap.push_back( new ClusterPair( cluster, *(*it) ) );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			}
#endif
			if(nTriangles>100)  if( ((nClusterNodes-nTriangles) % (nTriangles/100) )== 0 ) cout << "\r" << int ( (nClusterNodes-nTriangles) / (nTriangles/100) ) << "% " << flush;
		}
		else{
			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();
		}
	}
	cout << endl<< "Clustering Finish "<<endl;
	for(int i=0;i< nTriangles; i++){
		if(triangles[i].idCluster==-1) continue;
		ClusterImpl* c0= &clusterNodes[triangles[i].idCluster];
		while(! c0->isAliveNode) c0 = c0->parentNode;
		triangles[i].idCluster = c0->id;
	}

	for (int i=0;i< clusterPairHeap.size(); i++ ){
		ClusterPair& cp = *clusterPairHeap[i];
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
#ifdef OUTPUT_COMMENT
			cout <<cp.clusters[0]->id << " " <<cp.clusters[1]->id << " " <<cp.quadError << cp.quadCoefficient << endl;
			//cp.outputClusterQuadParameter();
#endif
		}
	}
	cout << endl ;


	ofstream fout("test.yaml");

	for(int i=0;i<nClusterNodes;i++){
		if(clusterNodes[i].isAliveNode){
#ifdef OUTPUT_COMMENT
			cout << clusterNodes[i].id <<" " << clusterNodes[i].neighborClusters.size() <<" "<<clusterNodes[i].quadError << " "<< clusterNodes[i].quadCoefficient.transpose() << endl;
			clusterNodes[i].calcQuadricSurfaceParameterOutput();
			cout << endl;
#endif
//			clusterNodes[i].calcQuadricSurfaceParameterSecond(clusterNodes[i].quadCoefficient, clusterNodes[i].sumQuadDistribution, clusterNodes[i].sumQuadError);
//			if(clusterNodes[i].quadCoefficient[0]==0 || fabs(clusterNodes[i].quadCoefficient[0]) >= 1.0e-20)
			clusterNodes[i].writeData(fout);
//			cout << endl;
			//cout << clusterNodes[i].sumQuadError << endl;
			//clusterNodes[i].calcQuadricSurfaceParameter();
			//cout << clusterNodes[i].sumQuadDistribution << endl << endl;
//			clusterNodes[i].outputClusterQuadParameter();
		}
	}

	calcVertexPositionOnQuadSurface();

	return;
}



void ObjectShape::calcVertexPositionOnQuadSurface(){

	if(!quadVerticies) quadVerticies = new Vertex[nVerticies];

	for(int j=0;j<nTriangles;j++){
		Triangle& t= triangles[j];
		ClusterImpl& cluster=clusterNodes[t.idCluster];

		for(int i=0; i<3;i++){
			Vector3& pos = quadVerticies[t.ver[i]->id].pos;
			pos[0] = t.ver[i]->pos[0];
			pos[1] = t.ver[i]->pos[1];
			pos[2] = t.ver[i]->pos[2];
		}
		continue;
		if(t.idCluster < nTriangles) continue;

		for(int i=0; i<3;i++){
			dmatrix dp = MatrixXd::Zero(3,10);
			double x= t.ver[i]->pos[0];
			double y= t.ver[i]->pos[1];
			double z= t.ver[i]->pos[2];

			for(int k=0;k<1;k++){
				dvector p(10);
				p[0] = x*x;
				p[1] = y*y;
				p[2] = z*z;
				p[3] = x*y;
				p[4] = y*z;
				p[5] = z*x;
				p[6] = x;
				p[7] = y;
				p[8] = z;
				p[9] = 1.0;

				dp(0,0) = 2.0*x;
				dp(1,1) = 2.0*y;
				dp(2,2) = 2.0*z;
				dp(0,3) = y; dp(1,3) = x;
				dp(1,4) = z; dp(2,4) = y;
				dp(2,5) = x; dp(0,5) = z;
				dp(0,6) = 1.0;
				dp(1,7) = 1.0;
				dp(2,8) = 1.0;

				Vector3& pos = quadVerticies[t.ver[i]->id].pos;

				dvector df (prod (dp,cluster.quadCoefficient));
				double norm = inner_prod( df, df);
				double f0 = inner_prod(p,cluster.quadCoefficient);

				if(norm>0) df = df/sqrt(norm);
				else {
					df = 0.0*df;
					continue;
				}

				dmatrix M3 = MatrixXd::Zero(3,3);
				M3(0,0)=cluster.quadCoefficient[0];
				M3(1,1)=cluster.quadCoefficient[1];
				M3(2,2)=cluster.quadCoefficient[2];
				M3(0,1)=cluster.quadCoefficient[3];
				M3(1,2)=cluster.quadCoefficient[4];
				M3(0,2)=cluster.quadCoefficient[5];
				for(int j1=0;j1<3;j1++){
					for(int k1=0;k1<3;k1++){
						double m=	( M3(k1,j1) + M3(j1,k1) )/2.0;
						M3(k1,j1) = M3(j1,k1) = m;
					}
				}
				dvector b(3),x0(3);
				b[0] = cluster.quadCoefficient[6];
				b[1] = cluster.quadCoefficient[7];
				b[2] = cluster.quadCoefficient[8];
				x0[0] = x;
				x0[1] = y;
				x0[2] = z;

				dmatrix evec = MatrixXd::Zero(3,3);
				dvector eval(3);
				int info = calcEigenVectors(M3, evec, eval);

				if( eval[0] || eval[1] || eval[2] ){
					dmatrix iM3;
					calcPseudoInverse(M3,iM3,1.0e-10);
					//cout << M3 << endl << iM3 << endl << eval << endl << endl;
					double c = cluster.quadCoefficient[9] - inner_prod( b, prod( iM3, b) );
					dvector x1 ( trans(evec) * (x0 - prod ( iM3, b) ) );
					double r[4];
					r[3]=-c*eval[0]*eval[1]*eval[2];
					r[2]=c*(eval[0]*eval[1]+eval[1]*eval[2]+eval[2]*eval[0]);
					r[1]=-c*(eval[0]+eval[1]+eval[2]);
					r[0] = c;
					for(int i=0;i<3;i++){
						r[0] += x1[i]*x1[i]*eval[i];
						r[1] += x1[i]*x1[i]*(eval[i]*eval[(i+1)%3]+eval[i]*eval[(i+2)%3]);
						r[2] += x1[i]*x1[i]*eval[0]*eval[1]*eval[2];
					}
					double s[3];
					if( sol3(&r[0],&s[0])  ){
						double min= s[0];
						if(fabs(s[1]) < fabs(min)) min=s[1];
						if(fabs(s[2]) < fabs(min)) min=s[2];
						dmatrix A2 = MatrixXd::Zero(3,3);
						A2(0,0) = eval[0]/(1.0-min*eval[0]);
						A2(1,1) = eval[1]/(1.0-min*eval[1]);
						A2(2,2) = eval[2]/(1.0-min*eval[2]);
						dvector n   ( (dvector)prod ( evec, (dvector)prod( A2 , (dvector)prod( trans(evec) ,x1 ) ) ) ) ;
						//cout << n << endl;
						if( inner_prod (n,n) ){
							df = n;
						}
						else{
							df*=0;
						}
					}
				}

				double a[3],s[2];
				a[2] = inner_prod ( df, prod( M3, df) ) ;
				a[1] = ( 2.0*inner_prod ( df, prod( M3, x0) ) + inner_prod (b , df) );
				a[0] = f0 ;

//				cout << "keisu " <<a[2] << "  " << a[1] << " " << a[0] << endl;
				if( sol2(&a[0],&s[0])  ){
					if(fabs(s[0]) < fabs(s[1]) ){
						df *= s[0];
					}else{
						df *=s[1];
					}
//					cout << "r " << s[0]*s[0]*a[2]+s[0]*a[1]+a[0] << endl;
				}
				else{
					df*=0;
				}
//				cout <<"TPOS" << s[0] <<" "<<s[1] << " " <<f0 << " " << norm  <<df << endl;

				pos[0] = pos[0] +df[0];
				pos[1] = pos[1] +df[1];
				pos[2] = pos[2] +df[2];
			}

		}

	}
}

void ObjectShape::calcVertexPositionOnOtherSurface(const dmatrix& qAffine, const dvector& qTrans){



	for(int j=0;j<nTriangles;j++){
		Triangle& t= triangles[j];
		ClusterImpl& cluster=clusterNodes[t.idCluster];

		for(int i=0; i<3;i++){
			Vector3& pos = quadVerticies[t.ver[i]->id].pos;
			pos[0] = t.ver[i]->pos[0];
			pos[1] = t.ver[i]->pos[1];
			pos[2] = t.ver[i]->pos[2];
		}
		//continue;
		if(t.idCluster < nTriangles) continue;

		for(int i=0; i<3;i++){
			double x= t.ver[i]->pos[0];
			double y= t.ver[i]->pos[1];
			double z= t.ver[i]->pos[2];

			dvector p(9), q(9);
			p[0] = x*x;
			p[1] = y*y;
			p[2] = z*z;
			p[3] = x*y;
			p[4] = y*z;
			p[5] = z*x;
			p[6] = x;
			p[7] = y;
			p[8] = z;
//			p[9] = 1.0;
			q = prod( qAffine, p) + qTrans;
			Vector3& pos = quadVerticies[t.ver[i]->id].pos;
			pos[0] = q[6];
			pos[1] = q[7];
			pos[2] = q[8];
		}

	}
}

void ObjectShape::calcClusterIncompleteModel(int nClusters,double volRatioThreshold,double magnification){
	double dist_tolerance = 0.001 * magnification; // If the distance between clusters is less than "dist_tolerance", these cluster pairs are candidate to be merged.

	makeNbr();

	thickness = 0.005 * magnification;
	generateClusterSeeds(1);
	mergeClustersFromSeed();

	generateInitialClusterCandidate(dist_tolerance);
	calcClusterBinaryTreeFromClusteredNodes(nClusters,volRatioThreshold);
}

void ObjectShape::generateInitialClusterCandidate(double dist_threshold){
	int num_cluster = parentList.size();

	clusterNodes = new ClusterImpl[num_cluster*2];
	nClusterNodes=num_cluster;

	for(int i=0;i<num_cluster*2;i++){
		clusterNodes[i].id = i;
	}

	int n_cnt=0;
	for(int i=0;i<num_cluster;i++){
		for(int j=0;j<clusters[parentList[i]].triangleList.size();j++){
			clusters[parentList[i]].triangleList[j]->idCluster = n_cnt;
		}
		clusterNodes[n_cnt++] = clusters[parentList[i]];
	}
	for(int i=0;i<num_cluster;i++){
		clusterNodes[i].calcQuadricAreaParameter();
	}

#ifdef THREAD
	ClusterPair** cpLink = new ClusterPair*[num_cluster * num_cluster];
	int cnt=0;
	boost::thread_group* thr_grp = new boost::thread_group();
	for(int i=0;i<num_cluster;i++){
		for(int j=0;j<num_cluster;j++){
			if(i==j) continue;
			if(!isNeighbor(&clusterNodes[i],&clusterNodes[j],dist_threshold)) continue;
			clusterNodes[i].neighborClusters.push_back(&clusterNodes[j]);
			if(i<j)thr_grp->create_thread(boost::bind(&thread_process, &cpLink[cnt++],&clusterNodes[i],&clusterNodes[j]));
			if(  cnt%16 ==0 ){
				thr_grp->join_all();
				delete thr_grp;
				thr_grp=  new boost::thread_group();
			}
		}
	}
	thr_grp->join_all();
	delete thr_grp;

	cout << endl <<"the number of cluster pair " << endl;

	for(int i=0;i<cnt;i++){
		clusterPairHeap.push_back( cpLink[i] );
	}
	delete[] cpLink;
#else
	for(int i=0;i<num_cluster;i++){
		for(int j=0;j<num_cluster;j++){
			if(i==j) continue;
			if(!isNeighbor(&clusterNodes[i],&clusterNodes[j],dist_threshold)) continue;
			clusterNodes[i].neighborClusters.push_back(&clusterNodes[j]);
			if(i<j) clusterPairHeap.push_back(new ClusterPair(clusterNodes[i],clusterNodes[j]));
		}
	}
#endif

	make_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );


	cout << endl <<"the number of cluster pair "<< clusterPairHeap.size() << endl;
	for(int i=0; i < clusterPairHeap.size(); i++){
		clusterPairHeap[i]->id= i;
	}
}

bool ObjectShape::isNeighbor(ClusterImpl* c1,ClusterImpl* c2,double threshold,bool useVer2TriDist){
	double sq_th = threshold * threshold;

	double min_dist = DBL_MAX;

	for(int i=0;i<c1->boundaryList.size();i++){
		for(int k=0;k<c1->boundaryList[i].size();k++){
			for(int j=0;j<c2->boundaryList.size();j++){
				for(int l=0;l<c2->boundaryList[j].size();l++){
					Vector3 diff = c1->boundaryList[i][k]->pos - c2->boundaryList[j][l]->pos;
					double dist = (diff(0)*diff(0)+diff(1)*diff(1)+diff(2)*diff(2));
					if(dist <= sq_th) return true;
					if(dist < min_dist) min_dist = dist;
				}
			}
		}
	}

	if(!useVer2TriDist) return false;
	double max_len = std::max(c1->bbedge.maxCoeff(),c2->bbedge.maxCoeff());
	if(max_len * max_len < min_dist) return false;

	for(int i=0;i<c1->boundaryList.size();i++){
		for(int k=0;k<c1->boundaryList[i].size();k++){
			Vector3 p = c1->boundaryList[i][k]->pos;
			Vector3 p_out;
			if(c2->lineIntersectToCluster(p,c2->normal,p_out)){
				Vector3 diff = p-p_out;
				double dist = (diff(0)*diff(0)+diff(1)*diff(1)+diff(2)*diff(2));
				if(dist <= sq_th) return true;
			}
		}
	}

	for(int i=0;i<c2->boundaryList.size();i++){
		for(int k=0;k<c2->boundaryList[i].size();k++){
			Vector3 p = c2->boundaryList[i][k]->pos;
			Vector3 p_out;
			if(c1->lineIntersectToCluster(p,c1->normal,p_out)){
				Vector3 diff = p-p_out;
				double dist = (diff(0)*diff(0)+diff(1)*diff(1)+diff(2)*diff(2));
				if(dist <= sq_th) return true;
			}
		}
	}

	return false;
}

void ObjectShape::calcClusterBinaryTreeFromClusteredNodes(const int nNodes,double volRatioThreshold){
	int num_cluster = parentList.size();
	cout << "Clustering Start"<<endl;

	for(int i=0;i<nClusterNodes;i++){
		Vector3 edge,center;
		Matrix3 R;
		calcBoundingBox(i,edge,center,R);
		clusterNodes[i].bbcenter = center;
		clusterNodes[i].bbedge = edge;
		clusterNodes[i].bbR = R;
	}

	while(  clusterPairHeap.size() > 0 ){
		ClusterPair& cp = * clusterPairHeap.front();
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
			if( (cp.error > 0.01) && (nNodes==0)) break;
			if(nNodes >= (num_cluster*2 - nClusterNodes)) break;
			if(cp.error > 0.01){
				double ratio = calcOverlapVolumeRatio();
#ifdef DEBUG_PRINT
				cout << "error:" << cp.error << endl;
				cout << "ratio:" << ratio << endl;
#endif
				if(ratio < volRatioThreshold) break;
			}

			ClusterImpl& cluster =  clusterNodes[nClusterNodes++];
			cluster.copyParameter(cp);
			for(int i=0;i<cp.clusters[0]->triangleList.size();i++){
				cluster.addTriangleToCluster(*(cp.clusters[0]->triangleList[i]));
			}
			for(int i=0;i<cp.clusters[1]->triangleList.size();i++){
				cluster.addTriangleToCluster(*(cp.clusters[1]->triangleList[i]));
			}
			cluster.mergeClusterForBinaryTree(*cp.clusters[0],*cp.clusters[1]);

			Vector3 edge,center;
			Matrix3 R;
			calcBoundingBox(cluster.id,edge,center,R);
			cluster.bbcenter = center;
			cluster.bbedge = edge;
			cluster.bbR = R;

			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();

#ifdef THREAD
			ClusterPair** cpLink = new ClusterPair*[cluster.neighborClusters.size()];
			int cnt=0;
			boost::thread_group* thr_grp = new boost::thread_group();
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				thr_grp->create_thread( boost::bind(&thread_process,  &cpLink[cnt++] , &cluster,(*it)));
			}
			thr_grp->join_all();
			delete thr_grp;

			for(int i=0;i<cnt;i++){
				clusterPairHeap.push_back( cpLink[i] );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			}
			delete[] cpLink;
#else
			for(list<ClusterImpl*>::iterator it = cluster.neighborClusters.begin();  it != cluster.neighborClusters.end(); it++ ){
				clusterPairHeap.push_back( new ClusterPair( cluster, *(*it) ) );
				push_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			}
#endif
		}
		else{
			delete clusterPairHeap.front();
			pop_heap ( clusterPairHeap.begin(), clusterPairHeap.end(), ClusterPair::compClusterHeap );
			clusterPairHeap.pop_back();
		}
	}
	cout << endl<< "Clustering Finish "<<endl;
	for(int i=0;i< nTriangles; i++){
		ClusterImpl* c0= &clusterNodes[triangles[i].idCluster];
		if(c0 == NULL) continue;
		while(! c0->isAliveNode) c0 = c0->parentNode;
		triangles[i].idCluster = c0->id;
	}

	for (int i=0;i< clusterPairHeap.size(); i++ ){
		ClusterPair& cp = *clusterPairHeap[i];
		if(cp.clusters[0]->isAliveNode && cp.clusters[1]->isAliveNode ){
			cout <<cp.clusters[0]->id << " " <<cp.clusters[1]->id << " " <<cp.quadError << cp.quadCoefficient << endl;
			//cp.outputClusterQuadParameter();
		}
	}
	cout << endl ;

	ofstream fout("test.yaml");

	for(int i=0;i<nClusterNodes;i++){
		if(clusterNodes[i].isAliveNode){
			cout << clusterNodes[i].id <<" " << clusterNodes[i].neighborClusters.size() <<" "<<clusterNodes[i].quadError << " "<< clusterNodes[i].quadCoefficient.transpose() << endl;
			clusterNodes[i].calcQuadricSurfaceParameterOutput();
			cout << endl;
			clusterNodes[i].writeData(fout);
//			cout << endl;
			//cout << clusterNodes[i].sumQuadError << endl;
			//clusterNodes[i].calcQuadricSurfaceParameter();
			//cout << clusterNodes[i].sumQuadDistribution << endl << endl;
//			clusterNodes[i].outputClusterQuadParameter();
		}
	}

	calcVertexPositionOnQuadSurface();

	return;
}

double ObjectShape::calcOverlapVolumeRatio()
{
	int max_index=-1;
	double max_vol = 0;

	for(int i=0;i<nClusterNodes;i++){
		if(!clusterNodes[i].isAliveNode) continue;
		double tmp_vol = clusterNodes[i].bbedge(0) * clusterNodes[i].bbedge(1) * clusterNodes[i].bbedge(2);
		if(tmp_vol > max_vol){
			max_vol = tmp_vol;
			max_index = i;
		}
	}

	if(max_vol == 0) return 1.0;

	double over_lap_vol = 0;

	for(int i=0;i<nClusterNodes;i++){
		if(!clusterNodes[i].isAliveNode) continue;
		if(i==max_index) continue;
		OverlapVolumeCalculator ovc;
		double vol = ovc.calcOverlapVolume(clusterNodes[max_index],clusterNodes[i]);
		over_lap_vol += vol;
	}
	return over_lap_vol/max_vol;
}


Matrix3 makeOrthogonal(const MatrixXd A, const VectorXd b)
{
	vector<double> c;
	for(size_t i=0; i<b.size(); i++)
			c.push_back(fabs(b(i)));

	Matrix3 A_ = d2v(A);
	int j=argmax(c);
	Vector3 v0 = A_.col(j);
	Vector3 v1 = A_.col((j+1)%3);

	double r=dot(v0,v1);
	v1 = (-r*v0 + v1)/sqrt(1-r*r);
	Vector3 v2 = cross(v0,v1);

	for(int i=0; i<3; i++){
		A_(i, (j+1)%3) = v1(i);
		A_(i, (j+2)%3) = v2(i);
	}

	return A_;
}

void ObjectShape::calcBoundingBox(Vector3 &edge, Vector3& center, Matrix3& Rot){
	std::vector<Triangle*> trgt_triangles;
	for(int i=0;i<nTriangles;i++){
		trgt_triangles.push_back(&(triangles[i]));
	}
	calcBoundingBoxImpl(trgt_triangles,edge,center,Rot);
}

void ObjectShape::calcBoundingBox(int cluster_id,Vector3 &edge, Vector3& center, Matrix3& Rot){
	std::vector<Triangle*> trgt_triangles;
	for(int i=0;i<nTriangles;i++){
		if(triangles[i].idCluster == cluster_id){
			trgt_triangles.push_back(&(triangles[i]));
		}
	}
	calcBoundingBoxImpl(trgt_triangles,edge,center,Rot);
}

void ObjectShape::calcBoundingBoxImpl(std::vector<Triangle*>& trgt_triangles, Vector3 &edge, Vector3& center, Matrix3& Rot){
	Vector3 pt;
	MatrixXd distribute = MatrixXd::Zero(3, 3);
	Vector3 average(0, 0, 0);

	for(int i=0;i<trgt_triangles.size();i++){
		Vector3 e1 (trgt_triangles[i]->ver[1]->pos - trgt_triangles[i]->ver[0]->pos);
		Vector3 e2 (trgt_triangles[i]->ver[2]->pos - trgt_triangles[i]->ver[0]->pos);
		trgt_triangles[i]->area = norm2 ( cross(e1,e2) ) /2.0;
	}


	Matrix3 aq,aq_n_sum,aq_p_sum;
	std::vector<double>* aq_p[3][3];
	std::vector<double>* aq_n[3][3];
	Vector3 sumCenter(0,0,0);
	double sumArea=0;

	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		aq(i,j)=0;
		aq_p_sum(i,j)=0;
		aq_n_sum(i,j)=0;
		aq_p[i][j] = new std::vector<double>();
		aq_n[i][j] = new std::vector<double>();
	}

	for(int l=0;l<trgt_triangles.size();l++){
		Triangle* t = trgt_triangles[l];
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				for(int k=0;k<3;k++){
					double tmp_aq;
					tmp_aq = t->area*(t->ver[i]->pos[j] * t->ver[i]->pos[k])/6.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t->area*(t->ver[i]->pos[j] * t->ver[(i+1)%3]->pos[k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
					tmp_aq = t->area*(t->ver[(i+1)%3]->pos[j] * t->ver[i]->pos[k])/12.0;
					tmp_aq > 0 ? aq_p[j][k]->push_back(tmp_aq) : aq_n[j][k]->push_back(tmp_aq);
				}
			}
		}
		sumArea +=t->area;
		sumCenter  =  sumCenter + t->area/3.0* Vector3 ( t->ver[0]->pos + t->ver[1]->pos + t->ver[2]->pos);
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
		std::sort(aq_p[i][j]->begin(),aq_p[i][j]->end(),LessAbs());
		for(int n=0;n<aq_p[i][j]->size();n++){
			aq_p_sum(i,j) += aq_p[i][j]->at(n);
		}
		std::sort(aq_n[i][j]->begin(),aq_n[i][j]->end(),LessAbs());
		for(int n=0;n<aq_n[i][j]->size();n++){
			aq_n_sum(i,j) += aq_n[i][j]->at(n);
		}
		delete aq_p[i][j];
		delete aq_n[i][j];
	}
	for(int i=0;i<3;i++) for(int j=0;j<3;j++) {aq(i,j) = aq_p_sum(i,j) + aq_n_sum(i,j);}
	average = sumCenter/sumArea;
	for(int j=0;j<3;j++){
		for(int k=0;k<3;k++){
			distribute(j,k) = aq(j,k) - average[j] * average[k] * sumArea;
		}
	}

	MatrixXd evec(3, 3);
	VectorXd eval(3);
	int info;
	info = calcEigenVectors(distribute, evec, eval);

	Rot = makeOrthogonal(evec, eval);

	Vector3 e[3];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			e[j][i] = Rot(i, j);

	Vector3 pt_max(0, 0, 0), pt_min(0, 0, 0);

	for(int l=0;l<trgt_triangles.size();l++){
		for(int k=0;k<3;k++){
			Vector3 pt = trgt_triangles[l]->ver[k]->pos;
			for (int j = 0; j < 3; j++) {
				double tmp = dot(e[j], Vector3(pt - average));
				if (tmp > pt_max[j]) pt_max[j] = tmp;
				if (tmp < pt_min[j]) pt_min[j] = tmp;
			}
		}
	}

	edge =  (pt_max - pt_min);
	center =  average + 0.5 * Rot * (pt_max + pt_min);

	return;
}

void ObjectShape::getClusterParameters(int clusters,vector<Vector3>* edges,vector<Vector3>* centers,vector<Matrix3>* rots,double volRatioThreshold,double magnification){
	calcClusterIncompleteModel(clusters,volRatioThreshold,magnification);

	for(int i=0;i<nClusterNodes;i++){
		ClusterImpl* cluster_node = &clusterNodes[i];
		if(cluster_node->isAliveNode){
			Vector3 edge;
			Vector3 center;
			Matrix3 rot;
			calcBoundingBox(cluster_node->id,edge,center,rot);
			edges->push_back(edge);
			centers->push_back(center);
			rots->push_back(rot);
		}
	}
}

void ObjectShape::getClusterTrianglePoints(int clusters,vector<Vector3>& points,vector<vector<int> >& pids,double volRatioThreshold,double magnification){
	calcClusterIncompleteModel(clusters,volRatioThreshold,magnification);
	points.clear();
	pids.clear();

	int ids = 0;
	for(int i=0;i<nClusterNodes;i++){
		ClusterImpl* cluster_node = &clusterNodes[i];
		if(cluster_node->isAliveNode){
			vector<int> pid;
			for(int i=0;i<nTriangles;i++){
				if(triangles[i].idCluster == cluster_node->id){
					for(int k = 0; k < 3; k++){
						points.push_back(triangles[i].ver[k]->pos);
						pid.push_back(ids++);
					}
				}
			}
			pids.push_back(pid);
		}
	}
}

void ObjectShape::getClusterQuadParameters(int clusters,vector<Matrix3>* qrot,vector<Vector3>* qcenter,vector<Vector3>* qradius,vector<double>* qscale,double volRatioThreshold,double magnification){
	calcClusterIncompleteModel(clusters,volRatioThreshold,magnification);

	for(int i=0;i<nClusterNodes;i++){
		ClusterImpl* cluster_node = &clusterNodes[i];
		if(cluster_node->isAliveNode){
			cluster_node->calcQuadricSurfaceParameter();
			qrot->push_back(cluster_node->quadRot);
			qcenter->push_back(cluster_node->quadCenter);
			qradius->push_back(cluster_node->quadRadius);
			qscale->push_back(cluster_node->quadScale);
		}
	}
}

void ObjectShape::generateContactCluster(const Vector3& contact_point,const Vector3& N,int& contact_cluster_id,bool PlaneMode)
{
	vector<int> tids;
	bool is_corner = isCornerPoint(contact_point,tids);
	if(is_corner){
		double max_dot = 0;
		for(int i=0;i<tids.size();i++){
			double tmp_dot = (dot(N,triangles[tids[i]].normal));
			if(tmp_dot > max_dot){
				max_dot = tmp_dot;
				contact_cluster_id = tids[i];
			}
		}
	}else{
		double min_dist = DBL_MAX;
		for(int i=0;i<nTriangles;i++){
			if(dot(triangles[i].normal,N) < 0.9) continue;
			Vector3 center = (triangles[i].ver[0]->pos + triangles[i].ver[1]->pos + triangles[i].ver[2]->pos) / 3;
			Vector3 intersect = intersectionPoint(contact_point,N,triangles[i].ver[0]->pos,triangles[i].normal);
			double dist = DBL_MAX;
			Vector3 Pout;
			if(!twoSegmentsIntersect(triangles[i].ver[0]->pos,triangles[i].ver[1]->pos,center,intersect,triangles[i].normal,Pout) &&
				!twoSegmentsIntersect(triangles[i].ver[1]->pos,triangles[i].ver[2]->pos,center,intersect,triangles[i].normal,Pout) &&
				!twoSegmentsIntersect(triangles[i].ver[2]->pos,triangles[i].ver[0]->pos,center,intersect,triangles[i].normal,Pout)){
				dist = norm2(intersect-contact_point);
			}else{
				for(int j=0;j<3;j++){
					dist = min(dist,norm2(contact_point-triangles[i].ver[j]->pos));
				}
			}
			if(dist < min_dist){
				min_dist = dist;
				contact_cluster_id = i;
			}
		}
	}

	while(true){
		bool withinthreshold = true;
		vector<double> normalList;
		vector<int>    numberList;
		for(unsigned int i=0; i<clusters[contact_cluster_id].neighborList.size(); i++){
			if(  clusters[contact_cluster_id].neighborList[i]->idCluster == -1) continue;
			if(  included(clusters[contact_cluster_id].neighborList[i]->idCluster, numberList) ) continue;
			normalList.push_back( fabs(dot(triangles[contact_cluster_id].normal, clusters[clusters[contact_cluster_id].neighborList[i]->idCluster].normal)));
			numberList.push_back(clusters[contact_cluster_id].neighborList[i]->idCluster);
		}

		if(normalList.size()==0) break;

		sort_by(numberList, normalList);

		int id2;
		if(PlaneMode){
			id2 = numberList[normalList.size()-1];
			if(fabs(dot(triangles[contact_cluster_id].normal,clusters[id2].normal)) < 0.99) withinthreshold = false;
		}else{
			for(int j=normalList.size()-1;j>=0;j--){
				id2 = numberList[j];
				withinthreshold= true;
				for(unsigned int i=0;i<clusters[id2].triangleList.size();i++){
					double cluster_thickness = clusters[contact_cluster_id].clusterTriangleDistance(*(clusters[id2].triangleList[i]));
					if(cluster_thickness > thickness){
						withinthreshold = false;
						break;
					}
				}
				if(withinthreshold){
					break;
				}
			}
		}

		if(!withinthreshold) break;

		ClusterImpl cluster(contact_cluster_id);

		cluster.calcTriangleList(clusters[contact_cluster_id], clusters[id2]);
		cluster.calcNeighborList(clusters[contact_cluster_id], clusters[id2]);
		cluster.calcClusterNormal();
		cluster.calcClusterBoundary();
		cluster.calcClusterBoundingBox();
		clusters[contact_cluster_id] = cluster;
	}
}

bool ObjectShape::isCornerPoint(const Vector3& point,vector<int>& contact_tids) const
{
	contact_tids.clear();
	double eps = 0.00001;

	for(int i=0;i<nTriangles;i++){
		for(int j=0;j<3;j++){
			if(minDistancePointLine(triangles[i].ver[j]->pos,triangles[i].ver[(j+1)%3]->pos,point) < eps){
				contact_tids.push_back(i);
				break;
			}
		}
	}

	if(contact_tids.size() < 2) return false;

	double min_dot = 1.0;

	for(int i=0;i<contact_tids.size()-1;i++){
		for(int j=i+1;j<contact_tids.size();j++){
			double tmp_dot = dot(triangles[contact_tids[i]].normal,triangles[contact_tids[j]].normal);
			if(min_dot > tmp_dot) min_dot = tmp_dot;
		}
	}

	//return ture if angle between normals is lesser than 75 degree
	if(min_dot < 0.259) return true;
	return false;
}

double ObjectShape::calcContactArea(const Vector3& contact_point,const Vector3& N)
{
	int target_id = 0;
	double area = 0;

	makeNbr();
	generateClusterSeeds(1);

	generateContactCluster(contact_point,N,target_id);

	Vector3 edge;
	Vector3 center;
	Matrix3 rot;
	calcBoundingBox(target_id,edge,center,rot);

	vector<double> normallist;
	normallist.push_back(fabs(dot(rot*Vector3(1,0,0),N)));
	normallist.push_back(fabs(dot(rot*Vector3(0,1,0),N)));
	normallist.push_back(fabs(dot(rot*Vector3(0,0,1),N)));

	if((normallist[0] > normallist[1]) && (normallist[0] > normallist[2])){
		area = edge(1)*edge(2);
	}else if((normallist[1] > normallist[2]) && (normallist[1] > normallist[0])){
		area = edge(0)*edge(2);
	}else{
		area = edge(0)*edge(1);
	}

	return area;
}

std::vector<std::vector<Vector3> > ObjectShape::getContactRegionBoundaryPoints(const Vector3& contact_point,const Vector3& N,int& tid)
{
	thickness = 0.003;

	makeNbr();
	generateClusterSeeds(1);

	// add close triangles to neighborList
	for(int i=0;i<clusters.size()-1;i++){
		for(int j=i+1;j<clusters.size();j++){
			if(isNeighbor(&clusters[i],&clusters[j],thickness,true)){
				clusters[i].neighborList.push_back(clusters[j].triangleList[0]);
				clusters[j].neighborList.push_back(clusters[i].triangleList[0]);
			}
		}
	}

	generateContactCluster(contact_point,N,tid,false);

	return getBoundaryPoints(tid);
}

std::vector<std::vector<Vector3> > ObjectShape::getBoundaryPoints(int cid)
{
	vector<vector<Vector3> > boundary_points_vec;
	vector<Vector3> boundary_points;
  clusters[cid].calcClusterBoundaryWithHole();
	vector<vector<VertexLink*> > boundary =  clusters[cid].boundaryList;

	// In the case of closed cluster
	if(boundary.size() == 0){
		Vector3 edge;
		Vector3 center;
		Matrix3 rot;
		calcBoundingBox(edge,center,rot);
		int min,max1,max2;
		edge = edge / 2.0;
		if(edge(0) < edge(1) && edge(0) < edge(2)){
			min = 0;max1 = 1;max2 = 2;
		}else if(edge(1) < edge(2) && edge(1) < edge(0)){
			min = 1;max1 = 0;max2 = 2;
		}else{
			min = 2;max1 = 0;max2 = 1;
		}
		edge(min) = 0;
		boundary_points.push_back(center + rot * edge);
		edge(max1) = -1 * edge(max1);
		boundary_points.push_back(center + rot * edge);
		edge(max2) = -1 * edge(max2);
		boundary_points.push_back(center + rot * edge);
		edge(max1) = -1 * edge(max1);
		boundary_points.push_back(center + rot * edge);
		boundary_points_vec.push_back(boundary_points);
		return boundary_points_vec;
	}

	/*vector<double> maxEdge;
	for(size_t i=0; i<boundary.size(); i++){
		Vector3 av(0,0,0);
		for(unsigned int j=0; j<boundary[i].size();j++)
			av += boundary[i][j]->pos;
		av = av/(double)boundary[i].size();

		vector<double> edge;
		for(unsigned int j=0; j<boundary[i].size(); j++)
			edge.push_back(norm2(boundary[i][j]->pos-av));

		maxEdge.push_back(vmax(edge));
	}
	vector<VertexLink*> max_boundary = boundary[argmax(maxEdge)];

	for(size_t i=0;i<max_boundary.size();i++){
		boundary_points.push_back(max_boundary[i]->pos);
	}*/

	for (size_t i = 0; i < boundary.size(); i++) {
		boundary_points.clear();
		for (size_t j = 0; j < boundary[i].size(); j++) {
			boundary_points.push_back(boundary[i][j]->pos);
		}
		boundary_points_vec.push_back(boundary_points);
	}

	return boundary_points_vec;
}

double ObjectShape::minDistancePointLine(const Vector3& lp1,const Vector3& lp2,const Vector3& p) const
{
	if(dot((lp2-lp1),(p-lp1))<0) return norm2(p-lp1);
	if(dot((lp1-lp2),(p-lp2))<0) return norm2(p-lp2);
	return norm2(cross(lp2-lp1,p-lp1)) / norm2(lp2-lp1);
}

void ObjectShape::saveClusters(std::ofstream& fout, int depth){

	vector<ClusterImpl*> saveClusterList;
	if(depth > 1){
		calcClusterBinaryTree(1);
		saveClusterList.push_back(&clusterNodes[nClusterNodes-1]);
		for( int i=1; i<depth;i++){
			vector<ClusterImpl*> cl;
			for(int j=0;j<saveClusterList.size();j++){
				if(saveClusterList[j] == NULL){
					cl.push_back(NULL);
					cl.push_back(NULL);
					continue;
				}
				saveClusterList[j]->isAliveNode=false;
				if( saveClusterList[j]->childNodes[0]==NULL){
					cl.push_back(saveClusterList[j]);
					cl.push_back(NULL);
					continue;
				}
				cl.push_back(saveClusterList[j]->childNodes[0]);
				cl.push_back(saveClusterList[j]->childNodes[1]);
			}
			saveClusterList = cl;
		}
		for(int j=0;j<saveClusterList.size();j++){
			if(saveClusterList[j] == NULL) continue;
			saveClusterList[j]->isAliveNode=true;
			saveClusterList[j]->id = j;
		}
	}else{
		int id=0;
		for(int i=0;i<nClusterNodes;i++){
			if( clusterNodes[i].isAliveNode == false) continue;
			clusterNodes[i].id = id++;
			saveClusterList.push_back(&clusterNodes[i]);
		}
	}
	for(int i=0;i< nTriangles; i++){
		triangles[i].idCluster = i;
	}
	for(int i=0;i< nTriangles; i++){
		if(triangles[i].idCluster==-1) continue;
		ClusterImpl* c0= &clusterNodes[triangles[i].idCluster];
		while(! c0->isAliveNode) c0 = c0->parentNode;
		triangles[i].idCluster = c0->id;
	}
	fout << "clusters: " << endl;

	for(int j=0;j<saveClusterList.size();j++){
		if(saveClusterList[j] == NULL) continue;
		saveClusterList[j]->writeData(fout);
		cout << "test" << endl;
	}
	fout << "faceClusterId: " << endl;
	for(int i=0;i< nTriangles; i++){
		fout << "  - " << triangles[i].idCluster << endl;
	}
}

bool ObjectShape::loadClusters(const cnoid::Mapping& clusterSetting){
	cnoid::ValueNode* clusters = clusterSetting.find("clusters");
	if( !clusters->isListing() ) return false;

	const cnoid::Listing& clusterslist = *(clusters->toListing());
	int nInitialClusters= clusterslist.size();

	clusterNodes = new ClusterImpl[nInitialClusters*2];
	nClusterNodes=0;
	for(int i=0;i<nInitialClusters*2;i++){
		clusterNodes[i].id = i;
	}
	for(int i=0;i<clusterslist.size();i++){
		cout  <<"initial cluster " <<  i<< endl;
	}
	nClusterNodes=nInitialClusters;

	cnoid::ValueNode* clusterid = clusterSetting.find("faceClusterId");
	if( !clusterid->isListing() ){
		cout << "there is no face cluster id list" << endl;
		return false;
	}
	const cnoid::Listing& clustersidlist = *(clusterid->toListing());
	if(clustersidlist.size() != nTriangles){
		cout << "size of cluster id list is wrong " << clustersidlist.size() << " " << nTriangles << endl;
		return false;
	}

	for(int i=0;i<nTriangles;i++){
		triangles[i].idCluster= clustersidlist[i].toInt();
	}

	cnoid::ValueNode* clustergroup = clusterSetting.find("clusterGroups");
	if( clustergroup->isListing() ){
		cout << "cluster group list" << endl;
		const cnoid::Listing& cgroup = *(clustergroup->toListing());
		vector<int> clusterNewId(nClusterNodes);
		for(int j=0;j<cgroup.size();j++){
			cout << cgroup.size() << endl;
			cout << cgroup[j].toListing()->size() << endl;
			for(int k=0;k<cgroup[j].toListing()->size();k++){
				cout << (*cgroup[j].toListing())[k].toInt() << endl;
				clusterNewId[(*cgroup[j].toListing())[k].toInt()] = j;
			}
		}
		for(int i=0;i<nTriangles;i++){
			triangles[i].idCluster= clusterNewId[triangles[i].idCluster];
		}
//		nClusterNodes = clustersgrouplist.size();
	}


/*
	for(int i=0;i<nTriangles;i++){
		clusterNodes[triangles[i].idCluster].addTriangleToCluster(triangles[i]);
		clusterNodes[i].calcQuadricAreaParameter();
#ifdef CENTER_SHIFT
		clusterNodes[i].calcQuadricAreaParameterLow();
#endif

		if(nTriangles>100)  if(  ( i % (nTriangles /100) ) == 0 ) cout << "\r" << (i / (nTriangles /100)) << "% " << flush;
	}
*/
	return true;
}

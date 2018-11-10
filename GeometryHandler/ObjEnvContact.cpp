// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include "ObjEnvContact.h"
#include "FindParallelPlane.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <cnoid/MessageView>
//#include <cnoid/SceneBody>
//#include <cnoid/SceneView>
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>
#include <cnoid/OSGSceneView>
#endif

//#define DEBUG

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

void ObjEnvContact::calcPlaneClusters(ObjectShape& object)
{
		object.thickness = 0.005;

		object.generateClusterSeeds(1);
		object.mergeClustersFromSeed(false);

		calcClusterConvexity(object);
		calcTableLegCluster(object);
		calcClusterConvexHull(object);

		for(size_t i=0; i<object.parentList.size(); i++)
				object.clusters[object.parentList[i]].Put = 0;

		//writeResult(object);
}

int ObjEnvContact::calcOuterBoundary(vector<vector<Vector3> >& boundary){

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

bool ObjEnvContact::lineIntersectToCluster(const Vector3& p, const Vector3& n, Vector3& p_out, const vector<Vector3>& Pset, bool ccw){

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
}

int ObjEnvContact::calcTargetCluster(ObjectShape& object)
{

		FindParallelPlane *fp = FindParallelPlane::instance();
		Vector3 pos = fp->objPressPos;
		Vector3 nor = fp->objPressNormal;

		for(unsigned int i=0; i<object.parentList.size(); i++){

				ClusterImpl c1 = object.clusters[object.parentList[i]];

				double eps = 0.0001, eps2 = 0.2;
				Vector3 x = c1.eigenVector[0]; //tangent;
				Vector3 y = cross(c1.normal, x);
				Vector3 z = c1.normal;
				Vector3 e = c1.bbedge;
				Vector3 c = c1.bbcenter;

				if(dot(z, nor) > eps2) continue;
				if(dot(pos,x)<dot(c,x)-e(0)-eps || dot(pos,x)>dot(c,x)+e(0)+eps ) continue;
				if(dot(pos,y)<dot(c,y)-e(1)-eps || dot(pos,y)>dot(c,y)+e(1)+eps ) continue;
				if(dot(pos,z)<dot(c,z)-e(2)-eps || dot(pos,z)>dot(c,z)+e(2)+eps ) continue;

				Vector3 p_out;
				vector<vector<Vector3> > boundaryPointList;
				for(size_t j=0; j<c1.boundaryList.size(); j++){
						vector<Vector3> boundary;
						for(size_t k=0; k<c1.boundaryList[j].size(); k++)
								boundary.push_back(c1.boundaryList[j][k]->pos);
						boundaryPointList.push_back(boundary);
				}

				int o1 = calcOuterBoundary(boundaryPointList);

				if(!lineIntersectToCluster(pos, z, p_out, boundaryPointList[o1], true))
						continue;

				return object.parentList[i];
		}

		vector<double> dist;
		for(unsigned int i=0; i<object.parentList.size(); i++){

				ClusterImpl c = object.clusters[object.parentList[i]];

				double f = 2.0*norm2(pos-c.bbcenter) + norm2(nor - c.normal);

				dist.push_back(f);
		}

		int i = argmin(dist);

		return object.parentList[i];
}

void ObjEnvContact::calcTableLegCluster(ObjectShape& obj)
{
		int i=0, j=1;

		while(i<obj.parentList.size()&& j<obj.parentList.size()){
				int ii = obj.parentList[i];
				int jj = obj.parentList[j];

				bool merge = true;
				if(merge && fabs(dot(obj.clusters[ii].bbcenter-obj.clusters[jj].bbcenter, obj.clusters[ii].normal)) > 0.5*obj.thickness) merge=false;
				if(merge && dot(obj.clusters[ii].normal, obj.clusters[jj].normal)<0.95) merge=false;
				if(merge && (obj.clusters[ii].Convexity != ClusterData::CONVEX && obj.clusters[ii].Convexity != ClusterData::TABLELEG )) merge = false;
				if(merge && (obj.clusters[jj].Convexity != ClusterData::CONVEX && obj.clusters[jj].Convexity != ClusterData::TABLELEG )) merge = false;
				if(merge){
						bool neighbor = false;
						for(size_t k=0; k<obj.clusters[ii].neighborList.size(); k++)
								if(obj.clusters[ii].neighborList[k]->idCluster == obj.clusters[jj].id){
										neighbor = true;
										break;
								}
						if(neighbor) merge = false;
				}

				if(merge){
#ifdef DEBUG
						cout << "merging" << obj.parentList[i] << " " << obj.parentList[j] << " " << obj.parentList.size() << endl;
						cout << "ca " << obj.clusters[ii].bbcenter.transpose() << endl;
						cout << "na " << obj.clusters[ii].normal.transpose() << endl;
						cout << "cb " << obj.clusters[jj].bbcenter.transpose() << endl;
						cout << "nb " << obj.clusters[jj].normal.transpose() << endl;
						cout << obj.clusters[ii].Convexity << " " << obj.clusters[jj].Convexity << endl;
#endif
						ClusterImpl cluster(ii);

						cluster.calcTriangleList(obj.clusters[ii], obj.clusters[jj]);
						cluster.calcNeighborList(obj.clusters[ii], obj.clusters[jj]);
						cluster.calcClusterNormal();
						cluster.calcClusterBoundary();
						cluster.calcClusterBoundingBox();
						cluster.Convexity = ClusterData::TABLELEG;
						cluster.closed = true;

						obj.clusters[ii] = cluster;

						for(vector<int>::iterator it = obj.parentList.begin(); it!=obj.parentList.end(); ++it){
								if((*it)==jj){
										it = obj.parentList.erase(it);
										break;
								}
						}

				}

				if(!merge)	j++;

				if(j==obj.parentList.size()){
						i++;
						j=i+1;								;
				}
		}
}

void ObjEnvContact::calcParentList(ObjectShape& object, vector<int>& o)
{
		vector<double> area;
		for(unsigned int i=0;i<object.parentList.size();i++){
				int j=object.parentList[i];

				area.push_back(1.0/object.clusters[j].area);
				o.push_back(j);
		}
		sort_by(o, area);

		vector<int>::iterator it = o.begin();
		while(it != o.end()) {

				if(object.clusters[*it].area < object.clusters[o[0]].area/1000.0)
						o.erase(it);
				else
						++it;
		}

		return;
}

void ObjEnvContact::calcClusterConvexity(ObjectShape& object)
{

		for(vector<int>::iterator it = object.parentList.begin(); it != object.parentList.end(); ++it){

				ClusterImpl cluster = object.clusters[*it];

				vector<vector<Vector3> > boundaryPointList;
				for(size_t j=0; j<cluster.boundaryList.size(); j++){
						vector<Vector3> boundary;
						for(size_t k=0; k<cluster.boundaryList[j].size(); k++)
								boundary.push_back(cluster.boundaryList[j][k]->pos);
						boundaryPointList.push_back(boundary);
				}
				int o1 = calcOuterBoundary(boundaryPointList);

				vector<int> idList;
				int c  = ClusterData::SADDLE;
				int id = cluster.id;

#ifndef HEURISTICS_FOR_PCL

				for(size_t i=0; i<cluster.boundaryList[o1].size(); i++){

						int nbr=0;
						for(size_t l=0; l<cluster.neighborList.size(); l++){
								//cout << "NB" << cluster.neighborList[l]->idCluster;
								for(int n=0; n<3; n++){
								if(cluster.neighborList[l]->ver[n]->id == cluster.boundaryList[o1][i]->id && !included(cluster.neighborList[l]->idCluster, idList))
										nbr = l;
								}
						}
						//cout << endl;

						Triangle *tn = cluster.neighborList[nbr];
#ifdef DEBUG
						cout << "ID " << cluster.id << endl;
						cout << "boundary " << o1 << "/" << boundaryPointList.size() << endl;
						cout << "Neighbor cluster " << tn->idCluster << endl;
#endif

						if(tn->idCluster == id) continue;
						if( included(tn->idCluster, idList)) continue;

#else
				for(size_t nbr=0; nbr<cluster.neighborList.size(); nbr++){

						Triangle *tn = cluster.neighborList[nbr];

						if( included(tn->idCluster, idList)) continue;
#endif

						Triangle *t;
						for(int j=0; j<3; j++){
								t = cluster.neighborList[nbr]->nbr[j];
								//cout << "#" << t->idCluster << "/" << id << "/";
								if(t != NULL && t->idCluster == id)
										break;
						}
						int id1, id2;
						for(int j=0; j<3; j++){
								for(int k=0; k<3; k++){
										if(t->ver[j]==tn->ver[k]) continue;
										id2 = k;
								}
						}

						Vector3 normal1 = cluster.normal;
						//Vector3 normal1 = t->normal;
						//Vector3 center1 = object.clusters[*it].bbcenter;
						Vector3 center1 = (t->ver[0]->pos + t->ver[1]->pos + t->ver[2]->pos)/3.0;

						Vector3 normal2 = object.clusters[tn->idCluster].normal;
						//Vector3 normal2 = tn->normal;
						//Vector3 center2 = object.clusters[tn->idCluster].bbcenter;
						Vector3 center2 = (tn->ver[0]->pos + tn->ver[1]->pos + tn->ver[2]->pos)/3.0;

						Vector3 x = normal1;
						Vector3 y = cross(cross(x, normal2),x);

						MatrixXd M(2,3);
						M(0,0) = x(0); M(0,1) = x(1); M(0,2) = x(2);
						M(1,0) = y(0); M(1,1) = y(1); M(1,2) = y(2);

						MatrixXd L(2,2);
						L(0,0) = (M*normal1)(0); L(0,1) = -(M*normal2)(0);
						L(1,0) = (M*normal1)(1); L(1,1) = -(M*normal2)(1);

						VectorXd u = inverse(L)*M*(center2-center1);

						if( ( (u(0)<=0 && u(1)<=0) && c==ClusterData::CONCAVE) || ((u(0)>0 && u(1)>0) && c==ClusterData::CONVEX)){
								c = ClusterData::SADDLE;
								break;
						}
						else if(u(0)<=0 && u(1)<=0)
								c = ClusterData::CONVEX;
						else if(u(0)>0 && u(1)>0)
								c = ClusterData::CONCAVE;

						idList.push_back(tn->idCluster);
#ifdef DEBUG
						cout << "Cluster id " << t->idCluster << "/" << id << "/" << cluster.id << endl;
						cout << "Convexity " << c << endl;
#endif
				}
				object.clusters[*it].Convexity = c;
		}

		return;
}

void ObjEnvContact::calcClusterConvexHull(ObjectShape& object)
{

		vector<int> o;
		calcParentList(object, o);

#ifdef DEBUG
		for(unsigned int k=0; k<object.clusters[o[0]].boundaryList.size(); k++){
				cout << "X=[ "; for(unsigned int l=0; l<object.clusters[o[0]].boundaryList[k].size(); l++)
						cout << object.clusters[o[0]].boundaryList[k][l]->pos(0) << ", "; cout << "]" << endl;
				cout << "Y=[ "; for(unsigned int l=0; l<object.clusters[o[0]].boundaryList[k].size(); l++)
						cout << object.clusters[o[0]].boundaryList[k][l]->pos(1) << ", "; cout << "]" << endl;
				cout << "Z=[ "; for(unsigned int l=0; l<object.clusters[o[0]].boundaryList[k].size(); l++)
						cout << object.clusters[o[0]].boundaryList[k][l]->pos(2) << ", "; cout << "]" << endl;
		}
#endif

		for(vector<int>::iterator it = o.begin(); it != o.end(); ++it){

				for(size_t k=0; k<object.clusters[*it].boundaryList.size(); k++)
						for(size_t l=0; l<object.clusters[*it].boundaryList[k].size(); l++)
								object.clusters[*it].convexHull.push_back(object.clusters[*it].boundaryList[k][l]->pos);


				if(object.clusters[*it].convexHull.size()<4) continue;

				vector<double> pt, pt_out;
				for(vector<Vector3>::iterator jt=object.clusters[*it].convexHull.begin(); jt != object.clusters[*it].convexHull.end(); ++jt){
						pt.push_back(dot(*jt, object.clusters[*it].eigenVector[0] ) );
						pt.push_back(dot(*jt, object.clusters[*it].eigenVector[1] ) );
				}

				ConvexAnalysis ca;
				ca.calcConvexHull(2, pt, pt_out, false);

				vector<Vector3> pSet;
				for(vector<Vector3>::iterator jt=object.clusters[*it].convexHull.begin(); jt != object.clusters[*it].convexHull.end(); ++jt){

						double p[2];
						vector<double> Pmin;
						for(unsigned int k=0; k<pt_out.size(); k++){
								p[k%2] = pt_out[k];

								if(k%2 == 0) continue;

								Pmin.push_back(fabs(dot(*jt, object.clusters[*it].eigenVector[0] )-p[0]) + fabs(dot(*jt, object.clusters[*it].eigenVector[1] )-p[1]));
						}

						if(vmin(Pmin)<0.0001)
								pSet.push_back(*jt);
				}

				object.clusters[*it].convexHull.clear();

				for(vector<Vector3>::iterator jt=pSet.begin(); jt != pSet.end(); ++jt)
						object.clusters[*it].convexHull.push_back(*jt);
		}
#ifdef DEBUG
		cout << "X=[ "; for(unsigned int k=0; k<object.clusters[o[0]].convexHull.size(); k++)
				cout << object.clusters[o[0]].convexHull[k](0) << ", "; cout << "]" << endl;
		cout << "Y=[ "; for(unsigned int k=0; k<object.clusters[o[0]].convexHull.size(); k++)
				cout << object.clusters[o[0]].convexHull[k](1) << ", "; cout << "]" << endl;
		cout << "Z=[ "; for(unsigned int k=0; k<object.clusters[o[0]].convexHull.size(); k++)
				cout << object.clusters[o[0]].convexHull[k](2) << ", "; cout << "]" << endl;
#endif

		for(vector<int>::iterator it = o.begin(); it != o.end(); ++it){

				Vector3 cog = average(object.clusters[*it].convexHull);

				vector<double> angle;
				for(unsigned int k=0; k<object.clusters[*it].convexHull.size(); k++){

						Vector3 p1 = object.clusters[*it].convexHull[k];
						p1 = p1 - dot(object.clusters[*it].normal, (p1 - cog))*object.clusters[*it].normal;

						Vector3 ex, ey;
						if(dot(object.clusters[*it].eigenVector[2], object.clusters[*it].normal)>0){
								ex =  object.clusters[*it].eigenVector[0];
								ey =  object.clusters[*it].eigenVector[1];
						}
						else{
								ex = -object.clusters[*it].eigenVector[0];
								ey =  object.clusters[*it].eigenVector[1];
						}

						double cosq = dot(ex, p1-cog)/norm2(p1-cog);
						double sinq = dot(ey, p1-cog)/norm2(p1-cog);
						angle.push_back(-atan2(sinq, cosq) );
				}
				sort_by(object.clusters[*it].convexHull, angle);
		}
#ifdef DEBUG
		cout << "X=[ "; for(unsigned int k=0; k<object.clusters[o[0]].convexHull.size(); k++)
				cout << object.clusters[o[0]].convexHull[k](0) << ", "; cout << "]" << endl;
		cout << "Y=[ "; for(unsigned int k=0; k<object.clusters[o[0]].convexHull.size(); k++)
				cout << object.clusters[o[0]].convexHull[k](1) << ", "; cout << "]" << endl;
		cout << "Z=[ "; for(unsigned int k=0; k<object.clusters[o[0]].convexHull.size(); k++)
				cout << object.clusters[o[0]].convexHull[k](2) << ", "; cout << "]" << endl;
#endif
}

void ObjEnvContact::writeResult(ObjectShape& object)
{

		//string filename = "extplugin/graspPlugin/PickAndPlacePlanner/PRM/" + object.name + ".yaml";
		string filename = object.name + "_merge.yaml";
		ofstream fout;
		fout.open(filename.c_str());

		vector<int> o;
		calcParentList(object, o);

		Vector3 a;
		int count=0;
		int sample=1;

		fout << "modelFile: ../../Samples/Object/" + object.name + "hrp.wrl" << endl;
		fout << endl;

		fout << "EnvCluster:" << endl;
		for(unsigned int j=0;j<o.size();j++){

				if(object.clusters[o[j]].Put==0) continue;

				fout << "  -" << endl;
				fout << "    id: [ " << o[j] << " ]" << endl;
				fout << "    area: [ " << object.clusters[o[j]].area << " ]" << endl;
				a = object.clusters[o[j]].normal;
				fout << "    outer_normal: [ " << a(0) << ", " << a(1) << ", " << a(2) << " ]" << endl;
				a = object.clusters[o[j]].eigenVector[0];
				fout << "    tangent_vector: [ " << a(0) << ", " << a(1) << ", " << a(2) << " ]" << endl;
				fout << "    convexhull: [ ";
				for(vector<Vector3>::iterator jt=object.clusters[o[j]].convexHull.begin(); jt!= object.clusters[o[j]].convexHull.end(); ++jt){

						if(count%sample !=0 && jt != object.clusters[o[j]].convexHull.end()-1) continue;

						fout << (*jt)(0) << ", " << (*jt)(1) << ", " << (*jt)(2);
						if(jt != object.clusters[o[j]].convexHull.end()-1)
								fout << ", ";

						count++;
				}
				fout << " ]" << endl;
#ifndef HEURISTICS_FOR_PCL
				if(object.clusters[o[j]].Convexity != ClusterData::TABLELEG){
						if(object.clusters[o[j]].boundaryList.size()>1 ){
								fout << "    boundaryList: [ ";
								for(size_t k=0; k<object.clusters[o[j]].boundaryList.size(); k++){
										fout << object.clusters[o[j]].boundaryList[k].size();
										if(k!=object.clusters[o[j]].boundaryList.size()-1)
											fout << ", ";
								}
								fout << " ]" << endl;
						}

						fout << "    boundary: [ ";
						for(size_t k=0; k<object.clusters[o[j]].boundaryList.size(); k++){
								for(size_t l=0; l<object.clusters[o[j]].boundaryList[k].size(); l++){

										if(count%sample!=0 && !(k==object.clusters[o[j]].boundaryList.size()-1 && l==object.clusters[o[j]].boundaryList[k].size()-1)) continue;

										a = object.clusters[o[j]].boundaryList[k][l]->pos;
										fout << a(0) << ", " << a(1) << ", " << a(2);
										if(k!=object.clusters[o[j]].boundaryList.size()-1 || l!=object.clusters[o[j]].boundaryList[k].size()-1)
												fout << ", ";

										count++;
								}
						}
						fout << " ]" << endl;
				}
#endif
				fout << "    convexity: [ "   << object.clusters[o[j]].Convexity   << " ]" << endl;
				fout << "    bounding_box_edge: [ "   << object.clusters[o[j]].bbedge(0)   << ", " << object.clusters[o[j]].bbedge(1)   << ", " << object.clusters[o[j]].bbedge(2)   << " ]" << endl;
				fout << "    bounding_box_center: [ " << object.clusters[o[j]].bbcenter(0) << ", " << object.clusters[o[j]].bbcenter(1) << ", " << object.clusters[o[j]].bbcenter(2) << " ]" << endl;
				fout << "    is_putting_cluster: [ " << object.clusters[o[j]].Put << " ]" << endl;
		}
}


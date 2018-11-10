// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 8; -*-

#include "FindParallelPlane.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <cnoid/MessageView>
//#include <cnoid/SceneBody>
//#include <cnoid/SceneView>

#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
#ifdef CNOID_ENABLE_OSG
#include <cnoid/OSGSceneBody>
#include <cnoid/OSGSceneView>
#endif
#else
#include <cnoid/SceneView>
#include <cnoid/EditableSceneBody>
#include <cnoid/SceneShape>
#include "../Grasp/ColdetConverter.h"
#endif

#ifdef WIN32
#include <windows.h>
void usleep(int t){
	Sleep(t/1000);
}
#endif

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

void FindParallelPlane::calcClusters(ObjectShape& object){
		int cnt=0;
		double thickness = 0.005;

		for(int j=0;j<object.nTriangles;j++){
				if(! (object.triangles[j].idCluster==-1) ) continue;
				ClusterImpl cluster(cnt++);
				cluster.clusterGrowingFromSeed( object.triangles[j], thickness);
				object.clusters.push_back(cluster);
		}
		for(unsigned int j=0;j<object.clusters.size();j++){
				object.parentList.push_back(j);
				object.clusters[j].calcClusterBoundary();
				object.clusters[j].calcClusterNormal();
				object.clusters[j].calcControlPoints(PlaneParam);
				if(object.clusters[j].controlPoints.size()==0)
						object.clusters[j].intention = 99;
		}

}

void FindParallelPlane::calcClusters2(ObjectShape& object){

		object.generateClusterSeeds();
		object.mergeClustersFromSeed();

		for(unsigned int j=0;j<object.parentList.size();j++){
				int idc = object.parentList[j];
				object.clusters[idc].calcClusterBoundary();
				object.clusters[idc].calcControlPoints(PlaneParam);
				if(object.clusters[idc].controlPoints.size()==0)
						object.clusters[idc].intention = 99;
		}

}

void FindParallelPlane::precalcParallelPlane(ObjectShape& object){

		object.generateClusterSeeds(1);
		object.mergeClustersFromSeed();

		double ratio = 30.0;
		vector<double> areaList;
		for(unsigned int i=0; i<object.parentList.size(); i++)
				areaList.push_back(object.clusters[object.parentList[i]].area);

		int nmax = argmax(areaList);

		for(unsigned int i=0; i<object.parentList.size(); i++)
				if(object.clusters[object.parentList[i]].area > object.clusters[object.parentList[nmax]].area/ratio)
						parentList2.push_back(object.parentList[i]);

		Vector3 ref(1.0/sqrt(3.0), 1.0/sqrt(3.0), 1.0/sqrt(3.0) );
		vector<Vector3> clusterNormal0, clusterNormal1;
		for(unsigned int i=0; i<parentList2.size(); i++)
				if(dot(object.clusters[parentList2[i]].normal, ref)>0)
						clusterNormal0.push_back(object.clusters[parentList2[i]].normal);
				else
						clusterNormal1.push_back(object.clusters[parentList2[i]].normal);

		if(clusterNormal0.size() >= clusterNormal1.size())
				for(unsigned int i=0; i<clusterNormal0.size(); i++)
						clusterNormal.push_back(clusterNormal0[i]);
		else
				for(unsigned int i=0; i<clusterNormal1.size(); i++)
						clusterNormal.push_back(clusterNormal1[i]);

}

bool FindParallelPlane::calcParallelPlane(ObjectShape& object, int i){

		idCluster.clear();

		double x_g=0.02, y_g=0.05, z_g=0.01;//gripper size
		double m_g=0.005;//Approach margin;
		const double fmax = 9.0;

		if(i>=clusterNormal.size())
				return false;

		//Clustering
		object.generateClusterSeeds(clusterSeedSize);
		object.mergeClustersFromSeed(Vector3(-clusterNormal[i]), objPressPos);
		cout << "clustering" << endl;

		//Control Points / Force Closure
		ForceClosureTest *fc;
		dvector wg=VectorXd::Zero(6);

		PlanBase* gc = PlanBase::instance();

		if ( !gc->initial() || !gc->targetObject || !gc->targetArmFinger) {
				gc->os << "Please select Grasped Object and Grasping Robot" << endl;
				return true;
		}

		ColdetLinkPairPtr *cPair = new ColdetLinkPairPtr[1];

#ifdef CNOID_10_11_12_13
		cPair[0] = new ColdetLinkPair(gc->bodyItemRobot()->body()->link(0), gc->object());
#else
		cPair[0] = boost::make_shared<ColdetLinkPair>(gc->bodyItemRobot()->body(), gc->bodyItemRobot()->body()->link(0), gc->targetObject->bodyItemObject->body(), gc->targetObject->bodyItemObject->body()->link(0));
#endif

		//create ObjectShape of finger
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		ColdetModelPtr coldet = gc->bodyItemRobot()->body()->link(0)->coldetModel();
#else
		SgNode* node = gc->bodyItemRobot()->body()->link(0)->collisionShape();
		SgMeshPtr mesh = ColdetConverter::ExtractMesh(node);
		SgVertexArrayPtr vertices = mesh->vertices();
#endif
		vector<double> vertex;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		float tx, ty, tz;
		for(int k=0;k<coldet->getNumVertices();k++){
			coldet->getVertex(k, tx, ty, tz);
			vertex.push_back( tx );
			vertex.push_back( ty );
			vertex.push_back( tz );
		}
		vector<int> crd;
		int t1, t2, t3;
		for(int k=0;k<coldet->getNumTriangles();k++){
			coldet->getTriangle(k, t1, t2, t3);
			crd.push_back( t1 );
			crd.push_back( t2 );
			crd.push_back( t3 );
			crd.push_back( -1 );
		}
#else
		for(int k = 0; k < vertices->size(); k++){
			cnoid::Vector3f vec = vertices->at(k);
			vertex.push_back(vec[0]);
			vertex.push_back(vec[1]);
			vertex.push_back(vec[2]);
		}
		vector<int> crd;
        cnoid::SgIndexArray indices = mesh->triangleVertices();
		for(int k = 0; k < mesh->numTriangles(); k++){
			crd.push_back( indices[k*3] );
			crd.push_back( indices[k*3+1] );
			crd.push_back( indices[k*3+2] );
			crd.push_back( -1 );
		}
#endif
		ObjectShape* fing_obj = new ObjectShape(vertex,crd);
		
		// obtain boundary of finger object
		Vector3 Pf = Vector3(0, 0, 0);
		Vector3 nf = Vector3(0, 0, -1.0);
		int fTid;
		std::vector<vector<Vector3> > boundaries = fing_obj->getContactRegionBoundaryPoints(Pf, nf, fTid);

		EnCalculator crc;
		crc.setObject(&object);
		crc.setHmax(z_g);
		
		vector<Vector3> d(8);
		map<int, vector<bool> > app;
		vector<int> idSet;

		for(unsigned int j=0;j<object.parentList.size();j++){
				int idc = object.parentList[j];

				if(app.count(idc) == 0){
					for(int l=0; l<8; l++){
						app[idc].push_back(true);
					}
				}

				if(object.clusters[idc].closed && dot(object.clusters[idc].normal,Vector3(-clusterNormal[i]))>0 ){

						for(unsigned int k=0; k<object.clusters[idc].idPair.size();k++){

								int idc2 = object.clusters[idc].idPair[k];
								object.clusters[idc2].controlPoints.clear();

								if(app.count(idc2) == 0){
									for(int l=0; l<8; l++){
										app[idc2].push_back(true);
									}
								}
								
								unsigned int control_point_size = object.clusters[idc].controlPoints.size();
								Vector3** p = new Vector3*[control_point_size];
								Vector3** n = new Vector3*[control_point_size];
								double*** en = new double**[control_point_size];
								for(unsigned int l=0; l<control_point_size; l++){
									p[l] = new Vector3[2];
									n[l] = new Vector3[2];
									en[l] = new double*[8];
									for(unsigned int m=0; m<8; m++){
										en[l][m] = new double[2];
									}
								}

								// approach vector
								for(unsigned int l=0; l<object.clusters[idc].controlPoints.size(); l++){

										Vector3 pt;

										p[l][0]=(object.clusters[idc].controlPoints[l]);
										n[l][0]=(object.clusters[idc].normal);

										if( object.clusters[idc2].lineIntersectToCluster(p[l][0], n[l][0], pt) ){

												p[l][1]=( intersectionPoint(p[l][0], n[l][0], object.clusters[idc2].center, object.clusters[idc2].normal) );//should be bottom
												n[l][1]=(object.clusters[idc2].normal);

												for(int m=0; m<8; m++){
													if(!app[idc][m] || !app[idc2][m] ) continue;
													double m_pi = 3.14159;
													d[m] = (cos(m_pi*m/4.0)*object.clusters[idc].eigenVector[0] + sin(m_pi*m/4.0)*object.clusters[idc].eigenVector[1]);
													
                                                    gc->bodyItemRobot()->body()->link(0)->p() = gc->object()->p() + gc->object()->attitude()*(p[l][0] + m_g*n[l][0]);
                                                    gc->bodyItemRobot()->body()->link(0)->R() = gc->bodyItemRobot()->body()->link(0)->calcRfromAttitude(Matrix33(gc->object()->attitude()*v3(cross(d[m], n[l][0]), d[m], n[l][0])));
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
                                                    cPair[0]->model(0)->setPosition(gc->bodyItemRobot()->body()->link(0)->R(), gc->bodyItemRobot()->body()->link(0)->p());
                                                    cPair[0]->model(1)->setPosition(gc->object()->R(), gc->object()->p());
#else
													cPair[0]->model(0)->setPosition(gc->bodyItemRobot()->body()->link(0)->T());
                                                    cPair[0]->model(1)->setPosition(gc->object()->T());
#endif

													double p1[3] = {0}, p2[3] = {0};
													int tid1, tid2;

													if(cPair[0]->computeDistance(tid1, &p1[0], tid2, &p2[0]) <= 0.0001){
															app[idc][m] = false;
															continue;
													}

													vector<vector<Vector3> > fing_boundary1;
													Matrix3 fingR = gc->bodyItemRobot()->body()->link(0)->R();
													Vector3 fingP = gc->bodyItemRobot()->body()->link(0)->p();
													for(int bn=0;bn<boundaries.size();bn++){
														vector<Vector3> fing_bound(boundaries[bn].size());
														for(int b=0;b<boundaries[bn].size();b++){
																fing_bound[b] = trans(Matrix3(gc->object()->R())) * ((fingR * boundaries[bn][b]) + fingP - gc->object()->p());
														}
														fing_boundary1.push_back(fing_bound);
													}

													Vector3 contact_point(p2[0], p2[1], p2[2]);
													crc.calc(p[l][0], trans(Matrix3(gc->object()->R())) * contact_point, trans(Matrix3(gc->object()->R())) * fingP, trans(Matrix3(gc->object()->R())) * fingR * nf, fing_boundary1, fmax);
													en[l][m][0] = crc.getEn();
													//en[l][m][0] = EnCalculator().calc(object, p[l][0], n[l][0], fing_boundary1, idc);

													gc->flush();

													usleep(1000);

                                                    gc->bodyItemRobot()->body()->link(0)->p() = gc->object()->p() + gc->object()->attitude()*(p[l][1] + m_g*n[l][1]);
                                                    gc->bodyItemRobot()->body()->link(0)->R() = gc->bodyItemRobot()->body()->link(0)->calcRfromAttitude(Matrix33(gc->object()->attitude()*v3(cross(d[m], n[l][1]), d[m], n[l][1])));
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
                                                    cPair[0]->model(0)->setPosition(gc->bodyItemRobot()->body()->link(0)->R(), gc->bodyItemRobot()->body()->link(0)->p());
                                                    cPair[0]->model(1)->setPosition(gc->object()->R(), gc->object()->p());
#else
                                                    cPair[0]->model(0)->setPosition(gc->bodyItemRobot()->body()->link(0)->T());
                                                    cPair[0]->model(1)->setPosition(gc->object()->T());
#endif

													if(cPair[0]->computeDistance(tid1, &p1[0], tid2, &p2[0]) <= 0.0001){
															app[idc2][m] = false;
															continue;
													}

													vector<vector<Vector3> > fing_boundary2;
													fingR = gc->bodyItemRobot()->body()->link(0)->R();
													fingP = gc->bodyItemRobot()->body()->link(0)->p();

													for(int bn=0;bn<boundaries.size();bn++){
														vector<Vector3> fing_bound(boundaries[bn].size());
														for(int b=0;b<boundaries[bn].size();b++){
																fing_bound[b] = trans(Matrix3(gc->object()->R())) * ((fingR * boundaries[bn][b]) + fingP - gc->object()->p());
														}
														fing_boundary2.push_back(fing_bound);
													}

													gc->flush();

													usleep(1000);

													contact_point = Vector3(p2[0], p2[1], p2[2]);
													crc.calc(p[l][1], trans(Matrix3(gc->object()->R())) * contact_point, trans(Matrix3(gc->object()->R())) * fingP, trans(Matrix3(gc->object()->R())) * fingR * nf, fing_boundary2, fmax);
													en[l][m][1] = crc.getEn();
													//en[l][m][1] = EnCalculator().calc(object, p[l][1], n[l][1], fing_boundary2, idc2);
												}
										}
								}

								// contorl points
								double mg = gc->targetObject->objMass * 9.8;
								for(unsigned int l=0; l<object.clusters[idc].controlPoints.size(); l++){
									Vector3 pt;
									if( object.clusters[idc2].lineIntersectToCluster(p[l][0], n[l][0], pt) ){
										for(int m=0; m<8; m++){
											if(!app[idc][m] || !app[idc2][m] ) continue;
												
												vector<double> en_vec;
												en_vec.push_back(en[l][m][0]);
												en_vec.push_back(en[l][m][1]);

												//Check if the force closure condition is satisfied
												//if(fc->forceClosureTestEllipsoidSoftFinger(wg, p[l], n[l], 2, 0.5, 3.0, en_vec)>0.0){
												if(fc->forceClosureTestEllipsoidSoftFingerSubspace(wg, p[l], n[l], 2, 0.5, fmax, en_vec, gc->targetObject->objCoM_)  - mg > 0.0){
													object.clusters[idc2].controlPoints.push_back( p[l][1] );
													break;
												}
										}
									}
								}

								if(object.clusters[idc2].controlPoints.size() > 0 && !included(idc2,idSet)){
									object.clusters[idc2].approach.resize(8);
									for(int l=0;l<8;l++){
										object.clusters[idc2].approach[l] = app[idc2][l] ? Vector3(-d[l]) : Vector3(0, 0, 0);
									}
									clusters_out.push_back( object.clusters[idc2] );
									idSet.push_back(idc2);
								}
								idCluster.push_back(idc);
								idCluster.push_back(idc2);

								for(unsigned int l=0; l<object.clusters[idc].controlPoints.size(); l++){
									for(unsigned int m=0; m<8; m++){
										delete [] en[l][m];
									}
									delete[] p[l];
									delete[] n[l];
									delete[] en[l];
								}
								delete [] p;
								delete [] n;
								delete [] en;
						}

						if(!included(idc,idSet)){
							object.clusters[idc].approach.resize(8);
							for(int l=0;l<8;l++){
								object.clusters[idc].approach[l] = app[idc][l] ? Vector3(-d[l]) : Vector3(0, 0, 0);
							}
							clusters_out.push_back( object.clusters[idc] );
							idSet.push_back(idc);
						}
				}
		}
		delete [] cPair;
		delete fing_obj;

		return true;
}

void FindParallelPlane::writeResults(ObjectShape& object){

		string filename = object.name + ".yaml";
		ofstream fout;
		fout.open(filename.c_str());

		vector<double> area;
		vector<int> o;
		for(unsigned int i=0;i<object.parentList.size();i++){
				int j=object.parentList[i];

				area.push_back(1.0/object.clusters[j].area);
				o.push_back(j);
		}
		sort_by(o, area);

		fout << "modelFile: ../../Samples/Object/" + object.name + "hrp.wrl" << endl;
		fout << endl;

		fout << "Cluster: " << endl;
		for(unsigned int j=0;j<object.parentList.size();j++){

				fout << "-" << endl;
				fout << "    id: ["               << o[j] << "]" << endl;
				fout << "    area: ["             << object.clusters[o[j]].area      << "]" << endl;
				fout << "    intention: [ "       << object.clusters[o[j]].intention << " ]" << endl;
				fout << "    outer_normal: [ "    << object.clusters[o[j]].normal(0)         << ", " << object.clusters[o[j]].normal(1)         << ". " << object.clusters[o[j]].normal(2)         << " ]" << endl;
				fout << "    tangent_vector: [ "  << object.clusters[o[j]].eigenVector[0](0) << ", " << object.clusters[o[j]].eigenVector[0](1) << ", " << object.clusters[o[j]].eigenVector[0](2) << " ]" << endl;
				fout << "    approach_vector: [ " << object.clusters[o[j]].approach[0](0)    << ", " << object.clusters[o[j]].approach[0](1)    << ", " << object.clusters[o[j]].approach[0](2)    << " ]" << endl;

				vector<double> distance;
				for(vector<Vector3>::const_iterator I=object.clusters[o[j]].controlPoints.begin(); I!=object.clusters[o[j]].controlPoints.end(); I++)
						distance.push_back(norm2(*I - object.clusters[o[j]].center));
				sort_by(object.clusters[o[j]].controlPoints, distance);

				fout << "    control_points [ ";
				for(vector<Vector3>::const_iterator I=object.clusters[o[j]].controlPoints.begin(); I!=object.clusters[o[j]].controlPoints.end(); I++){
						fout << (*I)(0) << ", " << (*I)(1) << ", " << (*I)(2) << ", ";
						if(I==object.clusters[o[j]].controlPoints.end()-1)
							fout << " ]" << endl;
						else
							fout << ", ";
				}
		}
}

void FindParallelPlane::writeResults2(vector<ClusterImpl>& clusters){


		string filename = PlanBase::instance()->targetObject->bodyItemObject->body()->name() + ".yaml";
		ofstream fout;
		fout.open(filename.c_str());

		vector<double> area;
		vector<int> o;
		for(unsigned int i=0;i<clusters.size();i++){

				area.push_back(1.0/clusters[i].area);
				o.push_back(i);
		}
		sort_by(o, area);

		fout << "modelFile: ../../Samples/Object/" + PlanBase::instance()->targetObject->bodyItemObject->body()->name() + "hrp.wrl" << endl;
		fout << endl;

		fout << "Cluster:" << endl;
		for(unsigned int j=0;j<clusters.size();j++){

				fout << "-" << endl;
				fout << "    id: [ " << clusters[o[j]].id << " ]" << endl;
				fout << "    pair: [ ";
				for(unsigned int k=0; k<clusters[o[j]].idPair.size(); k++){
						fout << clusters[o[j]].idPair[k];
						if(k==clusters[o[j]].idPair.size()-1)
							fout  << " ]" << endl;
						else
							fout << ", ";
				}
				fout << "    area: [ "           << clusters[o[j]].area << " ]" << endl;
				fout << "    intention: [ "      << clusters[o[j]].intention << " ]" << endl;
				fout << "    outer_normal: [ "   << clusters[o[j]].normal(0)         << ", " << clusters[o[j]].normal(1)         << ", " << clusters[o[j]].normal(2)         << " ]" << endl;
				fout << "    tangent_vector: [ " << clusters[o[j]].eigenVector[0](0) << ", " << clusters[o[j]].eigenVector[0](1) << ", " << clusters[o[j]].eigenVector[0](2) << " ]" << endl;
				fout << "    approach_vector: [ ";
				for(unsigned int i=0; i<clusters[o[j]].approach.size(); i++){
						fout << clusters[o[j]].approach[i](0) << ", " << clusters[o[j]].approach[i](1) << ", " << clusters[o[j]].approach[i](2);
						if(i==clusters[o[j]].approach.size()-1)
							fout << " ]" << endl;
						else
							fout << ", ";
				}

				vector<double> distance;
				for(vector<Vector3>::const_iterator I=clusters[o[j]].controlPoints.begin(); I!=clusters[o[j]].controlPoints.end(); I++)
						distance.push_back(norm2(*I - clusters[o[j]].center));
				sort_by(clusters[o[j]].controlPoints, distance);

				fout << "    control_points: [ ";
				for(vector<Vector3>::const_iterator I=clusters[o[j]].controlPoints.begin(); I!=clusters[o[j]].controlPoints.end(); I++){
						fout << (*I)(0) << ", " << (*I)(1) << ", " << (*I)(2);
						if(I==clusters[o[j]].controlPoints.end()-1)
							fout << " ]" << endl;
						else
							fout << ", ";
				}
				fout << "    bounding_box_edge: [ "   << clusters[o[j]].bbedge(0)   << ", " << clusters[o[j]].bbedge(1)   << ", " << clusters[o[j]].bbedge(2)   << " ]" << endl;
				fout << "    bounding_box_center: [ " << clusters[o[j]].bbcenter(0) << ", " << clusters[o[j]].bbcenter(1) << ", " << clusters[o[j]].bbcenter(2) << " ]" << endl;
		}

}

void generateRotMatFromVec( double vx, double vy, double vz, Matrix3 &rotMat )
{
    // Get our direction vector (the Z vector component of the matrix)
    // and make sure it's normalized into a unit vector
    double vnorm = sqrt( vx*vx+vy*vy+vz*vz );
    double zvx, zvy, zvz;
    zvx = vx/vnorm;
    zvy = vy/vnorm;
    zvz = vz/vnorm;

    // Build the Y vector of the matrix (handle the degenerate case
    // in the way that 3DS does) -- This is not the TRUE vector, only
    // a reference vector.
    double yvx, yvy, yvz;
    if ( !zvx && !zvz ) {
        yvx = -zvy;
        yvy = 0.;
        yvz = 0.;
    } else {
        yvx = 0.;
        yvy = 1.;
        yvz = 0.;
    }

    // Build the X axis vector based on the two existing vectors
    double xvx, xvy, xvz;
    xvx = zvy*yvz - zvz*yvy;
    xvy = zvz*yvx - zvx*yvz;
    xvz = zvx*yvy - zvy*yvx;

    vnorm = sqrt( xvx*xvx + xvy*xvy + xvz*xvz );
    xvx /= vnorm;
    xvy /= vnorm;
    xvz /= vnorm;

    // Correct the Y reference vector
    yvx = xvy*zvz - xvz*zvy;
    yvy = xvz*zvx - xvx*zvz;
    yvz = xvx*zvy - xvy*zvx;
    vnorm = sqrt( yvx*yvx + yvy*yvy + yvz*yvz );
    yvx = -yvx/vnorm;
    yvy = -yvy/vnorm;
    yvz = -yvz/vnorm;

    // Generate rotation matrix without roll included
    rotMat( 0, 0 ) = xvx;   rotMat( 0, 1 ) = yvx;   rotMat( 0, 2 ) = zvx;
    rotMat( 1, 0 ) = xvy;   rotMat( 1, 1 ) = yvy;   rotMat( 1, 2 ) = zvy;
    rotMat( 2, 0 ) = xvz;   rotMat( 2, 1 ) = yvz;   rotMat( 2, 2 ) = zvz;
}


bool FindParallelPlane::findTargetTriangle( cnoid::Link *targetLink, cnoid::Vector3 &pressPos, cnoid::BodyItemPtr pBody)
{

		const double precision = 0.001;

		if( targetLink == NULL ) return false;

		// compute the pressPos 3D coordinates in the object coordinates system
        objPressPos = targetLink->R().transpose()*(pressPos - targetLink->p());
		PlanBase::instance()->objPressPos = objPressPos;
		double x = objPressPos(0);
		double y = objPressPos(1);
		double z = objPressPos(2);

//		cout << " Press Pos " << objPressPos.transpose() << endl;
		cout << " Press Pos " << pressPos.transpose() << endl;

		objPressName = pBody->body()->name();
		PlanBase::instance()->objPressName = objPressName;

		vector<int> triangleList;
		vector<double> distanceList;
		double nnx, nny, nnz, mindist=0.;
		Vector3 nnp;
		int mindisti=-1;
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
		for( int i=0; i<targetLink->coldetModel()->getNumTriangles(); ++i ) {
				int t1, t2, t3;
				float tx, ty, tz;
				targetLink->coldetModel()->getTriangle( i, t1, t2, t3 );
				targetLink->coldetModel()->getVertex( t1, tx, ty, tz );
				Vector3 p1(tx,ty,tz);
				targetLink->coldetModel()->getVertex( t2, tx, ty, tz );
				Vector3 p2(tx,ty,tz);
				targetLink->coldetModel()->getVertex( t3, tx, ty, tz );
				Vector3 p3(tx,ty,tz);
#else
		SgMeshPtr mesh = ColdetConverter::ExtractMesh(targetLink->collisionShape());
		SgVertexArrayPtr vertices = mesh->vertices();
        SgIndexArray indices = mesh->triangleVertices();
		for( int i = 0; i < mesh->numTriangles(); ++i ) {
				Vector3f vec1 = vertices->at(indices[i*3]);
				Vector3f vec2 = vertices->at(indices[i*3+1]);
				Vector3f vec3 = vertices->at(indices[i*3+2]);
				Vector3 p1(vec1[0],vec1[1],vec1[2]);
				Vector3 p2(vec2[0],vec2[1],vec2[2]);
				Vector3 p3(vec3[0],vec3[1],vec3[2]);
#endif
				// check the bounding box
				double minx=p1(0), maxx=p1(0),  miny=p1(1), maxy=p1(1),  minz=p1(2), maxz=p1(2);
				if( p2(0)<minx ) minx = p2(0);
				if( p3(0)<minx ) minx = p3(0);
				if( p2(1)<miny ) miny = p2(1);
				if( p3(1)<miny ) miny = p3(1);
				if( p2(2)<minz ) minz = p2(2);
				if( p3(2)<minz ) minz = p3(2);
				if( p2(0)>maxx ) maxx = p2(0);
				if( p3(0)>maxx ) maxx = p3(0);
				if( p2(1)>maxy ) maxy = p2(1);
				if( p3(1)>maxy ) maxy = p3(1);
				if( p2(2)>maxz ) maxz = p2(2);
				if( p3(2)>maxz ) maxz = p3(2);

				if( x < minx-precision ) continue;
				if( x > maxx+precision ) continue;
				if( y < miny-precision ) continue;
				if( y > maxy+precision ) continue;
				if( z < minz-precision ) continue;
				if( z > maxz+precision ) continue;

				// triangle normal vector
				Vector3 v1 = p2 - p1;
				Vector3 v2 = p3 - p1;
				Vector3 np = unit(cross(v1, v2));

				// check the distance between the target point and the plane
				double planeD = -dot(np, p1);
				double ppdist = fabs( dot(np, objPressPos) + planeD); //nx*x + ny*y + nz*z + planeD );

				// the point should be on the surface of the object -> the distance should be close to 0
				// we do not test if the point is inside the triangle !! Thus we may get a list of triangles!
				if( ppdist < precision ) {
						distanceList.push_back( ppdist );
						triangleList.push_back( i );

						if( ppdist<mindist || mindisti<0 ) {
								mindist = ppdist;
								mindisti = i;

								// must rotate the normal vector to match the world coordinates!
                                nnp = targetLink->R()*np;
								objPressNormal = np;
						}
				}
		}

		// construct and display the target location as an arrow
		static bool firstRun = true;
		static cnoid::BodyItem *localArrowBI = new cnoid::BodyItem;
		if( firstRun ) {
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
  #ifdef CNOID_10_11
				boost::filesystem::path robotfullpath( pBody->modelFilePath() );
  #else
				boost::filesystem::path robotfullpath( pBody->lastAccessedFilePath() );
  #endif
#else
				boost::filesystem::path robotfullpath( pBody->filePath() );
#endif
				std::string bodyItemRobotPath = boost::filesystem::path( robotfullpath.branch_path() ).string();
				localArrowBI->loadModelFile( bodyItemRobotPath + "/../../GeometryHandler/project/arrowhrp.wrl" );
		}

		if( localArrowBI!=NULL ) {
                localArrowBI->body()->rootLink()->p() = pressPos;
				cnoid::Matrix3 rotMat;
				generateRotMatFromVec( nnp(0), nnp(1), nnp(2), rotMat );
				cnoid::Vector3 normVec( nnp );
				localArrowBI->body()->rootLink()->R() = rotMat;

				localArrowBI->notifyKinematicStateChange();

				// send the results to the controller
				clusterNormal.clear();
				clusterNormal.push_back(-trans(pBody->body()->link(0)->attitude())*normVec);
				counter=0;
		}

#ifdef CNOID_ENABLE_OSG
        static OSGSceneBody *localArrowSB = new OSGSceneBody( localArrowBI );
        if( firstRun ) OSGSceneView::mainInstance()->addSceneObject( localArrowSB );
		MessageView::mainInstance()->flush();
        OSGSceneView::mainInstance()->requestRedraw();
#endif
#if !defined(CNOID_10_11_12_13) && !defined(CNOID_14)
		SceneView::instance()->scene()->addChildOnce(localArrowBI->sceneBody(), true);
#endif
		firstRun = false;

		//  SceneView::mainInstance()->removeSceneObject( localArrowSB );
		return true;
}

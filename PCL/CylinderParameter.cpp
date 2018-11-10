#include "CylinderParameter.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

#define SAMPLE_CLOUD 2    //Resolution to sample cloud
#define GRASP_X      0.4  //Grasping position in X direction relative to robot's waist
#define GRASP_RADIUS 0.2  //Radius of grasping area centered at GRASP_X
#define Z_MARGIN     0.75 //Edge position of Grasping Rectangular in Z direction relative to cylinder center (rad*Z_MARGIN) //0.0 for success
#define Y_MARGIN     0.06 //Margin of Grasping Rectangular in Y direction (y_edge = 2*radius + Y_MARGIN) //0.01 for success
#define HAND_OPEN_WIDTH 0.08
#define DIST_RESOLUTION 10
#define NEW_VER
#define DEBUG

//inline double drand()
//{
//#ifdef WIN32
//  return ((double)rand())/((double)RAND_MAX);
//#else
//  return drand48();
//#endif
//}


CylinderFeatures* CylinderFeatures::instance()
{
	static CylinderFeatures* instance = new CylinderFeatures();
	return instance;
}

CylinderFeatures::CylinderFeatures(){
	minLength = 0.0;
	maxLength = 1.0;

#ifdef OBJECT_DISTRIBUTION_MODE
	data_num = 1;
#endif
}

CylinderParameter::CylinderParameter(){
	appEdge << 0.035, 0.04, 0.2;
	cpInterval = 0.02;
}

void CylinderParameter::setCylinderParameter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::ModelCoefficients::Ptr coefficient)
{
	pos << coefficient->values[0], coefficient->values[1], coefficient->values[2];
	dir << coefficient->values[3], coefficient->values[4], coefficient->values[5];
	radius = coefficient->values[6];
	dir = unit(dir);

	double lmax, lmin;
	com << 0, 0, 0;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=input->begin(); it!=input->end(); ++it){
			Vector3 r = Vector3(it->x, it->y, it->z);
			double l = Vector3(r - pos).dot(dir);
			com += r;

			if(it==input->begin()){
					lmax=l;
					lmin=l;
			}
			else if(l<lmin)
					lmin = l;
			else if(l>lmax)
					lmax = l;
	}

	com /= (double)(input->size());
	length = lmax-lmin;
	pos = pos + dir*(lmax+lmin)*0.5;

	connection[0] = -1;
	connection[1] = -1;
}

void CylinderParameter::setDesRadius(double desRadius)
{
	Vector3 o = pos + ((com - pos).dot(dir))*dir;
	Vector3 c = o + radius * unit(com - o);
	Vector3 ct = c + pos - o;

	pos = ct + (pos - ct)*desRadius/radius ;
	radius = desRadius;
}

void CylinderParameter::calcPointCloudDistribution(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	Vector3 o = pos + ((com-pos).dot(dir))*dir;
	Vector3 e = unit(com-o);
	Vector3 l = unit(dir.cross(e));

	stdDev = 0.0;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=input->begin(); it!=input->end(); ++it){
			Vector3 r = Vector3(it->x, it->y, it->z);
			double a = (r - com).dot(l);
			stdDev += a*a;
	}

	stdDev = sqrt( stdDev / (double)(input->size()) );
}

bool CylinderFeatures::extractFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::ModelCoefficients::Ptr coefficient, double desRadius)
{
	cylinderGenerated = false;

	CylinderParameter par;
	par.setCylinderParameter(input, coefficient);
	//par.calcPointCloudDistribution(input);

	if(par.length < 0.02) //minLength)
			return true;

	if(par.length > maxLength)
			return true;

	par.setDesRadius(desRadius); //Set desired radius

	if(includeHiddenPoints(par,input,0.3)) // || isSparseCloud(par,input,0.1))
			return true;

	cylinderParamSet.push_back(par);

	cylinderGenerated = true;

	//cout << "Limit for cylinder length, min:" << minLength << ", max:" << maxLength << ", length:" << par.length << endl;
	return true;
}

bool CylinderFeatures::isSparseCloud(CylinderParameter& par, pcl::PointCloud<pcl::PointXYZ>::Ptr input, double ratio)
{
	Vector3 cdir = unit(cameraPos - par.pos);

	vector<double> lengSet;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=input->begin(); it!=input->end(); ++it){

			Vector3 r = Vector3(it->x, it->y, it->z);

			if(Vector3(r - par.pos).dot(cdir)>0)
					lengSet.push_back( Vector3(r - par.pos).dot(par.dir) );
	}

	sort(lengSet.begin(), lengSet.end());

	double maxDist;
	for(size_t i=0; i<lengSet.size()-1; i++){
			if(i==0)
					maxDist = lengSet[1]-lengSet[0];
			else if(maxDist < lengSet[i+1]-lengSet[i])
					maxDist = lengSet[i+1]-lengSet[i];
	}

	//cout << maxDist << "/" << par.length << "/" << maxDist/par.length << endl;

	if(maxDist/par.length > ratio)
			return true;
	return false;
}


bool CylinderFeatures::includeHiddenPoints(CylinderParameter& par, pcl::PointCloud<pcl::PointXYZ>::Ptr input, double ratio)
{
	Vector3 dir = unit(cameraPos - par.pos);
	int positive=0;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it=input->begin(); it!=input->end(); ++it){
			Vector3 r = Vector3(it->x, it->y, it->z);
			if(Vector3(r - par.pos).dot(dir)>0)
					positive++;
	}

	if((double)positive / (double)( input->size() ) < ratio)
			return true;
	return false;
}

bool CylinderFeatures::checkOverlap(CylinderParameter& par1, CylinderParameter& par2, double ratio)
{
	Vector3 p[2] = {par1.pos, par2.pos}, d[2] = {par1.dir, par2.dir}, q[2];
	double r[2] = {par1.radius, par2.radius}, l[2] = {par1.length, par2.length}, ll[2];

	double dist = minDistancePoints(p[0], d[0], p[1], d[1], q[0], q[1]);

	if(dist > ratio*(r[0] + r[1]))
			return false;

	else if( norm2(q[0]-p[0]) < 0.5*l[0] && norm2(q[1]-p[1]) < 0.5*l[1] )
			return true;

	for(int i=0; i<4; i++){
			int j= (int)(i/2);
			int k= 1;

			Vector3 xj = p[j] + 0.5*k*l[j]*d[j];
			Vector3 nj = normalPoint(xj,  p[1-j], d[1-j] );
			if(norm2(nj - p[1-j]) < 0.5*l[1-j]  && norm2(xj - nj) < ratio*(r[0] + r[1]) )
						return true;
			k *= -1;
	}

	return false;
}

void CylinderFeatures::selectGraspedObject(const vector<Vector3>& cloud)
{
	double Zmargin = Z_MARGIN;
	double Ymargin = Y_MARGIN;


	PlanBase * pb = PlanBase::instance();
	CylinderParameter * cy;

	if(pb->targetArmFinger==NULL){
			cout << "targetArmFinger is not set" << endl;
			return;
	}

    Vector3 graspPos = pb->bodyItemRobot()->body()->link(0)->p() + Vector3(GRASP_X, 0, 0);
	double graspRadius = GRASP_RADIUS;

	idList.clear();
	vector<double> zpos;

	for(size_t i=0; i<cylinderParamSet.size(); i++){

			cy = &cylinderParamSet[i];
#ifdef NEW_VER
			cy->appEdge(1) = HAND_OPEN_WIDTH;
#else
			cy->appEdge(1) = cy->radius*2.0+Ymargin;
#endif
			cy->app.clear();

			if(  sqrt(dbl(cy->pos(0)-graspPos(0)) + dbl(cy->pos(1)-graspPos(1))) < graspRadius){
					idList.push_back(i);
					zpos.push_back(-cy->pos(2));
			}
	}
	sort_by(idList, zpos);

	int dataSize = 10;

	char line[1024];
	VectorXd maxVal(dataSize+1);

	FILE *ifp0 = fopen("extplugin/graspPlugin/PCL/maxvalue.txt", "rb");

	if(fgets(line,256,ifp0) != NULL)
			sscanf(line,"%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",&maxVal(0),&maxVal(1),&maxVal(2),&maxVal(3),&maxVal(4),&maxVal(5),&maxVal(6),&maxVal(7),&maxVal(8),&maxVal(9),&maxVal(10));
#ifdef NEW_VER
	ifstream fin("extplugin/graspPlugin/PCL/count.txt");
	vector<int> count;
	double tmp;
	for(int i=0; i<dataSize+1; i++){
		fin >> tmp;
		count.push_back((int)tmp);
	}
	ifstream fin2("extplugin/graspPlugin/PCL/radiusSet.txt");
	vector<double> radSample;
	double rad;
#if ((_MSC_VER >= 1900) || defined(CNOID_GE_16))
	while(fin2 >> rad)
#else
	while((fin2 >> rad) !=0)
#endif
		radSample.push_back(rad);
#endif

	double height = -100.0;
	Vector3 z(0,0,1);

	for(size_t j=0; j<idList.size(); j++){

			cy = &cylinderParamSet[idList[j]];

			Vector3 app = unit(cross(cy->dir, cross(z,cy->dir)));
			if(dot(app,z)>0) app = - app;

			double d0 = -0.5*cy->length+0.5*cy->appEdge(0);

			for(double d=d0; d<0.5*cy->length; d+=cy->cpInterval){

					for(int i=-2; i<=2; i++){
							Matrix3 R = rodrigues(cy->dir, i*3.14159/8.0);

							ApproachParameter app0;
							app0.dir  = R*app;
							app0.edge = cy->appEdge;
							app0.pos  = cy->pos + d * cy->dir;

							cy->app.push_back(app0);
					}
			}
	}

#ifdef DEBUG
	ofstream fout( "log.m" );
#endif

#ifndef NEW_VER
	for(size_t i=0; i<cloud.size(); i++){

			if(i%SAMPLE_CLOUD != 0) continue;

			for(size_t j=0; j<idList.size(); j++){

					cy = &cylinderParamSet[idList[j]];

					for(vector<ApproachParameter>::iterator ap = cy->app.begin(); ap != cy->app.end(); ++ap){

							vector<Vector3> po, no, e(3);
							e[2] = (*ap).dir;
							e[0] = cy->dir;
							e[1] = unit(cross(e[2], e[0]));

							double c0 = dot(cloud[i] -   cy->pos, e[0]);
							double c1 = dot(cloud[i] - (*ap).pos, e[1]);
							double c2 = dot(cloud[i] - (*ap).pos, e[2]);

							if(fabs(c0) > 0.5*cy->length) continue;

							double f = dataSize*( c0 / cy->length + 0.5 );
							int im = (int)f;
							int ip = im+1;

							if(sqrt(c1*c1 + c2*c2) < maxVal(im)*fabs(ip-f) + maxVal(ip)*fabs(f-im)) continue;

							po.push_back((*ap).pos + 0.5*(*ap).edge(0)*e[0]);
							po.push_back((*ap).pos - 0.5*(*ap).edge(0)*e[0]);
							po.push_back((*ap).pos + 0.5*(*ap).edge(1)*e[1]);
							po.push_back((*ap).pos - 0.5*(*ap).edge(1)*e[1]);
							po.push_back((*ap).pos + cy->radius * e[2] * Zmargin);
							po.push_back((*ap).pos + (cy->radius - (*ap).edge(2))* e[2]);

							no.push_back(-e[0]);
							no.push_back( e[0]);
							no.push_back(-e[1]);
							no.push_back( e[1]);
							no.push_back(-e[2]);
							no.push_back( e[2]);

							bool inside = true;
							for(int k=0; k<6; k++)
									if(dot(no[k], cloud[i]-po[k])<0 )inside = false;

							if(inside){
									ap = cy->app.erase(ap);
									--ap;
							}
					}
			}
	}

#ifdef DEBUG
	for(size_t j=0; j<idList.size(); j++){
			cy = &cylinderParamSet[idList[j]];
			fout << "id" << j << "=[";
			for(vector<ApproachParameter>::iterator ap = cy->app.begin(); ap != cy->app.end(); ++ap)
				fout << (*ap).dir.transpose() << endl;	fout << "];" << endl;
	}
#endif
#else

	for(size_t j=0; j<idList.size(); j++){

			cy = &cylinderParamSet[idList[j]];

			vector<vector<double> > radiusSet(cy->app.size()), pointSet0(cy->app.size()), pointSet1(cy->app.size()), pointSet2(cy->app.size());

			for(size_t i=0; i<cloud.size(); i++){

					if(i%SAMPLE_CLOUD != 0) continue;

					int n=0;
					for(vector<ApproachParameter>::iterator ap = cy->app.begin(); ap != cy->app.end(); ++ap, ++n){

							vector<Vector3> po, no, e(3);
							e[2] = (*ap).dir;
							e[0] = cy->dir;
							e[1] = unit(cross(e[2], e[0]));

							double c0 = dot(cloud[i] -   cy->pos, e[0]);
							double c1 = dot(cloud[i] - (*ap).pos, e[1]);
							double c2 = dot(cloud[i] - (*ap).pos, e[2]);

							if(fabs(c0) > 0.5*cy->length) continue;

							po.push_back((*ap).pos + 0.5*(*ap).edge(0)*e[0]);
							po.push_back((*ap).pos - 0.5*(*ap).edge(0)*e[0]);
							po.push_back((*ap).pos + 0.5*(*ap).edge(1)*e[1]);
							po.push_back((*ap).pos - 0.5*(*ap).edge(1)*e[1]);
							po.push_back((*ap).pos + cy->radius * e[2] * Zmargin);
							po.push_back((*ap).pos + (cy->radius - (*ap).edge(2))* e[2]);

							no.push_back(-e[0]);
							no.push_back( e[0]);
							no.push_back(-e[1]);
							no.push_back( e[1]);
							no.push_back(-e[2]);
							no.push_back( e[2]);

							bool inside = true;
							for(int k=0; k<6; k++)
									if(dot(no[k], cloud[i]-po[k])<0 )inside = false;

							double f = dataSize*( c0 / cy->length + 0.5 );
							int im = (int)f;
							int ip = im+1;

							if(sqrt(c1*c1 + c2*c2) < maxVal(im)*fabs(ip-f) + maxVal(ip)*fabs(f-im)) inside = false; //!!!!!!

							if(inside){
									radiusSet[n].push_back(sqrt(c1*c1+c2*c2));
									pointSet0[n].push_back(-c0);
									pointSet1[n].push_back( c1);
									pointSet2[n].push_back(-c2);
							}
					}
			}

			vector<double> mean(cy->app.size()), vari(cy->app.size());
//			vector<SegLinearRegression> slr(cy->app.size());

			int n=0;
			for(vector<ApproachParameter>::iterator ap = cy->app.begin(); ap != cy->app.end(); ++ap, ++n){
//			for(size_t n=0; n<cy->app.size(); n++){
#ifdef DEBUG
					fout << "R2_" << j << n << "=[";
#endif
					for(size_t k=0; k<pointSet1[n].size(); k++)
							(*ap).pt.push_back(Vector3(pointSet0[n][k], pointSet1[n][k], pointSet2[n][k]));

					sort_by((*ap).pt, pointSet1[n]);

					int h_min = 0;
					int step_size = 1;

					for(size_t m=0; m<(*ap).pt.size(); m+=step_size){

							vector<double> x, y, z;
							for(int h=h_min; h<m; h++){
									x.push_back((*ap).pt[h](0));
									y.push_back((*ap).pt[h](1));
									z.push_back((*ap).pt[h](2));
							}

							Segment seg;
							seg.start = h_min;

							double R2;
							if(x.size()<3){
									R2 = 1.0;
									seg.ls.a = 0.0;
									seg.ls.b = 0.0;
									seg.ls.c = average(z);
							}
							else{

									LeastSquare ls(x, y, z);

									double Ezz=0.0;
									for(size_t g=0; g<x.size(); g++)
											Ezz += dbl(z[g] - ls.a*x[g] - ls.b*y[g] - ls.c);
									Ezz /= (double)x.size();

//							    		fout << "%R2_" << j << n << m << endl;
//							    		fout << "x" << j<< n << m << "=[";for(size_t g=0; g<x.size(); g++)fout << x[g] << " ";	fout << "]" << endl;
//							    		fout << "y" << j<< n << m << "=[";for(size_t g=0; g<y.size(); g++)fout << y[g] << " ";	fout << "]" << endl;
//					    				fout << "LS" << j<< n << m << "=[" << h_min << " " << ls.a << " " << ls.b << " " << ls.Sxx << " " << ls.Syy << " " << ls.Sxy << " " << Eyy << "]" << endl;

									R2 = 1 - Ezz/(ls.Szz * ls.Szz);
									seg.ls = ls;
							}

							int df = (*ap).pt.size() - m;
							if(df < step_size && df > 1 ) step_size = df - 1;
#ifdef DEBUG
							fout << R2 << " ";
#endif
							if(R2 < 0.7 || m==(*ap).pt.size()-1){
									seg.end = m;
									(*ap).seg.push_back(seg);
									h_min = m+1;
							}
					}
					(*ap).weight = (double)((*ap).pt.size());
#ifdef DEBUG
					fout << "];" << endl;
					fout << "pointSet0"<< j << n << "=["; for(size_t m=0; m<(*ap).pt.size();  m++) fout << (*ap).pt[m](0) << " "; fout << "];" << endl;
					fout << "pointSet1"<< j << n << "=["; for(size_t m=0; m<(*ap).pt.size();  m++) fout << (*ap).pt[m](1) << " "; fout << "];" << endl;
					fout << "pointSet2"<< j << n << "=["; for(size_t m=0; m<(*ap).pt.size();  m++) fout << (*ap).pt[m](2) << " "; fout << "];" << endl;
					fout << "BPs"      << j << n << "=["; for(size_t m=0; m<(*ap).seg.size(); m++) fout << (*ap).seg[m].start << " "; fout << "];" << endl;
					fout << "BPe"      << j << n << "=["; for(size_t m=0; m<(*ap).seg.size(); m++) fout << (*ap).seg[m].end   << " "; fout << "];" << endl;
					fout << "BPa"      << j << n << "=["; for(size_t m=0; m<(*ap).seg.size(); m++) fout << (*ap).seg[m].ls.a  << " "; fout << "];" << endl;
					fout << "BPb"      << j << n << "=["; for(size_t m=0; m<(*ap).seg.size(); m++) fout << (*ap).seg[m].ls.b  << " "; fout << "];" << endl;
					fout << "BPc"      << j << n << "=["; for(size_t m=0; m<(*ap).seg.size(); m++) fout << (*ap).seg[m].ls.c  << " "; fout << "];" << endl;
#endif
					mean[n]=0.0;
					for(size_t k=0; k<radiusSet[n].size(); k++)
							mean[n] += radiusSet[n][k];
					mean[n] /= (double)radiusSet[n].size();

					vari[n]=0.0;
					for(size_t k=0; k<radiusSet[n].size(); k++)
							vari[n] += dbl(mean[n] - radiusSet[n][k]);
					vari[n] /= (double)radiusSet[n].size();
					vari[n] = sqrt(vari[n]);
			}

			for(vector<ApproachParameter>::iterator ap = cy->app.begin(); ap != cy->app.end(); ++ap){

					for(size_t m=0; m<(*ap).seg.size(); m++){

							if( ((*ap).pt[(*ap).seg[m].start](1)>0 && (*ap).seg[m].ls.b <0) || ((*ap).pt[(*ap).seg[m].end](1)<0 && (*ap).seg[m].ls.b >0)    ){

									ap = cy->app.erase(ap);
									--ap;
									break;
							}
					}
			}

//			double vari_m = vmin(vari);
//
//			n=0;
//			vector<double> appLength(cy->app.size());
//			for(vector<ApproachParameter>::iterator ap = cy->app.begin(); ap != cy->app.end(); ++ap, ++n){
//
//					double r = mean[n]; // - vari[n];
//
//					double c0 = dot((*ap).pos - cy->pos, cy->dir);
//					double f = dataSize*( c0 / cy->length + 0.5 );
//					int im = (int)f;
//					int ip = im+1;
//
//					//Possibility of collision
//					int c=0;
//					for(size_t k=0; k<im; k++)
//						c += count[k];
//
////					cout << "im=" << im << ", mean=" << mean[n] << ", " << "vari=" << vari[n] << ", ";
//					double colPosbl_m=0.0;
//					for(int k=c; k<c+count[im]; k++){
//						if(radSample[k] > r){
//							colPosbl_m = (double)(k-c)/(double)count[im];
////							cout << "[" << radSample[k] << "/" << r << ", " << c << "/" << k << "/" << c+count[im] << "]";
//							break;
//						}
//					}
//
//					c=0;
//					for(size_t k=0; k<ip; k++)
//						c += count[k];
//
//					double colPosbl_p=0.0;
//					for(int k=c; k<c+count[ip]; k++)
//						if(radSample[k] > r){
//							colPosbl_p = (double)(k-c)/(double)count[ip];
//							break;
//						}
//
//					double colPosbl = colPosbl_m*fabs(ip-f) + colPosbl_p*fabs(f-im);
//
//					int density=0;
//					for(size_t k=0; k<radiusSet[n].size(); k++)
//						if(radiusSet[n][k] > r-vari_m*0.5 && radiusSet[n][k] < r+vari_m*0.5)
//							density++;
//
//					appLength[n] = density*colPosbl;
//					(*ap).length = density*colPosbl;
////					cout << "(" << (*ap).length << ", " << density << ", " << colPosbl << ", " << colPosbl_p << ", " << colPosbl_m << ")"<< endl;
//			}
	}
#endif
	string cfile = "extplugin/graspPlugin/PCL/models/cylinder";

	BodyItemPtr item0(new BodyItem());
	item0->load(cfile+"_ref.yaml", "OpenHRP-VRML-MODEL");

	int size=10;
	if(idList.size()<10) size = idList.size();

	pb->multiTargetObject.resize(size);

	for(int i=0; i<size; i++){

			cy = &cylinderParamSet[idList[i]];

			stringstream ss;
			ss << i;
			write_cylinder_vrml(cfile+ss.str()+".wrl", *cy);

			vector<ClusterParameter> objCluster, envCluster;
			ParameterFileData::instance()->readObjYamlFile(item0, objCluster);
			ParameterFileData::instance()->readEnvYamlFile(item0, envCluster);

			writeYamlFile(i, objCluster, envCluster, cy->radius/0.02, cy->length/0.06);

			BodyItemPtr item = new BodyItem();

			if(item->load(cfile+ss.str()+".yaml", "OpenHRP-VRML-MODEL")) {
					RootItem::mainInstance()->addChildItem(item);
					ItemTreeView::mainInstance()->checkItem(item, true);
			}

			Vector3 y(0,1,0);
#if defined(CNOID_10_11_12_13) || defined(CNOID_14)
			item->body()->link(0)->coldetModel()->setPrimitiveType(ColdetModel::SP_CYLINDER);
			item->body()->link(0)->coldetModel()->setNumPrimitiveParams(2);
			item->body()->link(0)->coldetModel()->setPrimitiveParam(0, cy->radius);
			item->body()->link(0)->coldetModel()->setPrimitiveParam(1, cy->length);
#else
			SgMeshPtr mesh = ColdetConverter::ExtractMesh(item->body()->link(0)->collisionShape());
			mesh->setPrimitive(SgMesh::Cylinder(cy->radius, cy->length));
#endif
			item->body()->link(0)->p() = cy->pos;
			item->body()->link(0)->setAttitude( rodrigues(unit(cross(cy->dir, y)), -acos(dot(cy->dir,y))) );
			item->notifyKinematicStateChange();

			pb->multiTargetObject[i] = new TargetObject(item);

			Vector3 p = item->body()->link(0)->p();
			Matrix3 R = item->body()->link(0)->attitude();

			pb->multiTargetObject[i]->approach.resize(cy->app.size());

			for(size_t j=0; j<cy->app.size(); j++){

					Vector3 n = unit(cross(cy->dir, cy->app[j].dir));

					Vector3 direction = trans(R) * cy->app[j].dir;
					vector<Vector3> position;
					position.push_back( trans(R)*(cy->app[j].pos + (cy->radius-0.0025)*n - cy->pos) );
					position.push_back( trans(R)*(cy->app[j].pos - (cy->radius-0.0025)*n - cy->pos) );
					vector<Vector3> normal;
					normal.push_back( trans(R)*n );
					normal.push_back(-trans(R)*n );

					pb->multiTargetObject[i]->approach[j] = new Approach(direction, position, normal);
			}
	}

	if(size>0)
			pb->SetGraspedObject(pb->multiTargetObject[0]->bodyItemObject);
}

void CylinderFeatures::connectCylinders()
{
	double Length = 0.3;
	double difLength = 0.05;
	double EdgeDist = 0.0;
	double difEdgeDist = 0.01;
	double Direct = 0.0;
	double difDirect = 0.3;
	double SideDist = 0.0;
	double difSideDist = 0.01;
	double difRadius = 0.01;

	//Adjust Cylinder Length
	for(size_t i=0; i<cylinderParamSet.size(); i++)
			for(size_t j=i+1; j<cylinderParamSet.size(); j++){

					Vector3 p1 = cylinderParamSet[i].pos;
					Vector3 p2 = cylinderParamSet[j].pos;
					Vector3 r1 = cylinderParamSet[i].dir;
					Vector3 r2 = cylinderParamSet[j].dir;
					double l1 = cylinderParamSet[i].length;
					double l2 = cylinderParamSet[j].length;

					Vector3 q1, q2;
					minDistancePoints(p1,r1, p2,r2, q1, q2);

					Vector3 pp2 = p2 + q1 - q2;

					if(norm2(q1-q2) > difEdgeDist) continue;

					if(norm2(q1-p1)+0.5*l1 + norm2(q2-p2)+0.5*l2 > Length ) continue;

					if(cos_th(q1, p1, pp2) > -difDirect) continue;

					cylinderParamSet[i].length = norm2(q1-p1) + l1/2.0;
					cylinderParamSet[j].length = norm2(q2-p2) + l2/2.0;

					cylinderParamSet[i].pos = (q1+p1 - unit(q1-p1)*l1/2.0)/2.0;
					cylinderParamSet[j].pos = (q2+p2 - unit(q2-p2)*l2/2.0)/2.0;

					if(dot(q1-p1, r1)>0) cylinderParamSet[i].connection[0]=j;
					else                 cylinderParamSet[i].connection[1]=j;
					if(dot(q2-p2, r2)>0) cylinderParamSet[j].connection[0]=i;
					else                 cylinderParamSet[j].connection[1]=i;
			}

	//Adjust Cylinder Radius
	/*
	for(size_t i=0; i<cylinderParamSet.size(); i++)
			for(size_t j=i+1; j<cylinderParamSet.size(); j++){

					if(checkOverlap(cylinderParamSet[i], cylinderParamSet[j], 1.0))
							resizeCylinder(cylinderParamSet[i], cylinderParamSet[j], 1.0);
		}
		*/
}

void CylinderFeatures::write_cylinder_vrml(string file, CylinderParameter& par){

	ofstream fout(file.c_str());

	fout << "#VRML V2.0 utf8" << endl;
	fout << "            Transform {" << endl;
	fout << "              children[" << endl;
	fout << "                Shape{" << endl;
	fout << "                  geometry Cylinder{" << endl;
	fout << "                    radius " << par.radius << endl;
	fout << "                    height " << par.length << endl;
	fout << "                  }" << endl;
	fout << "                  appearance Appearance{" << endl;
	fout << "                    material Material{" << endl;
	fout << "                      diffuseColor 0.2 0.2 0.2" << endl;
	fout << "                    }" << endl;
	fout << "                  }" << endl;
	fout << "                }" << endl;
	fout << "              ]" << endl;
	fout << "            }" << endl;

}

void CylinderFeatures::writeYamlFile(int id, vector<ClusterParameter>& objClusters, vector<ClusterParameter>& envClusters, double r, double h){

		stringstream ss;
		ss << id;

		string filename = "extplugin/graspPlugin/PCL/models/cylinder" + ss.str() + ".yaml";

		ofstream fout;
		fout.open(filename.c_str());

		fout << "modelFile: ./cylinder" + ss.str() + "hrp.wrl" << endl;
		fout << endl;

		fout << "Cluster:" << endl;
		for(size_t j=0;j<objClusters.size();j++){

				fout << "-" << endl;
				fout << "    id: [ " << objClusters[j].id << " ]" << endl;
				fout << "    pair: [ ";
				for(unsigned int k=0; k<objClusters[j].idPair.size(); k++){
						fout << objClusters[j].idPair[k];
						if(k==objClusters[j].idPair.size()-1)
							fout  << " ]" << endl;
						else
							fout << ", ";
				}
				fout << "    area: [ "           << objClusters[j].area*r*h << " ]" << endl;
				fout << "    intention: [ "      << objClusters[j].intention << " ]" << endl;
				fout << "    outer_normal: [ "   << objClusters[j].normal(0)  << ", " << objClusters[j].normal(1)  << ", " << objClusters[j].normal(2)  << " ]" << endl;
				fout << "    tangent_vector: [ " << objClusters[j].tangent(0) << ", " << objClusters[j].tangent(1) << ", " << objClusters[j].tangent(2) << " ]" << endl;
				fout << "    approach_vector: [ ";
				for(unsigned int i=0; i<objClusters[j].approachVec.size(); i++){
						fout << objClusters[j].approachVec[i](0) << ", " << objClusters[j].approachVec[i](1) << ", " << objClusters[j].approachVec[i](2);
						if(i==objClusters[j].approachVec.size()-1)
							fout << " ]" << endl;
						else
							fout << ", ";
				}

				Vector3 a(r,h,r), e(h,r,r);
				if(dot(objClusters[j].normal, Vector3(0,1,0)) > 0.9 || dot(objClusters[j].normal, Vector3(0,1,0)) < -0.9)
					e << r, r, h;

				fout << "    control_points: [ ";
				for(vector<Vector3>::const_iterator I=objClusters[j].controlPoints.begin(); I!=objClusters[j].controlPoints.end(); I++){
						fout << (*I)(0)*a(0) << ", " << (*I)(1)*a(1) << ", " << (*I)(2)*a(2);
						if(I==objClusters[j].controlPoints.end()-1)
							fout << " ]" << endl;
						else
							fout << ", ";
				}
				fout << "    bounding_box_edge: [ "   << objClusters[j].bbEdge(0)*e(0)   << ", " << objClusters[j].bbEdge(1)*e(1)   << ", " << objClusters[j].bbEdge(2)*e(2)   << " ]" << endl;
				fout << "    bounding_box_center: [ " << objClusters[j].bbCenter(0)*a(0) << ", " << objClusters[j].bbCenter(1)*a(1) << ", " << objClusters[j].bbCenter(2)*a(2) << " ]" << endl;
		}

		fout << "EnvCluster:" << endl;
		for(size_t j=0;j<envClusters.size();j++){

				fout << "  -" << endl;
				fout << "    id: [ " << envClusters[j].id << " ]" << endl;
				fout << "    area: [ "           << envClusters[j].area*r*h << " ]" << endl;
				fout << "    outer_normal: [ "   << envClusters[j].normal(0)  << ", " << envClusters[j].normal(1)  << ", " << envClusters[j].normal(2)  << " ]" << endl;
				fout << "    tangent_vector: [ " << envClusters[j].tangent(0) << ", " << envClusters[j].tangent(1) << ", " << envClusters[j].tangent(2) << " ]" << endl;

				Vector3 a(r,h,r), e(h,r,r);
				if(dot(objClusters[j].normal, Vector3(0,1,0)) > 0.9 || dot(objClusters[j].normal, Vector3(0,1,0)) < -0.9)
					e << r, r, h;

				fout << "    convexhull: [ ";
				for(vector<Vector3>::iterator jt=envClusters[j].convexhullPoints.begin(); jt!= envClusters[j].convexhullPoints.end(); ++jt){
						fout << (*jt)(0)*a(0) << ", " << (*jt)(1)*a(1) << ", " << (*jt)(2)*a(2);
						if(jt != envClusters[j].convexhullPoints.end()-1)
								fout << ", ";
				}
				fout << " ]" << endl;

				for(size_t k=0; k<envClusters[j].boundaryPointList.size(); k++){
						fout << "    boundary: [ ";
						for(vector<Vector3>::iterator jt=envClusters[j].boundaryPointList[k].begin(); jt!= envClusters[j].boundaryPointList[k].end(); ++jt){
								fout << (*jt)(0)*a(0) << ", " << (*jt)(1)*a(1) << ", " << (*jt)(2)*a(2);
								if(jt != envClusters[j].boundaryPointList[k].end()-1)
										fout << ", ";
						}
						fout << " ]" << endl;
				}
				fout << "    convexity: [ "   << envClusters[j].Convexity   << " ]" << endl;
				fout << "    bounding_box_edge: [ "   << envClusters[j].bbEdge(0)*e(0)   << ", " << envClusters[j].bbEdge(1)*e(1)   << ", " << envClusters[j].bbEdge(2)*e(2)   << " ]" << endl;
				fout << "    bounding_box_center: [ " << envClusters[j].bbCenter(0)*a(0) << ", " << envClusters[j].bbCenter(1)*a(1) << ", " << envClusters[j].bbCenter(2)*a(2) << " ]" << endl;
				fout << "    is_putting_cluster: [ " << envClusters[j].isPuttingCluster << " ]" << endl;
		}
}

void CylinderFeatures::calcPointCloudDistribution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cyl, pcl::ModelCoefficients::Ptr coef, double desRadius)
{
	CylinderParameter par;
	par.setCylinderParameter(cloud_cyl, coef);

	par.setDesRadius(desRadius);

	vector<double> difRadius, difLength;
	Vector3 cog(0,0,0);
	int c=0;
	for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++){
			Vector3 p(it->x, it->y, it->z);

			Vector3 r = normalPoint(p, par.pos, par.dir);

			if(norm2(r-par.pos) > par.length/2.0) continue;

			cog += p;
			c++;

			difLength.push_back((p-par.pos).dot(par.dir));
			difRadius.push_back(norm2(p-r) - par.radius);
	}
	cog /= (double)c;

	cog = t2_ + R2_*(t1_ + R1_*Vector3(cog(0), cog(2), -cog(1)));

    Vector3 graspPos = PlanBase::instance()->bodyItemRobot()->body()->link(0)->p() + Vector3(GRASP_X, 0, 0);
	double graspRadius = GRASP_RADIUS;

	if(  sqrt(dbl(cog(0)-graspPos(0)) + dbl(cog(1)-graspPos(1))) < graspRadius ){

#if 1 //def DEBUG
			cout << "Cylinder length = " << par.length << endl;
			stringstream ss;
			ss << data_num;

			string data_file = "extplugin/graspPlugin/PCL/samples/rad" + ss.str() + ".m";
			ofstream fout(data_file.c_str());

			fout << "difRadius" << data_num << "=[";
			for(size_t i=0; i<difRadius.size(); i++)
					fout << difRadius[i]+desRadius << " ";
			fout << "];" << endl;
			fout << "difLength" << data_num << "=[";
			for(size_t i=0; i<difLength.size(); i++)
					fout << difLength[i] << " ";
			fout << "];" << endl;
			fout << "plot(difLength" << data_num << ", difRadius" << data_num << ",\"*\")" << endl;
			fout << "length" << data_num << "=" << par.length << ";" << endl;
			fout << "desRadius" << data_num << "=" << desRadius << ";" << endl;
			fout << "dataSize=" << DIST_RESOLUTION << ";" << endl;
			data_num++;
#endif
#if 0
			int dataSize = DIST_RESOLUTION;
			VectorXd xc(dataSize+1), sg2(dataSize+1);

			double dLength = par.length/(double)dataSize;
			for(int i=0; i<= dataSize; i++){
					double d = dLength*(i-dataSize/2);

					int count = 0;
					double mean = 0.0;
					for(size_t j=0; j<difLength.size(); j++){
							if(fabs(d-difLength[j]) > dLength) continue;
							count++;
							mean += difRadius[j];
					}
					mean /= (double)count;

					double variance = 0.0;
					for(size_t j=0; j<difLength.size(); j++){
							if(fabs(d-difLength[j]) > dLength) continue;
							variance += dbl(mean - difRadius[j]);
					}
					variance /= (double)count;
					variance = sqrt(variance);

					xc(i)  = mean + desRadius;
					sg2(i) = variance;
			}
			ofstream fout("extplugin/graspPlugin/PCL/mean.txt");
			fout << xc.transpose();
			ofstream fout2("extplugin/graspPlugin/PCL/variance.txt");
			fout2 << sg2.transpose();
#endif
	}
}


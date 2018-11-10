#include "MatchClusters.h"
#include "../Grasp/VectorMath.h"
#include <iostream>
#include <Eigen/SVD>

using namespace std;
using namespace cnoid;
using namespace grasp;
using namespace Eigen;

void permutation(vector<int>in, vector<int>& out, int j){
	if(j==in.size()){
		for(int i=0;i<in.size();i++){
			out.push_back(in[i]);
		}
		return;
	}
	for(int i=j;i<in.size();i++){
		vector<int>in2 = in;
		int t=in2[j];
		in2[j] = in2[i];
		in2[i] = t;
		permutation(in2,out,j+1);
	}
}

Matrix3 calcObjRot(const Matrix3 &a, int n) {
	Matrix3 ret;

	for (int i = 0; i < 3; i++) {
		if (n == 0) {    ret(i, 0) = a(i, 0);     ret(i, 1) = a(i, 1);     ret(i, 2) = a(i, 2);     }
		else if (n == 1) {    ret(i, 0) = a(i, 2);     ret(i, 1) = a(i, 0);     ret(i, 2) = a(i, 1);     }
		else if (n == 2) {    ret(i, 0) = a(i, 1);     ret(i, 1) = a(i, 2);     ret(i, 2) = a(i, 0);     }
		else if (n == 3) {    ret(i, 0) = -a(i, 1);     ret(i, 1) = -a(i, 0);     ret(i, 2) = -a(i, 2);     }
		else if (n == 4) {	ret(i, 0) = -a(i, 2);     ret(i, 1) = -a(i, 1);     ret(i, 2) = -a(i, 0);     }
		else if (n == 5) {	ret(i, 0) = -a(i, 0);     ret(i, 1) = -a(i, 2);     ret(i, 2) = -a(i, 1);     }
	}

	return ret;
}

Matrix3 calcHandRot(Matrix3 &a1, int n1) {
	Matrix3 ret;
	Vector3 x1(0, 0, 0);
	Vector3 y1(0, 0, 0);
	Vector3 z1(0, 0, 0);

//    for(int i=0; i<3; i++) z1[i]= -1.0*a1(i,2);
	z1 = -1.0 * col(a1, 2);

	if (n1 == 0)      x1 =      col(a1, 0);
	else if (n1 == 1)      x1 = -1.0 * col(a1, 0);
	else if (n1 == 2)      x1 =      col(a1, 1);
	else if (n1 == 3)      x1 = -1.0 * col(a1, 1);

	y1 = cross(z1, x1);

	for (int i = 0; i < 3; i++) {
		ret(i, 0) = x1(i);
		ret(i, 1) = y1(i);
		ret(i, 2) = z1(i);
	}
//	cout <<n1 << a1 << x1 << y1 << z1 << ret;

	return ret;
}


void MatchClusters::quadricalMatchClusters(ObjectShape& object1, ObjectShape& object2, ObjectShape& outObject){
	vector<ClusterImpl*> c1;
	vector<ClusterImpl*> c2;
	
	for(int i=0; i<object1.nClusterNodes; i++){
		if( object1.clusterNodes[i].isAliveNode == false) continue;
		c1.push_back(&object1.clusterNodes[i]);
	}
	for(int i=0; i<object2.nClusterNodes; i++){
		if( object2.clusterNodes[i].isAliveNode == false) continue;
		c2.push_back(&object2.clusterNodes[i]);
	}
	if(c1.size() != c2.size()) return;
	
	for(int i=0; i < c1.size(); i++){
		c1[i]->center = c1[i]->sumQuadDistribution.col(9).segment<3>(6)/c1[i]->sumQuadDistribution(9,9);
	}
	for(int i=0; i < c2.size(); i++){
		c2[i]->center = c2[i]->sumQuadDistribution.col(9).segment<3>(6)/c2[i]->sumQuadDistribution(9,9);
	}
	Matrix3 rot1,rot2;
	
	const int n = c1.size();
	vector<int>in;
	vector<int>out;
	for(int i=0;i<n;i++) in.push_back(i);
	permutation(in,out,0);
	
	double minError=1.0e10;
	int imin=0;
	for(int i=0;i<out.size()/n;i++){
		vector<ClusterImpl*>c2_;
		c2_.clear();
		for(int j=0;j<n;j++){
			c2_.push_back( c2[out[i*n+j]] );
		}
		vector<Matrix3> rot1_;
		Matrix3 rot2_ = Matrix3::Identity();
		calcRotation(c1,c2_,rot1_);
		double error = 0;
		for(int j=0;j<n;j++){
			error += matchingErrorClusters(*c1[j],*c2_[j],rot1_[0],rot2_);
			error += matchingErrorClusters(*c2_[j],*c1[j],rot2_,rot1_[0]);
		}
		cout << "sumerror "<< error << endl;
		if(error < minError){
			minError = error;
			imin = i*n;
			rot1 = rot1_[0];
			rot2 = rot2_;
		}
	}
	cout << minError << " " ;
	for(int j=0;j<n;j++){
		cout <<  out[imin+j] << " ";
	}
	cout << endl;
	quadricalTransform(*c1[0], *c2[out[imin]], object1, object2,rot1,rot2);
	quadricalTransform(*c1[1], *c2[out[imin]+1], object1, object2,rot1,rot2);
}

void MatchClusters::quadricalTransform(ClusterImpl& c1, ClusterImpl& c2, ObjectShape& object1, ObjectShape& object2, Matrix3 rot1, Matrix3 rot2){
	

	int nS=9;
	
	MatrixXd M (nS,nS);
	MatrixXd N (nS,nS);
	VectorXd m(nS);
	VectorXd n(nS);

	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)= c1.sumQuadDistribution(i,j)/c1.sumQuadDistribution(9,9) - c1.sumQuadDistribution(i,9)*c1.sumQuadDistribution(9,j)/c1.sumQuadDistribution(9,9)/c1.sumQuadDistribution(9,9);
			N(i,j)= c2.sumQuadDistribution(i,j)/c2.sumQuadDistribution(9,9) - c2.sumQuadDistribution(i,9)*c2.sumQuadDistribution(9,j)/c2.sumQuadDistribution(9,9)/c2.sumQuadDistribution(9,9);
		}
		m[i] = c1.sumQuadDistribution(i,9)/c1.sumQuadDistribution(9,9);
		n[i] = c2.sumQuadDistribution(i,9)/c2.sumQuadDistribution(9,9);
	}
	
	double minError=1.0e10;
	
	static int cnt=0;
	if(0){
		dmatrix dp;
		dp = MatrixXd::Zero(3,9);
		double x= 1;
		double y= 1;
		double z= 1;
		dp(0,0) = x;
		dp(1,1) = y;
		dp(2,2) = z;
		dp(0,3) = y; dp(1,3) = x;
		dp(1,4) = z; dp(2,4) = y;
		dp(2,5) = x; dp(0,5) = z;
		dp(0,6) = 1;
		dp(1,7) = 1;
		dp(2,8) = 1;
		MatrixXd ns = dp.transpose()*dp;
		MatrixXd sumQuadError2 = c2.sumQuadError;
		c2.sumQuadError = c2.sumQuadError.cwiseProduct(ns);
		c2.calcQuadricSurfaceParameter();
		c2.sumQuadError = sumQuadError2;
	}
	cnt++;
	

	Vector3 quadCenter1 = c1.quadCenter;
	Vector3 quadCenter2 = c2.quadCenter;

	Vector3 radius1 = c1.quadRadius;
	Vector3 radius2 = c2.quadRadius;

	Matrix3 rot1_ =  c1.quadRot;
	Matrix3 rot2_ = rot1.inverse()*rot2*c1.quadRot;;
	
	Matrix3 scale1 = rot1_.transpose()*M.block<3,3>(6,6)*rot1_;
	Matrix3 scale2 = rot2_.transpose()*N.block<3,3>(6,6)*rot2_;
	Vector3 linearCenter1 =  c1.sumQuadDistribution.col(9).segment<3>(6)/c1.sumQuadDistribution(9,9);
	Vector3 linearCenter2 =  c2.sumQuadDistribution.col(9).segment<3>(6)/c2.sumQuadDistribution(9,9);

	Eigen::SelfAdjointEigenSolver<MatrixXd> linearScaleEigen1 ( scale1);
	Matrix3 linearScale1 = linearScaleEigen1.eigenvectors() * linearScaleEigen1.eigenvalues().cwiseSqrt().asDiagonal()* linearScaleEigen1.eigenvectors().transpose();
	Eigen::SelfAdjointEigenSolver<MatrixXd> linearScaleEigen2 ( scale2);
	Matrix3 linearScale2 = linearScaleEigen2.eigenvectors() * linearScaleEigen2.eigenvalues().cwiseSqrt().asDiagonal()* linearScaleEigen2.eigenvectors().transpose();
	Matrix3 linearScale = linearScale2.inverse()*linearScale1;
	
	quadCenter1 =  rot2_* linearScale*rot1_.transpose()*(quadCenter1-linearCenter1)+linearCenter2 ;

	//radius1 = scale2.inverse()*scale1 *radius1;
	cout << "radius1 trans"<< radius1.transpose() ; 
	radius1 = scale2.inverse()*scale1 *radius1;
	cout << " to "<< radius1.transpose() << endl; 

	if(!object2.quadVerticies) object2.quadVerticies = new Vertex[object2.nVerticies];

	Matrix3 newQuadA = rot2_*radius1.asDiagonal()*rot2_.transpose();
	
	cout << "quadCenter "<< c1.quadCenter.transpose()  << "\t\t" << quadCenter1.transpose() <<endl;
	cout << "rot "<< rot2  << endl << rot1.transpose() <<endl;
	
	cout <<"coeff" <<c2.quadCoefficient.transpose() << endl;
//	cout << newQuadA << endl;
//	cout << c2.quadCenter.transpose() << endl;
	

	for(int k=0;k<object2.nTriangles;k++){
		Triangle& t= object2.triangles[k];
		if(t.idCluster != c2.id) continue;
		
		for(int i=0; i<3;i++){
			Vector3 pos = object2.verticies[t.ver[i]->id].pos;
			Vector3 pos2 = c2.quadRot.transpose() *( pos - quadCenter2 );
			double scale2 = double(pos2.transpose()*radius2.asDiagonal()*pos2)/c2.quadScale;
			Vector3 normal2 = c2.quadRot*radius2.asDiagonal()*pos2;

			Vector3 rpos = pos - quadCenter1;
			double scale1 = double(rpos.transpose()*newQuadA*rpos)/c1.quadScale;
			Vector3 normal1 = newQuadA*rpos;
			
			double scale3 = normal2.normalized().dot(normal1.normalized());
			if(scale3 <0.8) scale3=0;
			if(scale2 <0.7) scale3=0;
			if(scale1 <0.7) scale3=0;
			
//			cout << scale2/scale1 <<endl;
			if(scale2*scale1 > 0){
//				pos = rot2_*linearScale*rot2_.transpose()* ( rpos*(1.0+scale3* sqrt(scale2/scale1)-scale3) +quadCenter1 - linearCenter1)+ linearCenter1;
				pos = rot2_*linearScale*rot2_.transpose()* rpos*(1.0+scale3* sqrt(scale2/scale1)-scale3) +quadCenter1;
//				pos = linearScale*rpos*(1.0+scale3* sqrt(scale2/scale1)-scale3)+quadCenter1;
//				pos = rot2_*linearScale*rot2_.transpose()*rpos+quadCenter1;
//				pos = rot2_*linearScale*rot2_.transpose()*rpos*sqrt(scale2)+quadCenter1;
//				pos = rot2_*linearScale*rot2_.transpose()*rpos*sqrt(1.0/scale1)*scale3+quadCenter1;
//				pos = rot2_*linearScale*rot2_.transpose()*rpos*sqrt(1.0/1.0)+quadCenter1;
			}
			object2.quadVerticies[t.ver[i]->id].pos = pos;
	//		object2.verticies[t.ver[i]->id].pos = pos;
	//		object2.quadVerticies[i].pos = pos;
		}
	}
	for(int k=0;k<object2.nVerticies;k++){
//		object2.verticies[k].pos = object2.quadVerticies[k].pos;
	}
	
	//c1.calcQuadricSurfaceParameterOutput();
	//c2.calcQuadricSurfaceParameterOutput();
	
/*	
	for(int k=0;k<object2.nTriangles;k++){
		Triangle& t= object2.triangles[k];
		//if(t.idCluster != c2.id) continue;
		
		for(int i=0; i<3;i++){
			Vector3 pos = object2.verticies[t.ver[i]->id].pos;
			pos = rot2.transpose() *( pos - quadCenter2 );
			Vector3 sqrtRadiusPos;
			for(int j=0;j<3;j++){
				double temp = radius2[j] ;
				if(fabs(temp) < 1.0e-10) sqrtRadiusPos[j] =0;
				if(temp <0){
					sqrtRadiusPos[j] = temp*pos[j];
				}else{
					sqrtRadiusPos[j] = temp*pos[j];
				}
			}
			double scale = pos.transpose()*radius2.asDiagonal()*pos;
			Matrix3 a1= radius1.asDiagonal();
			Vector3 rpos = pos - rot2.transpose()*( quadCenter1-quadCenter2);
			double abc[3],x12[2];
			abc[2]= sqrtRadiusPos.transpose()*a1*sqrtRadiusPos;
			abc[1] = -2.0*sqrtRadiusPos.transpose()*a1*rpos;
			abc[0] = rpos.transpose()*a1*rpos - c1.quadScale;
			int n = sol2(abc,x12);
			
			double lambda=0;
			if(n==2){
				if(fabs(x12[0]) <fabs(x12[1])) lambda=x12[0];
				else lambda=x12[1];
			}
			else if(n==1){
				lambda = x12[0];
			}
			for(int j=0;j<3;j++){
				double rate = (rpos[j] - sqrtRadiusPos[j]*lambda)*sqrt(scale/c2.quadScale);
//				double rate = (rpos[j] - sqrtRadiusPos[j]*lambda);
				if( rate*rpos[j] < 0) pos[j] = 0;
				else pos[j]=rate;
			}
			object2.quadVerticies[t.ver[i]->id].pos = linearScale*pos;
	//		object2.quadVerticies[i].pos = pos;
		}
	}
*/	
	
	
}
double MatchClusters::matchingErrorClusters(ClusterImpl& c1, ClusterImpl& c2, cnoid::Matrix3 rot1, cnoid::Matrix3 rot2){
	
	int nS=9;
	
	MatrixXd M (nS,nS);
	MatrixXd N (nS,nS);
	VectorXd m(nS);
	VectorXd n(nS);

	for (int i = 0; i < nS; i++){
		for (int j = 0; j < nS; j++){
			M(i,j)= c1.sumQuadDistribution(i,j)/c1.sumQuadDistribution(9,9) - c1.sumQuadDistribution(i,9)*c1.sumQuadDistribution(9,j)/c1.sumQuadDistribution(9,9)/c1.sumQuadDistribution(9,9);
			N(i,j)= c2.sumQuadDistribution(i,j)/c2.sumQuadDistribution(9,9) - c2.sumQuadDistribution(i,9)*c2.sumQuadDistribution(9,j)/c2.sumQuadDistribution(9,9)/c2.sumQuadDistribution(9,9);
		}
		m[i] = c1.sumQuadDistribution(i,9)/c1.sumQuadDistribution(9,9);
		n[i] = c2.sumQuadDistribution(i,9)/c2.sumQuadDistribution(9,9);
	}
/*	//Matrix3 tempRot[24];
	//for(int i=0;i < 6; i++){
	//	Matrix3 oR = calcObjRot(Matrix3::Identity(), i);
	//	for(int j=0;j<4;j++){
	//		tempRot[i*4+j]= calcHandRot(oR, j); 
	//	}
	//}
*/	
	Vector3 quadCenter1 ;
	Vector3 quadCenter2 ;

	Vector3 radius1 ;
	Vector3 radius2 ;

	Matrix3 linearScale ;
	
	double minError=1.0e10;
	

	Vector3 quadCenter1_ = c1.quadCenter;
	Vector3 quadCenter2_ = c2.quadCenter;

	Vector3 radius1_ = c1.quadRadius;
	Vector3 radius2_ = c2.quadRadius;

	Matrix3 rot1_ = c1.quadRot;
	Matrix3 rot2_ = rot1.inverse()*rot2*c1.quadRot;
	
//	Vector3 k0 (radius1[0]*radius2[0],radius1[1]*radius2[1],radius1[2]*radius2[2]);
	
	Matrix3 scale1 = rot1_.transpose()*M.block<3,3>(6,6)*rot1_;
	Matrix3 scale2 = rot2_.transpose()*N.block<3,3>(6,6)*rot2_;
	Vector3 linearCenter1 =  c1.center;
	Vector3 linearCenter2 =  c2.center;
	Eigen::SelfAdjointEigenSolver<MatrixXd> linearScaleEigen1 ( scale1);
	Matrix3 linearScale1 = linearScaleEigen1.eigenvectors() * linearScaleEigen1.eigenvalues().cwiseSqrt().asDiagonal()* linearScaleEigen1.eigenvectors().transpose();
	Eigen::SelfAdjointEigenSolver<MatrixXd> linearScaleEigen2 ( scale2);
	Matrix3 linearScale2 = linearScaleEigen2.eigenvectors() * linearScaleEigen2.eigenvalues().cwiseSqrt().asDiagonal()* linearScaleEigen2.eigenvectors().transpose();
	Matrix3 linearScale_ = linearScale2.inverse()*linearScale1;
	quadCenter1_ =  rot2_* linearScale_*rot1_.transpose()*(quadCenter1_-linearCenter1)+linearCenter2 ;
	
	//cout << "linearScale " << linearScale_ <<endl;
	
	cout << "radius1 "<< radius1_.transpose() ; 
	radius1_ = scale2.inverse()*scale1 *radius1_;
	cout << " to "<< radius1_.transpose() << endl; 
	
	cout << "quadCoeff"<< c1.quadCoefficient.transpose() << endl;
	cout << c1.quadRot*c1.quadRadius.asDiagonal()*c1.quadRot.transpose() << endl;
	cout << (-2.0* c1.quadRot*c1.quadRadius.asDiagonal()*c1.quadRot.transpose()*c1.quadCenter).transpose() << endl;
	cout << (c1.quadCenter.transpose() * c1.quadRot*c1.quadRadius.asDiagonal()*c1.quadRot.transpose()*c1.quadCenter) -c1.quadScale << endl;
	cout << c1.quadScale << endl;
	
	
	VectorXd newQuadCoefficient(10);
	if(radius1_.norm() > 0){
		Matrix3 newQuadA = rot2_*radius1_.asDiagonal()*rot2_.transpose();
		Vector3 nquad678 = -2.0*newQuadA*quadCenter1_; 
		newQuadCoefficient[0] = newQuadA(0,0);
		newQuadCoefficient[1] = newQuadA(1,1);
		newQuadCoefficient[2] = newQuadA(2,2);
		newQuadCoefficient[3] = 2.0*newQuadA(0,1);
		newQuadCoefficient[4] = 2.0*newQuadA(1,2);
		newQuadCoefficient[5] = 2.0*newQuadA(2,0);
		newQuadCoefficient[6] = nquad678[0];
		newQuadCoefficient[7] = nquad678[1];
		newQuadCoefficient[8] = nquad678[2];
		newQuadCoefficient[9] = quadCenter1_.transpose()*newQuadA*quadCenter1_-c1.quadScale;
	}
	else{
		Vector3 nquad678 =   rot1.inverse()*rot2*c1.quadCoefficient.segment<3>(6); 
		newQuadCoefficient[0] = 0;
		newQuadCoefficient[1] = 0;
		newQuadCoefficient[2] = 0;
		newQuadCoefficient[3] = 0;
		newQuadCoefficient[4] = 0;
		newQuadCoefficient[5] = 0;
		newQuadCoefficient[6] = nquad678[0];
		newQuadCoefficient[7] = nquad678[1];
		newQuadCoefficient[8] = nquad678[2];
		newQuadCoefficient[9] = nquad678.dot(linearCenter2);
	}

	
	double error = (double)(newQuadCoefficient.transpose()*c2.sumQuadDistribution*newQuadCoefficient)  / (newQuadCoefficient.segment<9>(0).transpose()*c2.sumQuadError*newQuadCoefficient.segment<9>(0) );
	
	
	
		
	return error;
}

void MatchClusters::calcRotation(std::vector<ClusterImpl*> c1, std::vector<ClusterImpl*> c2, std::vector<cnoid::Matrix3>& rot1){
	vector<Vector3> p1;
	vector<Vector3> p2;
	if(c1.size() != c2.size()) return;
	
	if(c1.size() !=2) return;
	for(int i=0;i<c1.size();i++){
		Vector3 normal1= c1[i]->quadCenter-c1[i]->center;
		Vector3 normal2= c2[i]->quadCenter-c2[i]->center;
		//normal1 = -normal1;
		//cout << "calc rotation normal" << normal1.transpose() << " " << normal2.transpose() << endl;
		if( (normal1.norm() < normal2.norm()) && normal2.norm()!=0) normal2 = normal1.norm()/normal2.norm()*normal2;  
		if( (normal1.norm() > normal2.norm()) && normal1.norm()!=0) normal1 = normal2.norm()/normal1.norm()*normal1;  
		
		p1.push_back(c1[i]->center);
		p2.push_back(c2[i]->center);
		if(i>0){
			p1.push_back(c1[i]->center+100.0*normal1);
			p2.push_back(c2[i]->center+100.0*normal2);
		}
		
		cout << "position1 "<< c1[i]->center.transpose() << " " <<c1[i]->quadCenter.transpose() << endl;
		cout << "position2 "<< c2[i]->center.transpose() << " " <<c2[i]->quadCenter.transpose() << endl ;
	}

	Vector3 aveP1= Vector3::Zero();
	Vector3 aveP2= Vector3::Zero();
	for(int i=0;i<p1.size();i++){
		aveP1 += p1[i];
		aveP2 += p2[i];
	}
	aveP1 /= (double)p1.size();
	aveP2 /= (double)p2.size();
	
	Matrix3 sum= Matrix3::Zero();
	for(int i=0;i<p1.size();i++){
		sum += (p1[i]-aveP1)*(p2[i]-aveP2).transpose();
	}
	JacobiSVD<Matrix3> svd(sum,  ComputeThinU | ComputeThinV );
	Matrix3 h = Vector3(1,1,(svd.matrixU() * svd.matrixV().transpose()).determinant()).asDiagonal();
	Matrix3 r = svd.matrixU() * h *svd.matrixV().transpose();	
	//Matrix3 r = svd.matrixU() * svd.matrixV().transpose();
	cout << svd.singularValues().transpose() << endl;
	cout << r << endl;
	//rot2 = Matrix3::Identity();
	rot1.push_back(r);
}

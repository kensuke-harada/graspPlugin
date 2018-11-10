/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#include <fstream>
#include <iostream>

#include "ForceClosureTest.h"
#include "VectorMath.h"

double grasp::_drand(){
#ifdef WIN32
	  return ((double)rand())/((double)RAND_MAX);
#else
	  return drand48();
#endif
	}

using namespace cnoid;
using namespace std;
using namespace grasp;


double ForceClosureTest::forceClosureTestEllipsoid(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, double f_max, Vector3 center) {
	//thhd1 : thhd1*fmax : radius of ellipsoid (0<thhd1<1)
	//   1 <= thhd2 <= points
	// Pc[]:  the contact position when the object center is zero


	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	//Vector3 center(0.0, 0.0, 0.0);
	//center = center / (double)points;


	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j, j + 3*i) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
				G(j + 3, k + 3*i) = GAMMA * tm(j, k); ///
			}
	}

	double esize = 0.5;

	Matrix3 Si( diag(1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(f_max) / esize));
	Matrix3 Si2( diag(1.0 / dbl(f_max), 1.0 / dbl(f_max), 1.0 / dbl(f_max / 2.0)));

	double n1 = 1.0;
	double n2 = 0.5;

	MatrixXd U = MatrixXd::Zero(3 * points, 3 * points);
	MatrixXd U2 = MatrixXd::Zero(3 * points, 3 * points);
	VectorXd N(3*points);

	int pnum = 1;
	int bnum = 1;
	int pnum3 = 1;


	for (int i = 0;i < points;i++) {
		pnum *= 2;
	}

	vector <MatrixXd>  Up (pnum);  
	vector <VectorXd>  Np (pnum*2);  

	for (int i = 0;i < pnum;i++) {
		Up[i] = MatrixXd::Zero(3 * points, 3 * points);
		Np[i] = VectorXd(3 * points);
	}

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}

		Matrix3 Ui(v3(tx, ty, ni));

		Matrix3 USU(Ui*Si*trans(Ui));
		Matrix3 US2U(Ui*Si2*trans(Ui));

		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = USU(j, k);
					}
					Np[l](3*i + j) = ni(j) * n1; // normal
				}
			} else {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = US2U(j, k);
					}
					Np[l](3*i + j) = ni(j) * n2;
				}
			}
		}
		bnum *= 2;


	}

	VectorXd phi = G * N;
	MatrixXd UG = inverse(U) * trans(G);
	MatrixXd iGUG = inverse(G * UG);

	int flag = 1;
	double ming = 1000.0;


	vector <bool> iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector <double>  distance(pnum);

	double dp = (double)points;

	for (int i = 0;i < pnum;i++) {
		phi = G * Np[i];


		UG = inverse(Up[i]) * trans(G);
		MatrixXd GUG(G * UG);
		calcPseudoInverse(GUG, iGUG, 1.0e-10);


		double ans3 = inner_prod( VectorXd(wrench - f_max * phi),  VectorXd( iGUG *( wrench - f_max * phi)) );

		if (ans3 >= dp) {
			double temp = -  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			distance[i] = temp;
			if (ming < temp || ming > 0) ming = temp;
			flag = -1;
		} else {
			iflag[i] = true;
			double temp =  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			temp = min ( temp, EvaluateEllipsePointDistanceAxisDirection(iGUG, (wrench - f_max * phi), dp) );
			distance[i] = temp;
			if ( (temp) <= 0) {
				temp = 1000;
			}
			if (temp < ming) {
				ming = temp;
			}
		}

	}
	cout << ming << endl;
	
	if(!isOutputForceSpace && !isOutputTorqueSpace ){
		return ming;
	}

#ifdef VIEW_CONVEX


	vector <MatrixXd> iGUG_(pnum);
	vector <VectorXd> phi_(pnum);

	for (int i = 0;i < pnum;i++) {
		phi_[i] = G * Np[i];
#ifdef VIEW_FORCE_SPACE
		phi_[i][3] = phi_[i][4] = phi_[i][5] = 0;
#else
		phi_[i][0] = phi_[i][1] = phi_[i][2] = 0;  //torque space
#endif

		UG = inverse(Up[i])*trans(G);
		MatrixXd GUG(G* UG);

		for (int j = 0;j < 6;j++) for (int k = 0;k < 6;k++) {
#ifdef VIEW_FORCE_SPACE
				if (j > 2 || k > 2)  GUG(j, k) = (j == k) ? 1 : 0;
#else
				if (j < 3 || k < 3)  GUG(j, k) = (j == k) ? 1 : 0; //torque space
#endif
			}
		calcPseudoInverse(GUG, iGUG, 1.0e-10);
		iGUG_[i] = MatrixXd (iGUG);
	}


	vector<double> exfs, exfs_out;
	VectorXd w(6);
	w[0] =  w[1]  = w[2] = w[3] = w[4] = w[5] = 0;
	for (int j = -100;j < 100;j++) {
		if (j % 10 == 0) cout << j << endl;

		for (int k = -100;k < 100;k++) for (int l = -100;l < 100;l++) {

#ifdef VIEW_FORCE_SPACE
				w[0] = 0.3 * (j);
				w[1] = 0.3 * (k);
				w[2] = 0.3 * (l);
#else
				w[3] = (j); //torque space
				w[4] = (k);
				w[5] = (l);
#endif

				flag = 1;
				for (int i = 0;i < pnum;i++) {
					double ans3 =  inner_prod(w - f_max * phi_[i], iGUG_[i]*(w - f_max * phi_[i]));

					if (ans3 >= dp) {
						flag = -1;
						break;
					}
				}
				if (flag == 1) {
					for (int n = 0; n < 6; n++) exfs.push_back(w(n));
				}
			}
	}
//  calcConvexHull2(6, exfs, exfs_out, wrench, false);
	ConvexAnalysis::outputConvexHull(6, exfs, false);
	cout << "test" << endl;

#endif


}


double ForceClosureTest::forceClosureTestEllipsoidSoftFinger(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, double f_max, const vector<double>& en, Vector3 center) {

	//thhd1 : thhd1*fmax : radius of ellipsoid (0<thhd1<1)
	//   1 <= thhd2 <= points
	// Pc[]:  the contact position when the object center is zero

//	double GAMMA0 = 5.0;

	MatrixXd G = MatrixXd::Zero(6, 4 * points);

	//Vector3 center(0.0, 0.0, 0.0);
	//center = center / (double)points;
	
	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j,  4*i + j) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate

		for (int j = 0; j < 3; j++){
			for (int k = 0; k < 3; k++)
				G(j + 3, 4*i + k) = GAMMA * tm(j, k); ///
			G(j + 3, 4*i + 3) = GAMMA * Nc[i](j);
		}
	}

	double esize = 0.5;
	double n1 = 1.0;
	double n2 = 0.5;
	int pnum = 1;
	int bnum = 1;

	for (int i = 0;i < points;i++)
		pnum *= 2;

	vector <MatrixXd>  Up (pnum);
	vector <VectorXd>  Np (pnum);  

	for (int i = 0;i < pnum;i++) {
		Up[i]  = MatrixXd::Zero(4 * points, 4 * points);
		Np[i]  = VectorXd::Zero(4 * points);
	}

	MatrixXd Si  = MatrixXd::Zero(4,4);
	MatrixXd Si2 = MatrixXd::Zero(4,4);

	for (int i = 0; i < points; i++) {

		Si(0,0) = 1.0 / dbl(mu*f_max) / (1.0 - esize);
		Si(1,1) = 1.0 / dbl(mu*f_max) / (1.0 - esize);
		Si(2,2) = 1.0 / dbl(f_max) / esize;
		Si(3,3) = 1.0 / dbl(mu*f_max*en[i])/ (1.0 - esize);
		Si2(0,0) = 1.0 / dbl(f_max);
		Si2(1,1) = 1.0 / dbl(f_max);
		Si2(2,2) = 1.0 / dbl(f_max / 2.0);
		Si2(3,3) = 1.0 / dbl(f_max*en[i]);

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}
		
		MatrixXd Ui = MatrixXd::Identity(4,4);
		Matrix3 Utmp = v3(tx, ty, ni);
		for(int j=0; j<3; j++)
			for(int k=0; k<3; k++)
				Ui(j,k) = Utmp(j,k);

		MatrixXd Ut1 = Si *trans(Ui);
		MatrixXd USU = Ui * Ut1;

		MatrixXd Ut2 = Si2*trans(Ui);
		MatrixXd US2U = Ui* Ut2;

		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						Up[l](4*i + j, 4*i + k) = USU(j, k);
					}
				}
				for(int j=0; j<3; j++)
					Np[l](4*i + j) = ni(j) * n1;
			} else {
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						Up[l](4*i + j, 4*i + k) = US2U(j, k);
					}
				}
				for (int j = 0; j < 3; j++)
					Np[l](4*i + j) = ni(j) * n2;
			}
		}
		bnum *= 2;


	}
	
	int flag = 1;
	double ming = 1000.0;

	vector <bool> iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector <double>  distance(pnum);

	double dp = (double)points;

	for (int i = 0;i < pnum;i++) {

		VectorXd phi = G * Np[i];

		MatrixXd UG = (inverse(Up[i]) * trans(G));
		MatrixXd GUG  (G* UG), iGUG;
		calcPseudoInverse(GUG, iGUG, 1.0e-10);

		double ans3 = inner_prod(  (wrench - f_max * phi),  ( iGUG * (wrench - f_max * phi) ));

		if (ans3 >= dp) {
			double temp = -  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			distance[i] = temp;
			if (ming < temp || ming > 0) ming = temp;
			flag = -1;
		} else {
			iflag[i] = true;
			double temp =  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			temp = min ( temp, EvaluateEllipsePointDistanceAxisDirection(iGUG, (wrench - f_max * phi), dp) );
			distance[i] = temp;
			if ( (temp) <= 0) {
				temp = 1000;
			}
			if (temp < ming) {
				ming = temp;
			}
		}

	}
	cout << ming << endl;
	
	return ming;
}


double ForceClosureTest::forceClosureTestEllipsoidInternal(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, double f_max) {

	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	Vector3 center(0.0, 0.0, 0.0);
	center = center / (double)points;

	MatrixXd* Gi = new MatrixXd[points];
	//MatrixXd Gi[points];
	for (int i = 0;i < points;i++) {
		Gi[i] = MatrixXd::Zero(6, 3);
	}

	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j, j + 3*i) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate
//	  tm *= GAMMA;



		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
//	      G(j+3, k+3*i) = GAMMA*tm(j,k)*(0.5); ///under test tyu//////////////////////////////////
				G(j + 3, k + 3*i) = GAMMA * tm(j, k); ///
			}
		for (int j = 0; j < 3; j++) {
			Gi[i](j, j) = 1.0;
			for (int k = 0; k < 3; k++) Gi[i](j + 3, k) = GAMMA * tm(j, k);
		}
	}

	double esize = 0.5;
//  double esize = 0.3;

	Matrix3 Si( diag(1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(f_max) / esize));
//  Matrix3 Si( diag(1.0/dbl(mu*f_max), 1.0/dbl(mu*f_max), 1.0/dbl(f_max)));
	Matrix3 Si2( diag(1.0 / dbl(f_max), 1.0 / dbl(f_max), 1.0 / dbl(f_max / 2.0)));

//  double n1=1.0;
	double n1 = 1.0;
	double n2 = 0.5;
//  double n2 = 0.5;
//  double n2 = 0;


//  Matrix3 Si2( diag(1.0/dbl(f_max/2.0), 1.0/dbl(f_max/2.0), 1.0/dbl(f_max/2.0)));


	MatrixXd U = MatrixXd::Zero(3 * points, 3 * points);
	MatrixXd U2 = MatrixXd::Zero(3 * points, 3 * points);
	VectorXd N(3*points);

	//cout <<"Si " <<Si << Si2 << endl;

	int pnum = 1;
	int bnum = 1;
	int pnum3 = 1;


	for (int i = 0;i < points;i++) {
		pnum *= 2;
		pnum3 *= 3;
	}

	vector <MatrixXd>  Up (pnum);  
	vector <VectorXd>  Np (pnum*2);  
//	Up.reserve[pnum];
//	VectorXd Np[pnum*2];

	for (int i = 0;i < pnum;i++) {
		Up[i] = MatrixXd::Zero(3 * points, 3 * points);
		Np[i] = VectorXd(3 * points);
	}

	MatrixXd* Q = new MatrixXd[pnum*2];
	//MatrixXd Q[pnum*2];

	for (int i = 0;i < pnum*2;i++) {
		Q[i] = MatrixXd::Zero(6, 6);
	}
	for (int i = 0;i < pnum*2;i++) {
		Np[i] = VectorXd(3 * points);
	}
	VectorXd traceQ = VectorXd(pnum);

	VectorXd* N3 = new VectorXd[pnum];
	//VectorXd N3[pnum];

	for (int i = 0;i < pnum;i++) {
		N3[i] = VectorXd(6);
	}

	MatrixXd Sii = MatrixXd::Zero(3, 3);
	MatrixXd Si2i = MatrixXd::Zero(3, 3);
	for (int j = 0;j < 3;j++) {
//	    Sii(j,j) = 1.0/sqrt(Si(j,j));
//	    Si2i(j,j) = 1.0/sqrt(Si2(j,j));
		Sii(j, j) = 1.0 / (Si(j, j));
		Si2i(j, j) = 1.0 / (Si2(j, j));
	}

	VectorXd l1 = VectorXd(6);
	vector <MatrixXd> Qi1(points);
	vector <MatrixXd> Qi2(points);

	for (int i = 0; i < points; i++) {
		Qi1[i] = MatrixXd::Zero(6, 6);
		Qi2[i] = MatrixXd::Zero(6, 6);
	}

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}

		Matrix3 Ui(v3(tx, ty, ni));


		MatrixXd GiU =  (Gi[i]* v2d(Ui));
		VectorXd gn(3);

		MatrixXd USUd = GiU*Sii* trans(GiU);
		MatrixXd US2Ud =GiU *Si2i* trans(GiU);

		MatrixXd evec(6, 6);
		VectorXd eval(6);
		calcEigenVectors(USUd, evec, eval);
		double traceQi1 = 0, traceQi2 = 0;

		for (int j = 0;j < 6;j++) eval[j] = ( eval[j] > 1.0e-10) ? sqrt(eval[j]) : 0;
		for (int j = 0;j < 6;j++) traceQi1 += ( eval[j] > 1.0e-10) ? sqrt(eval[j]) : 0;

		calcEigenVectors(US2Ud, evec, eval);
		for (int j = 0;j < 6;j++) traceQi2 += ( eval[j] > 1.0e-10) ? sqrt(eval[j]) : 0;

//		if (thhd1 < 1) traceQi1 = traceQi2 = 1;

		VectorXd nid(3);
		for (int j = 0;j < 3;j++) nid[j] = ni[j];
		VectorXd wni = Gi[i] * nid;

		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				Q[l] += (USUd) / traceQi1;
				traceQ[l] += traceQi1;
				N3[l] += n1 * wni;
			} else {
				Q[l] += (US2Ud) / traceQi2;
				traceQ[l] += traceQi2;
				N3[l] += n2 * wni;
			}
		}

		bnum *= 2;


	}

	VectorXd phi = G* N;
	MatrixXd UG = (inverse(U)* trans(G));
	MatrixXd iGUG = inverse(G*UG);

	int flag = 1;
	double ming = 1000.0;


	vector <bool> iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector <double>  distance(pnum);

	double dp = (double)points;

	ming = 1.0e10;
	for (int i = 0;i < pnum;i++) {
		calcPseudoInverse(Q[i], iGUG, 1.0e-10);
		phi = N3[i];
		dp = traceQ[i];

		double ans3 =  inner_prod(wrench - f_max * phi, (iGUG *(wrench - f_max * phi)));
		if (ans3  < dp) {
			double temp =  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			if (temp < ming) ming = temp;

		} else {
			ming = -1;
		}
	}
	cout << ming << endl;
	//exit(0);

#ifdef VIEW_CONVEX
	//if(ming <= 0.5 || ming ==1000) return ming;


	vector <MatrixXd> iGUG_(pnum3);
	vector <VectorXd> phi_(pnum3);

	for (int i = 0;i < pnum;i++) {
		phi_[i] = N3[i];
#ifdef VIEW_FORCE_SPACE
		phi_[i][3] = phi_[i][4] = phi_[i][5] = 0;
#else
		phi_[i][0] = phi_[i][1] = phi_[i][2] = 0;  //torque space
#endif

		MatrixXd GUG = Q[i];

		for (int j = 0;j < 6;j++) for (int k = 0;k < 6;k++) {
#ifdef VIEW_FORCE_SPACE
				if (j > 2 || k > 2)  GUG(j, k) = (j == k) ? 1 : 0;
#else
				if (j < 3 || k < 3)  GUG(j, k) = (j == k) ? 1 : 0; //torque space
#endif
			}
		calcPseudoInverse(GUG, iGUG, 1.0e-10);
		iGUG_[i] = MatrixXd (iGUG);
	}


	vector<double> exfs, exfs_out;
	VectorXd w(6);
	w[0] =  w[1]  = w[2] = w[3] = w[4] = w[5] = 0;
	for (int j = -100;j < 100;j++) {
		if (j % 10 == 0) cout << j << endl;

		for (int k = -100;k < 100;k++) for (int l = -100;l < 100;l++) {

#ifdef VIEW_FORCE_SPACE
				w[0] = 0.3 * (j);
				w[1] = 0.3 * (k);
				w[2] = 0.3 * (l);
#else
				w[3] = (j); //torque space
				w[4] = (k);
				w[5] = (l);
#endif

				flag = 1;
				for (int i = 0;i < pnum;i++) {
					dp = traceQ[i];
					double ans3 =  inner_prod(w - f_max * phi_[i], (iGUG_[i]*( w - f_max * phi_[i]));

					if (ans3 >= dp) {
						flag = -1;
						break;
					}
				}
				if (flag == 1) {
//		cout << w << endl;
					for (int n = 0; n < 6; n++) exfs.push_back(w(n));
//			cout << w;
				}
			}
	}
//  calcConvexHull2(6, exfs, exfs_out, wrench, false);
	ConvexAnalysis::outputConvexHull(6, exfs, false);
	cout << "test" << endl;
//  exit(0);

//  exit(0);

#endif
	delete[] Gi;
	delete[] Q;
	delete[] N3;
//  if(flag > 0) return ming;

	return ming;

//  int bit2[4]={1,2,4,8};
	for (int i = 0;i < pnum;i++) {
		if (iflag[i] == true) continue;
		ming = 1000.0;
		/*
		      for(int j=0;j<pnum-1;j++){
			  if( (i & j) != 0){
				  cout << "overlap" << endl;
				  continue;
			  }
		*/
		for (int k = 0;k < points;k++) {
			VectorXd Npt (Np[i]);
			MatrixXd  Upt (Up[i]);
			dp = (double)points - 1.0;
			MatrixXd Gt (G);
			for (int l = 0; l < 3; l++) {
				for (int m = 0; m < 6; m++) {
					Gt(m, 3*k + l) = 0;
				}
			}
			/*
				  for(int k=0;k<points;k++){
					  if( j & bit2[k]){
						  dp -= 1.0;
						for(int l=0; l<3; l++){
							for(int m=0; m<6; m++){
								Gt(m, 3*k+l) = 0;
							}
						}
					  }
				  }
				  if(dp < 2.1) continue;
			*/
			phi = Gt* Npt;
			UG = inverse(Upt)* trans(Gt);
			MatrixXd GUG(Gt* UG);
			iGUG = inverse(GUG);

//	  cout << "det " << det(GUG)<< endl;

//	  cout << "Phi"<<phi << endl; //
//	  cout << GUG<< endl; //
//	  cout << Gt<< endl; //

//    calcPseudoInverse(GUG,iGUG,1.0e-10);

			double ans3 =  inner_prod(wrench - f_max * phi, (iGUG*( wrench - f_max * phi)));
			//cout << "a,"<<dp - ans3 << endl;

			if (ans3 >= dp) {
//		  double temp = -EvaluateEllipsePointDistance(iGUG,(wrench-f_max*phi),dp);
				//cout <<"d2,"  << temp << f_max*phi<<endl;
//		  if(ming < temp || ming > 0) ming = temp;
//		  flag=-1;
			} else {
				iflag[i] = true;
				double temp = EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp);
				//cout <<"d2,"  << temp << f_max*phi<< endl;
				if ( (temp) <= 0) {
					temp = 1000;
					//flag = -1;
				}
				if (temp < ming) {
					ming = temp;
				}
			}
		}
		if (iflag[i] == true) {
			cout << i << " b " << ming;
			distance[i] = ming;
		}
	}

	ming = distance[0];
	for (int i = 0;i < pnum;i++) {
		if (ming > distance[i]) ming = distance[i];
//	  cout << distance[i] << " ";
	}


//  if(flag < 0) return flag;
	if (ming == 1000) return -1000;
	//cout << " aaa " <<ming;


	//return 1.0;
	return ming;

}


double ForceClosureTest::ForceClosureTestManipulationForce(VectorXd &wrench, Vector3 Pc[], Vector3 Nc[], int points, double mu, double gamma, MatrixXd jacobi[]) {
	//thhd1 : thhd1*fmax : radius of ellipsoid (0<thhd1<1)
	//   1 <= thhd2 <= points
	// Pc[]:  the contact position when the object center is zero

// GAMMA is scale value for moment
//#define GAMMA 1000.0
//#define GAMMA 0.01
	double f_max = 10.0;

	MatrixXd Mf[6];

	MatrixXd L1 ( MatrixXd::Zero(5, 5) );
	MatrixXd L2 ( MatrixXd::Zero(4, 4) );

	L1(0, 0) = L1(1, 1) = L1(2, 2) = 1.0 / 0.9;
	L1(3, 3) = L1(4, 4) = 1.0 / 0.4;

	L2(0, 0) = L2(1, 1) = 1.0 / 0.9;
	L2(2, 2) = L2(3, 3) = 1.0 / 0.4;

	//Mf[0]  = MatrixXd::Zero(6,6);
	Mf[0] = MatrixXd (  jacobi[0]*  L1* L1*jacobi[0]  ) ;
	//cout << Mf[0] <<endl;
	for (int i = 1;i < 4;i++) {
		Mf[i] =  jacobi[i]*L2*L2*jacobi[i];

		//	cout << Mf[i]<<endl;
		//	cout << jacobi[i]<<endl;
	}

	//exit(0);



	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	Vector3 center(0.0, 0.0, 0.0);
	center = center / (double)points;

	for (int i = 0; i < points; i++) {

		for (int j = 0; j < 3; j++) G(j, j + 3*i) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate
//	  tm *= GAMMA;


		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
				G(j + 3, k + 3*i) = gamma * tm(j, k);
			}

	}


	double esize = 1.0 / 2.0;
//  double esize = 1.0;

	//Matrix3 Si( diag(1.0/dbl(mu*f_max), 1.0/dbl(mu*f_max), 1.0/dbl(thhd1*f_max)));
	Matrix3 Si( diag(1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(f_max) / esize));
//  Matrix3 Si( diag(1.0/dbl(f_max)/(1.0-esize), 1.0/dbl(f_max)/(1.0-esize), 1.0/dbl(f_max)/esize));

// Matrix3 Si( diag(3.0/dbl(mu*f_max), 3.0/dbl(mu*f_max), 9.0/dbl(f_max)));


	Matrix3 Si2( diag(1.0 / dbl(f_max), 1.0 / dbl(f_max), 1.0 / dbl(f_max / 2.0)));
//  Matrix3 Si2( diag(1.0/dbl(f_max/2.0), 1.0/dbl(f_max/2.0), 1.0/dbl(f_max/2.0)));

	MatrixXd U = MatrixXd::Zero(3 * points, 3 * points);
	MatrixXd U2 = MatrixXd::Zero(3 * points, 3 * points);
	VectorXd N(3*points);

	//cout <<"Si " <<Si << Si2 << endl;

	int pnum = 1;
	int bnum = 1;

	for (int i = 0;i < points;i++) {
		pnum *= 2;
	}

	vector <MatrixXd> Up(pnum);
	vector <VectorXd> Np(pnum);

	for (int i = 0;i < pnum;i++) {
		Up[i] = MatrixXd::Zero(3 * points, 3 * points);
		Np[i] = VectorXd(3 * points);
	}

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}

		Matrix3 Ui(v3(tx, ty, ni));


		Matrix3 USU(Ui*Si*trans(Ui));
//    Matrix3 USU(trans(Ui)*Si*(Ui));

		//cout << i << USU << endl;

		Matrix3 US2U(Ui*Si2*trans(Ui));
//    Matrix3 US2U(trans(Ui)*Si2*(Ui));


		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = USU(j, k);
					}
					Np[l](3*i + j) = ni(j); // normal
//	    Np[l](3*i+j) = 2.0/3.0*ni(j);
				}
			} else {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
//	      Up[l](3*i+j, 3*i+k) = US2U(j,k);
						Up[l](3*i + j, 3*i + k) = Mf[i](j, k);
					}
					//Np[l](3*i+j) = ni(j)/2.0;
				}
			}
		}
		bnum *= 2;

		/*
		      for(int j=0; j<3; j++){
			for(int k=0; k<3; k++){
			  U(3*i+j, 3*i+k) = USU(j,k);
			}
			N(3*i+j) = ni(j);
		      }

		      USU = Ui*Si2*trans(Ui);

		      for(int j=0; j<3; j++){
			for(int k=0; k<3; k++){
			      U2(3*i+j, 3*i+k) = USU(j,k);
			}
		      }
		*/

	}

//  cout << Up[0]<< endl;

	VectorXd phi = G*N;
	MatrixXd UG = inverse(U)*trans(G);
	MatrixXd iGUG = inverse(G*UG);


	//double ans1 = thhd2 - dot(wrench-f_max*phi, prod(iGUG, wrench-f_max*phi));
	//double ans1 = (double)points - dot(wrench-f_max*phi, prod(iGUG, wrench-f_max*phi));
	//UG = prod(inverse(U2),trans(G));
	//iGUG = inverse(prod(G,UG));
	//double ans2 = (double)points - dot(wrench-(f_max-lg)*phi, prod(iGUG, wrench-(f_max-lg)*phi));

	int flag = 1;
	double ming = 1000.0;

	//VectorXd dvectemp(6);
	//MatrixXd dmattemp(6,6);
	//double dtemp[2];

	vector <bool>  iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector<double> distance(pnum);

	double dp = (double)points;

	for (int i = 0;i < pnum;i++) {
//    cout << "NP"<<Np[i] << endl; //
		// cout << Up[i]<< endl; //
		phi = G*Np[i];
		UG = inverse(Up[i])* trans(G);
		MatrixXd GUG(G*UG);
//    iGUG = inverse(GUG);
		//iGUG_[i] = MatrixXd (iGUG);
		calcPseudoInverse(GUG, iGUG, 1.0e-10);





		double ans3 = inner_prod(wrench - f_max * phi, (iGUG* wrench - f_max * phi));

		if (ans3 >= dp) {
//	    return -1;
			double temp = -  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			distance[i] = temp;
			//cout << "d,"<<temp << f_max*phi<< endl;
			if (ming < temp || ming > 0) ming = temp;
			flag = -1;
		} else {
			iflag[i] = true;
			double temp =  min( EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp), EvaluateEllipsePointDistance2(iGUG, (wrench - f_max * phi), dp));
			distance[i] = temp;
			//cout << "d,"<<temp << ","<<EvaluateEllipsePointDistance(iGUG,(wrench-f_max*phi),dp)<< ","<<EvaluateEllipsePointDistance2(iGUG,(wrench-f_max*phi),dp)<< endl;
			if ( (temp) <= 0) {
				temp = 1000;
				//flag = -1;
			}
			if (temp < ming) {
				ming = temp;
			}
		}
	}
	//exit(0);

//#define ELLIPSE_CONVEX
#ifdef ELLIPSE_CONVEX
	if (ming <= 0.1 || ming == 1000) return ming;


	MatrixXd iGUG_[pnum];
	VectorXd phi_[pnum];

	for (int i = 0;i < pnum;i++) {
		phi_[i] = (G * Np[i]);
//	phi_[i][3] = phi_[i][4] = phi_[i][5] = 0;
		phi_[i][0] = phi_[i][1] = phi_[i][2] = 0;  //torque space

		UG = (inverse(Up[i]) * trans(G));
		MatrixXd GUG((G * UG));

		for (int j = 0;j < 6;j++) for (int k = 0;k < 6;k++) {
//		    if(j>2 || k>2)  GUG(j,k)=(j==k) ? 1 : 0;
				if (j < 3 || k < 3)  GUG(j, k) = (j == k) ? 1 : 0; //torque space
			}
		iGUG = inverse(GUG);
		iGUG_[i] = MatrixXd (iGUG);
	}
	cout << iGUG_[0] << endl;


	vector<double> exfs, exfs_out;
	VectorXd w(6);
	w[0] =  w[1]  = w[2] = w[3] = w[4] = w[5] = 0;
	for (int j = -130;j < 130;j++) {
		if (j % 10 == 0) cout << j << endl;

		for (int k = -130;k < 130;k++) for (int l = -130;l < 130;l++) {

//		w[0] = 0.3*(j);
//		w[1] = 0.3*(k);
//		w[2] = 0.3*(l);

				w[3] = 0.3 * (j); //torque space
				w[4] = 0.3 * (k);
				w[5] = 0.3 * (l);
				//cout << w << endl;

				flag = 1;
				for (int i = 0;i < pnum;i++) {
					double ans3 =  dot(w - f_max * phi_[i], (iGUG_[i] *( w - f_max * phi_[i])));

					if (ans3 >= dp) {
						flag = -1;
						break;
					}
				}
				if (flag == 1) {
					for (int n = 0; n < 6; n++) exfs.push_back(w(n));
					//cout << w;
				}
			}
	}
//  calcConvexHull2(6, exfs, exfs_out, wrench, false);
	displayConvexHull(6, exfs, false);
	cout << "test2 " << endl;

//  exit(0);

#endif

//  if(flag > 0) return ming;

	return ming;

//  int bit2[4]={1,2,4,8};
	for (int i = 0;i < pnum;i++) {
		if (iflag[i] == true) continue;
		ming = 1000.0;
		/*
		      for(int j=0;j<pnum-1;j++){
			  if( (i & j) != 0){
				  cout << "overlap" << endl;
				  continue;
			  }
		*/
		for (int k = 0;k < points;k++) {
			VectorXd Npt (Np[i]);
			MatrixXd  Upt (Up[i]);
			dp = (double)points - 1.0;
			MatrixXd Gt (G);
			for (int l = 0; l < 3; l++) {
				for (int m = 0; m < 6; m++) {
					Gt(m, 3*k + l) = 0;
				}
			}
			/*
				  for(int k=0;k<points;k++){
					  if( j & bit2[k]){
						  dp -= 1.0;
						for(int l=0; l<3; l++){
							for(int m=0; m<6; m++){
								Gt(m, 3*k+l) = 0;
							}
						}
					  }
				  }
				  if(dp < 2.1) continue;
			*/
			phi = (Gt* Npt);
			UG = (inverse(Upt)* trans(Gt));
			MatrixXd GUG(Gt* UG);
			iGUG = inverse(GUG);

//	  cout << "det " << det(GUG)<< endl;

//	  cout << "Phi"<<phi << endl; //
//	  cout << GUG<< endl; //
//	  cout << Gt<< endl; //

//    calcPseudoInverse(GUG,iGUG,1.0e-10);

			double ans3 =  inner_prod(wrench - f_max * phi, (iGUG*( wrench - f_max * phi)));
			//cout << "a,"<<dp - ans3 << endl;

			if (ans3 >= dp) {
//		  double temp = -EvaluateEllipsePointDistance(iGUG,(wrench-f_max*phi),dp);
//		  cout <<"d2,"  << temp << f_max*phi<<endl;
//		  if(ming < temp || ming > 0) ming = temp;
//		  flag=-1;
			} else {
				iflag[i] = true;
				double temp = EvaluateEllipsePointDistance(iGUG, (wrench - f_max * phi), dp);
//		  cout <<"d2,"  << temp << f_max*phi<< endl;
				if ( (temp) <= 0) {
					temp = 1000;
					//flag = -1;
				}
				if (temp < ming) {
					ming = temp;
				}
			}
		}
		if (iflag[i] == true) {
//	      cout << i << " b "<< ming;
			distance[i] = ming;
		}
	}

	ming = distance[0];
	for (int i = 0;i < pnum;i++) {
		if (ming > distance[i]) ming = distance[i];
//	  cout << distance[i] << " ";
	}


//  if(flag < 0) return flag;
	if (ming == 1000) return -1000;
	//cout << " aaa " <<ming;


	//return 1.0;
	return ming;

}

double ForceClosureTest::NormalForceClosureTest(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, int cface, double f_max) {

	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++)
			G(j, j + 3*i) = 1.0;

//      Matrix3 tm = sqew(Vector3(Pc[i]-objCoM));
		Matrix3 tm = sqew(Vector3(Pc[i])); // Confirm  objCoM

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				G(j + 3, k + 3*i) = GAMMA * tm(j, k);
	}

	MatrixXd V = MatrixXd::Zero(3 * points, cface * points);

	double mx = mu / ::sqrt(1 + mu * mu), mz = 1 / ::sqrt(1 + mu * mu);

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}
		
#define m_pi    (3.141596)

		for (int j = 0; j < cface; j++) {
			Vector3 vi( mz * ni + mx * ( sin(2.0* m_pi * j / (double)cface) * tx + cos(2.0 * m_pi * j / (double)cface) * ty));

			for (int k = 0; k < 3; k++) V(3*i + k, cface*i + j) = vi(k);
		}
	}

	MatrixXd GV (G* V);

	//Candidate of 3 points contact
	vector<double> exfs, exfs_out;

	double temp = f_max * sqrt(1.0 + mu * mu);

	if (points > 6) { //For the case of points>6
		int cfacep = 1;
		for (int i = 0;i < points;i++) {
			cfacep *= cface + 1;
		}
		for (int i = 0;i < cfacep;i++) {
			VectorXd lambda = VectorXd::Zero(cface * points);
			int p = 1;
			for (int j = 0;j < points;j++) {
				int l = i / p;
				int k = l % (cface + 1);
				if (k != cface)	   lambda(k + j*cface) = temp;
				p *= (cface + 1);
			}
			VectorXd w = (GV* lambda) - wrench;

			for (int n = 0; n < 6; n++)
				exfs.push_back(w(n));
		}
	}

	if (points == 6) { //For the case of points=6
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++)
						for (int o = 0; o < cface + 1; o++)
							for (int p = 0; p < cface + 1; p++) {

								VectorXd lambda = VectorXd::Zero(cface * points);

								if (j != cface)	   lambda(j)         = temp;
								if (k != cface)	   lambda(k + cface)   = temp;
								if (l != cface)	   lambda(l + 2*cface) = temp;
								if (m != cface)	   lambda(m + 3*cface) = temp;
								if (o != cface)	   lambda(o + 4*cface) = temp;
								if (p != cface)	   lambda(p + 5*cface) = temp;

								VectorXd w = (GV*lambda) - wrench;

								for (int n = 0; n < 6; n++)
									exfs.push_back(w(n));
							}
	}
	if (points == 5) { //For the case of points=5
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++)
						for (int o = 0; o < cface + 1; o++) {

							VectorXd lambda = VectorXd::Zero(cface * points);

							if (j != cface)	   lambda(j)         = temp;
							if (k != cface)	   lambda(k + cface)   = temp;
							if (l != cface)	   lambda(l + 2*cface) = temp;
							if (m != cface)	   lambda(m + 3*cface) = temp;
							if (o != cface)	   lambda(o + 4*cface) = temp;

							VectorXd w = (GV*lambda) - wrench;

							for (int n = 0; n < 6; n++)
								exfs.push_back(w(n));
						}
	}
	if (points == 4) { //For the case of points=4
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++) {

						VectorXd lambda = VectorXd::Zero(cface * points);

						if (j != cface)	   lambda(j)         = temp;
						if (k != cface)	   lambda(k + cface)   = temp;
						if (l != cface)	   lambda(l + 2*cface) = temp;
						if (m != cface)	   lambda(m + 3*cface) = temp;

						VectorXd w = (GV*lambda) - wrench;

						for (int n = 0; n < 6; n++)
							exfs.push_back(w(n));
					}
	} else if (points == 3) {
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++) {

					VectorXd lambda = VectorXd::Zero(cface * points);

					if (j != cface)   lambda(j)         = f_max * sqrt(1.0 + mu * mu);
					if (k != cface)   lambda(k + cface)   = f_max * sqrt(1.0 + mu * mu);
					if (l != cface)   lambda(l + 2*cface) = f_max * sqrt(1.0 + mu * mu);

					VectorXd w = (GV*lambda) - wrench;

					for (int n = 0; n < 6; n++)
						exfs.push_back(w(n));
					//cout << w << endl;
				}
	}
	if (points < 3) {
		cout << " Not supported number of contacts: " << points << endl;
//		if (fingtipGrasp())
//			return -100.0;
//		else
			return -100.0;
	}


	for (int m = 0; m < 6; m++) exfs.push_back(wrench[m]);

	double ret = 1.0e10;

/*	exfs.clear();
	for(int i=0;i<64;i++){
		if(i%2==0) exfs.push_back(-1);
		else exfs.push_back(1);
		if(i%4 < 2) exfs.push_back(-1);
		else exfs.push_back(1);
		if(i%8 < 4) exfs.push_back(-1);
		else exfs.push_back(1);
		if(i%16 < 8) exfs.push_back(-1);
		else exfs.push_back(1);
		if(i%32 < 16) exfs.push_back(-1);
		else exfs.push_back(1);
		if(i < 32) exfs.push_back(-1);
		else exfs.push_back(1);
	}
*/
	//calcConvexHull(6, exfs, exfs_out, false);
	ret = ConvexAnalysis::calcConvexHull2(6, exfs, exfs_out, wrench, false);
	/*
	  for(int i=0; i<(int)exfs_out.size(); i++){
	    w(i%6) = exfs_out[i]-wrench(i%6);
	    if(i%6==5){

	//	    cout << w ;

	      double tt = norm_2(w);
	//	  cout << w << endl;
	      if(tt < ret)
		  ret = tt;
	    }
	  }
	*/

#if 0 //3d distance 
	for (int i = 0;i < (int)exfs_out.size() / 18;i++) {
		double a[3][6] ;
		double w2[6];
		for (int j = 0;j < 18;j++) {
			a[j/6][j%6] = exfs_out[i*18+j];
		}
		for (int j = 0;j < 6;j++) {
			w2[j] = wrench[j] - a[0][j];
			a[2][j] -= a[0][j];
			a[1][j] -= a[0][j];
			//a[0][j] = 0;
		}
		double aa = 0, ab = 0, bb = 0, wa = 0, wb = 0;
		for (int j = 0;j < 6;j++) {
			aa += a[1][j] * a[1][j];
			bb += a[2][j] * a[2][j];
			ab += a[1][j] * a[2][j];
			wa += w2[j] * a[1][j];
			wb += w2[j] * a[2][j];
		}
		double adbc = aa * bb - ab * ab;
		double l = (bb * wa - ab * wb) / adbc;
		double m = (-ab * wa + aa * wb) / adbc;
//	  if(l < 0 || l > 1 || m < 0 || m >1) continue;
		/*	  cout << "l " << l << "m " << m << endl;
			  for(int j=0;j<18;j++){
				  cout << exfs_out[i*18+j] << " ";
			  }
			  cout << endl;
			  for(int j=0;j<6;j++){
				  cout << wrench[j] << " ";
			  }
			  cout << endl;
		*/


		VectorXd w3 = VectorXd(6);
		for (int j = 0;j < 6;j++) {
			w3[j] = w2[j] - l * a[1][j] - m * a[2][j];
		}
		double tt = norm_2(w3);
//	  cout << tt << endl;
		if (tt < ret)  ret = tt;
	}
#endif

// cout << "fc" << ret << endl;

#ifdef VIEW_CONVEX
	if (ret >= 0)
//      displayConvexHull(6, exfs_out, false);
		ConvexAnalysis::outputConvexHull(6, exfs, false);
#endif
	return ret;

}
/*
bool ForceClosureTest::NormalFormClosureTest(Vector3* Pc, Vector3* Nc, int points, vector<double>& spanVectors) {

	vector<double> unitTwist(points*7);

	for(int i=0; i<points; i++){

		for(int j=0; j<3; j++){
			unitTwist[i*points+j]   = Nc[i](j);
			unitTwist[i*points+j+3] = cross(Pc[i], Nc[i])(j);
		} 

		unitTwist[i*points+6] = 0.0;
	}

	ConvexAnalysis::calcHalfSpace(unitTwist, spanVectors, 6);

	if(spanVectors.size()==0)
	              return true;
	else
	              return false;

}
*/
double ForceClosureTest::normalForceClosureTestSoftFinger(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, int cface, int cface2, double f_max, const vector<double>& en,  Vector3 center) {

	MatrixXd G = MatrixXd::Zero(6, 6 * points);
	//Vector3 center(0.0, 0.0, 0.0);
//	double GAMMA0 = 5.0;

	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j,  6*i + j) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate
		Matrix3 tm2 = diag(-1.0*Pc[i][0]/Pc[i].norm(), -1.0*Pc[i][1]/Pc[i].norm(), -1.0*Pc[i][2]/Pc[i].norm());

		for (int j = 0; j < 3; j++){
			for (int k = 0; k < 3; k++){
				G(j + 3, 6*i + k) = GAMMA * tm(j, k); 
				G(j , 6*i + k + 3) = 0;
				G(j + 3, 6*i + j + 3) = GAMMA *1; 
			}
		}
	}
	
	int numface= cface2*cface*2 + cface+2;

	MatrixXd V = MatrixXd::Zero(6 * points, numface*points);

	//double mx = mu / ::sqrt(1 + mu * mu), mz = 1 / ::sqrt(1 + mu * mu);
	//double mx_mid = mu / ::sqrt(4 + mu * mu), mz_mid = 2 / ::sqrt(4 + mu * mu);
	
	int cnt = 0;

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}
		
#define m_pi    (3.141596)

		for (int j = 0; j < cface; j++) {
			Vector3 vi( ni +  mu*( sin(2.0* m_pi * j / (double)cface) * tx + cos(2.0 * m_pi * j / (double)cface) * ty));
			for (int k = 0; k < 3; k++){
				V(6*i + k,cnt) = vi(k); 
				V(6*i + k +3, cnt) = 0;
			}
			cnt++;
		}
		for(int l=0;l< cface2;l++){
			double div = (1.0+(double)l)/(double)(cface2+1.0);
			for (int j = 0; j < cface; j++) {
				Vector3 vi_mid( ni + sqrt(div)*mu * ( sin(2.0* m_pi * j / (double)cface) * tx + cos(2.0 * m_pi * j / (double)cface) * ty));
				for (int k = 0; k < 3; k++){
					V(6*i + k, cnt) = vi_mid(k); 
					V(6*i + k + 3, cnt) = mu*en[i]*ni[k]*sqrt(1.0-div); 
				}
				cnt++;
				for (int k = 0; k < 3; k++){
					V(6*i + k, cnt) = vi_mid(k); 
					V(6*i + k + 3, cnt) = -mu*en[i]*ni[k]*sqrt(1.0-div); 
				}
				cnt++;
			}
		}
		for (int k = 0; k < 3; k++){
			V(6*i + k, cnt) = ni[k];
			V(6*i + k + 3, cnt) = (mu*en[i]*ni[k]);
		}
		cnt++;
		for (int k = 0; k < 3; k++){
			V(6*i + k, cnt) = ni[k];
			V(6*i + k + 3, cnt) = -(mu*en[i]*ni[k]);
		}
		cnt++;
	}
	//cout  << G << endl;
	//cout  << V << endl;

	MatrixXd GV (G* V);

	vector<double> exfs, exfs_out;

	double temp = f_max * sqrt(1.0 + mu * mu);



	if (points > 6) { //For the case of points>6
		int cfacep = 1;
		for (int i = 0;i < points;i++) {
			cfacep *= cface + 1;
		}
		for (int i = 0;i < cfacep;i++) {
			VectorXd lambda = VectorXd(cface * points);
			int p = 1;
			for (int j = 0;j < points;j++) {
				int l = i / p;
				int k = l % (cface + 1);
				if (k != cface)	   lambda(k + j*cface) = temp;
				p *= (cface + 1);
			}
			VectorXd w = (GV* lambda) - wrench;

			for (int n = 0; n < 6; n++)
				exfs.push_back(w(n));
		}
	}

	if (points == 6) { //For the case of points=6
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++)
						for (int o = 0; o < cface + 1; o++)
							for (int p = 0; p < cface + 1; p++) {

								VectorXd lambda = VectorXd::Zero(cface * points);

								if (j != cface)	   lambda(j)         = temp;
								if (k != cface)	   lambda(k + cface)   = temp;
								if (l != cface)	   lambda(l + 2*cface) = temp;
								if (m != cface)	   lambda(m + 3*cface) = temp;
								if (o != cface)	   lambda(o + 4*cface) = temp;
								if (p != cface)	   lambda(p + 5*cface) = temp;

								VectorXd w = (GV*lambda) - wrench;

								for (int n = 0; n < 6; n++)
									exfs.push_back(w(n));
							}
	}
	if (points == 5) { //For the case of points=5
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++)
						for (int o = 0; o < cface + 1; o++) {

							VectorXd lambda = VectorXd::Zero(cface * points);

							if (j != cface)	   lambda(j)         = temp;
							if (k != cface)	   lambda(k + cface)   = temp;

							if (l != cface)	   lambda(l + 2*cface) = temp;
							if (m != cface)	   lambda(m + 3*cface) = temp;
							if (o != cface)	   lambda(o + 4*cface) = temp;

							VectorXd w = (GV*lambda) - wrench;

							for (int n = 0; n < 6; n++)
								exfs.push_back(w(n));
						}
	}
	if (points == 4) { //For the case of points=4
		for (int j = 0; j < cface + 1; j++)
			for (int k = 0; k < cface + 1; k++)
				for (int l = 0; l < cface + 1; l++)
					for (int m = 0; m < cface + 1; m++) {

						VectorXd lambda = VectorXd::Zero(cface * points);

						if (j != cface)	   lambda(j)         = temp;
						if (k != cface)	   lambda(k + cface)   = temp;
						if (l != cface)	   lambda(l + 2*cface) = temp;
						if (m != cface)	   lambda(m + 3*cface) = temp;

						VectorXd w = (GV*lambda) - wrench;

						for (int n = 0; n < 6; n++)
							exfs.push_back(w(n));
					}
	} else if (points == 3) {
		for (int j = 0; j < numface + 1; j++)
			for (int k = 0; k < numface + 1; k++)
				for (int l = 0; l < numface + 1; l++) {

					VectorXd lambda = VectorXd::Zero(numface*points);

					if(j != numface) lambda(j) = f_max;
					if(k != numface) lambda(k+numface) = f_max;
					if(l != numface) lambda(l+2*numface) = f_max;
					//cout << lambda.transpose() << endl;

					VectorXd w = (GV*lambda) - wrench;

					for (int n = 0; n < 6; n++)
						exfs.push_back(w(n));
					//cout << w << endl;
				}
	} else if (points == 2) {
		for(int j=0 ; j<numface+1 ; j++){
			for(int k=0 ; k<numface+1; k++){

				VectorXd lambda = VectorXd((numface)*points);
				lambda.setZero();
				
				if(j != numface) lambda(j) = f_max;
				if(k != numface) lambda(k+numface) = f_max;
				
				//cout << lambda.transpose() << endl;

				VectorXd w = (GV*lambda)-wrench;

				for (int n=0 ; n<6 ; n++)
					exfs.push_back(w(n));
			}
		}
	}


	VectorXd w0 = VectorXd(6);
	w0 << wrench[0],wrench[1],wrench[2],wrench[3],wrench[4],wrench[5];
	//w0.setZero();
	for (int m = 0; m < 6; m++) exfs.push_back(w0(m));
	
/*	vector<double>small_exfs ,small_exfs_out;
	for (int m = 0; m < 6; m++) small_exfs.push_back(w0(m));
	for(int i=0;i< numface*points;i++){
		for(int j=0;j<6;j++){
			small_exfs.push_back(GV(j,i));
		}
	}
	if( ConvexAnalysis::calcConvexHull2(6, small_exfs ,small_exfs_out, w0, false) == 0){
		return 0;
	}		
*/
	double ret = 1.0e10;

	//calcConvexHull(6, exfs, exfs_out, false);
	ret = ConvexAnalysis::calcConvexHull2(6, exfs, exfs_out, w0, false);
	cout << "ret:" << ret << endl;


#ifdef VIEW_CONVEX
	if (ret >= 0)
//      displayConvexHull(6, exfs_out, false);
		ConvexAnalysis::outputConvexHull(6, exfs, false);
#endif
	return ret;

}

void ForceClosureTest::forceClosureTestOnly(ColdetModelPtr c) { 	//Should modify for object read
	vector<double> vertex;
	vector<int>coord;

	int nVerticies = c->getNumVertices();
	int nTriangles = c->getNumTriangles();

	float tx, ty, tz;
	for(int i=0;i<nVerticies;i++){
		c->getVertex(i, tx, ty, tz);
		vertex.push_back(tx);
		vertex.push_back(ty);
		vertex.push_back(tz);
	}
	int t1, t2, t3;
	for(int i=0;i<nTriangles;i++){
		c->getTriangle(i, t1, t2, t3);
		coord.push_back(t1);
		coord.push_back(t2);
		coord.push_back(t3);
		coord.push_back(-1);
	}

	int cnum = (coord.end() - coord.begin()) / 4; //need to be traignlated;
	cout << cnum << endl;
	
	GAMMA = 200.0;

//	vector<double>::iterator I=vertex.begin()+100*3;

	int cf = 3;
	VectorXd wrench = VectorXd::Zero(6);
	vector <Vector3> objPos(cf), objN(cf);

	stringstream pos_file;
	pos_file <<  "forceclosureresult_"  << time(0) -1407305996 << ".csv";
	//string time_file = "timerecord.csv";
	ofstream fout_pos;
	ofstream fout_time;
	fout_pos.open ( pos_file.str().c_str() );
	
	fout_pos << "GAMMA," << GAMMA << endl;
	fout_pos << "Contact Point," << cf << endl;
	
	//fout_time.open ( time_file.c_str() );
	_initrand();
	double start0 = getrusage_sec();
	for(int j=0;j<1000;j++){
		vector<double>en;

		for (int i = 0;i < cf;i++) {
			int cn = (int)(_drand() * (cnum - 1));
			//cout << cn << " ";

			vector<int>::iterator I2 = coord.begin() + cn * 4;
			vector<double>::iterator I = vertex.begin() + (*I2) * 3;


			Vector3 Po((*I), (*(I + 1)), (*(I + 2)));
			Vector3 n1((*I), (*(I + 1)), (*(I + 2)));
			I2 += 1;
			I = vertex.begin() + (*I2) * 3;
			Vector3 n2((*I), (*(I + 1)), (*(I + 2)));
			I2 += 1;
			I = vertex.begin() + (*I2) * 3;
			Vector3 n3((*I), (*(I + 1)), (*(I + 2)));

			objPos[i] = Po;
			objN[i] = cross(Vector3(n2 - n1), Vector3(n3 - n1));
			objN[i] = objN[i] / norm2(objN[i]);
			
			en.push_back(0.005);

//			cout << "objPos[" << i << "] = Vector3(" << objPos[i][0] << "," << objPos[i][1] << "," << objPos[i][2] << ");" << endl;
//			cout << "objN[" << i << "] = Vector3(" << objN[i][0] << "," << objN[i][1] << "," << objN[i][2] << ");" << endl;
		}

		double efc = 0;
		double pfc = 0, pfc2 = 0;
#ifdef COMMENT
		//efc = forceClosureTestEllipsoid(wrench, &objPos[0], &objN[0], cf, 0.5, 5.0);
		efc = forceClosureTestEllipsoidSoftFinger(wrench, &objPos[0], &objN[0], cf, 0.5, 5.0,en); 

		double end1 = getrusage_sec();
		if (efc > 100 || efc < 0) efc = 0;

//	pfc2 = SimpleForceClosureTest(wrench, objPos, objN, cf, 0.5, 2.0, 4.0, 5.0);
//	if(pfc2 > 100 || pfc2<0) pfc2=0;

//	if(efc > 1){

//	pfc = ForceClosureTest(wrench, objPos, objN, cf, 0.5, 12, 5.0);
		double start2 = getrusage_sec();

		pfc = normalForceClosureTestSoftFinger(wrench, &objPos[0], &objN[0], cf, 0.5, 12, 1, 5.0,en);
		pfc2 = normalForceClosureTestSoftFinger(wrench, &objPos[0], &objN[0], cf, 0.5, 4, 1, 5.0,en);
#endif
		pfc2 = NormalForceClosureTest(wrench, &objPos[0], &objN[0], cf, 0.5, 4, 5.0);
		continue;

		double end2 = getrusage_sec();
//	}

//	if(pfc > 100) pfc=0;
//	if(efc > 0 && pfc==0) pfc = ForceClosureTest(wrench, objPos, objN, cf, 0.5, 12, 5.0);
		if (pfc > 100 || pfc < 0) pfc = -1;
		if (pfc2 > 100 || pfc2 < 0) pfc2 = -1;

//	if(pfc > 0)  pfc2 = ForceClosureTestFmin(wrench, objPos, objN, cf, 0.5, 6, 5.0, 5.0*(1-1.0/sqrt(2)) );
//	if(pfc > 0)  pfc2 = ForceClosureTest(wrench, objPos, objN, cf, 0.5, 4, 5.0);
		// 1/sqrt(2)


//	fout_pos << qtmp << " ";
//	cout << efc <<","<<pfc<<","<<pfc2<< endl;
//	fout_pos << efc <<","<<pfc<<","<<pfc2<< endl;

		cout << "result " << j << ","<< pfc << "," << efc << "," << pfc2 << endl;
		fout_pos << pfc << "," << efc << "," << pfc2 << endl;
//	fout_time << (end1-start1)/1000.0 <<"," <<(end2-start2) << endl;
		cout << j << "," << (getrusage_sec() - start0)  << endl;


//	if(fabs(efc -pfc) > 1.0 && pfc >=0 ) exit(0);


//	exit(0);
//	if(efc > 1.0e-3 && pfc < 0) exit(0);

	}
	cout  << "time ," << (getrusage_sec() - start0)  << endl;
	fout_pos.close();
	cout << "\a";
}



double ForceClosureTest::forceClosureTestEllipsoidSubspace(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, double f_max, Vector3 center) {
	//thhd1 : thhd1*fmax : radius of ellipsoid (0<thhd1<1)
	//   1 <= thhd2 <= points
	// Pc[]:  the contact position when the object center is zero


	MatrixXd G = MatrixXd::Zero(6, 3 * points);

	//Vector3 center(0.0, 0.0, 0.0);
	//center = center / (double)points;


	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j, j + 3*i) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
				G(j + 3, k + 3*i) = GAMMA * tm(j, k); ///
			}
	}

	double esize = 0.5;

	Matrix3 Si( diag(1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(mu*f_max) / (1.0 - esize), 1.0 / dbl(f_max) / esize));
	Matrix3 Si2( diag(1.0 / dbl(f_max), 1.0 / dbl(f_max), 1.0 / dbl(f_max / 2.0)));

	double n1 = 1.0;
	double n2 = 0.5;

	MatrixXd U = MatrixXd::Zero(3 * points, 3 * points);
	MatrixXd U2 = MatrixXd::Zero(3 * points, 3 * points);
	VectorXd N(3*points);

	int pnum = 1;
	int bnum = 1;
	int pnum3 = 1;


	for (int i = 0;i < points;i++) {
		pnum *= 2;
	}

	vector <MatrixXd>  Up (pnum);  
	vector <VectorXd>  Np (pnum*2);  

	for (int i = 0;i < pnum;i++) {
		Up[i] = MatrixXd::Zero(3 * points, 3 * points);
		Np[i] = VectorXd(3 * points);
	}

	for (int i = 0; i < points; i++) {

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}

		Matrix3 Ui(v3(tx, ty, ni));

		Matrix3 USU(Ui*Si*trans(Ui));
		Matrix3 US2U(Ui*Si2*trans(Ui));

		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = USU(j, k);
					}
					Np[l](3*i + j) = ni(j) * n1; // normal
				}
			} else {
				for (int j = 0; j < 3; j++) {
					for (int k = 0; k < 3; k++) {
						Up[l](3*i + j, 3*i + k) = US2U(j, k);
					}
					Np[l](3*i + j) = ni(j) * n2;
				}
			}
		}
		bnum *= 2;


	}

	VectorXd phi = G * N;
	MatrixXd UG = inverse(U) * trans(G);
	MatrixXd iGUG = inverse(G * UG);

	int flag = 1;
	double ming = 1000.0;


	vector <bool> iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector <double>  distance(pnum);

	double dp = (double)points;

	for (int i = 0;i < pnum;i++) {
		phi = G * Np[i];

		Vector3 gf = f_max*phi.head(3);
		Vector3 gtau = f_max*phi.tail(3);
		Vector3 wrenchf = wrench.head(3);

		UG = inverse(Up[i]) * trans(G);
		MatrixXd GUG(G * UG);
		calcPseudoInverse(GUG, iGUG, 1.0e-10);

		Matrix3 G11,G12,G21,G22;
		G11 = iGUG.topLeftCorner(3,3);
		G12 = iGUG.topRightCorner(3,3);
		G21 = iGUG.bottomLeftCorner(3,3);
		G22 = iGUG.bottomRightCorner(3,3);

		Vector3 b = -1 * (G12 + trans(G21)) * gtau;

		VectorXd point = VectorXd::Zero(6);
		point.block(0,0,3,0) << (wrenchf-gf + 0.5 * inverse(G11) * b);
		
		MatrixXd ellipse = MatrixXd::Zero(6,6);
		ellipse.block(0,0,3,3) << G11;

		double radius = -1 * inner_prod(gtau, G22 * gtau) + 0.25 * (inner_prod(b,inverse(G11) * b)) + dp;

		if(radius < 0){
			ming = -1000;
			continue;
		}

		double ans3 = inner_prod(point,G11*point);

		if(ans3 >= radius){
			double temp = -  min( EvaluateEllipsePointDistance(ellipse, point, radius), EvaluateEllipsePointDistance2(ellipse, point, radius));
			distance[i] = temp;
			if (ming < temp || ming > 0) ming = temp;
			flag = -1;
		} else {
			iflag[i] = true;
			double temp = min( EvaluateEllipsePointDistance(ellipse, point, radius), EvaluateEllipsePointDistance2(ellipse, point, radius));
			distance[i] = temp;
			if ( (temp) <= 0) {
				temp = 1000;
			}
			if (temp < ming) {
				ming = temp;
			}
		}

	}
	cout << ming << endl;
	
	if(!isOutputForceSpace && !isOutputTorqueSpace ){
		return ming;
	}

#ifdef VIEW_CONVEX


	vector <MatrixXd> iGUG_(pnum);
	vector <VectorXd> phi_(pnum);

	for (int i = 0;i < pnum;i++) {
		phi_[i] = G * Np[i];
#ifdef VIEW_FORCE_SPACE
		phi_[i][3] = phi_[i][4] = phi_[i][5] = 0;
#else
		phi_[i][0] = phi_[i][1] = phi_[i][2] = 0;  //torque space
#endif

		UG = inverse(Up[i])*trans(G);
		MatrixXd GUG(G* UG);

		for (int j = 0;j < 6;j++) for (int k = 0;k < 6;k++) {
#ifdef VIEW_FORCE_SPACE
				if (j > 2 || k > 2)  GUG(j, k) = (j == k) ? 1 : 0;
#else
				if (j < 3 || k < 3)  GUG(j, k) = (j == k) ? 1 : 0; //torque space
#endif
			}
		calcPseudoInverse(GUG, iGUG, 1.0e-10);
		iGUG_[i] = MatrixXd (iGUG);
	}


	vector<double> exfs, exfs_out;
	VectorXd w(6);
	w[0] =  w[1]  = w[2] = w[3] = w[4] = w[5] = 0;
	for (int j = -100;j < 100;j++) {
		if (j % 10 == 0) cout << j << endl;

		for (int k = -100;k < 100;k++) for (int l = -100;l < 100;l++) {

#ifdef VIEW_FORCE_SPACE
				w[0] = 0.3 * (j);
				w[1] = 0.3 * (k);
				w[2] = 0.3 * (l);
#else
				w[3] = (j); //torque space
				w[4] = (k);
				w[5] = (l);
#endif

				flag = 1;
				for (int i = 0;i < pnum;i++) {
					double ans3 =  inner_prod(w - f_max * phi_[i], iGUG_[i]*(w - f_max * phi_[i]));

					if (ans3 >= dp) {
						flag = -1;
						break;
					}
				}
				if (flag == 1) {
					for (int n = 0; n < 6; n++) exfs.push_back(w(n));
				}
			}
	}
//  calcConvexHull2(6, exfs, exfs_out, wrench, false);
	ConvexAnalysis::outputConvexHull(6, exfs, false);
	cout << "test" << endl;

#endif


}


double ForceClosureTest::forceClosureTestEllipsoidSoftFingerSubspace(VectorXd &wrench, Vector3* Pc, Vector3* Nc, int points, double mu, double f_max, const vector<double>& en, Vector3 center) {

	//thhd1 : thhd1*fmax : radius of ellipsoid (0<thhd1<1)
	//   1 <= thhd2 <= points
	// Pc[]:  the contact position when the object center is zero

	double GAMMA0 = 5.0;

	MatrixXd G = MatrixXd::Zero(6, 4 * points);

	//Vector3 center(0.0, 0.0, 0.0);
	//center = center / (double)points;
	
	for (int i = 0; i < points; i++) {
		for (int j = 0; j < 3; j++) G(j,  4*i + j) = 1.0;

		Matrix3 tm = sqew(Vector3(Pc[i] - center)); //Confirm ObjCom cordinate

		for (int j = 0; j < 3; j++){
			for (int k = 0; k < 3; k++)
				G(j + 3, 4*i + k) = GAMMA0 * tm(j, k); ///
			G(j + 3, 4*i + 3) = GAMMA0 * Nc[i](j);
		}
	}

	double esize = 0.5;
	double n1 = 1.0;
	double n2 = 0.5;
	int pnum = 1;
	int bnum = 1;

	for (int i = 0;i < points;i++)
		pnum *= 2;

	vector <MatrixXd>  Up (pnum);
	vector <VectorXd>  Np (pnum);  

	for (int i = 0;i < pnum;i++) {
		Up[i]  = MatrixXd::Zero(4 * points, 4 * points);
		Np[i]  = VectorXd::Zero(4 * points);
	}

	MatrixXd Si  = MatrixXd::Zero(4,4);
	MatrixXd Si2 = MatrixXd::Zero(4,4);

	for (int i = 0; i < points; i++) {

		Si(0,0) = 1.0 / dbl(mu*f_max) / (1.0 - esize);
		Si(1,1) = 1.0 / dbl(mu*f_max) / (1.0 - esize);
		Si(2,2) = 1.0 / dbl(f_max) / esize;
		Si(3,3) = 1.0 / dbl(f_max*en[i]);
		Si2(0,0) = 1.0 / dbl(f_max);
		Si2(1,1) = 1.0 / dbl(f_max);
		Si2(2,2) = 1.0 / dbl(f_max / 2.0);
		Si2(3,3) = 1.0 / dbl(5.0*f_max*en[i]);

		Vector3 tx(1.0, 0.0, 0.0), ty(0.0, 1.0, 0.0), ni(Nc[i]);

		if (fabs(ni(0)) <= 0.5) {

			Vector3 tmp( cross( cross(ni, tx), ni) );
			tx = tmp / norm2(tmp);

			ty = cross(ni, tx);
			ty = ty / norm2(ty);

		} else {

			Vector3 tmp( cross( cross(ni, ty), ni) );
			ty = tmp / norm2(tmp);

			tx = cross(ty, ni);
			tx = tx / norm2(tx);
		}
		
		MatrixXd Ui = MatrixXd::Identity(4,4);
		Matrix3 Utmp = v3(tx, ty, ni);
		for(int j=0; j<3; j++)
			for(int k=0; k<3; k++)
				Ui(j,k) = Utmp(j,k);

		MatrixXd Ut1 = Si *trans(Ui);
		MatrixXd USU = Ui * Ut1;

		MatrixXd Ut2 = Si2*trans(Ui);
		MatrixXd US2U = Ui* Ut2;

		for (int l = 0;l < pnum;l++) {
			if (l % (bnum*2) < bnum) {
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						Up[l](4*i + j, 4*i + k) = USU(j, k);
					}
				}
				for(int j=0; j<3; j++)
					Np[l](4*i + j) = ni(j) * n1;
			} else {
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						Up[l](4*i + j, 4*i + k) = US2U(j, k);
					}
				}
				for (int j = 0; j < 3; j++)
					Np[l](4*i + j) = ni(j) * n2;
			}
		}
		bnum *= 2;


	}
	
	int flag = 1;
	double ming = 1000.0;

	vector <bool> iflag(pnum);
	for (int i = 0;i < pnum;i++) iflag[i] = false;

	vector <double>  distance(pnum);

	double dp = (double)points;

	for (int i = 0;i < pnum;i++) {

		VectorXd phi = G * Np[i];

		Vector3 gf = f_max*phi.head(3);
		Vector3 gtau = f_max*phi.tail(3);
		Vector3 wrenchf = wrench.head(3);

		MatrixXd UG = (inverse(Up[i]) * trans(G));
		MatrixXd GUG  (G* UG), iGUG;
		calcPseudoInverse(GUG, iGUG, 1.0e-10);

		Matrix3 G11,G12,G21,G22;
		G11 = iGUG.topLeftCorner(3,3);
		G12 = iGUG.topRightCorner(3,3);
		G21 = iGUG.bottomLeftCorner(3,3);
		G22 = iGUG.bottomRightCorner(3,3);

		Vector3 b = -1 * (G12 + trans(G21)) * gtau;

		VectorXd point = VectorXd::Zero(6);
		point.block(0,0,3,0) << (wrenchf-gf + 0.5 * inverse(G11) * b);
		
		MatrixXd ellipse = MatrixXd::Zero(6,6);
		ellipse.block(0,0,3,3) << G11;

		double radius = -1 * inner_prod(gtau, G22 * gtau) + 0.25 * (inner_prod(b,inverse(G11) * b)) + dp;
		
		if(radius < 0){
			ming = -1000;
			continue;
		}

		double ans3 = inner_prod(point,G11*point);

		if(ans3 >= radius){
			double temp = -  min( EvaluateEllipsePointDistance(ellipse, point, radius), EvaluateEllipsePointDistance2(ellipse, point, radius));
			distance[i] = temp;
			if (ming < temp || ming > 0) ming = temp;
			flag = -1;
		} else {
			iflag[i] = true;
			double temp = min( EvaluateEllipsePointDistance(ellipse, point, radius), EvaluateEllipsePointDistance2(ellipse, point, radius));
			distance[i] = temp;
			if ( (temp) <= 0) {
				temp = 1000;
			}
			if (temp < ming) {
				ming = temp;
			}
		}
	}
	cout << ming << endl;
	
	return ming;
}



double ForceClosureTest::EvaluateEllipsePointDistance(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec, eigen);

	if(norm2(point) == 0){
		MatrixXd evec2 = MatrixXd::Zero(6,6);
		VectorXd eigen2(6);
		calcEigenVectors(ellipse/radius,evec2,eigen2);
		return sqrt(1/(eigen2.maxCoeff()));
	}

	VectorXd normal(6);

	normal = VectorXd ( evec* point);


	//cout <<" a "   << dot(point , normal ) << " " ;

	double a = inner_prod( VectorXd(normal), VectorXd ( ellipse* normal ) );
	double b = inner_prod( VectorXd( point) , VectorXd ( ( ellipse + trans(ellipse)* normal) ) );
	double c = inner_prod(point , VectorXd ( ellipse* point ) ) - radius;

	double D = (b * b - 4.0 * a * c);
	if (D > 0) {
		D = sqrt(D);
		double x1 = (-b + D) / (2.0 * a);
		double x2 = (-b - D) / (2.0 * a);
		double x =  min( fabs(x1) , fabs(x2) );
		return norm_2(normal)*x;
	} else {
		double ans = inner_prod(point, ellipse*point);
		return norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
	}
}



double ForceClosureTest::EvaluateEllipsePointDistance2(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	MatrixXd evec2 = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec2, eigen);

	if(norm2(point) == 0){
		MatrixXd evec3 = MatrixXd::Zero(6,6);
		VectorXd eigen2(6);
		calcEigenVectors(ellipse/radius,evec3,eigen2);
		return sqrt(1/(eigen2.maxCoeff()));
	}

	MatrixXd swap = MatrixXd::Zero(6, 6);
	int eipos[6] = {0, 1, 2, 3, 4, 5};

	for (int i = 0;i < 5;i++) {
		for (int j = 0;j < 5 - i;j++) {
			if (eigen[j] < eigen[j+1]) {
				double temp = eigen[j+1];
				eigen[j+1] = eigen[j];
				eigen[j] = temp;
				int itemp = eipos[j+1];
				eipos[j+1] = eipos[j];
				eipos[j] = itemp;
			}
		}
	}
	for (int i = 0;i < 6;i++) {
		swap(eipos[i], i) = 1.0;
	}
	evec = evec2* swap;

	//cout <<"eigen " <<eigen << endl;

	VectorXd normal(6);

	double xa[4];
	double xz[3];


	double a1, a2, a3, a4, a5, a6;
	a1 = radius / eigen[0];
	a2 = radius / eigen[1];
	a3 = radius / eigen[2];
	a4 = radius / eigen[3];
	a5 = radius / eigen[4];
	a6 = radius / eigen[5];

	VectorXd p2 ( trans(evec) * point);
//	double r2 = dot_(point , prod( ellipse, point) ) ;

	double b1, b2, b3, b4, b5, b6;
	b1 = p2(0) * p2(0);
	b2 = p2(1) * p2(1);
	b3 = p2(2) * p2(2);
	b4 = p2(3) * p2(3);
	b5 = p2(4) * p2(4);
	b6 = p2(5) * p2(5);

//	cout << b1/a1 + b2/a2 +b3/a3 + b4/a4 + b5/a5+ b6/a6 << endl;

	double offset = 1.0 - b4 / a4 - b5 / a5 - b6 / a6;

	xa[3] = offset;
	xa[2] = offset * (a1 + a2 + a3) - (b1 + b2 + b3);
	xa[1] = offset * (a1 * a2 + a2 * a3 + a3 * a1) - (b1 * (a2 + a3) + b2 * (a3 + a1) + b3 * (a1 + a2));
	xa[0] = offset * (a1 * a2 * a3) - (b1 * a2 * a3 + b2 * a3 * a1 + b3 * a1 * a2);

	sol3(xa, xz);
	//cout << " "<< xz[0] << " "<< xz[1] << " "<< xz[2]<< endl;
//	exit(0);

	double maxa = xz[0];
	for (int i = 0;i < 3;i++) {
		if ( (xz[i] != 0) && (fabs(xz[i]) < fabs(maxa)) ) {
			maxa = xz[i];
		}
	}

	MatrixXd eigenrad2 = MatrixXd::Zero(6, 6);
	for (int i = 0;i < 6;i++) {
		eigenrad2(i, i) = 1.0 / (radius / eigen[i] + maxa );
//		eigenrad2(i,i)= 1.0/ (1.0/eigen[i] );
	}
	VectorXd temp ( trans (evec) * point );
	VectorXd temp2 ( eigenrad2 * temp );
	normal = VectorXd ( evec *  temp2);


	//cout << offset << " "<< maxa << " "<<eigenrad2(0,0)*b1 + eigenrad2(1,1)*b2 + eigenrad2(2,2)*b3 + b4/a4 + b5/a5 + b6/a6

	//cout <<" a "   << dot(point , normal ) << " " ;
	//cout <<"er2" << eigenrad2 << endl;
	//cout <<"evec" << evec << endl;

	//cout <<"e2" << prod(evec, MatrixXd ( prod(eigenrad2,trans(evec) ) ) ) << endl;

	double a = inner_prod(normal, (ellipse* normal));
	double b = inner_prod(point, ( (ellipse + trans(ellipse))* normal));
	double c = inner_prod(point , ( ellipse* point) ) - radius;

	double D = (b * b - 4.0 * a * c);
	if (D > 0) {
		D = sqrt(D);
		double x1 = (-b + D) / (2.0 * a);
		double x2 = (-b - D) / (2.0 * a);

		double x =  min( fabs(x1) , fabs(x2) );

		//cout << prod ( trans(evec),point)  <<endl << eigen << endl;
		//cout << evec << endl;
		//cout << VectorXd (normal*x)<<endl ;
		//cout << "point,"<<point << endl;

		//cout <<norm_2(normal)*x << normal*x << endl;
		//exit(0);

		return norm_2(normal)*x;
	} else {
		double ans = inner_prod(point, ellipse* point);
		return norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
	}

}

double ForceClosureTest::EvaluateEllipsePointDistance3(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	MatrixXd evec2 = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec2, eigen);

	MatrixXd swap = MatrixXd::Zero(6, 6);
	int eipos[6] = {0, 1, 2, 3, 4, 5};

	for (int i = 0;i < 5;i++) {
		for (int j = 0;j < 5 - i;j++) {
			if (eigen[j] < eigen[j+1]) {
				double temp = eigen[j+1];
				eigen[j+1] = eigen[j];
				eigen[j] = temp;
				int itemp = eipos[j+1];
				eipos[j+1] = eipos[j];
				eipos[j] = itemp;
			}
		}
	}
	for (int i = 0;i < 6;i++) {
		swap(eipos[i], i) = 1.0;
	}
	evec = evec2 * swap;

	VectorXd normal(6);

	double xa[4];
	double xz[3];


	double a1, a2, a3, a4, a5, a6;
	a1 = radius / eigen[0];
	a2 = radius / eigen[1];
	a3 = radius / eigen[2];
	a4 = radius / eigen[3];
	a5 = radius / eigen[4];
	a6 = radius / eigen[5];

	VectorXd p2 (trans(evec)*point);

	double b1, b2, b3, b4, b5, b6;
	b1 = p2(0) * p2(0);
	b2 = p2(1) * p2(1);
	b3 = p2(2) * p2(2);
	b4 = p2(3) * p2(3);
	b5 = p2(4) * p2(4);
	b6 = p2(5) * p2(5);
	
	
	double offset2=0;
	double maxa=-1.0e10;	
	cout <<"convergence"<< maxa << endl;
	for(int j=0;j<10;j++){

		double offset = 1.0 - b4 / (a4+offset2) - b5 / (a5+offset2) - b6 / (a6+offset2);

		xa[3] = offset;
		xa[2] = offset * (a1 + a2 + a3) - (b1 + b2 + b3);
		xa[1] = offset * (a1 * a2 + a2 * a3 + a3 * a1) - (b1 * (a2 + a3) + b2 * (a3 + a1) + b3 * (a1 + a2));
		xa[0] = offset * (a1 * a2 * a3) - (b1 * a2 * a3 + b2 * a3 * a1 + b3 * a1 * a2);

		sol3(xa, xz);

		maxa = xz[0];
		for (int i = 0;i < 3;i++) {
			if ( (xz[i] != 0) && (fabs(xz[i]) < fabs(maxa)) ) {
				maxa = xz[i];
			}
		}
		offset2 = maxa;
		
		cout <<"convergence"<< maxa << endl;
	}

	MatrixXd eigenrad2 = MatrixXd::Zero(6, 6);
	for (int i = 0;i < 6;i++) {
		eigenrad2(i, i) = 1.0 / (radius / eigen[i] + maxa );
	}
	VectorXd temp ( trans (evec) * point );
	VectorXd temp2 ( eigenrad2* temp  );
	normal = VectorXd ( evec *  temp2);



	double a = inner_prod(normal, (ellipse* normal));
	double b = inner_prod(point, (ellipse + trans(ellipse))* normal);
	double c = inner_prod(point , ( ellipse * point) ) - radius;

	double D = (b * b - 4.0 * a * c);
	if (D > 0) {
		D = sqrt(D);
		double x1 = (-b + D) / (2.0 * a);
		double x2 = (-b - D) / (2.0 * a);

		double x =  min( fabs(x1) , fabs(x2) );

		return norm_2(normal)*x;
	} else {
		double ans = inner_prod(point, (ellipse * point));
		return norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
	}

}



double ForceClosureTest::EvaluateEllipsePointDistanceAxisDirection(MatrixXd ellipse, VectorXd point, double radius) {

	MatrixXd evec = MatrixXd::Zero(6, 6);
	VectorXd eigen(6);
	calcEigenVectors (ellipse, evec, eigen);
	
	MatrixXd evec2 = MatrixXd::Zero(6,6);
	VectorXd eigen2(6);
	calcEigenVectors(ellipse/radius,evec2,eigen2);
	double mini = sqrt(1/(eigen2.maxCoeff()));
	
	for(int i=0;i<6;i++){

		VectorXd normal ( evec.col(i) );

		double a = inner_prod( VectorXd(normal), VectorXd ( ellipse* normal ) );
		double b = inner_prod( VectorXd( point) , VectorXd ( ( ellipse + trans(ellipse)* normal) ) );
		double c = inner_prod(point , VectorXd ( ellipse* point ) ) - radius;

		double D = (b * b - 4.0 * a * c);
		if (D > 0) {
			D = sqrt(D);
			double x1 = (-b + D) / (2.0 * a);
			double x2 = (-b - D) / (2.0 * a);
			double x =  min( fabs(x1) , fabs(x2) );
			double temp = norm_2(normal)*x;
			if(temp < mini) mini= temp;
		} else {
			double ans = inner_prod(point, ellipse*point);
			double temp = norm_2(point)*(sqrt(ans) - sqrt(radius)) / sqrt(ans);
			if(temp < mini) mini= temp;
		}
	}
	return mini;
}

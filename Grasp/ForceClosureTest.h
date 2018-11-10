/**
   c) Tokuo Tsuji (Kyushu univ./AIST) and Kensuke Harada (AIST)
*/

#ifndef _ForceClosureTest_H
#define _ForceClosureTest_H

#include <cnoid/EigenTypes>
#include <cnoid/ColdetModel>

#include "ConvexAnalysis.h"

#include <algorithm>
#include <time.h>
#ifndef WIN32
#include <sys/resource.h>
#endif

#include "exportdef.h"


namespace grasp{

double getrusage_sec();
double _drand();
void _initrand();
		
class EXCADE_API ForceClosureTest
{
	public:
		ForceClosureTest() {
			 GAMMA = 5.0;
			 isOutputForceSpace=false;
			 isOutputTorqueSpace=false;
		}
		~ForceClosureTest(){}
			
		void SetGAMMA(double GAMMA){
			this->GAMMA= GAMMA;
		}

	static ForceClosureTest* instance(ForceClosureTest *fct=NULL) {
		static ForceClosureTest* instance = (fct) ? fct : new ForceClosureTest();
		return instance;
	}
			

	double forceClosureTestEllipsoid(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max, cnoid::Vector3 center = cnoid::Vector3(0.0,0.0,0.0));
	double forceClosureTestEllipsoidSoftFinger(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max, const std::vector<double>& en, cnoid::Vector3 center = cnoid::Vector3(0.0,0.0,0.0));
	double forceClosureTestEllipsoidInternal(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max);
	double NormalForceClosureTest(cnoid::VectorXd& wrench, cnoid::Vector3 cpos[], cnoid::Vector3 Nc[], int points, double mu, int cface, double f_max);
	double normalForceClosureTestSoftFinger(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, int cface,  int cface2, double f_max, const std::vector<double>& en, cnoid::Vector3 center = cnoid::Vector3(0.0,0.0,0.0));
	//bool NormalFormClosureTest(cnoid::Vector3* Pc, cnoid::Vector3* Nc, int points, std::vector<double>& spanVectors);
	double ForceClosureTestManipulationForce(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double gamma,cnoid::MatrixXd jacobi[]);
	
	void forceClosureTestOnly(cnoid::ColdetModelPtr c);
	
	double forceClosureTestEllipsoidSubspace(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max, cnoid::Vector3 center = cnoid::Vector3(0.0,0.0,0.0));
	double forceClosureTestEllipsoidSoftFingerSubspace(cnoid::VectorXd &wrench, cnoid::Vector3 Pc[], cnoid::Vector3 Nc[], int points, double mu, double f_max, const std::vector<double>& en, cnoid::Vector3 center = cnoid::Vector3(0.0,0.0,0.0));

	protected: 


	double EvaluateEllipsePointDistance(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius);
	double EvaluateEllipsePointDistance2(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius);
	double EvaluateEllipsePointDistance3(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius);
	double EvaluateEllipsePointDistanceAxisDirection(cnoid::MatrixXd ellipse, cnoid::VectorXd point, double radius) ;
	
	// GAMMA is scale value for moment
	double GAMMA;
	bool isOutputForceSpace;
	bool isOutputTorqueSpace;

			
};

}

#endif

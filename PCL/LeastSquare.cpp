#include "LeastSquare.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

LeastSquare::LeastSquare(){
}

LeastSquare::LeastSquare(const vector<double>& x, const vector<double>& y){

	double xa = average(x);
	double ya = average(y);

	Sxx = 0.0;
	for(size_t i=0; i<x.size(); i++)
		Sxx += (x[i] - xa)*(x[i] - xa);
	Sxx = sqrt( Sxx / (double)x.size());

	Syy = 0.0;
	for(size_t i=0; i<y.size(); i++)
		Syy += (y[i] - ya)*(y[i] - ya);
	Syy = sqrt(Syy / (double)y.size());

	Sxy = 0.0;
	for(size_t i=0; i<x.size(); i++)
		Sxy += (x[i] - xa)*(y[i] - ya);
	Sxy = Sxy / (double)x.size();

	a = Sxy/(Sxx*Sxx);
	b = ya - a*xa;

}

LeastSquare::LeastSquare(const vector<double>& x, const vector<double>& y, const vector<double>& z){

	double Axx=0.0, Ayy=0.0, Axy=0.0, Ayz=0.0, Azx=0.0, Ax=0.0, Ay=0.0, Az=0.0;
	for(size_t i=0; i<x.size(); i++){
		Axx += x[i]*x[i];
		Ayy += y[i]*y[i];
		Axy += x[i]*y[i];
		Ayz += y[i]*z[i];
		Azx += z[i]*x[i];
		Ax += x[i];
		Ay += y[i];
		Az += z[i];
	}

	Matrix3 R;
	R << Axx, Axy, Ax,
  		 Axy, Ayy, Ay,
		 Ax, Ay, x.size();
	Vector3 p = Vector3(Azx, Ayz, Az);

	Vector3 u = inverse(R)*p;

	a = u(0);
	b = u(1);
	c = u(2);

	double za = average(z);
	Szz = 0.0;
	for(size_t i=0; i<z.size(); i++)
		Szz += (z[i] - za)*(z[i] - za);
	Szz = sqrt(Szz / (double)z.size());
}


#ifndef _LEASTSQUARE_H
#define _LEASTSQUARE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include "../Grasp/VectorMath.h"

class LeastSquare
{

public :
	LeastSquare();
	LeastSquare(const std::vector<double>& x, const std::vector<double>& y);
	LeastSquare(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);
	double a, b, c, Sxx, Syy, Szz, Sxy, Syz, Szx;

protected:


};

#endif

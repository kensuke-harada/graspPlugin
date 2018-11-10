#ifndef _TOUR_
#define _TOUR_

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "tsp.h"
#include "tspCity.h"

#include "exportdef.h"


#define INFTY 10e6

class Tour{
 public:
  Tour(vector<vector<double> > tours, double *table[], int dim, double **ltour, int* index);
  ~Tour(void);

 private:
  City* first;
  City* current;
  TSP tsp;
  double **ltour_;
  int *index_;

  Real mincost;
  int insertion_point;
  void insert(Vectornd);
  void init(Vectornd p);
  Real cost(void);
  void print(void);
};

#endif

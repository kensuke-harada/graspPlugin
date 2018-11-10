#ifndef _TSP_
#define _TSP_

#include "tspVectornd.h"

#include "exportdef.h"


class TSP{
 public:
  TSP(void);
  TSP(vector<vector<double> > tours, double *table[], int dim);
  ~TSP(void);

  int size;
  Vectornd* pos;
};

#endif

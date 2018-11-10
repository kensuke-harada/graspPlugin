
#include "tsp.h"

TSP::TSP(void){
  ;
}

TSP::TSP(vector<vector<double> > tours, double *table[], int dim){

  int i,j;
  size=tours.size();

  //pos = new Vectornd[size](table, dim);
  pos = new Vectornd[size];
  for (i = 0; i < size; i++){
	pos[i].pt.resize(dim);
	pos[i].vdim=dim;
	pos[i].local_table=table;
  }
  for (i = 0; i < size; i++){
    for(j=0;j<dim;j++) pos[i].pt[j] = tours[i][j];
    pos[i].index = i ;
  }
}

TSP::~TSP(void){
  ;
}



#include "tspTour.h"

Tour::Tour(vector<vector<double> > tours, double *table[], int dim, double **ltour, int *index){
  first = NULL;
  current = NULL;

  tsp = TSP(tours, table, dim);

  insertion_point = 0;
  int i = 0;

  if (tsp.size > i){
    init(tsp.pos[i]);
    i++;
    while(i < tsp.size){
      mincost = INFTY;
      insert(tsp.pos[i]);
      i++;
    }
  }
  ltour_=ltour;
  index_=index;
  print();
  
}

Tour::~Tour(void){
  ;
}

void Tour::init(Vectornd p){
  // set first city into tour
  first = new City();
  first->pos = p;
  first->next = new City();
  first->next->pos = p;
}

void Tour::insert(Vectornd p){
  // insert new city into tour
  current = first;
  Real ins_cost;
  insertion_point = 0;
  int i = 0;
  while(current->next != NULL){
    ins_cost = current->insertion_cost(p);
    if (ins_cost < mincost){
      insertion_point = i;
      mincost = ins_cost;
    }
    i++;
    current = current->next;
  }
  current = first;
  i = 0;
  while(i != insertion_point){
    i++;
    current = current->next;
  }
  City* newcity = new City();
  newcity->next = current->next;
  newcity->pos = p;
  current->next = newcity;
}

Real Tour::cost(void){
  // compute cost of tour
  Real cost = 0;
  current = first;
  while(current->next != NULL){
    cost += current->pos.dist(current->next->pos);
  }
  return(cost);
}

void Tour::print(void){
  // print tour

  if (first == NULL){
    return;
  }

  //printf("%d ", first->pos.index);
  //first->print();
  
  int j=0;
  index_[j]=first->pos.index;

  for(int i=0; i<first->pos.vdim; i++)  {
    ltour_[j][i]=first->pos.pt[i];
  }

  current = first;
  while(current->next != NULL){
    current = current->next;
    //printf("%d ", current->pos.index);
    //current->print();
    j++;
    index_[j]=current->pos.index;

    for(int i=0; i<current->pos.vdim; i++)  {
      ltour_[j][i]=current->pos.pt[i];
    }
  }
}


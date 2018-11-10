
#include "tspCity.h"

City::City(void){
  next = NULL;
  pos = nullvec;
}

City::~City(void){
  ;
}

void City::print(void){
  pos.print();
}

Real City::insertion_cost(Vectornd p){
  if (next == NULL){
    cout << "error: no insertion after last city!";
    return(0);
  }
  Real cost = 0;
  cost += pos.dist(p);
  cost += next->pos.dist(p);
  cost -= pos.dist(next->pos);
  return(cost);
}

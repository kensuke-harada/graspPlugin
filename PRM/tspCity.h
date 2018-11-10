#ifndef _CITY_
#define _CITY_

#include "tspVectornd.h"

#include "exportdef.h"


class City{
 public:
  City(void);
  ~City(void);

  City* next;
  Vectornd pos;
  void print(void);
  Real insertion_cost(Vectornd p);
};

#endif

#ifndef FMRAND_H
#define FMRAND_H

#include <stdlib.h>
#include <time.h>

#include "exportdef.h"

inline double mpk_drand()
{
#ifdef WIN32
  return ((double)rand())/((double)RAND_MAX);
#else
  return drand48();
#endif
}

inline long int mpk_lrand()
{
#ifdef WIN32
  return rand();
#else
  return lrand48();
#endif
}

inline void mpk_initrand(unsigned seed=0)
{
#ifdef WIN32
  if(seed)
    srand(seed);
  else
    srand((unsigned)time(NULL));
#else
  if (seed)
    srand48(seed);
  else
    srand48(time(0));
#endif
}

#endif

#ifndef SBL_RANDVAL_H
#define SBL_RANDVAL_H

#include <sys/types.h>
#include <time.h>

#include "exportdef.h"


#define CACHE_SIZE	1024		/* must be > KK_DOUBLE */
#define	KK_RD		100 

/**@memo Generates uniform random numbers in [0, 1).
*/

class sblRandVal {

public:

  ///
  sblRandVal();

  ///
  sblRandVal(long seed);

  ///@memo Returns a pointer to {\bf num} random values, each in [0,1).
  const double* get(int num);

protected:

  double cache[CACHE_SIZE];		/* cache of random numbers */ 
  double *cacheEnd;
  double *current;			
  double ran_val_x[KK_RD];     	/* the generator state     */

  void ranValStart(long seed);
  void ranValArray(double *aa, int n);
};


inline sblRandVal::sblRandVal() 
{
  current = cacheEnd = cache + CACHE_SIZE;
  long key = 0;//time(NULL);
  ranValStart( key );
}

inline sblRandVal::sblRandVal(long seed)
{  
  current = cacheEnd = cache + CACHE_SIZE;
  ranValStart(seed);
}

/* get
 *   return a pointer to an array of  random doubles
 *
 *   num: number of random doubles requested
 */

inline const double* sblRandVal::get(int num)
{
  double* rd = current;
  if ( current+num >= cacheEnd ) {
    ranValArray(cache, CACHE_SIZE);
    rd = current = cache;
  }
  current += num;
  return rd;
}

#undef CACHE_SIZE

#endif

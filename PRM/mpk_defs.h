#ifndef MPK_VALUES_H
#define MPK_VALUES_H

#ifdef WIN32
#define DBL_MAX 1.7976931348623158e+308
#define FLT_MAX 3.40282347e+38F
#else
#include <values.h>
#include <stdlib.h>
#endif

#define MAX_UNSIGNED ((unsigned)0xffffffff)

#include "exportdef.h"


typedef short mpk_idx;  // index type

inline void mpk_exit(int code)
{
#ifdef WIN32
  getchar();
#endif
  exit(code);
}

#endif

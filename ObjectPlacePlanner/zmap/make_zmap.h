#ifndef _MAKE_ZMAP_H_
#define _MAKE_ZMAP_H_

// constants
#include "/usr/local/VVV/include/vvvstd.h"
#include "/usr/local/VVV/include/range.h"
#include "/usr/local/VVV/include/brep.h"
#include "/usr/local/VVV/include/matutil.h"

#define DEF_Z_TH	(-50.0)
#define DEF_DTH		(5.0) 
#define DEF_PITCH	(1.0) 

#define REGPROP_EIGEN_OK (1)
#define REGPROP_EIGEN_NG (0)

#define NSTR	(256)

#define CAP_MIN	(25.0)
#define CAP_MAX	(40.0)

#define SHOLDER_RAD	(32.5)
#define DIFF_Z	(30.0)

struct seg_args {
  DOUBLE64	lshort;
  DOUBLE64	lext;
  DOUBLE64	deltalen;
  DOUBLE64	width0;
  DOUBLE64	width1;
  INT32		mergespeed;
};
typedef struct seg_args	Seg_Args;

typedef struct _opt_make_zmap_ {
  double	z_th; // height of frame
  double	dth; // depth edge thresh
  Seg_Args  	segarg;
  double	pitch; // pitch size of XY grid
} OptMakeZmap;

typedef struct _zmap_ {
  int	mapsize_x, mapsize_y; // mapsize 
  double	offset_x, offset_y; // location of (0,0)
  double	pitch; // x and y pitch
  double	*z; /* (x,y,z)=
		       (offset_x + pitch * ix,
		       offset_y + pitch * iy,
		       z[mapsize_x * iy + ix])
		       0 <= ix < mapsize_x
		       0 <= iy < mapsize_y
		     */
  double	Z_NoData;
  double	Z_OutRect;
  double	dz; // thresh for labeling
} ZMAP;

typedef struct _rectangle_ {
  double	xmin, xmax, ymin, ymax;
  double	corner[4][2];
  double	center[2];
  double	dir[2][2];
  double	umax, umin, vmax, vmin;
  
} RectAngle;

typedef struct _reg_prop_ {
  double	x2, xy, y2;
  double	x, y, z, n;
  double	g[3];
  int	stat;
  double	v[2][2];
  double	max[2];
  double	min[2];
} RegProp;

typedef struct _cap_list_ {
  int	n;
  int	*d;
} CapList;

// zmap_io.c
int
output_zmap(char	*fout,
	    OptMakeZmap	*opt_make_zmap, //
	    RectAngle	*rect,  //
	    ZMAP	*zmap,
	    int	nlabel,
	    RegProp	*regprop,
	    CapList	*caplist
	    );

int
input_zmap(char	*fin,
	   OptMakeZmap	*opt_make_zmap, //
	   RectAngle	*rect,  //
	   ZMAP	*zmap,
	   int	*nlabel,
	   RegProp	**regprop,
	   CapList	*caplist
	   );

// make_zmap.c
void
filter_by_z(RANGE	*rng,
	    double	z_th);

ZMAP
create_map_data(RANGE	*rng,
		RANGE	*rng2, // データ参照用
		BREP	*brp,
		short	*rtob,
		RectAngle	*rect,
		OptMakeZmap	*opt_make_zmap,
		VERR	*err);

int
comp_zmap(const int	a,
	  const int	b,
	  const void	*vzmap);

CapList
find_cap(EPBM	*label,
	 int	nlabel,
	 ZMAP	*zmap,
	 double	capmin,
	 double	capmax,
	 RegProp	*regprop,
	 VERR	*err);

void
add_bottle(ZMAP	*zmap,
	   int	capid,
	   RegProp	*regprop,
	   EPBM	*label,
	   double	sholder_rad,
	   double	diff_z);

#endif

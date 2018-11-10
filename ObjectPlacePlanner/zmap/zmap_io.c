/*
  output_zmap()
  input_zmap()
 */

/* return code
   0 = OK
   -1 = NG
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "make_zmap.h"

int
output_zmap(char	*fout,
	    OptMakeZmap	*opt_make_zmap, //
	    RectAngle	*rect,  //
	    ZMAP	*zmap,
	    int	nlabel,
	    RegProp	*regprop,
	    CapList	*caplist
	    )
{
  FILE	*fp, *fp2, *fp3;
  int	ic, id, idir, i;
  int	x, y;

  if(fout != NULL){
    fp = fopen(fout, "w");
    if(fp == NULL){
      fprintf(stderr, "File Open Error in output_zmap(): fname=\"%s\"\n",
	      fout);
      return -1;
    }
  }else{
    fp = stdout;
  }

  char filename2[] = "extplugin/graspPlugin/ObjectPlacePlanner/data/data_cap.mat";
  fp2 = fopen(filename2,"w");

  char filename3[] = "extplugin/graspPlugin/ObjectPlacePlanner/data/data_box.mat";
  fp3 = fopen(filename3,"w");


#if 0
  // opt_make_zmap
  fprintf(fp, "opt_make_zmap=%g %g %g\n",
	  opt_make_zmap->z_th,
	  opt_make_zmap->dth,
	  opt_make_zmap->pitch);
  fprintf(fp, "opt_make_zmap.segarg=%g %g %g %g %g %ld\n",
	  opt_make_zmap->segarg.lshort,
	  opt_make_zmap->segarg.lext,
	  opt_make_zmap->segarg.deltalen,
	  opt_make_zmap->segarg.width0,
	  opt_make_zmap->segarg.width1,
	  opt_make_zmap->segarg.mergespeed);

  // rect
  fprintf(fp, "rectangle.[xy][min|max]=%g %g %g %g\n",
	  rect->xmin,
	  rect->xmax,
	  rect->ymin,
	  rect->ymax);

  fprintf(fp, "rectangle.corner=");
  for(ic = 0; ic < 4; ic++){
    for(id = 0; id < 2; id++){
      fprintf(fp, "%g ", rect->corner[ic][id]);
    }
  }
  fprintf(fp, "\n");
#endif
  /*Box corner */
  for(ic = 0; ic < 4; ic++){

    double minvalue = 100000.0;
    int x_min=0, y_min=0;
    for(y = 0; y < zmap->mapsize_y; y++){
      for(x = 0; x < zmap->mapsize_x; x++){
	double value = (x+zmap->offset_x - rect->corner[ic][0])*(x+zmap->offset_x - rect->corner[ic][0])
	  + (y+zmap->offset_y - rect->corner[ic][1])*(y+zmap->offset_y - rect->corner[ic][1]);
	if( minvalue> value){
	  minvalue = value;
	  x_min = x;
	  y_min = y;
	}
      }
    }
    fprintf(fp3, "%lg %lg %lg\n",
		x_min+zmap->offset_x,
		y_min+zmap->offset_y,
		zmap->z[y_min*zmap->mapsize_x+x_min]);
  }
  /*******/
#if 0
  fprintf(fp, "rectangle.center=");
  for(id = 0; id < 2; id++){
    fprintf(fp, "%g ", rect->center[id]);
  }
  fprintf(fp, "\n");

  fprintf(fp, "rectangle.dir=");
  for(idir = 0; idir < 2; idir++){
    for(id = 0; id < 2; id++){
      fprintf(fp, "%g ", rect->dir[idir][id]);
    }
  }
  fprintf(fp, "\n");

  fprintf(fp, "rectangle.[uv][min|max]=%g %g %g %g\n",
	  rect->umin,
	  rect->umax,
	  rect->vmin,
	  rect->vmax);

  // zmap

  fprintf(fp, "zmap.mapsize_[xy]=%d %d\n",
	  zmap->mapsize_x,
	  zmap->mapsize_y);
  fprintf(fp, "zmap.offset_[xy]=%g %g\n",
	  zmap->offset_x,
	  zmap->offset_y);
  fprintf(fp, "zmap.pitch=%g\n",
	  zmap->pitch);
  for(y = 0; y < zmap->mapsize_y; y++){
    for(x = 0; x < zmap->mapsize_x; x++){
      fprintf(fp, "%lg %lg %lg\n",
	      x+zmap->offset_x,
	      y+zmap->offset_y,
	      zmap->z[y*zmap->mapsize_x+x]);
    }
    fprintf(fp, "\n");
  }
  /*
  for(i = 0; i < zmap->mapsize_x*zmap->mapsize_y; i++){
    fprintf(fp, "%g\n", zmap->z[i]);
  }
  */
  fprintf(fp, "zmap.Z_[NoData|OutRect]=%g %g\n",
	  zmap->Z_NoData,
	  zmap->Z_OutRect);
  fprintf(fp, "zmap.dz=%g\n",
	  zmap->dz);

  // regprop
  fprintf(fp, "nlabel=%d\n", nlabel);
  for(i = 0; i < nlabel; i++){
    fprintf(fp, "regprop[%d].g[0-1]=%g %g\n",
	    i,
	    regprop[i].g[0],
	    regprop[i].g[1]);
    fprintf(fp, "regprop[%d].v[0-1][0-1]=%g %g %g %g\n",
	    i,
	    regprop[i].v[0][0],
	    regprop[i].v[0][1],
	    regprop[i].v[1][0],
	    regprop[i].v[1][1]);
    fprintf(fp, "regprop[%d].[min|max][0-1]=%g %g %g %g\n",
	    i,
	    regprop[i].min[0],
	    regprop[i].max[0],
	    regprop[i].min[1],
	    regprop[i].max[1]);
  }

  // caplist
  fprintf(fp, "caplist.n=%d\n", caplist->n);
  for(i = 0; i < caplist->n; i++){
    fprintf(fp, "caplist.d[%d]=%d\n", i, caplist->d[i]);
  }
#endif
  if(fp != stdout){
    fclose(fp);
  }

  fprintf(fp2, "#\n");
  for(i=0; i< caplist->n; i++){
	  fprintf(fp2,"# Candidate - %d\n", i);
	  fprintf(fp2, "4 4\n");
	  fprintf(fp2, "1.0  0.0  0.0  %g\n", regprop[caplist->d[i]].g[0]);
	  fprintf(fp2, "0.0  1.0  0.0  %g\n", regprop[caplist->d[i]].g[1]);
	  fprintf(fp2, "0.0  0.0  1.0  %g\n", regprop[caplist->d[i]].g[2]);
	  fprintf(fp2, "0.0  0.0  0.0  1.0\n");
	  fprintf(fp2, "\n");
	  /* fprintf(fp3, "%g %g %g\n", regprop[caplist->d[i]].g[0], regprop[caplist->d[i]].g[1], regprop[caplist->d[i]].g[2]);*/
  }

  return 0;
}

static void
fetch_string(FILE	*fp,
	     char	*str)
{
  int	i, len, ieq;
  char	str0[NSTR];

  fgets(str0, NSTR, fp);

  len = strlen(str0);
  ieq = -1;
  for(i = 0; i < len && ieq == -1; i++){
    if(str0[i] == '='){
      ieq = i;
    }
  }

  if(ieq == -1){
    str[0] = '\0';
    return;
  }

  for(i = 0; i < len-(ieq+1); i++){
    str[i] = str0[i+ieq+1];
  }

  return;
}


static void
escape_func(char	*form,
	    int	valint,
	    char	*str,
	    FILE	*fp,
	    double	*zmap_z,
	    RegProp	*regprop,
	    int	*caplist_d)
{
  fprintf(stderr, form, valint, str);
  if(fp != stdin){
    fclose(fp);
  }
  if(zmap_z != NULL) free(zmap_z);
  if(regprop !=NULL) free(regprop);
  if(caplist_d != NULL) free(caplist_d);
  return;
}

int
input_zmap(char	*fin,
	   OptMakeZmap	*opt_make_zmap, //
	   RectAngle	*rect,  //
	   ZMAP	*zmap,
	   int	*nlabel,
	   RegProp	**regprop,
	   CapList	*caplist
	   )
{
  FILE	*fp;
  char	str[NSTR];
  int	x, y;
  int	i;
  int	count;

  // 0 clear for lists to be malloced
  zmap->z = NULL;
  *regprop = NULL;
  caplist->d = NULL;

  if(fin != NULL){
    fp = fopen(fin, "r");
    if(fp == NULL){
      fprintf(stderr, "File Open Error in input_zmap(): fname=\"%s\"\n",
	      fin);
      return -1;
    }
  }else{
    fp = stdin;
  }

  // opt_make_zmap
  /*
  fprintf(fp, "opt_make_zmap=%g %g %g\n",
	  opt_make_zmap->z_th,
	  opt_make_zmap->dth,
	  opt_make_zmap->pitch);

   */
  fetch_string(fp, str);
  if(sscanf(str, "%lg %lg %lg",
	    &opt_make_zmap->z_th,
	    &opt_make_zmap->dth,
	    &opt_make_zmap->pitch) != 3){
    escape_func("Error in input_zmap(): num of numbers(opt_make_zmap) < %d\"%s\"\n",
		3, str, fp, NULL, NULL, NULL);
    return -1;
  }

  /*
  fprintf(fp, "opt_make_zmap.segarg=%g %g %g %g %g %ld\n",
	  opt_make_zmap->segarg.lshort,
	  opt_make_zmap->segarg.lext,
	  opt_make_zmap->segarg.deltalen,
	  opt_make_zmap->segarg.width0,
	  opt_make_zmap->segarg.width1,
	  opt_make_zmap->segarg.mergespeed);
   */
  fetch_string(fp, str);
  if(sscanf(str,
	    "%lg %lg %lg %lg %lg %ld",
	    &opt_make_zmap->segarg.lshort,
	    &opt_make_zmap->segarg.lext,
	    &opt_make_zmap->segarg.deltalen,
	    &opt_make_zmap->segarg.width0,
	    &opt_make_zmap->segarg.width1,
	    &opt_make_zmap->segarg.mergespeed) != 6){
    escape_func("Error in input_zmap(): num of numbers(opt_make_zmap.segarg) < %d\"%s\"\n",
		6, str, fp, NULL, NULL, NULL);
    return -1;
  }

  // rect
  /*
  fprintf(fp, "rectangle.[xy][min|max]=%g %g %g %g\n",
	  rect->xmin,
	  rect->xmax,
	  rect->ymin,
	  rect->ymax);
   */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg %lg %lg",
	    &rect->xmin,
	    &rect->xmax,
	    &rect->ymin,
	    &rect->ymax) != 4){
    escape_func("Error in input_zmap(): num of numbers(rectangle.[xy][min|max]) < %d\"%s\"\n",
		4, str, fp, NULL, NULL, NULL);
    return -1;
  }

  /*
  fprintf(fp, "rectangle.corner=");
  for(ic = 0; ic < 4; ic++){
    for(id = 0; id < 2; id++){
      fprintf(fp, "%g ", rect->corner[ic][id]);
    }
  }
  fprintf(fp, "\n");
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg %lg %lg %lg %lg %lg %lg",
	    &rect->corner[0][0],
	    &rect->corner[0][1],
	    &rect->corner[1][0],
	    &rect->corner[1][1],
	    &rect->corner[2][0],
	    &rect->corner[2][1],
	    &rect->corner[3][0],
	    &rect->corner[3][1]) != 8){
    escape_func("Error in input_zmap(): num of numbers(rectangle.corner) < %d\"%s\"\n",
		8, str, fp, NULL, NULL, NULL);
    return -1;
  }

  /*
  fprintf(fp, "rectangle.center=");
  for(id = 0; id < 2; id++){
    fprintf(fp, "%g ", rect->center[id]);
  }
  fprintf(fp, "\n");
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg",
	    &rect->center[0],
	    &rect->center[1]) != 2){
    escape_func("Error in input_zmap(): num of numbers(rectangle.center) < %d\"%s\"\n",
		2, str, fp, NULL, NULL, NULL);
    return -1;
  }

  /*
  fprintf(fp, "rectangle.dir=");
  for(idir = 0; idir < 2; idir++){
    for(id = 0; id < 2; id++){
      fprintf(fp, "%g ", rect->dir[idir][id]);
    }
  }
  fprintf(fp, "\n");
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg %lg %lg",
	    &rect->dir[0][0],
	    &rect->dir[0][1],
	    &rect->dir[1][0],
	    &rect->dir[1][1]) != 4){
    escape_func("Error in input_zmap(): num of numbers(rectangle.dir) < %d\"%s\"\n",
		4, str, fp, NULL, NULL, NULL);
    return -1;
  }

  /*
  fprintf(fp, "rectangle.[uv][min|max]=%g %g %g %g\n",
	  rect->umin,
	  rect->umax,
	  rect->vmin,
	  rect->vmax);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg %lg %lg",
	    &rect->umin,
	    &rect->umax,
	    &rect->vmin,
	    &rect->vmax) != 4){
    escape_func("Error in input_zmap(): num of numbers(rectangle.uv_minmax) < %d\"%s\"\n",
		4, str, fp, NULL, NULL, NULL);
    return -1;
  }

  // zmap

  /*
  fprintf(fp, "zmap.mapsize_[xy]=%d %d\n",
	  zmap->mapsize_x,
	  zmap->mapsize_y);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%d %d",
	    &zmap->mapsize_x,
	    &zmap->mapsize_y) != 2){
    escape_func("Error in input_zmap(): num of numbers(zmap.mapsize_[xy]) < %d\"%s\"\n",
		2, str, fp, NULL, NULL, NULL);
    return -1;
  }else if(zmap->mapsize_x <= 0){
    escape_func("Error in input_zmap(): (zmap.mapsize_x=%d) <= 0 \"%s\"\n",
		zmap->mapsize_x, str, fp, NULL, NULL, NULL);
    return -1;
  }else if(zmap->mapsize_y <= 0){
    escape_func("Error in input_zmap(): (zmap.mapsize_y=%d) <= 0 \"%s\"\n",
		zmap->mapsize_y, str, fp, NULL, NULL, NULL);
    return -1;
  }

  /*
  fprintf(fp, "zmap.offset_[xy]=%g %g\n",
	  zmap->offset_x,
	  zmap->offset_y);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg",
	    &zmap->offset_x,
	    &zmap->offset_y) != 2){
    escape_func("Error in input_zmap(): num of numbers(zmap.offset_[xy]) < %d\"%s\"\n",
		2, str, fp, NULL, NULL, NULL);
    return -1;
  }
  /*
  fprintf(fp, "zmap.pitch=%g\n",
	  zmap->pitch);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg",
	    &zmap->pitch) != 1){
    escape_func("Error in input_zmap(): num of numbers(zmap.pitch) < %d\"%s\"\n",
		1, str, fp, NULL, NULL, NULL);
    return -1;
  }

  if(zmap->mapsize_x * zmap->mapsize_y <= 0){
    fprintf(stderr, "Warning: no zmap, escape\n");
    if(fp != stdin){
      fclose(fp);
    }
    return -1;
  }

  zmap->z = (double *)calloc(zmap->mapsize_x * zmap->mapsize_y,
			     sizeof(double));
  if(zmap->z == NULL){
    fprintf(stderr, "Error in input_zmap(): malloc zmap->z\n");
    if(fp != stdin){
      fclose(fp);
    }
    return -1;
  }

  count = 0;
  for(y = 0; y < zmap->mapsize_y; y++){
    for(x = 0; x < zmap->mapsize_x; x++, count++){
      /*
      fprintf(fp, "%lg %lg %lg\n",
	      x+zmap->offset_x,
	      y+zmap->offset_y,
	      zmap->z[y*zmap->mapsize_x+x]);
      */
      fgets(str, NSTR, fp);
      if(sscanf(str,"%*g %*g %lg",
		&zmap->z[count]) != 1){
	escape_func("Error in input_zmap(): lost data(zmap.z[%d]) \"%s\"\n",
		    count, str, fp, zmap->z, NULL, NULL);
	return -1;
      }
    }
    /*
    fprintf(fp, "\n");
    */
    fgets(str, NSTR, fp);
  }
  /*
  fprintf(fp, "zmap.Z_[NoData|OutRect]=%g %g\n",
	  zmap->Z_NoData,
	  zmap->Z_OutRect);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg %lg",
	    &zmap->Z_NoData,
	    &zmap->Z_OutRect) != 2){
    escape_func("Error in input_zmap(): num of numbers(zmap.Z_[NoData|OutRect]) < %d\"%s\"\n",
		2, str, fp, zmap->z, NULL, NULL);
    return -1;
  }
  /*
  fprintf(fp, "zmap.dz=%g\n",
	  zmap->dz);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%lg",
	    &zmap->dz) != 1){
    escape_func("Error in input_zmap(): num of numbers(zmap.dz) < %d\"%s\"\n",
		1, str, fp, zmap->z, NULL, NULL);
    return -1;
  }

  // regprop
  /*
  fprintf(fp, "nlabel=%d\n", nlabel);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%d",
	    nlabel) != 1){
    escape_func("Error in input_zmap(): num of numbers(nlabel) < %d\"%s\"\n",
		1, str, fp, zmap->z, NULL, NULL);
    return -1;
  }else if(*nlabel <= 0){
    *regprop = NULL;
  }else{
    (*regprop) = (RegProp *)calloc(*nlabel, sizeof(RegProp));
    if(*regprop == NULL){
      fprintf(stderr, "Error in input_zmap(): malloc regprop\n");
      free(zmap->z);
      if(fp != stdin){
	fclose(fp);
      }
      return -1;
    }

    for(i = 0; i < *nlabel; i++){
      /*
      fprintf(fp, "regprop[%d].g[0-1]=%g %g\n",
	      i,
	      regprop[i].g[0],
	      regprop[i].g[1]);
      */
      fetch_string(fp, str);
      if(sscanf(str,"%lg %lg",
		&((*regprop)[i].g[0]),
		&((*regprop)[i].g[1])) != 2){
	escape_func("Error in input_zmap(): region data lost(regprop[%d].g[01])\"%s\"\n",
		    i, str, fp, zmap->z, *regprop, NULL);
	return -1;
      }
      /*
      fprintf(fp, "regprop[%d].v[0-1][0-1]=%g %g %g %g\n",
	      i,
	      regprop[i].v[0][0],
	      regprop[i].v[0][1],
	      regprop[i].v[1][0],
	      regprop[i].v[1][1]);
      */
      fetch_string(fp, str);
      if(sscanf(str,"%lg %lg %lg %lg",
		&((*regprop)[i].v[0][0]),
		&((*regprop)[i].v[0][1]),
		&((*regprop)[i].v[1][0]),
		&((*regprop)[i].v[1][1])) != 4){
	escape_func("Error in input_zmap(): region data lost(regprop[%d].v[0-3])\"%s\"\n",
		    i, str, fp, zmap->z, *regprop, NULL);
	return -1;
      }
      /*
      fprintf(fp, "regprop[%d].[min|max][0-1]=%g %g %g %g\n",
	      i,
	      regprop[i].min[0],
	      regprop[i].max[0],
	      regprop[i].min[1],
	      regprop[i].max[1]);
      */
      fetch_string(fp, str);
      if(sscanf(str,"%lg %lg %lg %lg",
		&((*regprop)[i].min[0]),
		&((*regprop)[i].max[0]),
		&((*regprop)[i].min[1]),
		&((*regprop)[i].max[1])) != 4){
	escape_func("Error in input_zmap(): region data lost(regprop[%d].(min|max)[01])\"%s\"\n",
		    i, str, fp, zmap->z, *regprop, NULL);
	return -1;
      }
    }
  }

  // caplist
  /*
  fprintf(fp, "caplist.n=%d\n", caplist->n);
  */
  fetch_string(fp, str);
  if(sscanf(str,"%d",
	    &caplist->n) != 1){
    escape_func("Error in input_zmap(): num of caplist->n < %d\"%s\"\n",
		1, str, fp, zmap->z, *regprop, NULL);
    return -1;
  }

  if(caplist->n <= 0){
    caplist->d = NULL;
  }else{
    caplist->d = (int *)calloc(caplist->n, sizeof(int));
    if(caplist->d == NULL){
      fprintf(stderr, "Error in input_zmap(); malloc caplist->d\n");
      free(zmap->z);
      if(*regprop != NULL){
	free(*regprop);
      }
      if(fp != stdin){
	fclose(fp);
      }
      return -1;
    }
  }
  /*
  for(i = 0; i < caplist->n; i++){
    fprintf(fp, "caplist.d[%d]=%d\n", i, caplist->d[i]);
  }
  */
  for(i = 0; i < caplist->n; i++){
    fetch_string(fp, str);
    if(sscanf(str,"%d", &caplist->d[i]) != 1){
      escape_func("Error in input_zmap(): caplist->[%d] lost\"%s\"\n",
		  i, str, fp, zmap->z, *regprop, caplist->d);
      return -1;
    }
  }

  if(fp != stdin){
    fclose(fp);
  }

  return 0;

}

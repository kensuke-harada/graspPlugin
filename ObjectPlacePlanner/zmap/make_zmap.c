#include <math.h>
#include <string.h>

#include "/usr/local/VVV/include/matutilP.h"
#include "make_zmap.h"

void
filter_by_z(RANGE	*rng,
	    double	z_th)
{
  PixelData	**pm;
  int	i;
  pm = rng->Map;
  for(i = 0; i < rng->MapSize; i++, pm++){
    if(*pm && ((*pm)->Label & RANGE_LABEL_DOT)){
      if((*pm)->Dot[2] < z_th){
	*pm = NULL;
      }
    }
  }
  return;
}

static int
comp_ipmax(const void *va,
	   const void *vb)
{
  int	*a, *b;

  a = (int *)va;
  b = (int *)vb;

  return *a - *b;
}

static int
check_inside_rect(double	x,
		  double	y,
		  RectAngle	*rect)
{
  double	m0, m1;

  /*
  x -= rect->center[0];
  y -= rect->center[1];
  */
  m0 = x * rect->dir[0][0] + y * rect->dir[0][1];
  if(m0 < rect->umin || m0 > rect->umax) return 0;

  m1 = x * rect->dir[1][0] + y * rect->dir[1][1];
  if(m1 < rect->vmin || m1 > rect->vmax) return 0;

  return 1;
}


static void
rectangle_from_boundary(BREP_Bound	*pbou,
			RectAngle	*rect,
			double	r_corner_2)
{
  BREP_Point	*pp;
  int	ip;
  double	sumx2, sumxy, sumy2, sumx[4], sumy[4], sum1[4];
  double	dx, dy;
  MAT_Matrix	a;
  double	d[2], e[2];
  int	i, j;
  double	maxlen2[4];
  int	ipmax[4];
  double	inprod[2];
  int	zone;
  double	tmplen2;
  int	ip0, ip1;
  double	gx, gy;
  double	g0[4];
  int	iseg;
  int	ip0cond, ip1cond;
  int	ipx;
  BREP_Point	*ppx;
  double	x0, y0, x1, y1;
  int	cont;
  int	ipseg[4][2];
  char	debugfile[256];
  FILE	*fp;

  a = mat_new_matrix(2, 2, NULL);

  // xmin, xmax, ymin, ymax
  rect->xmin = rect->xmax = pbou->point[0].x;
  rect->ymin = rect->ymax = pbou->point[0].y;
  
  pp = &pbou->point[1];
  for(ip = 1; ip < pbou->npoint; ip++, pp++){
    if(rect->xmin > pp->x) rect->xmin = pp->x;
    if(rect->xmax < pp->x) rect->xmax = pp->x;
    if(rect->ymin > pp->y) rect->ymin = pp->y;
    if(rect->ymax < pp->y) rect->ymax = pp->y;
  }

  // center
  rect->center[0] = (rect->xmin + rect->xmax)/2.0;
  rect->center[1] = (rect->ymin + rect->ymax)/2.0;

  // eigen matrix with center=gravity
  sumx2 = sumxy = sumy2 = 0.0;
  pp = &pbou->point[0];
  for(ip = 0; ip < pbou->npoint; ip++, pp++){
    dx = pp->x- rect->center[0];
    dy = pp->y- rect->center[1];
    sumx2 += dx*dx;
    sumxy += dx*dy;
    sumy2 += dy*dy;
  }

  // dir
  a[0][0] = sumx2;
  a[0][1] = a[1][0] = sumxy;
  a[1][1] = sumy2;

  mat_eigen(2, a, d, e);

  for(i = 0; i < 2; i++){
    for(j = 0; j < 2; j++){
      rect->dir[i][j] = a[i][j];
    }
  }

  // corner
  for(i = 0; i < 4; i++){
    maxlen2[i] = 0.0;
    ipmax[i] = -1;
  }

  // zone の中で center からの距離が最大の点をさがすー>コーナー
  pp = &pbou->point[0];
  for(ip = 0; ip < pbou->npoint; ip++, pp++){
    dx = pp->x- rect->center[0];
    dy = pp->y- rect->center[1];
    for(i = 0; i < 2; i++){
      inprod[i] = rect->dir[i][0] * dx + rect->dir[i][1] * dy;
    }

    zone = 0;
    if(inprod[0] < 0.0){
      zone = 2;
    }
    if(inprod[1] < 0.0){
      zone += 1;
    }

    tmplen2 = dx * dx + dy * dy;
    if(maxlen2[zone] < tmplen2){
      ipmax[zone] = ip;
      maxlen2[zone] = tmplen2;
    }
  }

  // ipmax をソートし，点列の区切りとする
  qsort(ipmax, 4, sizeof(int), comp_ipmax);

  // ipmax から 距離 r_corner の点をさがし、端点とする
  for(iseg = 0; iseg < 4; iseg++){
    // 仮端点のセット
    ip0cond = ipmax[iseg];
    ip1cond = ipmax[(iseg+1)%4];
    if(ip1cond < ip0cond){
      ip1cond += pbou->npoint;
    }
    x0 = pbou->point[ip0cond%pbou->npoint].x;
    y0 = pbou->point[ip0cond%pbou->npoint].y;
    x1 = pbou->point[ip1cond%pbou->npoint].x;
    y1 = pbou->point[ip1cond%pbou->npoint].y;

    // 開始点の探索
    ip = ip0cond;
    ip0 = -1;
    do{
      ipx = ip % pbou->npoint;
      ppx = &pbou->point[ipx];
      dx = ppx->x - x0;
      dy = ppx->y - y0;
      if(dx * dx + dy * dy > r_corner_2){
	cont = 0;
	ip0 = ip;
      }else{
	ip++;
	if(ip < ip1cond){
	  cont = 1;
	}else{
	  cont = 0;
	}
      }
    }while(cont);

    if(ip0 == -1){
      fprintf(stderr, "Error: cannot find seg%d-start\n", iseg);
      exit(1);
    }
    
    // 終了点の探索
    ip = ip1cond;
    ip1 = -1;
    do{
      ipx = ip % pbou->npoint;
      ppx = &pbou->point[ipx];
      dx = ppx->x - x1;
      dy = ppx->y - y1;
      if(dx * dx + dy * dy > r_corner_2){
	cont = 0;
	ip1 = ip;
      }else{
	ip--;
	if(ip > ip0cond){
	  cont = 1;
	}else{
	  cont = 0;
	}
      }
    }while(cont);

    if(ip1 == -1){
      fprintf(stderr, "Error: cannot find seg%d-goal\n", iseg);
      exit(1);
    }

    // 整合性チェック
    if(ip0 > ip1){
      fprintf(stderr, "Error: consistensy seg%d start=%d,goal=%d\n",
	      iseg, ip0, ip1);
      exit(1);
    }

    ipseg[iseg][0] = ip0;
    ipseg[iseg][1] = ip1;
  }

  {
    // debug 表示
    for(iseg = 0; iseg < 4; iseg++){
      sprintf(debugfile, "line%d.d", iseg);
      fp = fopen(debugfile, "w");
      fprintf(fp, "%g %g\n",
	      pbou->point[ipseg[iseg][0] % pbou->npoint].x,
	      pbou->point[ipseg[iseg][0] % pbou->npoint].y);
      fprintf(fp, "%g %g\n",
	      pbou->point[ipseg[iseg][1] % pbou->npoint].x,
	      pbou->point[ipseg[iseg][1] % pbou->npoint].y);
      fclose(fp);
    }
  }
  // ipseg[is][0-1] を使ってセグメントデータの共分散行列をつくり、
  // それを足し合わせて軸ベクトル dir を安定させる

  mat_fill_matrix(2, 2, a, 0.0);
  for(iseg = 0; iseg < 4; iseg++){
    sumx2 = sumxy = sumy2 = sumx[iseg] = sumy[iseg] = sum1[iseg] = 0.0;
    for(ip = ipseg[iseg][0]; ip <= ipseg[iseg][1]; ip++){
      pp = &pbou->point[ip%pbou->npoint];
      sumx2 += (pp->x * pp->x);
      sumxy += (pp->x * pp->y);
      sumy2 += (pp->y * pp->y);
      sumx[iseg] += pp->x;
      sumy[iseg] += pp->y;
      sum1[iseg] += 1.0;
    }

    if((iseg % 2) == 0){ // iseg = 0, 2
      a[0][0] += (sumx2 - sumx[iseg]*sumx[iseg]/sum1[iseg]);
      a[0][1] += (sumxy - sumx[iseg]*sumy[iseg]/sum1[iseg]);
      a[1][1] += (sumy2 - sumy[iseg]*sumy[iseg]/sum1[iseg]);
    }else{ // iseg = 1,3  90deg rotation
      a[0][0] += (sumy2 - sumy[iseg]*sumy[iseg]/sum1[iseg]);
      a[0][1] -= (sumxy - sumx[iseg]*sumy[iseg]/sum1[iseg]);
      a[1][1] += (sumx2 - sumx[iseg]*sumx[iseg]/sum1[iseg]);
    }
  }

  a[1][0] = a[0][1];

  mat_eigen(2, a, d, e);

  for(i = 0; i < 2; i++){
    for(j = 0; j < 2; j++){
      rect->dir[i][j] = a[i][j];
    }
  }

  // dir[0] は seg0,seg2方向、dir[1] は seg1,seg3 方向
  for(iseg = 0; iseg < 4; iseg++){
    gx = sumx[iseg]/sum1[iseg];
    gy = sumy[iseg]/sum1[iseg];
    g0[iseg] = gx*rect->dir[(iseg+1)%2][0] + gy*rect->dir[(iseg+1)%2][1];
  }

  if(g0[0] > g0[2]){
    rect->vmax = g0[0];
    rect->vmin = g0[2];
  }else{
    rect->vmax = g0[2];
    rect->vmin = g0[0];
  }    

  if(g0[1] > g0[3]){
    rect->umax = g0[1];
    rect->umin = g0[3];
  }else{
    rect->umax = g0[3];
    rect->umin = g0[1];
  }    

  {
    // debug 表示
    fp = fopen("rect.d", "w");

    // umin,vmin
    fprintf(fp, "%g %g\n",
	    rect->umin * rect->dir[0][0] +
	    rect->vmin * rect->dir[1][0],
	    rect->umin * rect->dir[0][1] +
	    rect->vmin * rect->dir[1][1]);
    // umin, vmax
    fprintf(fp, "%g %g\n",
	    rect->umin * rect->dir[0][0] +
	    rect->vmax * rect->dir[1][0],
	    rect->umin * rect->dir[0][1] +
	    rect->vmax * rect->dir[1][1]);
    // umax, vmax
    fprintf(fp, "%g %g\n",
	    rect->umax * rect->dir[0][0] +
	    rect->vmax * rect->dir[1][0],
	    rect->umax * rect->dir[0][1] +
	    rect->vmax * rect->dir[1][1]);
    // umax, vmin
    fprintf(fp, "%g %g\n",
	    rect->umax * rect->dir[0][0] +
	    rect->vmin * rect->dir[1][0],
	    rect->umax * rect->dir[0][1] +
	    rect->vmin * rect->dir[1][1]);
    // umin,vmin
    fprintf(fp, "%g %g\n",
	    rect->umin * rect->dir[0][0] +
	    rect->vmin * rect->dir[1][0],
	    rect->umin * rect->dir[0][1] +
	    rect->vmin * rect->dir[1][1]);

    fclose(fp);
  }

  // corner
  // umin,vmin
  rect->corner[0][0] = (rect->umin * rect->dir[0][0] +
			rect->vmin * rect->dir[1][0]);
  rect->corner[0][1] = (rect->umin * rect->dir[0][1] +
			rect->vmin * rect->dir[1][1]);
  // umin, vmax
  rect->corner[1][0] = (rect->umin * rect->dir[0][0] +
			rect->vmax * rect->dir[1][0]);
  rect->corner[1][1] = (rect->umin * rect->dir[0][1] +
			rect->vmax * rect->dir[1][1]);
  // umax, vmax
  rect->corner[2][0] = (rect->umax * rect->dir[0][0] +
			rect->vmax * rect->dir[1][0]);
  rect->corner[2][1] = (rect->umax * rect->dir[0][1] +
			rect->vmax * rect->dir[1][1]);
  // umax, vmin
  rect->corner[3][0] = (rect->umax * rect->dir[0][0] +
			rect->vmin * rect->dir[1][0]);
  rect->corner[3][1] = (rect->umax * rect->dir[0][1] +
			rect->vmin * rect->dir[1][1]);

  // {x|y}{min|max}の再計算
  rect->xmin = rect->xmax = rect->corner[0][0];
  rect->ymin = rect->ymax = rect->corner[0][1];
  for(i = 1; i < 4; i++){
    if(rect->corner[i][0] < rect->xmin) rect->xmin = rect->corner[i][0];
    if(rect->corner[i][0] > rect->xmax) rect->xmax = rect->corner[i][0];
    if(rect->corner[i][1] < rect->ymin) rect->ymin = rect->corner[i][1];
    if(rect->corner[i][1] > rect->ymax) rect->ymax = rect->corner[i][1];
  }

  mat_free_matrix(a);

  return;
}

// 三点の三角形の面積を計算
static double
calc_s_tri(double	*p0,
	   double	*p1,
	   double	*p2)
{
  int	k;
  double	vec0[3], vec1[3], prod[3];

  for(k = 0; k < 3; k++){
    vec0[k] = p0[k] - p2[k];
  }
  for(k = 0; k < 3; k++){
    vec1[k] = p1[k] - p2[k];
  }

  mat_outerproduct2(vec0, vec1, prod);
  
  return sqrt(mat_innerproduct(3, prod, prod));
}

	   
static void
remap_rng_to_zmap(RANGE	*rng,
		  RectAngle	*rect,
		  ZMAP	*zmap,
		  VERR	*err)
{
  PixelData	**pm, **pm00, **pm01, **pm10, **pm11;
  int	r, c;
  double	*p[4];
  double	xmin, xmax, ymin, ymax;
  int	i;
  int	ixmin, ixmax, iymin, iymax;
  int	ix, iy;
  double	z;
  double	*pz;

  double	s012, s312, s130, s230;
  double	*patch[2][3];
  int	ipatch, k, n;
  MAT_Matrix	m22, m22inv;
  long	work0[2];
  double	work1[2];
  double	vecxy[2], vecst[2];

  /*
    rngデータの連続的なzmapへのマッピング
    隣接する四点のレンジデータ点の内部をぬりつぶすようにzmapのデータをつくる
    四点がつくる四角形の内部かどうかを判定し，内部であればzを内挿して
    計算する
    
    p00,p01,p10,p11 があるとする
    内挿する関数を次のようにおく
    q = (1-t)*((1-s)*p00+s*p01)+t*((1-s)*p10+s*p11)
    これで、0<s,t<1の内部点は一葉双曲面上の四辺形の内部になる
    x,yに対する s,tを計算し，それが 0<s,t<1を満たせば内部点として
    zを計算する
    
    まず、x,yは、四点のxmin,xmax,ymin,ymaxを計算し，
    xmin<ixmin<=ixmax<xmax
    ymin<iymin<=iymax<ymax
    となる整数 ixmin, ixmax, iymin, iymax をきめる。
    (pitch は省略）
    この条件を満たす整数がx あるいはy方向で見付からなければその四角は終了
    (この条件は次の格子点のループ条件で自動的にされるので明示的な判定は
    不要)

    次に，[ixmin,ixmax][iymin,iymax]内の格子点すべてについて上記計算により
    s,tを計算し，s,tが内部条件を満たしていれば z を計算し，
    ix,iyにzを格納する

    s,tの計算方法
    qx = (1-t)*((1-s)*x00+s*x01)+t*((1-s)*x10+s*x11)
    qy = (1-t)*((1-s)*y00+s*y01)+t*((1-s)*y10+s*y11)

    展開して
    (1-t) (1-s) x00+(1-t) s x01 + t (1-s) x10 + t s x11 - qx = 0
    s t x00 - s x00 - t x00 + x00 -s t x01 + s x01 -s t x10 + t x10 + s t x11 - qx = 0
    s t (x00-x01-x10+x11) + s (-x00+x01) + t (-x00+x10) + (x00-qx) = 0
    これを次のようにおく
    a s t + b s + c t + d = 0
    これは次のように変形できる
    a (s-s0)(t-t0) =e
    展開すると
    a s t - a t0 s - a s0 t + a s0 t0 - e = 0
    b = -a t0
    c = -a s0
    d = a s0 t0 - e

    a がゼロかどうかで処理がちがう
    (1) a != 0 のとき
    t0 = -b/a
    s0 = -c/a
    e = a s0 t0 - d
    と計算し，
    s = e/a(t-t0) + s0

    (2) a = 0 のとき(めったにない)
    b s + c t + d = 0
    (2-1) b!=0のとき
    s=-(ct+d)/b
    (2-2)b=0のとき
    ct+d=0
    (2-2-1)c!=0のとき
    t=-d/c
    (2-2-2)c=0のとき
    (2-2-2-0) d=0 なら不定
    (2-2-2-1) d!=0 なら解無し
    
    同様な計算がyについても可能
    全部やると場合分けが大変なので、めったにない(2)以降は解無しにして
    問題ないとしてしまう

    ********* 実は a==0となる場合がたくさんあることが判明．
    これはkinectのデータの奥行きが離散的であるため。
    経験的に平面の場合には ax,ay < 1e-13
    値はそこから急に 1e-6 のオーダーまであがるので、
    閾値を 1e-6 として、どちらも 1e-6 より大きい場合には双曲面
    それ以外の場合には ax = ay = 0 として平面の式をとく
    ++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    したがって 
    s = ex/ax(t-t0x)+s0x = ey/ay(t-t0y)+s0y

    これはtについての二次方程式なので、tについてとき，
    tが0<t<1であればsを計算して 0<s<1であれば
    qz = (1-t)*((1-s)*z00+s*z01)+t*((1-s)*z10+s*z11)
    としてzを計算する
   */
  /*
    2012.03.28
    上記の閾値をつかって双曲面か平面かを判定するやりかたは
    うまくない
    閾値は経験的なものなので、もし微妙にあてはまらないことが起きると問題

    そこまでいくのなら、四角形を三角パッチ二個で表してそれぞれに平面を
    あてはめたほうが問題がおきにくい
    
    したがってつぎのようにする
    
    隣接する４個のデータ点があったとき、対角線で分割する二通りの
    三角パッチでわける

    三角パッチの面積の和が小さいほうの分割を採用する
    面積は外積の絶対値の和で計算
    
    それぞれの三角形の内部にXY座標値がある頂点をしらべ、そのZ座標値を内挿して
    記録する
   */

  m22 = mat_new_matrix(2, 2, err);
  if(VVV_ERR_P(err)){
    return;
  }
  m22inv = mat_new_matrix(2, 2, err);
  if(VVV_ERR_P(err)){
    mat_free_matrix(m22);
    return;
  }
  pm = rng->Map;
  for(r = 0; r < rng->Height-1; r++, pm+=rng->Width){
    pm00 = pm;
    pm01 = pm+1;
    pm10 = pm+rng->Width;
    pm11 = pm+rng->Width+1;
    for(c = 0; c < rng->Width-1; c++, pm00++,pm01++,pm10++,pm11++){
      if(*pm00 && ((*pm00)->Label & RANGE_LABEL_DOT) &&
	 ((*pm00)->Surf & (REGION_AROUND_EDGE|REGION_DEPTH_EDGE)) == 0 &&
	 *pm01 && ((*pm01)->Label & RANGE_LABEL_DOT) &&
	 ((*pm01)->Surf & (REGION_AROUND_EDGE|REGION_DEPTH_EDGE)) == 0 &&
	 *pm10 && ((*pm10)->Label & RANGE_LABEL_DOT) &&
	 ((*pm10)->Surf & (REGION_AROUND_EDGE|REGION_DEPTH_EDGE)) == 0 &&
	 *pm11 && ((*pm11)->Label & RANGE_LABEL_DOT) &&
	 ((*pm11)->Surf & (REGION_AROUND_EDGE|REGION_DEPTH_EDGE)) == 0){
	// all pm has dot
	if(check_inside_rect((*pm00)->Dot[0],(*pm00)->Dot[1],rect) &&
	   check_inside_rect((*pm01)->Dot[0],(*pm01)->Dot[1],rect) &&
	   check_inside_rect((*pm10)->Dot[0],(*pm10)->Dot[1],rect) &&
	   check_inside_rect((*pm11)->Dot[0],(*pm11)->Dot[1],rect)){
	  // all dot is inside rect
	  p[0] = (*pm00)->Dot;
	  p[1] = (*pm01)->Dot;
	  p[2] = (*pm10)->Dot;
	  p[3] = (*pm11)->Dot;
	  xmin = xmax = p[0][0];
	  ymin = ymax = p[0][1];
	  for(i = 1; i < 4; i++){
	    if(p[i][0] < xmin) xmin = p[i][0];
	    if(p[i][0] > xmax) xmax = p[i][0];
	    if(p[i][1] < ymin) ymin = p[i][1];
	    if(p[i][1] > ymax) ymax = p[i][1];
	  }
	  ixmin = ceil(xmin/zmap->pitch);
	  ixmax = floor(xmax/zmap->pitch);
	  iymin = ceil(ymin/zmap->pitch);
	  iymax = floor(ymax/zmap->pitch);


	  // 120328
	  // 三角パッチx2 のやりかた

	  // 格子点の有無の判定
	  if(ixmin <= ixmax && iymin <= iymax){
	    // 面積によるパッチくみの判定
	    s012 = calc_s_tri(p[0], p[1], p[2]);
	    s312 = calc_s_tri(p[3], p[1], p[2]);
	    s130 = calc_s_tri(p[1], p[3], p[0]);
	    s230 = calc_s_tri(p[2], p[3], p[0]);

	    if(s012 + s312 < s130 + s230){
	      // patch 012,312
	      patch[0][0] = p[0];
	      patch[0][1] = p[1];
	      patch[0][2] = p[2];
	      patch[1][0] = p[3];
	      patch[1][1] = p[1];
	      patch[1][2] = p[2];
	    }else{
	      // patch 130,230
	      patch[0][0] = p[1];
	      patch[0][1] = p[3];
	      patch[0][2] = p[0];
	      patch[1][0] = p[2];
	      patch[1][1] = p[3];
	      patch[1][2] = p[0];
	    }

	    // 各パッチについての処理
	    for(ipatch = 0; ipatch < 2; ipatch++){
	      // s,t を計算するための係数行列
	      for(k = 0; k < 2; k++){
		for(n = 0; n < 2; n++){
		  m22[k][n] = patch[ipatch][n][k] - patch[ipatch][2][k];
		}
	      }
	      if(m22[0][0]*m22[1][1] - m22[0][1]*m22[1][0] != 0.0){
		mat_matinv2(2, m22, m22inv, work0, work1); // work0[2],work1[2]
		for(iy = iymin; iy <= iymax; iy++){
		  for(ix = ixmin; ix <= ixmax; ix++){
		    vecxy[0] = (double)ix - patch[ipatch][2][0];
		    vecxy[1] = (double)iy - patch[ipatch][2][1];
		    mat_multvector2(2, 2, m22inv, vecxy, vecst);
		    if(vecst[0] >= 0.0 && vecst[1] >= 0.0 &&
		       vecst[0] + vecst[1] <= 1.0){
		      // 三角形の内部
		      z = ((patch[ipatch][0][2]-patch[ipatch][2][2])*vecst[0]+
			   (patch[ipatch][1][2]-patch[ipatch][2][2])*vecst[1]+
			   patch[ipatch][2][2]);
		      pz = &zmap->z[(int)((iy-zmap->offset_y/zmap->pitch) 
					  *zmap->mapsize_x+ 
					  ix-zmap->offset_x/zmap->pitch)]; 
		      if(*pz < z){
			*pz = z;
		      } 
		    } // if(vecst) end
		  } // for(ix) end
		} // for(iy) end
	      }// if(det(m22)) end
	    } // for(ipatch) end
	  } // if(ixmin,ixmax,iymin,iymax) end
	} // if(check_inside) end
      }// if(LABEL_DOT) end
    } //for(c) end
  } // for(r) end

  mat_free_matrix(m22);
  mat_free_matrix(m22inv);

  return;
}

ZMAP
create_map_data(RANGE	*rng,
		RANGE	*rng2, // データ参照用
		BREP	*brp,
		short	*rtob,
		RectAngle	*rect,
		OptMakeZmap	*opt_make_zmap,
		VERR	*err)
{
  RANGE_Region	*prl;
  int	max_area;
  int	ir_max_area;
  int	irr;
  int	ib_max_area;
  BREP_Region	*pbrmax;
  int	ibholemax;
  int	holemax;
  BREP_Bound	*pb;
  int	ib;
  ZMAP	zmap;
  double	*pz;
  int	ix, iy;
  double	y, x;
  double	zmin;
  double	*pdl;
  int	i;
  int	zminflag;
  
  // get hole boundary
  // 最大サイズの領域をさがす
  prl = rng->RegionList;
  max_area = 0;
  ir_max_area = -1;
  for(irr = 0; irr < rng->RegionCount; irr++, prl++){
    if(prl->a > max_area){
      max_area = prl->a;
      ir_max_area = irr;
    }
  }
  if(ir_max_area == -1){
    fprintf(stderr, "Err No region found\n");
    exit(1);
  }

  // 最大サイズの領域内の最大サイズの穴を探す
  ib_max_area = rtob[ir_max_area];
  pbrmax = &brp->region[ib_max_area];
  if(pbrmax->nbound < 2){
    fprintf(stderr, "Err nbound of max area is 1.\n");
    exit(1);
  }

  ibholemax = 1;
  holemax = pbrmax->bound[1].npoint;
  pb = &pbrmax->bound[2];
  for(ib = 2; ib < pbrmax->nbound; ib++, pb++){
    if(pb->npoint > holemax){
      holemax= pb->npoint;
      ibholemax = ib;
    }
  }

  // 点列の (x,y) に対して長方形の頂点を得る
  rectangle_from_boundary(&pbrmax->bound[ibholemax],
			  rect,
			  50.0*50.0);

  // zmap の作成
  // 初期化
  memset(&zmap, 0, sizeof(ZMAP));

  zmap.pitch = opt_make_zmap->pitch;

  // zmap メンバのセット
  zmap.offset_x = (floor(rect->xmin/zmap.pitch)-1.0)*zmap.pitch;
  zmap.offset_y = (floor(rect->ymin/zmap.pitch)-1.0)*zmap.pitch;

  zmap.mapsize_x = (int)(ceil(rect->xmax/zmap.pitch)+1.0 - zmap.offset_x/zmap.pitch) +1;
  zmap.mapsize_y = (int)(ceil(rect->ymax/zmap.pitch)+1.0 - zmap.offset_y/zmap.pitch) +1;

  zmap.z = (double *)calloc(zmap.mapsize_x*zmap.mapsize_y, sizeof(double));
  if(zmap.z == NULL){
    VVV_ERROR(VVV_ERROR_MEMALLOC, "fail in malloc zmap.z", err);
    return zmap;
  }

  // Z_NoData, Z_OutRect のセット
  zmin = -1.0;
  pdl = rng->DotList;
  zminflag = 0;
  for(i = 0; i < rng->DotCount; i++, pdl+=3){
    if(check_inside_rect(pdl[0], pdl[1], rect)){
      if(zminflag == 0 || pdl[2] < zmin){
	zminflag = 1;
	zmin = pdl[2];
      }
    }
  }
  zmap.Z_NoData = zmin - 1.0;
  zmap.Z_OutRect = zmin - 2.0;

  pz = zmap.z;
  for(iy = 0; iy < zmap.mapsize_y; iy++){
    y = zmap.offset_y + zmap.pitch * (double)iy;
    for(ix = 0; ix < zmap.mapsize_x; ix++, pz++){
      x = zmap.offset_x + zmap.pitch * (double)ix;
      if(check_inside_rect(x, y, rect)){
	*pz = zmap.Z_NoData;  // 計測データとかぶらない数字
      }else{
	*pz = zmap.Z_OutRect; // 計測データとかぶらない数字
      }
    }
  }

  remap_rng_to_zmap(rng2, rect, &zmap, err);

  return zmap;
}

int
comp_zmap(const int	a,
	  const int	b,
	  const void	*vzmap)
{
  ZMAP	*zmap;
  int	ret;
  double	vala, valb;

  zmap = (ZMAP *)vzmap;

  vala = zmap->z[a];
  valb = zmap->z[b];

  if(vala == zmap->Z_NoData || vala == zmap->Z_OutRect){
    if(valb == zmap->Z_NoData || valb == zmap->Z_OutRect){
      ret = 0;
    }else{
      ret = 1;
    }
  }else{
    if(valb == zmap->Z_NoData || valb == zmap->Z_OutRect){
      ret = 1;
    }else{ // both point has valid data
      if(fabs(vala - valb) > zmap->dz){
	ret = 1;
      }else{
	ret = 0;
      }
    }
  }

  return ret;
}


CapList
find_cap(EPBM	*label,
	 int	nlabel,
	 ZMAP	*zmap,
	 double	capmin,
	 double	capmax,
	 RegProp	*regprop,
	 VERR	*err)
{
  RegProp	*prp;
  short	*plbl;
  int	r, c;
  double	x, y, z;
  int	ir;
  MAT_Matrix	a;
  double	d[2], e[2];
  int	j, i;
  double	dvec[2];
  double	tmpr;
  CapList	caplist;

  caplist.n = 0;
  caplist.d = NULL;


  a = mat_new_matrix(2, 2, err);
  if(VVV_ERR_P(err)){
    //free(regprop);
    return caplist;
  }
  
  plbl = (short *)(label->Image);
  for(r = 0; r < label->Height; r++){
    y = r * zmap->pitch + zmap->offset_y;
    for(c = 0; c < label->Width; c++, plbl++){
      if(*plbl > 0){
	prp = &regprop[*plbl-1];
	x = c * zmap->pitch + zmap->offset_x;

	z = zmap->z[zmap->mapsize_x*r + c];/*Harada*/

	prp->x2 += (x * x);
	prp->xy += (x * y);
	prp->y2 += (y * y);
	prp->x += x;
	prp->y += y;
	prp->z += z;
	prp->n += 1.0;
      }
    }
  }

  prp = regprop;
  for(ir = 0; ir < nlabel; ir++, prp++){
    if(prp->n > 0.0){
      prp->g[0] = prp->x / prp->n;
      prp->g[1] = prp->y / prp->n;
      prp->g[2] = prp->z / prp->n;

      a[0][0] = prp->x2 - prp->x * prp->x / prp->n;
      a[0][1] =
	a[1][0] = prp->xy - prp->x * prp->y / prp->n;
      a[1][1] = prp->y2 - prp->y * prp->y / prp->n;

      if(mat_eigen(2, a, d, e) == MAT_OK){
	prp->stat = REGPROP_EIGEN_OK; // 1
	for(j = 0; j < 2; j++){
	  for(i = 0; i < 2; i++){
	    prp->v[j][i] = a[j][i];
	  }
	}

	prp->max[0] = prp->max[1] = prp->min[0] = prp->min[1] = 0.0;
      }else{
	prp->stat = REGPROP_EIGEN_NG; // 0
	for(j = 0; j < 2; j++){
	  for(i = 0; i < 2; i++){
	    prp->v[j][i] = 0.0;
	  }
	}
      }
    }else{
      prp->stat = REGPROP_EIGEN_NG; // 0
    }
  }
      
  plbl = (short *)(label->Image);
  for(r = 0; r < label->Height; r++){
    for(c = 0; c < label->Width; c++, plbl++){
      if(*plbl > 0){
	prp = &regprop[*plbl-1];

	if(prp->stat == REGPROP_EIGEN_OK){
	  dvec[0] = c * zmap->pitch + zmap->offset_x - prp->g[0];
	  dvec[1] = r * zmap->pitch + zmap->offset_y - prp->g[1];
	  for(j = 0; j < 2; j++){
	    tmpr = mat_innerproduct(2, dvec, prp->v[j]);
	    if(tmpr > prp->max[j]){
	      prp->max[j] = tmpr;
	    }else if(tmpr < prp->min[j]){
	      prp->min[j] = tmpr;
	    }
	  }
	}
      }
    }
  }

  caplist.n = 0;
  prp = regprop;
  for(ir = 0; ir < nlabel; ir++, prp++){
    if(prp->stat == REGPROP_EIGEN_OK){
      if(prp->max[0] - prp->min[0] > capmin &&
	 prp->max[0] - prp->min[0] < capmax &&
	 prp->max[1] - prp->min[1] > capmin &&
	 prp->max[1] - prp->min[1] < capmax){
	caplist.n++;
      }
    }
  }

  if(caplist.n > 0){

    caplist.d = (int *)calloc(caplist.n, sizeof(int));
    if(caplist.d == NULL){
      VVV_ERROR(VVV_ERROR_MEMALLOC, "malloc err at caplist.d", err);
      mat_free_matrix(a);
    }

    caplist.n = 0;
    prp = regprop;
    for(ir = 0; ir < nlabel; ir++, prp++){
      if(prp->stat == REGPROP_EIGEN_OK){
	if(prp->max[0] - prp->min[0] > capmin &&
	   prp->max[0] - prp->min[0] < capmax &&
	   prp->max[1] - prp->min[1] > capmin &&
	   prp->max[1] - prp->min[1] < capmax){
	  caplist.d[caplist.n] = ir;
	  caplist.n++;
	}
      }
    }
  }else{
    caplist.d = NULL;
  }

  mat_free_matrix(a);

  return caplist;
}


void
add_bottle(ZMAP	*zmap,
	   int	capid,
	   RegProp	*regprop,
	   EPBM	*label,
	   double	sholder_rad,
	   double	diff_z)
{
  double	sumz;
  int	n;
  int	all, i;
  short	*plbl;
  double	*pz;
  double	capz, sholderz;
  int	min_c, max_c, min_r, max_r;
  double	rad2;
  int	r, c;
  double	y, x, dy, dx;

  sumz = 0.0;
  n = 0;
  all = label->Width * label->Height;
  plbl = (short *)(label->Image);
  pz = zmap->z;
  for(i = 0; i < all; i++, plbl++, pz++){
    if(*plbl - 1 == capid){
      sumz += *pz;
      n++;
    }
  }

  capz = sumz / (double)n;
  sholderz = capz - diff_z;

  min_c = (int)floor((regprop[capid].g[0] - sholder_rad - zmap->offset_x)
		     / zmap->pitch);
  if(min_c < 0) min_c = 0;

  max_c = (int)ceil((regprop[capid].g[0] + sholder_rad - zmap->offset_x)
		    / zmap->pitch);
  if(max_c >= label->Width) max_c = label->Width-1;

  min_r = (int)floor((regprop[capid].g[1] - sholder_rad - zmap->offset_y)
		     / zmap->pitch);
  if(min_r < 0) min_r = 0;

  max_r = (int)ceil((regprop[capid].g[1] + sholder_rad - zmap->offset_y)
		    / zmap->pitch);
  if(max_r >= label->Height) max_r = label->Height-1;

  rad2 = sholder_rad * sholder_rad;
  plbl = (short *)(label->Image);
  for(r = min_r; r <= max_r; r++){
    y = r * zmap->pitch + zmap->offset_y;
    dy = y - regprop[capid].g[1];
    for(c = min_c; c <= max_c; c++){
      if(plbl[r * label->Width+c]-1 != capid){
	x = c * zmap->pitch + zmap->offset_x;
	dx = x - regprop[capid].g[0];
	if(dx * dx + dy * dy < rad2){
	  zmap->z[r * label->Width+c] = sholderz;
	}
      }
    }
  }
  
  return;
}


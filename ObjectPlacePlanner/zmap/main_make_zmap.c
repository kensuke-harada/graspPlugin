
//  (1) プログラム内でレンジデータを複製
//  (2) 高さの低い点をNULLに
//  (3) Aエッジ、Dエッジをつける
//  (4) セグメンテーション
//  (5) 最大領域をさがし、穴の輪郭をえる
//  (6) 穴の輪郭にXY平面内の長方形をフィット
//  (7) 穴長方形を含む1mm間隔のメッシュをつくる
//  (8) 内部の点をプロット
//  (9) ペットボトルのふたをさがす
//  (10)ペットボトルのふたの中心をさがし、そのまわりにびんの上部をつくる

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "/usr/local/VVV/include/rangeopen.h"
#include "/usr/local/VVV/include/rangeedge.h"
#include "/usr/local/VVV/include/range_to_brepA.h"
#include "/usr/local/VVV/include/range_segmentation.h"
#include "/usr/local/VVV/include/epbmlabeling.h"

#include "make_zmap.h"

static int
find_opt(int	argc,
	 char	**argv,
	 char	*key)
{
  while(--argc > 0){
    if(strcmp(argv[argc], key) == 0){
      return argc;
    }
  }

  return 0;
}

static void
show_help()
{
  char	*help[] = {
    "usage make_zmap [-i fin][-o fout][options]",
    " ",
    "   -i f_in input file range data (def = STDIN)",
    "   -o f_out output file (def=STDOUT)",
    " ",
    "   --dth depth   edge distance ()",
    "   --zth z_th    z filter height ()",
    "   --pitch pitch XY grid pitch size ()",
    ""};
  int	i;

  i = 0;
  while(help[i][0]){
    printf("%s\n", help[i]);
    i++;
  }

  return;
}


static void
arg_p(int	argc,
      char	**argv,
      char	**fin,
      char	**fout,
      OptMakeZmap	*opt)
{
  int	i0;

  i0 = find_opt(argc, argv, "-h");
  if(i0 > 0){
    show_help();
    exit(0);
  }

  *fin = *fout = NULL;
  opt->z_th = DEF_Z_TH; // -5.0
  opt->dth = DEF_DTH; // 5.0
  opt->pitch = DEF_PITCH; // 1.0

  // range_segmentatio.h から
  opt->segarg.lshort = DEF_SEG_LSHORT; // 8.0;
  opt->segarg.lext = DEF_SEG_LEN_EXT; //2.0;
  opt->segarg.deltalen = DEF_SEG_DELTALEN; //2.0;
  opt->segarg.width0 = DEF_SEG_WIDTH0; //4.0;
  opt->segarg.width1 = DEF_SEG_WIDTH1; //4.0;
  opt->segarg.mergespeed = DEF_SEG_MERGE_SPEED; //1;

  i0 = find_opt(argc, argv, "-i");
  if(i0 > 0){
    if(i0+1 < argc){
      *fin = argv[i0+1];
    }else{
      fprintf(stderr, "Error: No value for option \"%s\"\n",
	      argv[i0]);
      exit(1);
    }
  }

  i0 = find_opt(argc, argv, "-o");
  if(i0 > 0){
    if(i0+1 < argc){
      *fout = argv[i0+1];
    }else{
      fprintf(stderr, "Error: No value for option \"%s\"\n",
	      argv[i0]);
      exit(1);
    }
  }

  i0 = find_opt(argc, argv, "--dth");
  if(i0 > 0){
    if(i0+1 < argc){
      if(sscanf(argv[i0+1], "%lg", &opt->dth ) != 1){
	fprintf(stderr, "Error: option Conversion\"%s %s\"\n",
		argv[i0], argv[i0+1]);
	exit(1);
      }
    }else{
      fprintf(stderr, "Error: No value for option \"%s\"\n",
	      argv[i0]);
      exit(1);
    }
  }

  i0 = find_opt(argc, argv, "--zth");
  if(i0 > 0){
    if(i0+1 < argc){
      if(sscanf(argv[i0+1], "%lg", &opt->z_th ) != 1){
	fprintf(stderr, "Error: option Conversion\"%s %s\"\n",
		argv[i0], argv[i0+1]);
	exit(1);
      }
    }else{
      fprintf(stderr, "Error: No value for option \"%s\"\n",
	      argv[i0]);
      exit(1);
    }
  }

  i0 = find_opt(argc, argv, "--pitch");
  if(i0 > 0){
    if(i0+1 < argc){
      if(sscanf(argv[i0+1], "%lg", &opt->pitch ) != 1){
	fprintf(stderr, "Error: option Conversion\"%s %s\"\n",
		argv[i0], argv[i0+1]);
	exit(1);
      }
    }else{
      fprintf(stderr, "Error: No value for option \"%s\"\n",
	      argv[i0]);
      exit(1);
    }
  }

  return;
}


int
main(int	argc,
     char	**argv)
{
  char	*fin, *fout;
  OptMakeZmap	opt_make_zmap;
  RANGE	rng, rng2;
  BREP	brp;
  short	*rtob;
  ZMAP	mapdata;
  EPBM	label;
  int	nlabel;
  CapList	caplist;
  RegProp	*regprop;
  int	icap;
  RectAngle	rect;

  arg_p(argc, argv, &fin, &fout, &opt_make_zmap);

//  (0) rng データ読み込み
  range_open(fin, &rng, NULL);

//  (1) プログラム内でレンジデータを複製
  rng2 = range_duplicate(&rng, NULL);
  
//  (2) 高さの低い点をNULLに
  filter_by_z(&rng, opt_make_zmap.z_th);

//  (3) Aエッジ、Dエッジをつける
   set_edge_A(&rng, NULL);
   set_edge_D(&rng, opt_make_zmap.dth, NULL);

   set_edge_A(&rng2, NULL);
   set_edge_D(&rng2, opt_make_zmap.dth, NULL);

//  (4) セグメンテーション
// rng_segmentation() の err を塗るにするとエラーが発生するバグを発見

   rng_segmentation(&rng, 0.0, 0.0,
		    opt_make_zmap.segarg.width0,
		    opt_make_zmap.segarg.width1,
		    opt_make_zmap.segarg.lshort,
		    opt_make_zmap.segarg.lext,
		    opt_make_zmap.segarg.mergespeed,
		    opt_make_zmap.segarg.deltalen,
		    0, NULL);
		    
//  (5) 最大領域をさがし、穴の輪郭をえる
   brp = r2b_range2brepA(&rng, &rtob, NULL);

   // (6) brpおよび rng を使って、マップデータを作成する
   // マップデータは、二次元(XY)のインデックスをもつZ方向の値を持つ
   // 二次元配列と，ピッチ，オフセットなどをもつ構造体
   mapdata = create_map_data(&rng, &rng2, &brp, rtob,
			     &rect, &opt_make_zmap, NULL);

   {
     FILE	*fp;
     int	x, y;
     fp = fopen("zmap.plot", "w");
     for(y = 0; y < mapdata.mapsize_y; y++){
       for(x = 0; x < mapdata.mapsize_x; x++){
	 fprintf(fp, "%lg %lg %lg\n",
		 x+mapdata.offset_x,
		 y+mapdata.offset_y,
		 mapdata.z[y*mapdata.mapsize_x+x]);
       }
       fprintf(fp, "\n");
     }
     fclose(fp);
   }


   // (7) 高さでラベリング

   label = epbm_create(EPBM_BINARY_GRAY,
		       mapdata.mapsize_x, mapdata.mapsize_y,
                       EPBM_SIGNED, EPBM_SHORT16, NULL);

   mapdata.dz = 10.0;
   nlabel = epbm_label_by_pixel(&label, comp_zmap, (void *)&mapdata, NULL);

   printf("nlabel = %d\n", nlabel);

  regprop = (RegProp *)calloc(nlabel, sizeof(RegProp));
  if(regprop == NULL){
    fprintf(stderr,  "malloc error at regprop");
    return -1;
  }

  caplist = find_cap(&label, nlabel, &mapdata,
		     CAP_MIN, //25.0,
		     CAP_MAX, //40.0,
		     regprop, NULL);
     
//  (9) ペットボトルのふたをさがす
//  (10)ペットボトルのふたの中心をさがし、そのまわりにびんの上部をつくる

  for(icap = 0; icap < caplist.n; icap++){
    add_bottle(&mapdata, caplist.d[icap], regprop, &label,
	       SHOLDER_RAD, //32.5,
	       DIFF_Z // 30.0
	       );
  }

   {
     FILE	*fp;
     int	x, y;
     fp = fopen("zmap_bottle.plot", "w");
     for(y = 0; y < mapdata.mapsize_y; y++){
       for(x = 0; x < mapdata.mapsize_x; x++){
	 fprintf(fp, "%lg %lg %lg\n",
		 x+mapdata.offset_x,
		 y+mapdata.offset_y,
		 mapdata.z[y*mapdata.mapsize_x+x]);
       }
       fprintf(fp, "\n");
     }
     fclose(fp);
   }

   output_zmap(fout, &opt_make_zmap, &rect, &mapdata, 
	       nlabel, regprop, &caplist);

   range_free(&rng);
   range_free(&rng2);

   if(caplist.d){
     free(caplist.d);
   }

   epbm_free(&label);

   free(mapdata.z);

   free(regprop);

   brep_free(&brp);

   free(rtob);

   return 0;
}

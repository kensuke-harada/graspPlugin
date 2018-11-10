#include <stdio.h>
#include "set_define.h"

struct list_data{
	void *element;
	struct list_data *next;
};

struct ring_list_data{
	int type;
	void *element;
	struct ring_list_data *next;
	struct ring_list_data *previous;
//	struct ring_list_data *nbr;
#ifdef MAKE_CLUSTER_HIERARCHY
	struct ring_list_data *parent;
//	int Nvertex;
//	int Nremainvertex;
	int number;
	int ring_number;
	struct cluster_data *cluster;
	struct ring_list_data *pair;
#endif
};

struct vertex_data{
#ifdef FLOAT_VERTEX
	float normal[3];
	float pos[3];
#else
	double normal[3];
	double pos[3];
#endif
	int number;
	int check;
	struct list_data neighbor_triangle_list;
#ifdef MAKE_CLUSTER_HIERARCHY
	int Nvtalias;
	int cluster_mark;
	int mark;
	int closed_check;
//	int edge_mark;
//	struct cluster_data *clusters[2];
	struct edge_data *edge;
	struct cluster_data *bothcluster;
//	struct ring_list_data *alias;
	int ring_number;
	struct vertex_tree_data *treevertex;
	struct vertex_tree_data *vtalias;
//	struct list_data vtlist;
#endif
#ifdef VIEW_CLUSTER_HIERARCHY
	struct cluster_data *clusters[2];
	struct cluster_data *bothcluster;
	int ring_number;
	int raster_pos[2];
	int inout;
	double depth;
#endif
}; /* 頂点データ */

struct vertex_hierarchy_data{
	int check;
	struct vertex_data *vertex;
	struct vertex_hierarchy_data *next;
}; /* 頂点階層データ */

struct vertex_tree_data{
	int check;
	int number;
	int boundary_order;
	struct vertex_data *vertex;
	struct edge_data *edge;
	struct cluster_data *bothcluster;
	struct cluster_data *onecluster;
	struct cluster_data *parentcluster;
//	struct vertex_tree_data *children[2];
	struct vertex_tree_data *parent;
	struct vertex_tree_data *reverse_parent;
	struct vertex_tree_data *vtalias;
	struct vertex_tree_data *next;

}; /* 頂点階層データ */

struct neighbor_vertex_data{
	int check;
	double length;
	struct vertex_data *ver;
	struct vertex_data *belong;
};

struct color_data{
	float rgb[3];
};

struct edge_data{
	int loop;
	int number;
//	int check; 
	struct vertex_tree_data *vtop[2];
//	struct cluster_data *bothcluster;
};

struct triangle_data{
	int number;
	int check;
	int cluster_number;
//	int before_cluster_number;
	double area;
	double normal[3];
//	double outer_product[3];
	double center[3];
	struct vertex_data *ver[3];
	struct triangle_data *nbr[3];
//	struct color_data *color[3];
#ifdef MAKE_CLUSTER_HIERARCHY
//	struct cluster_data *cluster;
	double area_quadrics[3][3];
//	double gravity_center[3];
//	double normal_center[3];
#endif
#ifdef TRIANGLE_HIERARCHY
	double inner_product;
	double dir_range;
	double radius;
	int fill_hole_flag;
	double error_value;
	struct triangle_data *parent;
//	struct vertex_data *midpoint[3];
	struct triangle_data *children[2];
	struct triangle_data *pair;
	struct vertex_data *newvertex;
#endif //TRIANGLE_HIERARCHY
#ifdef MAKE_TRIANGLE_HIERARCHY
	int mark;
	int Npolygon_vertex;
	struct triangle_data *center_triangle;
	struct triangle_data *trianglelink;
	struct list_data triangle_posterity;
	struct list_data boundary_vertex_list;
	struct list_data polygon_vertex_list;
#endif
};

//#define MINIMUM_CLUSTER_DATA

struct cluster_data{
	
	int number;
	int check;
	int loop;
	int number_of_vertex;
	int Ncluster_ver_link;
//	float rgb[3];
	struct cluster_data *children[2];
	struct cluster_data *parent;
#ifndef MINIMUM_CLUSTER_DATA
	double area;
	double quadrics_error;
	double correlation_error;
	double normal_center[3];
#endif
	double center[3];
//	double p2n[3][3];
#ifdef MAKE_CLUSTER_HIERARCHY
//	int NnewBoundary;
//	int NBoundary;
	double direction_range;
	double position_range;
	double bounding_distance;
	double normal[3];
	double pca[3][3];
	double cca[3][3];
	double cca_center[3];
	int number_of_triangle;
//	struct ring_list_data *boundary_vertex_cluster_list;
	struct triangle_data **triangles_que;
//	struct triangle_data *trianglelink;
	struct list_data polygons;
	struct list_data vertexlink;
#endif
#ifdef MAKE_EDGE_HIERARCHY
//	struct vertex_tree_data *vt_top[2];
//	struct vertex_tree_data *vt_temp[2];
//	struct ring_list_data *edgelist;
#endif
#ifdef RANDOM_CLUSTER_COLOR
	float m[3];
#endif
#ifdef VIEW_CLUSTER_HIERARCHY
	int Nhvertices;
	double direction_range;
	double upper_distance;
	double depth;
	double normal[3];
//	double cca[3][3];
	struct vertex_data **polygon_ver;
	struct cluster_data **polygon_edge;
	struct vertex_data **cluster_ver_link;
	struct vertex_hierarchy_data *hvertices;
#endif
};

struct view_vertex_data{
	float normal[3];
	float pos[3];
	char check;
}; /* 頂点データ */

struct view_vertex_tree_data{
	char check;
//	int number;
	unsigned short boundary_order; //0 から 65,535
	struct view_vertex_data *vertex;
	struct view_vertex_tree_data *next;
};

struct view_vertex_tree_link_data{
//	struct view_vertex_tree_data *myself;
	struct view_vertex_tree_data *parent;
	struct view_vertex_tree_data *reverse_parent;
	struct view_vertex_tree_data *alias;
};

struct view_cluster_data{
	char check;
	char number_of_vertex;
	char Ncluster_ver_link;
//	int number;
	int verlinknumber;
	struct view_cluster_data *children;
//	struct view_cluster_data *parent;
	float area;
	float quadrics_error;
//	double correlation_error;
	float position_range;
	float direction_range;
	float center[3];
	float normal[3];
	struct view_vertex_tree_data **polygon;
};

struct object_data
{
    char name[256];
	char filetype[256];
	int list_id;
	int nbr_status;
	int vertex_normal_status;
	int num_of_ver;
    int num_of_tex_ver;
	int num_of_all_tri;
	int num_of_init_tri;
	int num_of_process;
	int number_of_usemtl;
	int number_of_cluster;
	int Nheap;
	int Ncolor;
	int Nring_lists;
	int Nvpolygon;
	int Nhvertices;
	int Nvertex_que;
	int Naddvertices;
	int Ntree_verticies;
	int Nvertexlink;
	int Nedge;
	int Nclusterque;
	int max_number_of_vertiecs_of_polygon;
	int number_of_base_triangle;
	int *pnumber;
	int cqtop;
	int cqend;
	double pos[3];
	double nml[3];
	double data_range_high[3];
	double data_range_low[3];
	double size;
	double center[3];
	struct vertex_data *vertices;
	struct edge_data *edges;
	struct texture_vertex_data *texture_vertices;
	struct triangle_data *triangles;
	struct cluster_data *clusters;
	struct triangle_data *base_triangles;
	struct color_data *colors;
	struct ring_list_data **ring_lists;
	struct vertex_data **polygon_ver;
	struct cluster_data **polygon_edge;
	struct vertex_hierarchy_data *hvertices;
	struct vertex_hierarchy_data *addvertices;
	struct vertex_data **cluster_ver_link;
	struct vertex_data **vertex_que;
	struct vertex_tree_data *tree_vertices;

	struct view_vertex_data *view_verticies;
	struct view_cluster_data *view_clusters;
	struct view_vertex_tree_data *view_vertex_trees;
	struct view_vertex_tree_data **view_polygons;
	struct view_vertex_tree_link_data *view_verlink;
	struct view_cluster_data **clusterque;

  //HANDLE hFileMap;
}; /* オブジェクトデータ */

struct heap_data{
	int cluster_number;
	double quadrics;
};

struct octree_data{
	int depth;
	int check;
	double range[3][2];
	struct octree_data *children[8];
	struct octree_data *parent;
};

typedef struct triangle_data triangle_data;
typedef struct vertex_data vertex_data;
typedef struct object_data object_data;
typedef struct cluster_data cluster_data;
typedef struct edge_data edge_data;
typedef struct heap_data heap_data;
typedef struct list_data list_data;
typedef struct ring_list_data ring_list_data;
typedef struct octree_data octree_data;
typedef struct color_data color_data;
typedef struct vertex_hierarchy_data vertex_hierarchy_data;
//typedef struct edge_data edges;
typedef struct vertex_tree_data vertex_tree_data;

typedef struct view_vertex_data view_vertex_data;
typedef struct view_cluster_data view_cluster_data;
typedef struct view_vertex_tree_data view_vertex_tree_data;
typedef struct view_vertex_tree_link_data view_vertex_tree_link_data;

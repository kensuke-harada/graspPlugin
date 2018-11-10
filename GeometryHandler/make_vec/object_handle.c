#include <stdio.h>
#include <stdlib.h>

#include "set_define.h"
#include "struct.h"
#include "list_handle.h"
#include "tri_ver_math.h"
#include "mymath.h"
#include "mymath_float.h"
//#include "tri_ver_math.h"

//extern triangle_data BOUNDARY;
extern vertex_data   DUMMY_VERTEX,*TMP_VER;


//void make_neighbor_data(triangle_data *t)
	 /* 三角形tの情報を三角形tの各頂点に記録する */
//{
//	int i;

//	add_ver_to_list(&t->ver[0]->ver_list,t->ver[1]);
//	add_ver_to_list(&t->ver[0]->ver_list,t->ver[2]);
//  add_ver_to_list(&t->ver[1]->ver_list,t->ver[0]);
//  add_ver_to_list(&t->ver[1]->ver_list,t->ver[2]);
//  	add_ver_to_list(&t->ver[2]->ver_list,t->ver[0]);
//  	add_ver_to_list(&t->ver[2]->ver_list,t->ver[1]);
//  	add_tri_to_list(&t->ver[0]->tri_list,t);
//  	add_tri_to_list(&t->ver[1]->tri_list,t);
//  	add_tri_to_list(&t->ver[2]->tri_list,t);
//	for(i=0;i<3;i++)
//	if(t->nbr[i] == &BOUNDARY){
//		add_tri_to_list(&t->ver[i]->tri_list,&BOUNDARY);
//		add_tri_to_list(&t->ver[(i+1)%3]->tri_list,&BOUNDARY);
//	}
//}

int search_next_boundary(vertex_data *v0, vertex_data *v1, vertex_data **v2){
	extern triangle_data BOUNDARY;
	list_data *tl;
	triangle_data *t;
	vertex_data *vt1;
	int i,flag;

	vt1 = v0;
	tl = &v1->neighbor_triangle_list;
	while(tl->next != NULL){
		flag=0;
		t = (triangle_data*)tl->next->element;
		for(i=0;i<3;i++){
			if(t->ver[i]==vt1 && t->ver[(i+1)%3]==v1){
				vt1 = t->ver[(i+2)%3];
				tl = &v1->neighbor_triangle_list;
				flag=1;
				if(t->nbr[(i+1)%3] == &BOUNDARY){
					*v2 = vt1;
					return 1;
				}
			}
		}
		if(flag==0){
			tl = tl->next;
		}
	}
	return -2;
}

void make_nbr(object_data *o){
	int i,j,l,flag,cnt;
	list_data *tl;
	triangle_data *t;
	extern triangle_data BOUNDARY;

	for (i=0; i<o->num_of_ver;i++){
		o->vertices[i].neighbor_triangle_list.next = NULL;
	}
	

	for (i=0; i<o->num_of_all_tri;i++){
		add_element_to_list(&o->triangles[i].ver[0]->neighbor_triangle_list,&o->triangles[i]);
		add_element_to_list(&o->triangles[i].ver[1]->neighbor_triangle_list,&o->triangles[i]);
		add_element_to_list(&o->triangles[i].ver[2]->neighbor_triangle_list,&o->triangles[i]);
		o->triangles[i].nbr[0] = o->triangles[i].nbr[1] = o->triangles[i].nbr[2] = &BOUNDARY;
	}

	for(i=0;i<o->num_of_all_tri;i++){
//		o->triangles[i].check = DEAD;
		cnt = 0;
		for(j=0;j<3;j++){
			flag = 0;
			tl = &o->triangles[i].ver[j]->neighbor_triangle_list;
			while (tl->next != NULL){
				t = (triangle_data*)tl->next->element;
				for(l=0;l<3;l++)
				if(o->triangles[i].ver[j] == t->ver[l] && &o->triangles[i] != t){
					if(o->triangles[i].ver[(j+1)%3] == t->ver[(l+2)%3]){
						o->triangles[i].nbr[j] = t;
						flag++;
					}
				}
				tl = tl->next;
			}
			if(flag == 0){
				o->triangles[i].nbr[j] = &BOUNDARY;
//				printf("nbr error%d %d\n",flag,i);
//				cnt++;
//				o->triangles[i].check = ALIVE;
			}
			if(flag > 1){
				o->triangles[i].nbr[j] = &BOUNDARY;
				printf("nbr error%d %d\n",flag,i);

				tl = &o->triangles[i].ver[j]->neighbor_triangle_list;
				while (tl->next != NULL){
					t = (triangle_data*)tl->next->element;
					for(l=0;l<3;l++)
						if(o->triangles[i].ver[j] == t->ver[l] && &o->triangles[i] != t){
							if(o->triangles[i].ver[(j+1)%3] == t->ver[(l+2)%3]){
								o->triangles[i].nbr[j] = t;
								flag++;
								printf("%d %d\n",i,t->number);
								printf("t %d ",o->triangles[i].ver[0]->number);
								printf("%d ",o->triangles[i].ver[1]->number);
								printf("%d ",o->triangles[i].ver[2]->number);
								printf("tn %d ",t->ver[0]->number);
								printf("tn %d ",t->ver[1]->number);
								printf("tn %d \n",t->ver[2]->number);
							}
						}
						tl = tl->next;
				}
			}

		}
	}
	printf("make nbr\n");

}

#ifdef MAKE_CLUSTER_HIERARCHY
void make_closed_check(object_data *o){
	int i,j,k,flag,cnt;
	list_data *tl;
	triangle_data *t;
	vertex_data *v,*v0,*v1,*v2,*vi0,*vi1,*vi2;
	extern triangle_data BOUNDARY;
	struct list_data *l;

	for (i=0; i<o->num_of_ver;i++){
		o->vertices[i].closed_check = 0;
	}
	

	for(i=0;i<o->num_of_all_tri;i++){
		for(j=0;j<3;j++){
			if(o->triangles[i].nbr[j] == &BOUNDARY || o->triangles[i].nbr[j] == NULL){
				o->triangles[i].ver[j]->closed_check++;
				o->triangles[i].ver[(j+1)%3]->closed_check++;
			}
		}
	}

//	for (i=0; i<o->num_of_all_tri;i++){
//		add_element_to_list(&o->triangles[i].ver[0]->neighbor_triangle_list,&o->triangles[i]);
//		add_element_to_list(&o->triangles[i].ver[1]->neighbor_triangle_list,&o->triangles[i]);
//		add_element_to_list(&o->triangles[i].ver[2]->neighbor_triangle_list,&o->triangles[i]);
//	}

	for (i=0; i<o->num_of_ver;i++){
		if(o->vertices[i].closed_check > 2){
			printf("not closed loop %d,%d\n", i,o->vertices[i].closed_check);
			l = &o->vertices[i].neighbor_triangle_list;
			t = (triangle_data*)load_list_element_and_next_pointer(&l);
			while(t!=NULL){
				for(j=0;j<3;j++){
					for(k=0;k<3;k++){
						if(t->nbr[j]->nbr[k] == t) t->nbr[j] = &BOUNDARY;
					}
					t->nbr[j] = &BOUNDARY;
				}
				t = (triangle_data*)load_list_element_and_next_pointer(&l);
			}
		}
	}

}
#endif

#ifndef FLOAT_VERTEX
void make_normal_of_vertices(object_data *o){
	int i,j;
	triangle_data *t;
	double vec[3];
	
	
	for(i=0;i<o->num_of_ver;i++){
		o->vertices[i].normal[0] = o->vertices[i].normal[1] = o->vertices[i].normal[2] = 0; 
	}
	for(i=0;i<o->num_of_all_tri;i++){
		t = &o->triangles[i];
		outer_product_3_points(vec,t->ver[0]->pos,t->ver[1]->pos,t->ver[2]->pos);
		for(j=0;j<3;j++){
			add_2_vector(t->ver[j]->normal,vec,t->ver[j]->normal);
		}
	}
	for(i=0;i<o->num_of_ver;i++){
		make_unit_vector(o->vertices[i].normal);
	}

}
#else
void make_normal_of_vertices(object_data *o){
	int i,j;
	triangle_data *t;
	float vec[3];
	
	
	for(i=0;i<o->num_of_ver;i++){
		o->vertices[i].normal[0] = o->vertices[i].normal[1] = o->vertices[i].normal[2] = 0; 
	}
	for(i=0;i<o->num_of_all_tri;i++){
		t = &o->triangles[i];
		outer_product_3_points_float(vec,t->ver[0]->pos,t->ver[1]->pos,t->ver[2]->pos);
		for(j=0;j<3;j++){
			add_2_vector_float(t->ver[j]->normal,vec,t->ver[j]->normal);
		}
	}
	for(i=0;i<o->num_of_ver;i++){
		make_unit_vector_float(o->vertices[i].normal);
	}

}
#endif

void initial_ver_tri_data(object_data *o){
	int i;
	extern triangle_data BOUNDARY;
	
	for(i=0;i<o->num_of_ver;i++){
		o->vertices[i].number = i;
		o->vertices[i].neighbor_triangle_list.next = NULL;
//		o->vertices[i].neighbor_vertex_list = NULL;
		o->vertices[i].check = DEAD;
#ifdef MAKE_CLUSTER_HIERARCHY
		o->vertices[i].cluster_mark = -1;
		o->vertices[i].mark = -1;
		o->vertices[i].Nvtalias = 0;
#endif
	}
	for (i=0; i<o->num_of_all_tri;i++){
		add_element_to_list(&o->triangles[i].ver[0]->neighbor_triangle_list,&o->triangles[i]);
		add_element_to_list(&o->triangles[i].ver[1]->neighbor_triangle_list,&o->triangles[i]);
		add_element_to_list(&o->triangles[i].ver[2]->neighbor_triangle_list,&o->triangles[i]);
	}

	
	for (i=0; i<o->num_of_all_tri;i++){
		o->triangles[i].number = i;
//		o->triangles[i].check = DEAD;
#ifdef MAKE_CLUSTER_HIERARCHY
		o->triangles[i].cluster_number = 0;
		make_triangles_area_and_normal(&o->triangles[i].area,o->triangles[i].normal,&o->triangles[i]);
		make_gravity_center_of_triangle(o->triangles[i].center,&o->triangles[i]);
		make_area_quadrics(o->triangles[i].area_quadrics,&o->triangles[i]);
#endif

#ifdef TRIANGLE_HIERARCHY
		o->triangles[i].inner_product = -1;
#endif
#ifdef MAKE_TRIANGLE_HIERARCHY
		o->triangles[i].mark = -1;
		o->triangles[i].triangle_posterity.next = NULL;
		o->triangles[i].children[0] = &BOUNDARY;
		o->triangles[i].children[1] = &BOUNDARY;
		o->triangles[i].parent= &BOUNDARY;
#endif

	}
	if(o->nbr_status==DEAD){
		make_nbr(o);
		o->nbr_status = ALIVE;
	}
	if(o->vertex_normal_status == DEAD){
//#ifndef FLOAT_VERTEX
		make_normal_of_vertices(o);
//#endif
	}


//	BOUNDARY.check = DEAD;
	BOUNDARY.cluster_number = -1;
#ifdef MAKE_TRIANGLE_HIERARCHY
	BOUNDARY.cluster = NULL;
	BOUNDARY.parent = &BOUNDARY;
	BOUNDARY.nbr[0] = &BOUNDARY;
	BOUNDARY.nbr[1] = &BOUNDARY;
	BOUNDARY.nbr[2] = &BOUNDARY;
#endif
}

#ifdef MAKE_CLUSTER_HIERARCHY
void
initial_cluster_data(object_data *o){
	int i,j;
	int nt;

	nt = o->num_of_all_tri;
//	o->num_of_all_tri = 1000;
//	o->hFileMap = CreateFileMapping( (HANDLE)0xFFFFFFFF, NULL, PAGE_READWRITE, 0, sizeof(cluster_data)*o->num_of_all_tri*2 , "test") ;
//	if(o->hFileMap==NULL) exit(-1);
//	o->clusters = (cluster_data *)MapViewOfFile( o->hFileMap, FILE_MAP_WRITE, 0, 0, 0 );


	o->clusters = (cluster_data *)malloc(sizeof(cluster_data)*o->num_of_all_tri*2);
//	o->tree_vertices = (vertex_tree_data *)malloc(sizeof(vertex_tree_data)*o->num_of_all_tri*3);

	o->number_of_cluster=0;
	for(i=0;i<o->num_of_ver;i++){
		o->vertices[i].bothcluster = NULL;
		o->vertices[i].vtalias = NULL;
		o->vertices[i].mark = -1;
	}	
	for(i=0;i<o->num_of_all_tri*2;i++){
		for(j=0;j<3;j++){
			o->clusters[i].center[j] = 0;
//			o->clusters[i].normal_average[j] = 0;
		}
		o->clusters[i].area = 0;
		o->clusters[i].number_of_vertex = 0;
		o->clusters[i].number_of_triangle = 0;
		o->clusters[i].parent = NULL;
		o->clusters[i].children[0] = NULL;
		o->clusters[i].children[1] = NULL;
	}
	printf("closed_check\n");

//	o->num_of_all_tri = nt;
	make_closed_check(o);


}
#endif

void 
make_unit_10000mm_data(object_data *o){
	int i,j;
	double size;


	for(j=0;j<3;j++){
		o->data_range_high[j] = MIN_DOUBLE;
		o->data_range_low[j] =  MAX_DOUBLE;
	}
	for(i=0;i<o->num_of_ver;i++){
		o->vertices[i].number = i;
		for(j=0;j<3;j++){
			if(o->data_range_high[j] < o->vertices[i].pos[j])
				o->data_range_high[j] = o->vertices[i].pos[j];
			if(o->data_range_low[j] > o->vertices[i].pos[j])
				o->data_range_low[j] = o->vertices[i].pos[j];
		}
	}
	size = 0;
	for(i=0;i<3;i++){
		if(size < (o->data_range_high[i] - o->data_range_low[i]))
			size = o->data_range_high[i] - o->data_range_low[i];
	}
	printf("size\t%lf\n",size);

	//	for(i=0;i<o->num_of_ver;i++){
	//for(j=0;j<3;j++){
	//	o->vertices[i].pos[j] -= (o->data_range_high[j]+o->data_range_low[j])/2.0;
									//o->data_range_low[j];
	  //o->vertices[i].pos[j] *= 1000/size;
	  //}
	 //}
    for(i=0;i<o->num_of_ver;i++){
        o->vertices[i].number = i;
    }
	make_nbr(o);
}

void
make_unit_objects(object_data *o,int number_of_object){
	int i,j,k;
	double size;
    double data_range_high[3];
    double data_range_low[3];

    
	for(j=0;j<3;j++){
		data_range_high[j] = MIN_DOUBLE;
		data_range_low[j] =  MAX_DOUBLE;
	}
    for(k=0;k<number_of_object;k++){
        for(i=0;i<o[k].num_of_ver;i++){
            for(j=0;j<3;j++){
                if(data_range_high[j] < o[k].vertices[i].pos[j])
                    data_range_high[j] = o[k].vertices[i].pos[j];
                if(data_range_low[j] > o[k].vertices[i].pos[j])
                    data_range_low[j] = o[k].vertices[i].pos[j];
            }
        }
    }
    printf("\n");
	for(j=0;j<3;j++){
            printf("range           %12lf %12lf\n",data_range_high[j],data_range_low[j]);
    }
    printf("\n");


	size = 0;
	for(i=0;i<3;i++){
		if(size < (data_range_high[i] - data_range_low[i]))
			size = data_range_high[i] - data_range_low[i];
	}

    for(k=0;k<number_of_object;k++){
        for(i=0;i<o[k].num_of_ver;i++){
            for(j=0;j<3;j++){
                o[k].vertices[i].pos[j] -= data_range_low[j];
                o[k].vertices[i].pos[j] /= size;
            }
        }
    }
}

void
make_unit_2object(object_data *bo,object_data *wo,int number_of_object){
	int i,j,k=0;
	double size;
    double data_range_high[3];
    double data_range_low[3];
	object_data *o;

    
	for(j=0;j<3;j++){
		data_range_high[j] = MIN_DOUBLE;
		data_range_low[j] =  MAX_DOUBLE;
	}
	o = bo;
	for(i=0;i<o[k].num_of_ver;i++){
		for(j=0;j<3;j++){
			if(data_range_high[j] < o[k].vertices[i].pos[j])
				data_range_high[j] = o[k].vertices[i].pos[j];
			if(data_range_low[j] > o[k].vertices[i].pos[j])
				data_range_low[j] = o[k].vertices[i].pos[j];
		}
	}
	o = wo;
	for(i=0;i<o[k].num_of_ver;i++){
		for(j=0;j<3;j++){
			if(data_range_high[j] < o[k].vertices[i].pos[j])
				data_range_high[j] = o[k].vertices[i].pos[j];
			if(data_range_low[j] > o[k].vertices[i].pos[j])
				data_range_low[j] = o[k].vertices[i].pos[j];
		}
	}
    printf("\n");
	for(j=0;j<3;j++){
            printf("range           %12lf %12lf\n",data_range_high[j],data_range_low[j]);
    }
    printf("\n");


	size = 0;
	for(i=0;i<3;i++){
		if(size < (data_range_high[i] - data_range_low[i]))
			size = data_range_high[i] - data_range_low[i];
	}

	o = bo;
	for(i=0;i<o[k].num_of_ver;i++){
		for(j=0;j<3;j++){
			o[k].vertices[i].pos[j] -= (data_range_high[j]+data_range_low[j])/2.0;
			o[k].vertices[i].pos[j] /= size;
		}
	}
	o = wo;
	for(i=0;i<o[k].num_of_ver;i++){
		for(j=0;j<3;j++){
			o[k].vertices[i].pos[j] -= (data_range_high[j]+data_range_low[j])/2.0;
			o[k].vertices[i].pos[j] /= size;
		}
	}
}


void
magnification(object_data *o,double m){
    int i,j;

	for(i=0;i<o->num_of_ver;i++){
		for(j=0;j<3;j++){
			o->vertices[i].pos[j] *= m;
		}
	}
}

void 
initial_data_range(object_data *o){
	int i,j;

	for(j=0;j<3;j++){
		o->data_range_high[j] = MIN_DOUBLE;
		o->data_range_low[j] = MAX_DOUBLE;
	}
	for(j=0;j<3;j++){
		for(i=0;i<o->num_of_ver;i++){
			if(o->data_range_high[j] < o->vertices[i].pos[j])
				o->data_range_high[j] = o->vertices[i].pos[j];
			if(o->data_range_low[j] > o->vertices[i].pos[j])
				o->data_range_low[j] = o->vertices[i].pos[j];
		}
		printf("data_range[%d] %lf %lf\n",j,o->data_range_high[j],o->data_range_low[j]);
	}
	for(i=0;i<o->num_of_ver;i++){
		for(j=0;j<3;j++){
		  //			o->vertices[i].pos[j] -= (o->data_range_high[j]+o->data_range_low[j])/2;
		}
	}
}

void initial(object_data *o){
	initial_data_range(o);
	initial_ver_tri_data(o);
#ifdef MAKE_CLUSTER_HIERARCHY
	initial_cluster_data(o);
//	make_closed_check(o);
#endif
}

void initial2(object_data *o){
	initial_data_range(o);
	initial_ver_tri_data(o);
	make_nbr(o);
}


#ifdef FLOAT_VERTEX
void parallel_transform(object_data *o,float t[3]){
    int i;

	for(i=0;i<o->num_of_ver;i++){
		add_2_vector_float(o->vertices[i].pos,o->vertices[i].pos,t);
	}
}
#else
void parallel_transform(object_data *o,double t[3]){
    int i;

	for(i=0;i<o->num_of_ver;i++){
		add_2_vector(o->vertices[i].pos,o->vertices[i].pos,t);
	}
}
#endif

#ifdef FLOAT_VERTEX
void parallel_transform_view(object_data *o,float t[3]){
    int i;

	for(i=0;i<o->num_of_ver;i++){
		add_2_vector_float(o->view_verticies[i].pos,o->view_verticies[i].pos,t);
	}
	for(i=0;i<o->number_of_cluster;i++){
		add_2_vector_float(o->view_clusters[i].center,o->view_clusters[i].center,t);
	}
}
#endif

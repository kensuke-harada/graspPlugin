#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "set_define.h"
#include "struct.h"
#include "mymath.h"
#include "list_handle.h"
/* search-node-function */

//static FILE *FP;

//vertex_tree_data DUMMY_TREEVERTEX;

/*************************************************************************************/


void output_one_cluster_data(FILE *fp,cluster_data *c){
	list_data *l;
	vertex_tree_data *vt;


	if(c->parent == NULL)	fprintf(fp,"-1");
	else fprintf(fp,"%d",c->parent->number);
	if(c->children[0] == NULL)	fprintf(fp,"\t-1 \t-1");
	else fprintf(fp,"\t%d \t%d",c->children[0]->number,c->children[1]->number);


	fprintf(fp,"\t%d",c->number_of_vertex);
	l = &c->polygons;
	if(l != NULL){
		vt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
		while(vt != NULL){
			fprintf(fp,"\t%d",vt->number);
			vt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
		}
	}
	fprintf(fp,"\t%d",c->Ncluster_ver_link);
	l = &c->vertexlink;
	if(l != NULL){
		vt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
		while(vt != NULL){
			fprintf(fp,"\t%d",vt->number);
			vt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
		}
	}

	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\t%lf\t%lf\t%lf",c->center[0],c->center[1],c->center[2]);

	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\t%lf\t%lf\t%lf",c->normal[0],c->normal[1],c->normal[2]);

	fprintf(fp,"\t%lf",c->area);
	fprintf(fp,"\t%lf",c->quadrics_error);
	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\n");
}


void output_one_cluster_data_trivec(FILE *fp,cluster_data *c){
	list_data *l;
	vertex_tree_data *vt;

	if(c->parent == NULL)	fprintf(fp,"-1");
	else fprintf(fp,"%d",c->parent->number);
	if(c->children[0] == NULL)	fprintf(fp,"\t-1 \t-1");
	else fprintf(fp,"\t%d \t%d",c->children[0]->number,c->children[1]->number);

	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\t%lf\t%lf\t%lf",c->center[0],c->center[1],c->center[2]);

	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\t%lf\t%lf\t%lf",c->normal[0],c->normal[1],c->normal[2]);

	fprintf(fp,"\t%lf",c->area);
	fprintf(fp,"\t%lf",c->quadrics_error);
	fprintf(fp,"\t%lf",-1.0);
	fprintf(fp,"\n");
}


void output_one_tree_vertex_data(FILE *fp,vertex_tree_data *vt){

	fprintf(fp,"%d",vt->vertex->number);
	fprintf(fp,"\t%d",vt->boundary_order);
	fprintf(fp,"\t%d",(vt->parent) ? vt->parent->number : -1);
	fprintf(fp,"\t%d",(vt->reverse_parent) ? vt->reverse_parent->number : -1);
	fprintf(fp,"\t%d",(vt->vtalias) ? vt->vtalias->number : -1);
	fprintf(fp,"\n");

}

void
write_vec_data(char *file,object_data *o){
    FILE *fp;
	int i,j,cnt;
	list_data *l;
	vertex_tree_data *vt,*tvt;
	int Npolygon=0;
	int Nvertexlink=0;

	printf("edge num%d\n",o->Nedge);

	if ((fp = fopen(file,"wb")) == NULL){
	  printf("Cannot open write vec file!\n");
	  exit(-1);
	}

	vt = o->tree_vertices; 

	for (i=0; i<o->num_of_ver; i++){
		o->vertices[i].number = i;
	}
	for(i=0;i<o->Ntree_verticies;i++){
		vt[i].number = i;
	}
	for(i=0;i<o->Nedge;i++){
		for(j=0;j<2;j++){
			if(o->edges[i].vtop[j] == NULL) continue;
			cnt=0;
			if(o->edges[i].loop == 1) cnt++;
			o->edges[i].vtop[j]->boundary_order=cnt++;
			tvt= o->edges[i].vtop[j]->next;
			while(tvt!=NULL && o->edges[i].vtop[j] != tvt){
				tvt->boundary_order = cnt++;
				tvt = tvt->next;
			}
		}
	}

	for(i=0;i<o->Ntree_verticies;i++){
		if(vt[i].vtalias == NULL){
			vt[i].reverse_parent = NULL;
			continue;
		}
		if(vt[i].reverse_parent!=NULL){
			vt[i].reverse_parent = vt[i].reverse_parent->vtalias;
			if(vt[i].reverse_parent != NULL && vt[i].vtalias !=NULL && vt[i].vtalias->boundary_order < vt[i].reverse_parent->boundary_order){
				vt[i].reverse_parent = vt[i].edge->vtop[(vt[i].onecluster->number+1)%2];
			}
			if(vt[i].reverse_parent && vt[i].reverse_parent->onecluster == vt[i].onecluster){
				printf("reverse error %d\n",i);
				vt[i].reverse_parent = NULL;
			}
		}
//		if(vt[i].reverse_parent==NULL || vt[i].edge->loop == 1){
		if(vt[i].reverse_parent==NULL){
			vt[i].reverse_parent = vt[i].edge->vtop[(vt[i].onecluster->number+1)%2];
		}
//		vt[i].parent = vt[i].edge->vtop[(vt[i].onecluster->number)%2];/////////////////////////////////////////
//		vt[i].reverse_parent = vt[i].edge->vtop[(vt[i].onecluster->number+1)%2];//////////////////////////////////

		if(vt[i].reverse_parent == vt[i].vtalias || vt[i].vtalias == NULL || vt[i].reverse_parent == NULL || vt[i].edge != vt[i].vtalias->edge){
			vt[i].reverse_parent = vt[i].vtalias = NULL;
		}
	}
	for(i=0;i<o->Nedge;i++){
		if(o->edges[i].vtop[0]!=NULL && o->edges[i].vtop[1]!=NULL){
			o->edges[i].vtop[0]->reverse_parent = o->edges[i].vtop[1];
			o->edges[i].vtop[1]->reverse_parent = o->edges[i].vtop[0];
			if(o->edges[i].vtop[0]->vtalias !=NULL && o->edges[i].vtop[1]->vtalias != NULL && o->edges[i].vtop[0]->vtalias != o->edges[i].vtop[1]){
				o->edges[i].vtop[0]->vtalias->vtalias = o->edges[i].vtop[1]->vtalias->vtalias = NULL;
			}
		}
	}

	for(i=0;i<o->num_of_ver;i++){
		o->vertices[i].check = 0;
	}
	for(i=0;i<o->Ntree_verticies;i++){
		if(vt[i].boundary_order==0) continue;
		vt[i].vertex->check++;
	}

	for(i=0;i<o->num_of_ver;i++){
		if(o->vertices[i].check > 2) printf("Too much %d %d\n",i,o->vertices[i].check);
	}

	for(i=0;i<o->Ntree_verticies;i++){
		vt[i].number = -1;
	}

	for(i=0;i<o->number_of_cluster;i++){
		Npolygon += o->clusters[i].number_of_vertex;
		Nvertexlink += o->clusters[i].Ncluster_ver_link;
	}

	cnt=0;
	for(i=0;i<o->number_of_cluster;i++){
		l = &o->clusters[i].vertexlink;
		if(l != NULL){
			tvt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
			while(tvt != NULL){
				tvt->number=cnt++;
				tvt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
			}
		}
	}
	for(i=0;i<o->Ntree_verticies;i++){
		if(vt[i].number != -1) continue;
		if(vt[i].boundary_order == 0) continue;
		vt[i].number=cnt++;
		Nvertexlink++;
	}
	for(i=0;i<o->Ntree_verticies;i++){
		if(vt[i].number != -1) continue;
		vt[i].number=cnt++;
	}

	fprintf(fp,"%d %d %d %d %d\n",o->num_of_ver,o->Ntree_verticies,o->number_of_cluster,Npolygon,Nvertexlink);

	for (i=0; i<o->num_of_ver; i++){
		fprintf(
			fp,"%lf %lf %lf %lf %lf %lf ",
			o->vertices[i].pos[0],
			o->vertices[i].pos[1],
			o->vertices[i].pos[2],
			o->vertices[i].normal[0],
			o->vertices[i].normal[1],
			o->vertices[i].normal[2]
			);
		fprintf(fp,"\t%d" ,(o->vertices[i].bothcluster) ? o->vertices[i].bothcluster->number : -1);
		fprintf(fp,"\t%d" ,o->vertices[i].ring_number);
		fprintf(fp,"\n");
	}

	cnt=0;
	for(i=0;i<o->number_of_cluster;i++){
		l = &o->clusters[i].vertexlink;
		if(l != NULL){
			tvt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
			while(tvt != NULL){
				output_one_tree_vertex_data(fp,tvt);
				cnt++;
				tvt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
			}
		}
	}
	for(i=0;i<o->Ntree_verticies;i++){
		if(vt[i].number != cnt) continue;
		output_one_tree_vertex_data(fp,&vt[i]);
		cnt++;
	}
	for(i=0;i<o->Ntree_verticies;i++){
		if(vt[i].number != cnt) continue;
		output_one_tree_vertex_data(fp,&vt[i]);
		cnt++;
	}

	for (i=0; i<o->number_of_cluster; i++) output_one_cluster_data(fp,&o->clusters[i]);

	fclose(fp);
	fprintf(stderr,"Vec file was written\n");
}


void
write_trivec_data(char *file,object_data *o){
    FILE *fp;
	int i,j,cnt;
	list_data *l;
	vertex_tree_data *vt,*tvt;
	int Npolygon=0;
	int Nvertexlink=0;

	printf("edge num%d\n",o->Nedge);

	if ((fp = fopen(file,"wb")) == NULL){
	  printf("Cannot open write vec file!\n");
	  exit(-1);
	}

	fprintf(fp,"%d %d %d\n",o->num_of_ver,o->num_of_all_tri,o->number_of_cluster);

	for (i=0; i<o->num_of_ver; i++){
		fprintf(
			fp,"%lf %lf %lf %lf %lf %lf ",
			o->vertices[i].pos[0],
			o->vertices[i].pos[1],
			o->vertices[i].pos[2],
			o->vertices[i].normal[0],
			o->vertices[i].normal[1],
			o->vertices[i].normal[2]
			);
		fprintf(fp,"\t%d" ,(o->vertices[i].bothcluster) ? o->vertices[i].bothcluster->number : -1);
		fprintf(fp,"\t%d" ,o->vertices[i].ring_number);
		fprintf(fp,"\n");
	}

	for (i=0;i<o->num_of_all_tri;i++){
			fprintf(fp,"%d %d %d %d\n",
				o->triangles[i].ver[0]->number,
				o->triangles[i].ver[1]->number,
				o->triangles[i].ver[2]->number,
				o->triangles[i].cluster_number
			);
	}

	for (i=0; i<o->number_of_cluster; i++) output_one_cluster_data_trivec(fp,&o->clusters[i]);

	fclose(fp);
	fprintf(stderr,"Vec file was written\n");
}


int search_start_vertices_of_cluster_boundary(vertex_data **v0,vertex_data **v1, int cn, cluster_data *c){
	int i,j;
	triangle_data *tn;
	extern triangle_data BOUNDARY;

	for(i=0;i<c->number_of_triangle;i++){
		for(j=0;j<3;j++){
			tn = c->triangles_que[i]->nbr[j];
			if(tn->cluster_number == cn) continue;
			if(c->triangles_que[i]->ver[j]->mark == cn) continue;
			*v0 = c->triangles_que[i]->ver[j];
			*v1 = c->triangles_que[i]->ver[(j+1)%3];
			if(tn == &BOUNDARY) return -1;
			return tn->cluster_number;
		}
	}
	return -2;
}

int count_cluster_boundary(cluster_data *c){
	int i,j,cn,cnt=0;
	triangle_data *tn;
	extern triangle_data BOUNDARY;

	cn = c->number;
	for(i=0;i<c->number_of_triangle;i++){
		for(j=0;j<3;j++){
			tn = c->triangles_que[i]->nbr[j];
			if(tn->cluster_number == cn) continue;
			cnt++;
		}
	}
	return cnt;
}


int search_next_vertices_of_boundary(vertex_data *v0, vertex_data *v1, vertex_data **v2, int cn){
	list_data *tl;
	triangle_data *t;
	vertex_data *vt1;
	int i,flag;
	extern triangle_data BOUNDARY;

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
				if(t->nbr[(i+1)%3]->cluster_number != cn){
					*v2 = vt1;
					if(t->nbr[(i+1)%3] == &BOUNDARY) return -1;
					return t->nbr[(i+1)%3]->cluster_number;
				}
			}
		}
		if(flag==0){
			tl = tl->next;
		}
	}
	return -2;
}



vertex_tree_data * malloc_vertex_tree_data(vertex_data *v){
	extern object_data OBJECT;
	static int first = 0;
	vertex_tree_data *nvt;

	if(first == 0){
		first = 1;
		OBJECT.Ntree_verticies = 0;
		OBJECT.tree_vertices = (vertex_tree_data *)malloc(OBJECT.num_of_all_tri*3*sizeof(vertex_tree_data));
		if( OBJECT.tree_vertices == NULL){
			fprintf(stderr, "tree_vertices cannot be allocated\n");
			exit(0);
		}
	}
	nvt = &OBJECT.tree_vertices[OBJECT.Ntree_verticies++];
	nvt->vertex = v;
	nvt->bothcluster = v->bothcluster;
	nvt->parent = NULL;
	nvt->reverse_parent = NULL;
	nvt->vtalias = NULL;
	nvt->boundary_order = 0;
	nvt->onecluster = NULL;
	nvt->parentcluster = NULL;
	nvt->edge = v->edge;
	nvt->next = NULL;
	nvt->number = OBJECT.Ntree_verticies-1;
	return nvt;
}

edge_data * malloc_edge_data(){
	static int first=0;
	extern object_data OBJECT;
	edge_data *e;

	if(first==0){
		first=1;
		e = OBJECT.edges = (edge_data *)malloc(sizeof(edge_data)*OBJECT.num_of_ver);
		OBJECT.Nedge = 0;
	}
	e = &OBJECT.edges[OBJECT.Nedge++];
	e->vtop[0] = e->vtop[1] = NULL;
	e->loop = 0;
	e->number = OBJECT.Nedge-1;
	return e;
}

int plus_minus_zero(int a){
	if(a > 0) return 1;
	if(a < 0) return -1;
	return 0;
}

void add_vertex_to_edge(vertex_tree_data *vt){
	edge_data *e;
	cluster_data *tc,*bc;
	int cn,a,b,c,sign;
	vertex_tree_data *v1,*v2;

	e =vt->edge;
	tc = vt->onecluster;
	bc = vt->bothcluster;
	if(bc == NULL) return;

	while(tc->parent != NULL && tc->parent != bc) tc = tc->parent;
	vt->onecluster = tc;
	cn = tc->number % 2;

	if(e->vtop[cn] == NULL){
		e->vtop[cn] = vt;
		vt->parent = vt;
		if(vt->boundary_order != 0) vt->next = vt;
		return;
	}
	if(vt->boundary_order == 0) return;


	if(cn==0) sign = -1;
	else 	sign = 1;

	v1 = e->vtop[cn];

	b = vt->vertex->ring_number;
	if(v1->boundary_order != 0) a = v1->vertex->ring_number;
	else a = -10;

	c = 0;

	while(v1->next){
		c = v1->next->vertex->ring_number;
		if((sign * plus_minus_zero(a - b) * plus_minus_zero(b - c) * plus_minus_zero(c - a)) >= 0) break;
		v1 = v1->next;
		a = c;
	}

	if(v1->vertex == vt->vertex || (v1->next && vt->vertex == v1->next->vertex)){
		printf("onaji%d ",(vt->edge) ? vt->edge->number : -1);
		return;
	}

	v2 = v1->next;
	v1->next = vt;
	vt->next = v2;

	vt->parent = v1;

	while(v2){
		if(v2 == v1) break;
		tc = vt->parentcluster;
		while(tc){
			if(tc == v2->parentcluster){
				vt->reverse_parent = v2;
				return;
			}
			tc = tc->parent;
		}
		v2 = v2->next;
	}
	vt->reverse_parent = NULL;
}

void make_polygon_of_cluster2(cluster_data *c, cluster_data *bc, cluster_data *c2){
	extern object_data OBJECT;
	int i,flag,sign=1;
	vertex_data *v0,*v1,*vi0,*vi1,*vi2,*nv0,*nv1;
	int cn,tcnt,cnn,bcnn,tb,c2n;
	int cnt = 0,cnt2=0; 
	int ring_number = 1;
	vertex_tree_data *vt,*tvt;
	list_data *l;
	static int first = 0;
	edge_data *e;
	int NBoundary;


	c->number_of_vertex = 0;
	c->Ncluster_ver_link = 0;
	c->polygons.next = NULL;
	c->vertexlink.next = NULL;


	NBoundary = count_cluster_boundary(c);

	l = &bc->polygons;
	tvt = NULL;
	if(l != NULL){
		vt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
		while(vt != NULL){
			vt->vertex->cluster_mark = bc->number;
			vt->vertex->treevertex = vt;
			vt = (vertex_tree_data*)load_list_element_and_next_pointer(&l);
		}
	}
	tvt = vt;

	cn = c->number;
	c2n = c2->number;

	tcnt=0;
	
	

	do{
		tb = bcnn = search_start_vertices_of_cluster_boundary(&v0,&v1,cn,c);
		if(bcnn < -1){
			fprintf(stderr,"Can't find boundary vetices of cluster ? %d\n",cn);
			break;
		}
		
		vi0 = v0;
		vi1 = v1;
		
		flag = 0;

		ring_number = 1;
		do{
			cnn = search_next_vertices_of_boundary(vi0,vi1,&vi2,cn);

			vi1->mark = cn;
			tcnt++;

			if(vi1->check == DEAD) ring_number++;
			if(bcnn != cnn && cnn == c2n){
				if(flag == 0 || bcnn != -1){
					flag = 1;
					nv0 = vi0;
					nv1 = vi1;
					tb = bcnn;
				}
				flag = 1;
			}
			vi0 = vi1;
			vi1 = vi2;
			bcnn = cnn;
		}while(vi0 != v0 && vi1 != v1);
		
		if(flag == 1){
			vi0 = v0 = nv0;
			vi1 = v1 = nv1;
			bcnn = tb;
		}

		if(cn%2) ring_number=1;
		flag = 1;
		cnt= cnt2= 0;
		do{
			cnt++;

			bcnn = search_next_vertices_of_boundary(vi0,vi1,&vi2,cn);
			if(vi1->check == ALIVE){
				flag = 1;
			}
			if(vi1->check == DEAD){
				cnt2++;

				if(flag==1) e =malloc_edge_data();
				flag = 0;

				vi1->check = ALIVE;
				vi1->bothcluster = bc;
				vi1->ring_number = (cn%2) ? ring_number++ : --ring_number;
				vi1->edge = e;
			}
			vi0 = vi1;
			vi1 = vi2;
		}while(vi0 != v0 && vi1 != v1);

		if(cnt2 > 0 && cnt==cnt2){
			e->loop = 1;
			printf("loop %d %d %d\n",e->number,bc->number,c->number);
		}

		do{
			vt = NULL;
			cnn = search_next_vertices_of_boundary(vi0,vi1,&vi2,cn);

			if(vi1->cluster_mark == bc->number) vt = vi1->treevertex;

			if( bcnn != cnn && (bcnn == c2n || cnn == c2n) ){
				if(vi1->cluster_mark == cn) printf("onevertexloop %d %d %d\n",cn,vi1->number,(vi1->bothcluster) ? vi1->bothcluster->number : -1);
				if(vt == NULL){
					if(bcnn==c2n){
						vt = malloc_vertex_tree_data(vi1);
						
						add_any_element_to_list(&bc->vertexlink,vt);
						bc->Ncluster_ver_link++;
						
						vt->boundary_order = 1;
						vt->onecluster = c;
						vt->parentcluster = bc;
						
						add_vertex_to_edge(vt);

						vt->reverse_parent = NULL;

						if(vi1->vtalias == NULL){
							vi1->vtalias = vt;
							vi1->Nvtalias = 1;
						}
						else{
							if(vi1->Nvtalias == 1){ 
								if(vt->onecluster != vi1->vtalias->onecluster){
									vt->vtalias = vi1->vtalias;
									vi1->vtalias->vtalias = vt;
									vi1->Nvtalias = 2;
								}
							}
						}
					}
				}
				
				if(cnn == c2n){
					vt = malloc_vertex_tree_data(vi1);
					vt->bothcluster = bc;
					vt->onecluster = c;
					vt->parentcluster = bc;
					if(vi2->bothcluster == bc ){
						vt->edge = vi2->edge;
						add_vertex_to_edge(vt);
					}
				}
			}
			if(vt != NULL){
				add_any_element_to_list(&c->polygons,vt);
				c->number_of_vertex++;
				vi1->cluster_mark = cn;
				vi1->treevertex = vt;

			}
			vi0 = vi1;
			vi1 = vi2;
			bcnn = cnn;
		}while(vi0 != v0 && vi1 != v1);
	}while(tcnt < NBoundary);

	if(tcnt > NBoundary) printf("tigau %d %d v %d b %d\n",bc->number,c->number,tcnt,NBoundary);


	if(c->number_of_triangle == 1 && c->number_of_vertex < 3){
		for(i=0;i<3;i++){
			vt = NULL;
			vi1 = c->triangles_que[0]->ver[i];
			if(vi1->Nvtalias == 2) continue;
			if(vi1->cluster_mark == bc->number || vi1->cluster_mark == c2n || vi1->cluster_mark == cn) vt = vi1->treevertex;

			if(vt == NULL){
				vt = malloc_vertex_tree_data(vi1);
				vt->boundary_order = 1;
				vt->onecluster = c;
				vt->parentcluster = bc;

				add_vertex_to_edge(vt);

				if(vi1->vtalias == NULL){
					vi1->vtalias = vt;
					vi1->Nvtalias = 1;
				}
				else{
					if(vi1->Nvtalias == 1){ 
						if(vt->onecluster != vi1->vtalias->onecluster){
							vt->vtalias = vi1->vtalias;
							vi1->vtalias->vtalias = vt;
							vi1->Nvtalias = 2;
						}
					}
				}
			}
		}
//		c->polygons.next = NULL;
	}
}

void initial_edge_data(object_data *o){
	int cnt,tcnt,bcnn,ring_number;
	cluster_data *c;
	extern triangle_data BOUNDARY;
	vertex_data *v0,*v1,*vi0,*vi1,*vi2;
	edge_data *e;

	c = &o->clusters[0];
	cnt = count_cluster_boundary(c);
	tcnt = 0;

	if(cnt==0) return;




	do{
		e = malloc_edge_data();
		e->loop = 1;
		bcnn = search_start_vertices_of_cluster_boundary(&v0,&v1,0,c);
		if(bcnn < -1){
			fprintf(stderr,"Can't find boundary vetices of cluster\n");
			exit(0);
		}
		
		vi0 = v0;
		vi1 = v1;
		
		ring_number = 1;
		do{
			search_next_vertices_of_boundary(vi0,vi1,&vi2,0);

			vi1->mark = 0;
			tcnt++;

			if(vi1->check == DEAD){
				vi1->check = ALIVE;
				vi1->bothcluster = NULL;
				vi1->ring_number = ring_number++;
				vi1->edge = e;
			}
			vi0 = vi1;
			vi1 = vi2;
		}while(vi0 != v0 && vi1 != v1);
	}while(tcnt != cnt);
	printf("first loop %d\n",e->number);
	e = malloc_edge_data();



}
	
	

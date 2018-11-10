#include <stdio.h>
#include "set_define.h"
#include "struct.h"
#include "mymath.h"
#include "jacobi.h"
#include "list_handle.h"
#include "cluster2mesh.h" 

int Counter;

void make_polygon_of_cluster(cluster_data *c,int dir);


/*//////////////////////////// Make Cluster Paremeter ////////////////////////////*/
/*
double check_distance_of_cluster_descendant(triangle_data *t, int nt, cluster_data *c){
	int i,j;
	double d,tmp,max;
	vertex_data *v;

	if(c == NULL) return 0;

	d = 0x100000000;
	tmp = 0;
	max = 0;
	for(i=0;i<c->Ncluster_ver_link;i++){
		v = c->cluster_ver_link[i];
		for(j=0;j<nt;j++){
			tmp = distance_point_and_triangle_mesh(v->pos,&t[j]);
			if(tmp < d) d = tmp;
		}
		if(d > max) max = d;
	}
	tmp = check_distance_of_cluster_descendant(t,nt,c->children[0]);
	if(tmp > max) max = tmp;
	tmp = check_distance_of_cluster_descendant(t,nt,c->children[1]);
	if(tmp > max) max = tmp;

	return max;
}
*/
void make_bounding_distance_of_cluster_data(cluster_data *rc){
	int i,j,k,cnt;
//	double d,tmp;
	triangle_data pt[100];
	list_data *l;
	vertex_data *v;
	vertex_tree_data *svt,*bvt,*vt;
	double d,tmp,max;
	double bounding_distance;//////////shoud be store

	l = &rc->polygons;

	if(l==NULL || l->next == NULL || l->next->next == NULL){
		bounding_distance = 0x100000;
		return ;
	}

	svt = (vertex_tree_data*)l->next->element;
	l = l->next;
	bvt = (vertex_tree_data*)l->next->element;
	l = l->next;
	cnt=0;
	while(l->next != NULL){
		vt = (vertex_tree_data*)l->next->element;
		pt[cnt].ver[0] = svt->vertex;
		pt[cnt].ver[1] = bvt->vertex;
		pt[cnt].ver[2] = vt->vertex;
		bvt = vt;
		cnt++;
		l = l->next;
	}
//	if(cnt < 1){
//		rc->bounding_distance = 0x100000;
//		return ;
//	}

	max = 0;
	for(i=0;i<rc->number_of_triangle;i++){
		for(j=0;j<3;j++){
			v = rc->triangles_que[i]->ver[j];
			d = 3.40282347e38;
			for(k=0;k<cnt;k++){
				tmp = distance_point_and_triangle_mesh(v->pos,&pt[k]);
				if(tmp < d) d = tmp;
			}
			if(d > max) max = d;
		}
	}
	bounding_distance = max;
//	printf("%d %lf\t",rc->number,max);
}


void make_direction_range_of_cluster(cluster_data *c,double normal[3], double *range){
	int i;
	double ip,min;
	triangle_data *t;

	if( c->number_of_triangle == 1){
		for(i=0;i<3;i++){
			c->normal[i] = c->triangles_que[0]->normal[i];
//			c->normal_center[i] =  c->triangles_que[0]->normal[i];
		}
		*range = 0;
		return;
	}

	min = 2;
	for(i=0;i<c->number_of_triangle;i++){
		t = c->triangles_que[i];
		ip = inner_product_2_vector(t->normal,normal);
		if(ip < min) min = ip;
	}
	if(min<0){
		*range = 2;
	}
	else{
		*range = sqrt(1-min*min);
	}
}

void make_direction_range_nbr_of_cluster(cluster_data *c,double normal[3], double *range){
	int i,j,k;
	double ip,min;
	triangle_data *t;
	double normal2[3];

	if( c->number_of_triangle == 1){
		for(i=0;i<3;i++){
			c->normal[i] = c->triangles_que[0]->normal[i];
		}
//		*range = 0;
//		return;
	}else{
		for(i=0;i<3;i++){
			normal2[i] = 0;
		}
		for(i=0;i<c->number_of_triangle;i++){
			t = c->triangles_que[i];
			for(j=0;j<3;j++){
				normal2[j]+=t->normal[j];
			}
			for(k=0;k<3;k++){
				if(t->nbr[k]->cluster_number == c->number) continue;
				for(j=0;j<3;j++){
					normal2[j]+=t->nbr[k]->normal[j];
				}
			}
		}
		make_unit_vector(normal2);
		for(i=0;i<3;i++){
			c->normal[i] = normal2[i];
		}
	}
	

	min = 2;
	for(i=0;i<c->number_of_triangle;i++){
		t = c->triangles_que[i];
		ip = inner_product_2_vector(t->normal,normal);
		if(ip < min) min = ip;
		for(j=0;j<3;j++){
			ip = inner_product_2_vector(t->nbr[j]->normal,normal);
			if(ip < min) min = ip;
		}
	}
	if(min<0){
		*range = 2;
	}
	else{
		*range = sqrt(1-min*min);
	}
}


void make_position_range_of_cluster(cluster_data *c,double center[3], double *range){
	int i,j;
	double d,max;
	triangle_data *t;
	double sumCenter[3] = {0,0,0};
	double sumArea=0;
	
	for(i=0;i<c->number_of_triangle;i++){
		t = c->triangles_que[i];
		sumCenter[0] += t->area*t->center[0];
		sumCenter[1] += t->area*t->center[1];
		sumCenter[2] += t->area*t->center[2];
		sumArea+=t->area;
	}
	center[0] = sumCenter[0]/sumArea;
	center[1] = sumCenter[1]/sumArea;
	center[2] = sumCenter[2]/sumArea;
	

	max = -1;
	for(i=0;i<c->number_of_triangle;i++){
		t = c->triangles_que[i];
		for(j=0;j<3;j++){
			d = distance_2_point(center,t->ver[j]->pos);
			if(d > max) max = d;
		}
	}
	*range = max;
	
}

/*///////////////////////////////////////////

Hierarchical Clustering

///////////////////////////////////////////*/




/*//////////////////////////// Parameter for Partition ////////////////////////////*/



int compareing_two_parameter_length(triangle_data *t,triangle_data *myt,triangle_data *othert){
	if(distance_2_point(t->center,myt->center) > distance_2_point(t->center,othert->center)) return FALSE;
	return TRUE;
}

int compareing_two_parameter_normal(triangle_data *t,triangle_data *myt,triangle_data *othert){
	if(inner_product_2_vector(t->normal,myt->normal) > inner_product_2_vector(t->normal,othert->normal)) return TRUE;
	return FALSE;
}

int compareing_two_parameter(triangle_data *t,triangle_data *myt,triangle_data *othert){
	if(inner_product_2_vector(t->normal,myt->normal)/distance_2_point(t->center,myt->center)
		> inner_product_2_vector(t->normal,othert->normal)/distance_2_point(t->center,othert->center)) return TRUE;
	return FALSE;
}

/*//////////////////////////// Continuous Partition ////////////////////////////*/

int countinuous_cluster_by_normal(triangle_data *st, int bcn, int cn, 
										 triangle_data *myt, triangle_data *othert, cluster_data *c){
	extern triangle_data BOUNDARY;
	int top=0,end=0;
	int i;
	triangle_data *tn,*t,**tq;

	tq = c->triangles_que;
	end = c->number_of_triangle;

	while(top!=end){
		t = tq[top++];
		for(i=0;i<3;i++){
			tn = t->nbr[i];
			if(tn == &BOUNDARY || tn == othert) continue;
			if(tn->cluster_number != bcn) continue;
			if(compareing_two_parameter_normal(tn,myt,othert)==FALSE) continue;
			tn->cluster_number = cn;
			tq[end++] = tn;
		}
	}
	return end;
}


int cut_parameter(triangle_data *st, double center[3], double main_axis[3]){
	double rel[3];

	subtract_2_vector(rel,st->center,center);
	if(inner_product_2_vector(rel,main_axis)>0) return TRUE;
	return FALSE;
}


int q_countinuous_cluster_by_cut_plane(triangle_data *st, int bcn, int cn, 
										 double center[3], double main_axis[3],cluster_data *c){
	extern triangle_data BOUNDARY;
	int top=0,end=0;
	int i;
	triangle_data *tn,*t,**tq;

	top = 0;
	end = c->number_of_triangle;
	tq = c->triangles_que;
	while(top!=end){
		t = tq[top++];
		for(i=0;i<3;i++){
			tn = t->nbr[i];
			if(tn == &BOUNDARY) continue;
			if(tn->cluster_number != bcn) continue;
			if(cut_parameter(tn,center,main_axis)==FALSE) continue;
			tn->cluster_number = cn;
			tq[end++] = tn;
		}
	}
	return end;
}

int smooth_boundary(int bcn, int cn, cluster_data *c){
	extern triangle_data BOUNDARY;
	int top=0,end,end1;
	int i,j,flag;
	triangle_data *tn,*t;
	triangle_data **tq;

	tq = c->triangles_que;
	end1 = end = c->number_of_triangle;
//	return end1;
	while(top!=end1){
//	while(top!=end){
		t = tq[top++];
		for(i=0;i<3;i++){
			tn = t->nbr[i];
			if(tn == &BOUNDARY) continue;
			if(tn->cluster_number != bcn) continue;
			flag = 0;
			for(j=0;j<3;j++) if(tn->nbr[j]->cluster_number == cn) flag++;
			if(flag < 2) continue;
			tn->cluster_number = cn;
			tq[end++] = tn;
		}
	}
	return end;
}

int fill_cluster(triangle_data *st, int bcn, int cn, cluster_data *c){
	extern triangle_data BOUNDARY;
	int top=0,end=0;
	int i;
	triangle_data *tn,*t;
	triangle_data **tq;

	tq = c->triangles_que;
	end = c->number_of_triangle;
	st->cluster_number = cn;
	while(top!=end){
		t = tq[top++];
		for(i=0;i<3;i++){
			tn = t->nbr[i];
			if(tn == &BOUNDARY) continue;
			if(tn->cluster_number != bcn) continue;
			tn->cluster_number = cn;
			tq[end++] = tn;
		}
	}
	return end;
}


int change_cluster_number(int bcn, int cn, cluster_data *c){
	extern triangle_data BOUNDARY;
	int top,end;
	int i;
	triangle_data *tn,*t,**tq;

	top = 0;
	end = c->number_of_triangle;
	tq = c->triangles_que;
	while(top!=end){
		t = tq[top++];
		for(i=0;i<3;i++){
			tn = t->nbr[i];
			if(tn == &BOUNDARY) continue;
			if(tn->cluster_number != bcn) continue;
			tn->cluster_number = cn;
			tq[end++] = tn;
		}
	}
	return end;
}

int make_closed_loop(cluster_data *c1,int bcn, triangle_data *st1){
//	extern triangle_data BOUNDARY;
	int i,j,flag=0;
	triangle_data *t,**tq1,**tq2;
	list_data *l;


	tq1 = c1->triangles_que;
	for(i=0;i<c1->number_of_triangle;i++){
		for(j=0;j<3;j++) tq1[i]->ver[j]->closed_check = 0;
	}
	for(i=0;i<c1->number_of_triangle;i++){
		for(j=0;j<3;j++){
			if(tq1[i]->nbr[j]->cluster_number == c1->number) continue;
			tq1[i]->ver[j]->closed_check++;
			tq1[i]->ver[(j+1)%3]->closed_check++;
		}
	}
	for(i=0;i<c1->number_of_triangle;i++){
		for(j=0;j<3;j++){
			if(tq1[i]->ver[j]->closed_check > 2){
				l = &tq1[i]->ver[j]->neighbor_triangle_list;
				t = (triangle_data*)load_list_element_and_next_pointer(&l);
				while(t!=NULL){
					if(t->cluster_number == c1->number && t != st1)	t->cluster_number = bcn;
					t = (triangle_data*)load_list_element_and_next_pointer(&l);
				}
				tq1[i]->ver[j]->closed_check = 0;
				flag = 1;
			}
		}
	}
	return flag;
}

int closed_loop_check(cluster_data *c1){

	int i,j,flag=0;
	triangle_data *t,**tq1,**tq2;
	list_data *l;

	c1->loop = 0;
	tq1 = c1->triangles_que;
	for(i=0;i<c1->number_of_triangle;i++){
		for(j=0;j<3;j++) tq1[i]->ver[j]->closed_check = 0;
	}
	for(i=0;i<c1->number_of_triangle;i++){
		for(j=0;j<3;j++){
			if(tq1[i]->nbr[j]->cluster_number == c1->number) continue;
			tq1[i]->ver[j]->closed_check++;
			tq1[i]->ver[(j+1)%3]->closed_check++;
		}
	}
	for(i=0;i<c1->number_of_triangle;i++){
		for(j=0;j<3;j++){
			if(tq1[i]->ver[j]->closed_check > 2){
				c1->loop = 1;
				printf("Not closed loop is generated %d\n",c1->number);
				return c1->loop;
			}
		}
	}
	return c1->loop;
}


double parameter(object_data *o, cluster_data *c){
	return area_normal_and_position_covariance(o,c);
//	return area_quadrics_error_from_triangle_q(o,c);
//	return area_normal_quadrics_error_from_triangle_q(o, c);
}


void another_clustering(object_data *o,cluster_data *bc,cluster_data *c1,cluster_data *c2,double *p1,double *p2,double center[3],double direction[3],int f1,int f2){
	int bcn,cn1,cn2,i;
	double min,max,tempd,np1,np2;
	double vec[3];
	triangle_data *st1, *st2, *t, dumtri;
	static heap_data *heap;
	static int nh;
	static int first=TRUE;
	triangle_data **ntq1,**ntq2;
	cluster_data nc1,nc2;
	int flag1,flag2;

	nc1.triangles_que = ntq1 = (triangle_data **)malloc(sizeof(triangle_data*)*bc->number_of_triangle);
	nc2.triangles_que = ntq2 = (triangle_data **)malloc(sizeof(triangle_data*)*bc->number_of_triangle);
	bcn = bc->number;
	cn1 = nc1.number = c1->number;
	cn2 = nc2.number = c2->number;

	for(i=0;i<bc->number_of_triangle;i++){
		bc->triangles_que[i]->cluster_number = bcn; 
	}

	min = 0;
	max = 0;
	st1 = &dumtri;
	st2 = &dumtri;
	for(i=0;i<bc->number_of_triangle;i++){
		t = bc->triangles_que[i];
		subtract_2_vector(vec,t->center,center);
		tempd = inner_product_2_vector(direction,vec);
		if(tempd > max){
			max = tempd;
			st1 = t;
		}
		if(tempd < min){
			min = tempd;
			st2 = t;
		}
	}

	if((st1==&dumtri) || (st2==&dumtri) || (st1==st2)){
		for(i=0;i<c1->number_of_triangle;i++){
			c1->triangles_que[i]->cluster_number = cn1;
		}
		for(i=0;i<c2->number_of_triangle;i++){
			c2->triangles_que[i]->cluster_number = cn2; 
		}
		free(ntq1);
		free(ntq2);
		return;
	}

	ntq1[0] = st1;
	ntq2[0] = st2;
	st1->cluster_number = cn1;
	st2->cluster_number = cn2;
	nc1.number_of_triangle = 1;
	nc2.number_of_triangle = 1;


	nc1.number_of_triangle = q_countinuous_cluster_by_cut_plane(st1,bcn,cn1,center,direction,&nc1);
	nc1.number_of_triangle = smooth_boundary(bcn,cn1,&nc1);
	flag1 = flag2 = 0;
	flag1 = make_closed_loop(&nc1,bcn,st1);

	nc2.number_of_triangle = fill_cluster(st2,bcn,cn2,&nc2);
	flag2 = make_closed_loop(&nc2,bcn,st2);

	if(flag1==1){
		for(i=0;i<nc1.number_of_triangle;i++){
			if(nc1.triangles_que[i]->cluster_number == cn1){
				nc1.triangles_que[i]->cluster_number = bcn;
			}
		}
		nc1.triangles_que[0] =st1;
		st1->cluster_number = cn1;
		nc1.number_of_triangle = 1;
	}
	nc1.number_of_triangle= change_cluster_number(bcn,cn1,&nc1);

	if(flag2==1){
		for(i=0;i<nc2.number_of_triangle;i++){
			if(nc2.triangles_que[i]->cluster_number == cn2){
				nc2.triangles_que[i]->cluster_number = bcn;
			}
		}
		nc2.triangles_que[0] = st2;
		st2->cluster_number = cn2;
		nc2.number_of_triangle = 1;
	}
	nc2.number_of_triangle = change_cluster_number(bcn,cn2,&nc2);
	nc1.number_of_triangle = change_cluster_number(bcn,cn1,&nc1);

	flag1 = closed_loop_check(&nc1);
	flag2 = closed_loop_check(&nc2);

	np1 = parameter(o,&nc1);
	np2 = parameter(o,&nc2);

	if( (flag1+flag2) <  (f1+f2) ){
		printf("loop modified %d\n", flag1+flag2);
		*p1 = np1;
		*p2 = np2;
		free(c1->triangles_que);
		free(c2->triangles_que);
		*c1 = nc1;
		*c2 = nc2;
		c1->triangles_que = ntq1;
		c2->triangles_que = ntq2;
		return;
	}
	if( (flag1+flag2) == (f1+f2)){
		if((np1+np2) < (*p1+*p2)){
			*p1 = np1;
			*p2 = np2;
			free(c1->triangles_que);
			free(c2->triangles_que);
			*c1 = nc1;
			*c2 = nc2;
			c1->triangles_que = ntq1;
			c2->triangles_que = ntq2;
			return;
		}
	}else{
		printf("loop modified %d\n", f1+f2);
	}
	for(i=0;i<c1->number_of_triangle;i++){
		c1->triangles_que[i]->cluster_number = cn1;
	}
	for(i=0;i<c2->number_of_triangle;i++){
		c2->triangles_que[i]->cluster_number = cn2; 
	}
	free(ntq1);
	free(ntq2);
}

void parameter_for_one_face(cluster_data *c){
	int i,j,k;
	c->quadrics_error = 0;
//	c->correlation_error = 0;
	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			c->pca[j][k] = 0;
			c->cca[j][k] = 0;
		}
	}
	c->children[0] = c->children[1] = NULL;
//	c->direction_range = 0;
//	c->direction_range_nbr = 0;
//	c->bounding_distance = 0;
	c->area = c->triangles_que[0]->area;
	for(i=0;i<3;i++){
//		c->normal[i] = c->triangles_que[0]->normal[i];
//		c->center[i] = c->triangles_que[0]->center[i];
	}
}

int hierarchical_clustering_by_quadrics(object_data *o){
	int bcn,cn1,cn2,i,j,k,flag1,flag2;
	double min,max,tempd,p1,p2;
	double vec[3];
	triangle_data *st1, *st2, *t, dumtri;
	cluster_data *bc,*c1,*c2;
	static heap_data *heap;
	static int nh;
	static int first=TRUE;

	if(first==TRUE){
		first = FALSE;
		heap = (heap_data *)malloc(sizeof(heap_data)*(o->num_of_all_tri+1));

		o->clusters[0].number_of_triangle = o->num_of_all_tri;
		o->clusters[0].triangles_que = (triangle_data **)malloc(sizeof(triangle_data*)*o->num_of_all_tri);
		o->clusters[0].number = 0;
		for(i=0;i<o->num_of_all_tri;i++){
			o->clusters[0].triangles_que[i] = &o->triangles[i];
		}
		o->number_of_cluster++;
		heap[1].cluster_number = 0;
		heap[1].quadrics = parameter(o,&o->clusters[0]);
		nh=1;
//		make_direction_range_of_cluster(&o->clusters[0],o->clusters[0].normal,&o->clusters[0].direction_range);
//		make_direction_range_nbr_of_cluster(&o->clusters[0],o->clusters[0].normal,&o->clusters[0].direction_range_nbr);
//		make_position_range_of_cluster(&o->clusters[0],o->clusters[0].center,&o->clusters[0].position_range);
		o->clusters[0].number_of_vertex = 0;
		o->clusters[0].Ncluster_ver_link = 0;
		o->clusters[0].polygons.next = NULL;
		o->clusters[0].vertexlink.next = NULL;
		o->clusters[0].parent = NULL;
		o->clusters[0].loop = 0;

		initial_edge_data(o);


	}



#ifdef WITHOUT_HEAP
	bcn = heap[nh].cluster_number;
	bc = &o->clusters[bcn];
	--nh;
#else	
	bcn = heap[1].cluster_number;
	bc = &o->clusters[bcn];

	heap[1] = heap[nh];
	downheap(--nh,heap,1);
#endif

	min = 0;
	max = 0;
	st1 = &dumtri;
	st2 = &dumtri;
	for(i=0;i<bc->number_of_triangle;i++){
		t = bc->triangles_que[i];
		subtract_2_vector(vec,t->center,bc->center);
		tempd = bc->pca[0][0]*vec[0]+bc->pca[0][1]*vec[1]+bc->pca[0][2]*vec[2];
		if(tempd > max){
			max = tempd;
			st1 = t;
		}
		if(tempd < min){
			min = tempd;
			st2 = t;
		}
	}

	if((st1==&dumtri) || (st2==&dumtri) || (st1==st2)){
		st1 = bc->triangles_que[0];
		st2 = bc->triangles_que[bc->number_of_triangle-1];
	}


	cn1 = o->number_of_cluster++;
	cn2 = o->number_of_cluster++;
	c1 = &o->clusters[cn1];
	c2 = &o->clusters[cn2];
	c1->number = cn1;
	c2->number = cn2;


	c1->triangles_que = (triangle_data **)malloc(sizeof(triangle_data*)*bc->number_of_triangle);
	c2->triangles_que = (triangle_data **)malloc(sizeof(triangle_data*)*bc->number_of_triangle);
	
	if(c2->triangles_que == NULL ) {
		fprintf(stdout,"no more memory");
		
	}
	c1->triangles_que[0] =st1;
	c2->triangles_que[0] =st2;
	st1->cluster_number = cn1;
	st2->cluster_number = cn2;
	c1->number_of_triangle = 1;
	c2->number_of_triangle = 1;

	double direction[3] ;
	direction[0] = bc->pca[0][0];
	direction[1] = bc->pca[0][1];
	direction[2] = bc->pca[0][2];

	c1->number_of_triangle = q_countinuous_cluster_by_cut_plane(st1,bcn,cn1,bc->center,direction,c1);
	c1->number_of_triangle = smooth_boundary(bcn,cn1,c1);

	flag1 = flag2 = 0;
	flag1 = make_closed_loop(c1,bcn,st1);

	c2->number_of_triangle = fill_cluster(st2,bcn,cn2,c2);
	flag2 = make_closed_loop(c2,bcn,st2);

	if(flag1==1){
		for(i=0;i<c1->number_of_triangle;i++){
			if(c1->triangles_que[i]->cluster_number == cn1){
				c1->triangles_que[i]->cluster_number = bcn;
			}
		}
		if(st1==NULL) exit(-1);
		c1->triangles_que[0] = st1;
		st1->cluster_number = cn1;
		c1->number_of_triangle = 1;
	}
	c1->number_of_triangle= change_cluster_number(bcn,cn1,c1);

	if(flag2==1){
		for(i=0;i<c2->number_of_triangle;i++){
			if(c2->triangles_que[i]->cluster_number == cn2){
				c2->triangles_que[i]->cluster_number = bcn;
			}
		}
		if(st2==NULL) exit(-1); 
		c2->triangles_que[0] =st2;
		st2->cluster_number = cn2;
		c2->number_of_triangle = 1;
	}
	c2->number_of_triangle = change_cluster_number(bcn,cn2,c2);
	c1->number_of_triangle = change_cluster_number(bcn,cn1,c1);

	flag1 = closed_loop_check(c1);
	flag2 = closed_loop_check(c2);

	if(bc->number_of_triangle > 2) {
		p1 = parameter(o,c1);
		p2 = parameter(o,c2);
		if( (p1 > 1.0e-12) && (p2 > 1.0e-12) ){
			double direction[3];
			direction[0] = bc->cca[0][0];
			direction[1] = bc->cca[0][1];
			direction[2] = bc->cca[0][2];
			double center[3];
			center[0] = bc->cca_center[0];
			center[1] = bc->cca_center[1];
			center[2] = bc->cca_center[2];
			another_clustering(o,bc,c1,c2,&p1,&p2,center,direction ,flag1, flag2);
		}
//		another_clustering(o,bc,c1,c2,&p1,&p2,bc->center,bc->cca[0] ,flag1 ,flag2);
	}

	closed_loop_check(c1);
	closed_loop_check(c2);

	
	bc->children[0] = c1;
	bc->children[1] = c2;
	c1->parent = bc;
	c2->parent = bc;

	make_polygon_of_cluster2(c1,bc,c2);
	make_polygon_of_cluster2(c2,bc,c1);
	
//	make_direction_range_of_cluster(c1,c1->normal,&c1->direction_range);
//	make_direction_range_of_cluster(c2,c2->normal,&c2->direction_range);

//	make_direction_range_nbr_of_cluster(c1,c1->normal,&c1->direction_range_nbr);
//	make_direction_range_nbr_of_cluster(c2,c2->normal,&c2->direction_range_nbr);
	
//	make_position_range_of_cluster(c1,c1->center,&c1->position_range);
//	make_position_range_of_cluster(c2,c2->center,&c2->position_range);

	make_bounding_distance_of_cluster_data(c1);
	make_bounding_distance_of_cluster_data(c2);

 	bc->check = DEAD;
 	c1->check = ALIVE;
 	c2->check = ALIVE;
	
	if(c1->number_of_triangle > 1){
		++nh;
		heap[nh].cluster_number = cn1;
//		heap[nh].quadrics =p1;
		heap[nh].quadrics =c1->quadrics_error;
#ifndef WITHOUT_HEAP
		upheap(nh,heap,nh);
#endif
	}else{
		parameter_for_one_face(c1);
		free(c1->triangles_que);
	}

	if(c2->number_of_triangle > 1){
		++nh;
		heap[nh].cluster_number = cn2;
//		heap[nh].quadrics = p2;
		heap[nh].quadrics = c2->quadrics_error;
#ifndef WITHOUT_HEAP
		upheap(nh,heap,nh);
#endif
	}else{
		parameter_for_one_face(c2);
		free(c2->triangles_que);
	}


	free(bc->triangles_que);///////////////////////////////////////////////////////////
	
	if(nh == 0){
		printf("cluster number %d\n",o->number_of_cluster);
		return FINISH;
	}
	return CONTINUE;
}

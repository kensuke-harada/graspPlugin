#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "struct.h"
#include "mymath.h"

//extern triangle_data BOUNDARY;



double
distance_point_and_triangle(double *p,triangle_data *t){
	double s[3],d[3];
	double a[3][3];
	double ia[3][3];

	subtract_2_vector(d,p,t->ver[0]->pos);

	subtract_2_vector(a[0],t->ver[1]->pos,t->ver[0]->pos);
	subtract_2_vector(a[1],t->ver[2]->pos,t->ver[0]->pos);
	outer_product_2_vector(a[2],a[0],a[1]);
	make_unit_vector(a[2]);
	if(inverse_matrix(ia,a)==0) return -1;
	translate_cordinate(s,ia,d);

	if(s[0]>0 && s[1]>0 && s[0]+s[1]<1){
		if(s[2] > 0) return s[2];
		return -s[2];
	}
	return -1;

}

double
distance_point_and_triangle_mesh(double *p,triangle_data *t){
	double d,tmp;

	d = MAX_DOUBLE;
	tmp = distance_point_and_triangle(p,t);
	if(tmp >= 0) return tmp;
	tmp = distance_point_and_line(p,t->ver[0]->pos,t->ver[1]->pos);
	if(tmp >= 0 && tmp < d) d = tmp;
	tmp = distance_point_and_line(p,t->ver[1]->pos,t->ver[2]->pos);
	if(tmp >= 0 && tmp < d) d = tmp;
	tmp = distance_point_and_line(p,t->ver[2]->pos,t->ver[0]->pos);
	if(tmp >= 0 && tmp < d) d = tmp;
	tmp = distance_2_point(p,t->ver[0]->pos);
	if(tmp >= 0 && tmp < d) d = tmp;
	tmp = distance_2_point(p,t->ver[1]->pos);
	if(tmp >= 0 && tmp < d) d = tmp;
	tmp = distance_2_point(p,t->ver[2]->pos);
	if(tmp >= 0 && tmp < d) d = tmp;

	return d;
}



/*
void
make_triangle_inverse_matrix(triangle_data *t){
	double a[3][3];
	
	subtract_2_vector(a[0],t->ver[1]->pos,t->ver[0]->pos);
	subtract_2_vector(a[1],t->ver[2]->pos,t->ver[0]->pos);
	subtract_2_vector(a[2],t->nml,POS_O);
	inverse_matrix(t->ia,a);
}

*/

void make_circumscrived_circle(double *radius,double *center,triangle_data *t){
	double kk,ll,kl;
	double m,n;
	double vk[3],vl[3],v[3];
	double c[3];

	kk = distance_2_point_power(t->ver[0]->pos,t->ver[1]->pos);
	ll = distance_2_point_power(t->ver[0]->pos,t->ver[2]->pos);
	kl = inner_product_3_point(t->ver[0]->pos,t->ver[1]->pos,t->ver[2]->pos);

	m = 0.5*(ll*kk-kl*ll)/(kk*ll-kl*kl);
	n = 0.5*(-kk*kl+kk*ll)/(kk*ll-kl*kl);

	if(m<0){
		*radius = sqrt(ll)*0.5;
		add_2_vector(c,t->ver[0]->pos,t->ver[2]->pos);
		multiple_scalar_vector(center,0.5,c);
	}
	else if(n<0){
		*radius = sqrt(kk)*0.5;
		add_2_vector(c,t->ver[0]->pos,t->ver[1]->pos);
		multiple_scalar_vector(center,0.5,c);
	}
	else if((m+n)>1.0){
		*radius = distance_2_point(t->ver[1]->pos,t->ver[2]->pos)*0.5;
		add_2_vector(c,t->ver[1]->pos,t->ver[2]->pos);
		multiple_scalar_vector(center,0.5,c);
	}
	else{
		subtract_2_vector(vk,t->ver[1]->pos,t->ver[0]->pos);
		subtract_2_vector(vl,t->ver[2]->pos,t->ver[0]->pos);
		
		multiple_scalar_vector(vk,m,vk);
		multiple_scalar_vector(vl,n,vl);
		add_2_vector(v,vk,vl);
		*radius = vector_length(v);
		add_2_vector(center,t->ver[0]->pos,v);
	}
}

void make_gravity_center_of_triangle(double result[3],triangle_data *t){
	int j;
	
	for(j=0;j<3;j++){
		result[j] = t->ver[0]->pos[j]+t->ver[1]->pos[j]+t->ver[2]->pos[j];
		result[j] /=3.0;
	}
}

void
make_triangles_normal(double *normal,triangle_data *t){
	outer_product_3_points(normal,t->ver[0]->pos,t->ver[1]->pos,t->ver[2]->pos);
	make_unit_vector(normal);
}

void
make_triangles_area_and_normal(double *area,double *normal,triangle_data *t){
	outer_product_3_points(normal,t->ver[0]->pos,t->ver[1]->pos,t->ver[2]->pos);
	*area = vector_length(normal)/2.0;
	make_unit_vector(normal);
}

void
make_outer_product_of_triangle(double *normal,triangle_data *t){
	outer_product_3_points(normal,t->ver[0]->pos,t->ver[1]->pos,t->ver[2]->pos);
}

void
make_area_quadrics(float aq[3][3],triangle_data *t){
	int i,j,k;

	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			aq[i][j]=0;
		}
	}
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			for(k=0;k<3;k++){
				aq[j][k] += (t->ver[i]->pos[j] * t->ver[i]->pos[k])/6.0;
				aq[j][k] += (t->ver[i]->pos[j] * t->ver[(i+1)%3]->pos[k])/12.0;
				aq[j][k] += (t->ver[(i+1)%3]->pos[j] * t->ver[i]->pos[k])/12.0;
			}
		}
	}
	for(j=0;j<3;j++){
		for(k=0;k<3;k++){
			aq[j][k] *= t->area;
		}
	}
}

#ifdef TRIANGLE_HIERARCHY
double check_normal_of_triangle_descendant(double *vector,triangle_data *t){
	double p0,p1,p2;
	extern triangle_data BOUNDARY;

	if(t == &BOUNDARY) return 1;

	p0 = inner_product_2_vector(t->normal,vector);
	if(t->children[0] != NULL){
		p1 = check_normal_of_triangle_descendant(vector,t->children[0]);
		p2 = check_normal_of_triangle_descendant(vector,t->children[1]);
	}
	else{
		p1=1;
		p2=1;
	}
	if(p0<p1 && p0<p2){
		return p0;
	}
	if(p1<p2){
		return  p1;
	}
	return p2;
}

void make_direction_of_data(double *vector,double *range,triangle_data *t){
	int i;
	double ip;

	for(i=0;i<3;i++){
		vector[i] = t->normal[i];
	}
	ip = check_normal_of_triangle_descendant(vector,t);
	if(ip<0){
		*range = 2;
	}
	else{
		*range = sqrt(1-ip*ip);
	}
}

double check_distance_of_triangle_descendant(triangle_data *dt,triangle_data *t){
	double d,tmp;
	extern triangle_data BOUNDARY;

	if(dt->children[0] == &BOUNDARY) return 0;

	d = distance_point_and_triangle_mesh(dt->ver[0]->pos,t);
	tmp = distance_point_and_triangle_mesh(dt->ver[1]->pos,t);
	if(tmp > d) d = tmp;
	tmp = distance_point_and_triangle_mesh(dt->ver[2]->pos,t);
	if(tmp > d) d = tmp;

	if(dt->children[0] != &BOUNDARY){
		tmp = check_distance_of_triangle_descendant(dt->children[0],t);
		if(tmp > d) d = tmp;
		tmp = check_distance_of_triangle_descendant(dt->children[1],t);
		if(tmp > d) d = tmp;
	}
	return d;
}

double make_error_value_of_data(triangle_data *t){
	return check_distance_of_triangle_descendant(t,t);
}

#endif

#ifdef MAKE_CLUSTER_HIERARCHY




#endif

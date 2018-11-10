
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/**********************************************

  ŽZp“I”ŠÖ”
  
**********************************************/

double
maxd(double a, double b){
	if(a > b) return a;
	return b;
}

void
subtract_2_vector(double *result,double *plus,double *minus){
	int i;
	
	for(i=0;i<3;i++){
		result[i] = plus[i] - minus[i];
	}
}

void
add_2_vector(double *result,double *pos1,double *pos2){
	int i;
	
	for(i=0;i<3;i++){
		result[i] = pos1[i] + pos2[i];
	}
}


double
inner_product_2_vector(double *nml1,double *nml2){
	
	return (nml1[0]*nml2[0]+nml1[1]*nml2[1]+nml1[2]*nml2[2]);
}



double
inner_product_3_point(double *minus,double *pos1,double *pos2){
	
	return (
		 (pos1[0]-minus[0])*(pos2[0]-minus[0])
		+(pos1[1]-minus[1])*(pos2[1]-minus[1])
		+(pos1[2]-minus[2])*(pos2[2]-minus[2])
		);
	
}

void
outer_product_3_points(double *result,double *minus,double *pos1,double *pos2){
	int i;
	
	for(i=0;i<3;i++){
		result[i]
			=(pos1[(i+1)%3]-minus[(i+1)%3])*(pos2[(i+2)%3]-minus[(i+2)%3])
			-(pos1[(i+2)%3]-minus[(i+2)%3])*(pos2[(i+1)%3]-minus[(i+1)%3]);
	}
}

void
outer_product_2_vector(double *result,double *vec1,double *vec2){
	int i;
	
	for(i=0;i<3;i++){
		result[i]
			=vec1[(i+1)%3]*vec2[(i+2)%3] - vec1[(i+2)%3]*vec2[(i+1)%3];
	}
}

double
make_unit_vector(double *nml){
	double norm;
	int i;
	
	norm = sqrt(nml[0] * nml[0] + nml[1] * nml[1] + nml[2] * nml[2]);
	if(norm == 0) return 0;
	for(i=0;i<3;i++) nml[i] /= norm;
	return norm;
}
/*
float
make_unit_vector_float(float *nml){
	float norm;
	int i;
	
	norm = (float)sqrt(nml[0] * nml[0] + nml[1] * nml[1] + nml[2] * nml[2]);
	if(norm == 0) return 0;
	for(i=0;i<3;i++) nml[i] /= norm;
	return norm;
}
*/
void
multiple_scalar_vector(double *result,double s,double *vec){
	int i;

	for(i=0;i<3;i++){
		result[i] = s*vec[i];
	}

}


double
determinant(double a[3][3]){
	return (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
		+a[1][0]*(a[2][1]*a[0][2]-a[0][1]*a[2][2])
		+a[2][0]*(a[0][1]*a[1][2]-a[0][2]*a[1][1]));
}

double
determinant_of_3_vector(double a[3], double b[3], double c[3]){
	return (a[0]*(b[1]*c[2]-b[2]*c[1])
		   +b[0]*(c[1]*a[2]-c[2]*a[1])
		   +c[0]*(a[1]*b[2]-a[2]*b[1]));
}


double
inverse_matrix(double result[3][3],double a[3][3]){
	double det;
	
	det = determinant(a);
	if(det == 0) return det;
	
	result[0][0]= (a[1][1]*a[2][2]-a[1][2]*a[2][1])/det;
	result[0][1]=-(a[0][1]*a[2][2]-a[0][2]*a[2][1])/det;
	result[0][2]= (a[0][1]*a[1][2]-a[0][2]*a[1][1])/det;
	result[1][0]=-(a[1][0]*a[2][2]-a[1][2]*a[2][0])/det;
	result[1][1]= (a[0][0]*a[2][2]-a[0][2]*a[2][0])/det;
	result[1][2]=-(a[0][0]*a[1][2]-a[0][2]*a[1][0])/det;
	result[2][0]= (a[1][0]*a[2][1]-a[1][1]*a[2][0])/det;
	result[2][1]=-(a[0][0]*a[2][1]-a[0][1]*a[2][0])/det;
	result[2][2]= (a[0][0]*a[1][1]-a[0][1]*a[1][0])/det;
	
	return det;
}

void
translate_cordinate(double result[3],double a[3][3],double r[3]){
	int i;
	
	for(i=0;i<3;i++)
		result[i] = a[0][i]*r[0]+a[1][i]*r[1]+a[2][i]*r[2];
}

double
distance_2_point_power(double *pos1,double *pos2){
	
	return
		(	 (pos1[0]-pos2[0])*(pos1[0]-pos2[0])
		+(pos1[1]-pos2[1])*(pos1[1]-pos2[1])
		+(pos1[2]-pos2[2])*(pos1[2]-pos2[2])
		);
}

double
distance_2_point(double *pos1,double *pos2){
	return sqrt(distance_2_point_power(pos1,pos2));
}

double
distance_2_point_2dimension(double *pos1,double *pos2){
	return sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0])+(pos1[1]-pos2[1])*(pos1[1]-pos2[1]));
}



double
vector_length(double *vec){
	double o[3] = {0.0,0.0,0.0};
	return distance_2_point(vec,o);
}

double
distance_point_and_line(double *pos,double *posl1,double *posl2){
	double aa,bb,ab;
	
	aa = distance_2_point_power(posl1,posl2);
	bb = distance_2_point_power(pos,posl1);
	ab = inner_product_3_point(posl1,pos,posl2);
	
	if(ab > aa || ab < 0) return -1;
	return sqrt(bb - ab*ab/aa);
}

void
rotation_vector(double vec[3], double ra, double rb, double rg){
	double v1[3],v2[3];

	v1[0] = cos(rg)*vec[0]-sin(rg)*vec[1];
	v1[1] = sin(rg)*vec[0]+cos(rg)*vec[1];
	v1[2] = vec[2];
	
	v2[0] = cos(rb)*v1[0]+sin(rb)*v1[2];
	v2[1] = v1[1];
	v2[2] = -sin(rb)*v1[0]+cos(rb)*v1[2];
	
	vec[0] = v2[0];
	vec[1] = cos(ra)*v2[1]-sin(ra)*v2[2];
	vec[2] = sin(ra)*v2[1]+cos(ra)*v2[2];
}

void multiply_matrix(double result[3][3],double a[3][3], double b[3][3]){  //result = a x b
	int i,j;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			result[i][j] = a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j];
		}
	}
}

void transpose_matrix(double result[3][3], double input[3][3]){
	int i,j;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			result[i][j] = input[j][i];
		}
	}
}


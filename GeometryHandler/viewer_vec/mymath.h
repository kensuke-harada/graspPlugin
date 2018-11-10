#include <stdio.h>
#include <stdlib.h>
#include <math.h>


double maxd(double a, double b);

void subtract_2_vector(double *result,double *pos1,double *pos2);
void add_2_vector(double *result,double *pos1,double *pos2);
double distance_2_point_power(double *pos1,double *pos2);

double inner_product_2_vector(double *nml1,double *nml2);
double inner_product_3_point(double *minus,double *pos1,double *pos2);

void outer_product_3_points(double *result,double *minus,double *pos1,double *pos2);
void outer_product_2_vector(double *result,double *vec1,double *vec2);

double make_unit_vector(double *nml);
void multiple_scalar_vector(double *result,double s,double *vec);


double determinant(double a[3][3]);
double determinant_of_3_vector(double a[3], double b[3], double c[3]);
double inverse_matrix(double result[3][3],double a[3][3]);
void translate_cordinate(double result[3],double a[3][3],double r[3]);

double distance_2_point(double *pos1,double *pos2);
double distance_2_point_2dimension(double *pos1,double *pos2);
double distance_point_and_line(double *pos,double *posl1,double *posl2);
double vector_length(double *vec);

void rotation_vector(double vec[3], double ra, double rb, double rg);

void multiply_matrix(double result[3][3],double a[3][3], double b[3][3]); //result = A x B
void transpose_matrix(double result[3][3], double input[3][3]);

float make_unit_vector_float(float *nml);

double distance_point_and_triangle_mesh(double *p,triangle_data *t);

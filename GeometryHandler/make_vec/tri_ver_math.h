#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//#include "struct.h"
//#include "mymath.h"

double distance_point_and_triangle(double *p,triangle_data *t);
double distance_point_and_triangle_mesh(double *p,triangle_data *t);
void make_triangle_inverse_matrix(triangle_data *t);
void make_circumscrived_circle(double *radius,double *center,triangle_data *t);
void make_gravity_center_of_triangle(double result[3],triangle_data *t);
double check_normal_of_triangle_descendant(double *vector,triangle_data *t);
void make_triangles_normal(double *normal,triangle_data *t);
void make_direction_of_data(double *vector,double *range,triangle_data *t);
void make_outer_product_of_triangle(double *normal,triangle_data *t);
void make_triangles_area_and_normal(double *area,double *normal,triangle_data *t);
double make_error_value_of_data(triangle_data *t);
void make_area_quadrics(float aq[3][3],triangle_data *t);

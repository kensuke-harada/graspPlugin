#include <stdio.h>
#include <stdlib.h>
#include <math.h>


float maxd_float(float a, float b);

void subtract_2_vector_float(float *result,float *pos1,float *pos2);
void add_2_vector_float(float *result,float *pos1,float *pos2);
float distance_2_point_power_float(float *pos1,float *pos2);

float inner_product_2_vector_float(float *nml1,float *nml2);
float inner_product_3_point_float(float *minus,float *pos1,float *pos2);

void outer_product_3_points_float(float *result,float *minus,float *pos1,float *pos2);
void outer_product_2_vector_float(float *result,float *vec1,float *vec2);

float make_unit_vector_float(float *nml);
void multiple_scalar_vector_float(float *result,float s,float *vec);


float determinant_float(float a[3][3]);
float determinant_of_3_vector_float(float a[3], float b[3], float c[3]);
float inverse_matrix_float(float result[3][3],float a[3][3]);
void translate_cordinate_float(float result[3],float a[3][3],float r[3]);

float distance_2_point_float(float *pos1,float *pos2);
float distance_2_point_2dimension_float(float *pos1,float *pos2);
float distance_point_and_line_float(float *pos,float *posl1,float *posl2);
float vector_length_float(float *vec);

void rotation_vector_float(float vec[3], float ra, float rb, float rg);

void multiply_matrix_float(float result[3][3],float a[3][3], float b[3][3]); //result = A x B
void transpose_matrix_float(float result[3][3], float input[3][3]);


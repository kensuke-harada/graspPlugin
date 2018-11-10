void initial_ver_tri_data(object_data *o);
void make_unit_object(object_data *o,int number_of_object);
//void magnification(object_data *o);
void magnification(object_data *o,double m);
void initial_data_range(object_data *o);
void initial(object_data *o);
void make_normal_of_vertices(object_data *o);

#ifdef FLOAT_VERTEX
void parallel_transform(object_data *o,float t[3]);
#else
void parallel_transform(object_data *o,double t[3]);
#endif
void make_nbr(object_data *o);

//void init_data_and_set_range(object_data *o);
//void make_vertex_hierarchies(object_data *o);
//void make_unit_object(object_data *o,int number_of_object);
//void init_data(object_data *o);
//void transform(object_data *o,double t[3], double m[3], double r[3]);
//void stdinput_transformation_value(double t[3], double m[3], double r[3]);
//void transform2(object_data *o,int number_of_object);

void make_unit_10000mm_data(object_data *o);

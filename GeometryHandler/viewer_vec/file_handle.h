#include <stdio.h>

void read_tri_sample_data(object_data *o);
void read_triangle_hierarchy_data(char *file,object_data *o);

void read_tri_data(char *file,object_data *o);
void read_ascii_data(char *file,object_data *o);
void read_ply_data(char *file,object_data *o);
void read_stl_data(char *file, object_data *o);

void write_cluster_data(char *file, object_data *o);
//int write_data(object_data *object);
//void write_tri_data(object_data *o, char *file);

void write_vrml_data(char *file,object_data *wo);
void write_hierarchical_tri_data(char *file,object_data *wo, object_data *eo);

void write_hierarchical_clusters_as_CMF(char *file,object_data *o);

void read_mesh_data(char *name,object_data *o);



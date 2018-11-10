
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<float.h>

#include "set_define.h"
#include "struct.h"
#include "file_handle.h"
#include "list_handle.h"

#ifdef DUMMY_VERTEX_NOT_DEFINED
vertex_data   DUMMY_VERTEX,*TMP_VER;
#endif

#ifndef DUMMY_VERTEX_DEFINED
vertex_data   DUMMY_VERTEX,*TMP_VER;
#endif


#ifdef __LINUX__
int _isnan(float f){};
#endif


void
read_tri_sample_data(object_data *o)
{
    FILE *ifp;
    int i,j;
    int tmp[6],itmp;
    double ddum;
	extern triangle_data BOUNDARY;

#ifdef LOD_MODE
    if ((ifp = fopen("buny_on_table.tri","rb")) == NULL){
        printf("Cannot open file!\n");
        exit(0);
    }
#else
    if ((ifp = fopen("rbn.tri","rb")) == NULL){
        printf("Cannot open file!\n");
        exit(0);
    }
#endif

    fscanf(ifp,"%d%d%d",&o->num_of_ver,&o->num_of_init_tri,&o->num_of_process);
    fprintf(stderr,"%d %d %d\n",o->num_of_ver,o->num_of_init_tri,o->num_of_process);
    
    o->num_of_all_tri = o->num_of_init_tri + o->num_of_process * 4;
    
    o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
    o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_all_tri);
    
    for (i=0; i<o->num_of_ver; i++){	
#ifdef FLOAT_VERTEX
        fscanf(ifp,"%f%f%f%f%f%f%lf%lf%lf",
#else
        fscanf(ifp,"%lf%lf%lf%lf%lf%lf%lf%lf%lf",
#endif
            &o->vertices[i].pos[0],
            &o->vertices[i].pos[1],
            &o->vertices[i].pos[2],
            &o->vertices[i].normal[0],
            &o->vertices[i].normal[1],
            &o->vertices[i].normal[2],
            &ddum,&ddum,&ddum);
    }
    
    for (i=0; i<o->num_of_all_tri; i++){
        fscanf(ifp,"%d%d%d",tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->vertices[tmp[j]];
    }

#ifdef TRIANGLE_HIERARCHY
    
    if(o->num_of_process != 0){
        for (i=0; i<o->num_of_all_tri; i++){
            for (j=0; j<3; j++){
                fscanf(ifp,"%d",&itmp);
                if (itmp == -1) o->triangles[i].nbr[j] = &BOUNDARY;
                else o->triangles[i].nbr[j] = &o->triangles[itmp];
            }
            fscanf(ifp,"%d",&itmp);
            if (itmp == -1) o->triangles[i].parent = &BOUNDARY;
            else o->triangles[i].parent = &o->triangles[itmp];
            for (j=0; j<2; j++){
                fscanf(ifp,"%d",&itmp);
                if (itmp == -1) o->triangles[i].children[j] = &BOUNDARY;
                else o->triangles[i].children[j] = &o->triangles[itmp];
            }
            fscanf(ifp,"%lf",&o->triangles[i].error_value);
        }
    }
	else{
        for (i=0; i<o->num_of_all_tri; i++){
            for (j=0; j<3; j++){
                o->triangles[i].nbr[j] = &BOUNDARY;
            }
            o->triangles[i].parent = &BOUNDARY;
            for (j=0; j<2; j++){
                o->triangles[i].children[j] = &BOUNDARY;
            }
            o->triangles[i].error_value = 0;
        }
	}

#else //TRIANGLE_HIERARCHY
	for (i=0; i<o->num_of_all_tri; i++){
		for (j=0; j<3; j++){
			if(fscanf(ifp,"%d",&itmp)!=1){
				o->nbr_status = DEAD;
				continue;
			}
			if (itmp == -1) o->triangles[i].nbr[j] = &BOUNDARY;
			else o->triangles[i].nbr[j] = &o->triangles[itmp];
		}
	}
#endif //TRIANGLE_HIERARCHY



    
	fclose(ifp);
}

void
read_tri_data(char *file,object_data *o){
    FILE *ifp;
    int i,j;
    int tmp[6];
    double ddum;
	double dtmp[256];
    int inbr[3],iprt=-1,icld[2]={-1,-1};
    double eb=0;
	extern triangle_data BOUNDARY;
    
	
    if ((ifp = fopen(file,"rb")) == NULL){
        printf("Cannot open file!\n");
        exit(0);
    }
	
	o->vertex_normal_status = ALIVE;
	o->nbr_status = ALIVE;
    
	fscanf(ifp,"%d%d%d",&o->num_of_ver,&o->num_of_init_tri,&o->num_of_process);
    fprintf(stderr,"%d %d %d\n",o->num_of_ver,o->num_of_init_tri,o->num_of_process);
    
    o->num_of_all_tri = o->num_of_init_tri + o->num_of_process * 4;
    
    o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
    o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_all_tri);
    
    for (i=0; i<o->num_of_ver; i++){	
        fscanf(ifp,"%lf%lf%lf%lf%lf%lf%lf%lf%lf",
			&dtmp[0],&dtmp[1],&dtmp[2],&dtmp[3],&dtmp[4],&dtmp[5],&ddum,&ddum,&ddum);
		o->vertices[i].pos[0] = dtmp[0];
		o->vertices[i].pos[1] = dtmp[1];
		o->vertices[i].pos[2] = dtmp[2];
		o->vertices[i].normal[0]= dtmp[3];
		o->vertices[i].normal[1] = dtmp[4];
		o->vertices[i].normal[2] = dtmp[5];
    }
    
    for (i=0; i<o->num_of_all_tri; i++){
        fscanf(ifp,"%d%d%d",tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->vertices[tmp[j]];
    }
	
#ifdef TRIANGLE_HIERARCHY
    if(o->num_of_process != 0){
        for (i=0; i<o->num_of_all_tri; i++){
            fscanf(ifp,"%d%d%d%d%d%d%lf",inbr,inbr+1,inbr+2,&iprt,icld,icld+1,&eb);
            for (j=0; j<3; j++){
                if (inbr[j] == -1) o->triangles[i].nbr[j] = &BOUNDARY;
                else o->triangles[i].nbr[j] = &o->triangles[inbr[j]];
            }
            if (iprt == -1) o->triangles[i].parent = &BOUNDARY;
            else o->triangles[i].parent = &o->triangles[iprt];
            for (j=0; j<2; j++){
                if (icld[j] == -1) o->triangles[i].children[j] = &BOUNDARY;
                else o->triangles[i].children[j] = &o->triangles[icld[j]];
            }
            o->triangles[i].error_value = eb;
        }
    }
	else{
		for (i=0; i<o->num_of_all_tri; i++){
			for (j=0; j<3; j++){
				if(fscanf(ifp,"%d",inbr)!=1){
					o->nbr_status = DEAD;
					continue;
				}
				if (inbr[0] == -1) o->triangles[i].nbr[j] = &BOUNDARY;
				else o->triangles[i].nbr[j] = &o->triangles[inbr[0]];
			}
		}
        o->triangles[i].error_value= 0;
	}
#else //TRIANGLE_HIERARCHY
	for (i=0; i<o->num_of_all_tri; i++){
		for (j=0; j<3; j++){
			if(fscanf(ifp,"%d",inbr)!=1){
				o->nbr_status = DEAD;
				continue;
			}
			if (inbr[0] == -1) o->triangles[i].nbr[j] = &BOUNDARY;
			else o->triangles[i].nbr[j] = &o->triangles[inbr[0]];
		}
	}
#endif //TRIANGLE_HIERARCHY
	fclose(ifp);
}



void write_cluster_data(char *file, object_data *o){
	FILE *fp;
	int i;
	double ddum=0;
	
	if ((fp = fopen(file,"wb")) == NULL){
		printf("Cannot open file!\n");
		exit(0);
	}
	
    fprintf(fp,"%d %d %d\n",o->num_of_ver,o->num_of_init_tri,o->num_of_process);
    
    for(i=0; i<o->num_of_ver; i++){	
        fprintf(fp,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
            o->vertices[i].pos[0],
            o->vertices[i].pos[1],
            o->vertices[i].pos[2],
            o->vertices[i].normal[0],
            o->vertices[i].normal[1],
            o->vertices[i].normal[2],
            ddum,ddum,ddum);
    }
    for(i=0; i<o->num_of_all_tri; i++){
        fprintf(fp,"%d %d %d\n",
            o->triangles[i].ver[0]->number,
            o->triangles[i].ver[1]->number,
            o->triangles[i].ver[2]->number
            );
    }
    
	for (i=0; i<o->num_of_all_tri; i++){
		fprintf(fp,"%d %d %d\n",
			o->triangles[i].nbr[0]->number,
			o->triangles[i].nbr[1]->number,
			o->triangles[i].nbr[2]->number
			);
	}
	fclose(fp);
}

void read_ascii_data(char *file,object_data *o){
    FILE *fp;
    int i,j;
    int tmp[6],itmp;
	char line[256];
	
	o->nbr_status = DEAD;
	o->vertex_normal_status = DEAD;
    
    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ascii file!\n");
        exit(0);
    }
	fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	fgets(line,256,fp);
    if(memcmp(line,"format ascii 1.0",16)!=0){
		printf("Not format ascii 1.0\n");
		exit(0);
	}
	
    while(fgets(line,256,fp) != NULL){
		if(memcmp(line,"element vertex",14)==0){
            sscanf(line,"element vertex %d",&o->num_of_ver);
        }
		if(memcmp(line,"element face",12)==0){
            sscanf(line,"element face %d",&o->num_of_init_tri);
        }
        if(memcmp(line,"end_header",6)==0){
			break;
        }
	}
	o->num_of_process = 0;
	o->num_of_all_tri = o->num_of_init_tri;
	
    fprintf(stderr,"%d %d %d\n",o->num_of_ver,o->num_of_init_tri,o->num_of_process);
    
    o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
    o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_all_tri);
	
	if(o->vertices == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
	if(o->triangles == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
    for (i=0; i<o->num_of_ver; i++){
		fgets(line,256,fp);
#ifdef FLOAT_VERTEX
        sscanf(line,"%f%f%f",
#else
		sscanf(line,"%lf%lf%lf",
#endif
            &o->vertices[i].pos[0],
            &o->vertices[i].pos[1],
            &o->vertices[i].pos[2]
			);
    }
    for (i=0; i<o->num_of_all_tri; i++){
		fgets(line,256,fp);
        sscanf(line,"%d%d%d%d",&itmp,tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->vertices[tmp[j]];
    }
    fclose(fp);
}


void read_ply_bindary_big_endian_data(char *file,object_data *o){
    FILE *fp;
    int i,j,cnt;
    int tmp[6],itmp;
	char line[256];
	char *tpoint;
	unsigned char uctmp;
	float ftmp[3];
	
	o->nbr_status = DEAD;
	o->vertex_normal_status = DEAD;
    
    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ply file!\n");
        exit(0);
    }
	fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	fgets(line,256,fp);
    if(memcmp(line,"format binary_big_endian 1.0",28)!=0){
		printf("Not format binary_big_endian 1.0\n");
		exit(0);
	}
	
    while(fgets(line,256,fp) != NULL){
		if(memcmp(line,"element vertex",14)==0){
            sscanf(line,"element vertex %d",&o->num_of_ver);
        }
		if(memcmp(line,"element face",12)==0){
            sscanf(line,"element face %d",&o->num_of_init_tri);
        }
        if(memcmp(line,"end_header",6)==0){
			break;
        }
	}
	o->num_of_process = 0;
	o->num_of_all_tri = o->num_of_init_tri;
	
    fprintf(stderr,"%d %d %d\n",o->num_of_ver,o->num_of_init_tri,o->num_of_process);
    
    o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
    o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_all_tri);
	
	if(o->vertices == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
	if(o->triangles == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}

    for (i=0; i<o->num_of_ver; i++){
		fread(line,sizeof(float)*6+sizeof(char)*3,1,fp);
		cnt = 0;
        for (j=0; j<3; j++){
			tpoint = (char *)ftmp;
			tpoint[3] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[0] = line[cnt++];
			o->vertices[i].pos[j] =  ftmp[0];
		}
	}
    for (i=0; i<o->num_of_all_tri; i++){
		fread(&uctmp,sizeof(char),1,fp);
		fread(line,sizeof(int)*uctmp+3,1,fp);
		if(uctmp != 3) printf("not triangle %d",uctmp);
		cnt = 0;
		for(j=0;j<3;j++){ 
			tpoint = (char *)&itmp;
			tpoint[3] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[0] = line[cnt++];
            o->triangles[i].ver[j] = &o->vertices[itmp];
		}
/*
        sscanf(line,"%d%d%d%d",&itmp,tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->vertices[tmp[j]];
*/    }
    fclose(fp);
}

void read_ply_bindary_little_endian_data(char *file,object_data *o){
    FILE *fp;
    int i,j,cnt;
    int tmp[6],itmp;
	char line[256];
	char *tpoint;
	unsigned char uctmp;
	float ftmp[3];
	
	o->nbr_status = DEAD;
	o->vertex_normal_status = DEAD;
    
    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ply file!\n");
        exit(0);
    }
	fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	fgets(line,256,fp);
    if(memcmp(line,"format binary_little_endian 1.0",28)!=0){
		printf("Not format binary_little_endian 1.0\n");
		exit(0);
	}
	
    while(fgets(line,256,fp) != NULL){
		if(memcmp(line,"element vertex",14)==0){
            sscanf(line,"element vertex %d",&o->num_of_ver);
        }
		if(memcmp(line,"element face",12)==0){
            sscanf(line,"element face %d",&o->num_of_init_tri);
        }
        if(memcmp(line,"end_header",6)==0){
			break;
        }
	}
	o->num_of_process = 0;
	o->num_of_all_tri = o->num_of_init_tri;
	
    fprintf(stderr,"%d %d %d\n",o->num_of_ver,o->num_of_init_tri,o->num_of_process);
    
    o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
    o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_all_tri);
	
	if(o->vertices == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
	if(o->triangles == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}

    for (i=0; i<o->num_of_ver; i++){
		fread(line,sizeof(float)*5,1,fp);
		cnt = 0;
        for (j=0; j<3; j++){
			tpoint = (char *)ftmp;
			tpoint[0] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[3] = line[cnt++];
			o->vertices[i].pos[j] =  ftmp[0];
		}
	}
    for (i=0; i<o->num_of_all_tri; i++){
		fread(&uctmp,sizeof(char),1,fp);
		fread(line,sizeof(int)*uctmp,1,fp);
		if(uctmp != 3) printf("not triangle %d",uctmp);
		cnt = 0;
		for(j=0;j<3;j++){ 
			tpoint = (char *)&itmp;
			tpoint[0] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[3] = line[cnt++];
            o->triangles[i].ver[j] = &o->vertices[itmp];
		}
/*
        sscanf(line,"%d%d%d%d",&itmp,tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->vertices[tmp[j]];
*/    }
    fclose(fp);
}

void read_ply_data(char *file,object_data *o){
    FILE *fp;
    int i,j;
    int tmp[6],itmp;
	char line[256];
	float ftmp[3];
	
	o->nbr_status = DEAD;
	o->vertex_normal_status = DEAD;
    
    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ply file!\n");
        exit(0);
    }
	fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	fgets(line,256,fp);
    if(memcmp(line,"format ascii 1.0",16)==0){
		fclose(fp);
		printf("Format ascii 1.0\n");
		//read_ply_ascii_data(file,o);
		exit(0);
		return ;
	}
    if(memcmp(line,"format binary_big_endian 1.0",28)==0){
		fclose(fp);
		printf("format binary_big_endian 1.0\n");
		read_ply_bindary_big_endian_data(file,o);
		return ;
	}
    if(memcmp(line,"format binary_little_endian 1.0",28)==0){
		fclose(fp);
		printf("format binary_little_endian 1.0\n");
		read_ply_bindary_little_endian_data(file,o);
		return ;
	}
	fclose(fp);
	exit(0);
}

#ifdef MAKE_TRIANGLE_HIERARCHY
void write_hierarchical_tri_data(char *file,object_data *wo, object_data *eo){
	FILE *ofp;
	int i,cnt=0;
	extern triangle_data BOUNDARY;
	vertex_data dummy;

	
	if ((ofp = fopen(file,"wb")) == NULL){
		printf("Cannot open file!\n");
		exit(0);
	}
	for (i=eo->num_of_all_tri-1; i>=0; i--){
		if(eo->triangles[i].parent == &BOUNDARY){
			eo->triangles[i].number = cnt++;
		}
		if(eo->triangles[i].newvertex == NULL) eo->triangles[i].newvertex = &dummy;
	}
	eo->num_of_init_tri = cnt;

	for (i=eo->num_of_all_tri-1; i>=0; i--){
		if(eo->triangles[i].parent != &BOUNDARY){
			eo->triangles[i].number = cnt++;
		}
		if(eo->triangles[i].newvertex == NULL) eo->triangles[i].newvertex = &dummy;
	}

	fprintf(ofp,"%d %d %d\n", wo->num_of_ver,eo->num_of_init_tri,eo->num_of_all_tri);
	
	for (i=0; i<wo->num_of_ver; i++){
		fprintf(
			ofp,"%lf %lf %lf %lf %lf %lf 0 0 0\n",
			wo->vertices[i].pos[0],
			wo->vertices[i].pos[1],
			wo->vertices[i].pos[2],
			wo->vertices[i].normal[0],
			wo->vertices[i].normal[1],
			wo->vertices[i].normal[2]
		);
	}
	

	BOUNDARY.number = -1;
	dummy.number = -1;
	cnt = 0;

	for (i=eo->num_of_all_tri-1; i>=0; i--){
		if(eo->triangles[i].parent == &BOUNDARY){
			fprintf(ofp,"%d %d %d %d %d %d %d\n",
				eo->triangles[i].ver[0]->number,
				eo->triangles[i].ver[1]->number,
				eo->triangles[i].ver[2]->number,
				eo->triangles[i].parent->number,
				eo->triangles[i].children[0]->number,
				eo->triangles[i].children[1]->number,
				eo->triangles[i].newvertex->number
			);
			if(cnt != eo->triangles[i].number){
				printf("write data broken\n");
				exit(-1);
			}
			++cnt;
		}
	}
	for (i=eo->num_of_all_tri-1; i>=0; i--){
		if(eo->triangles[i].parent != &BOUNDARY){
			fprintf(ofp,"%d %d %d %d %d %d %d\n",
				eo->triangles[i].ver[0]->number,
				eo->triangles[i].ver[1]->number,
				eo->triangles[i].ver[2]->number,
				eo->triangles[i].parent->number,
				eo->triangles[i].children[0]->number,
				eo->triangles[i].children[1]->number,
				eo->triangles[i].newvertex->number
			);
			if(cnt != eo->triangles[i].number){
				printf("write data broken\n");
				exit(-1);
			}
			++cnt;
		}
	}
	fclose(ofp);
}

#endif //MAKE_TRIANGLE_HIERARCHY

void write_tri_data(char *file,object_data *wo){
	FILE *ofp;
	int i,cnt=0;
	extern triangle_data BOUNDARY;
	vertex_data dummy;

	
	if ((ofp = fopen(file,"wb")) == NULL){
		printf("Cannot open file!\n");
		exit(0);
	}

	fprintf(ofp,"%d %d %d\n", wo->num_of_ver,wo->num_of_all_tri,0);
	
	for (i=0; i<wo->num_of_ver; i++){
		fprintf(
			ofp,"%lf %lf %lf %lf %lf %lf 0 0 0\n",
			wo->vertices[i].pos[0],
			wo->vertices[i].pos[1],
			wo->vertices[i].pos[2],
			wo->vertices[i].normal[0],
			wo->vertices[i].normal[1],
			wo->vertices[i].normal[2]
		);
	}

	for (i=0;i<wo->num_of_all_tri;i++){
			fprintf(ofp,"%d %d %d\n",
				wo->triangles[i].ver[0]->number,
				wo->triangles[i].ver[1]->number,
				wo->triangles[i].ver[2]->number
			);
	}
	fclose(ofp);
}

void
count_data(FILE *fp, object_data *o){
    char line[256];
    int count=0;
	//    int count_of_object = -1;
    int temp = 0;
    int itemp[8];
    
    while(fgets(line,256,fp) != NULL){
        switch(line[0]){
        case '#':
            break;
        case 'v':
            if(line[1] == 't'){
                o->num_of_tex_ver++;
				break;
            }
            if(line[1] == 'c'){
                o->Ncolor++;
				break;
            }
            o->num_of_ver++;
            break;
        case 'f':
            if(line[1] == ' '){
	            o->num_of_init_tri++;
				if(sscanf(line,"f %d//%d %d//%d %d//%d %d//%d",itemp,itemp+1,itemp+2,itemp+3,itemp+4,itemp+5,itemp+6,itemp+7) == 8){
					o->num_of_init_tri++;
				}
				break;
			}
            if(line[1] == '^'){
				o->number_of_cluster++;
				break;
			}

        }
    }
	
}

void
read_vertex_face_in_obj_file(FILE *fp, object_data *o){
    char line[256];
    double temp_pos[256];
    float *ftemp_pos;
    int itemp[8];
    int count_of_ver=0,count_of_tri=0,count_of_color=0;
//    int count_of_tex_ver=0,count_of_tex_before_ver= 1;
    int i;
    int readnumber;

    while(fgets(line,256,fp) != NULL){
        switch(line[0]){
        case '#':
            break;
        case 'g':
//            count_of_tri = 0;
//            count_of_ver = 0;
            break;
        case 'v':
            if(line[1] == 'c'){
				ftemp_pos = o->colors[count_of_color++].rgb;
				sscanf(line,"vc %f %f %f",ftemp_pos,ftemp_pos+1,ftemp_pos+2);
				//printf("vc %f %f %f\n",ftemp_pos[0],ftemp_pos[1],ftemp_pos[2]);
				break;
            }
            if(line[1] == 't'){

				break;
            }
			sscanf(line,"v %lf %lf %lf",temp_pos,temp_pos+1,temp_pos+2);
			o->vertices[count_of_ver].pos[0] = temp_pos[0];
			o->vertices[count_of_ver].pos[1] = temp_pos[1];
			o->vertices[count_of_ver].pos[2] = temp_pos[2];
            count_of_ver++;
            break;
        case 'f':
//            readnumber = sscanf(line,"f %d/%d %d/%d %d/%d",itemp,itemp+1,itemp+2,itemp+3,itemp+4,itemp+5);
//            readnumber = sscanf(line,"f %d//%d %d//%d %d//%d",itemp,itemp+1,itemp+2,itemp+3,itemp+4,itemp+5);
			readnumber = sscanf(line,"f %d//%d %d//%d %d//%d %d//%d",itemp,itemp+1,itemp+2,itemp+3,itemp+4,itemp+5,itemp+6,itemp+7);
            if(readnumber == 8){
                for (i=0; i<3; i++){
                    o->triangles[count_of_tri].ver[i] = &o->vertices[itemp[i*2]-1];
//                    o->triangles[count_of_tri].color[i] = &o->colors[itemp[i*2+1]-1];
                }
	            count_of_tri++;
                for (i=0; i<3; i++){
                    o->triangles[count_of_tri].ver[i] = &o->vertices[itemp[((i+2)%4)*2]-1];
//                    o->triangles[count_of_tri].color[i] = &o->colors[itemp[((i+2)%4)*2+1]-1];
                }
	            count_of_tri++;
				break;
			}
            if(readnumber == 6){ 
                for (i=0; i<3; i++){
                    o->triangles[count_of_tri].ver[i] = &o->vertices[itemp[i*2]-1];
//                    o->triangles[count_of_tri].color[i] = &o->colors[itemp[i*2+1]-1];
                }
	            count_of_tri++;
				break;
            }
            sscanf(line,"f %d %d %d",itemp,itemp+1,itemp+2);
            for (i=0; i<3; i++){
				o->triangles[count_of_tri].ver[i] = &o->vertices[itemp[i]-1];
			}
            count_of_tri++;
            break;
        }	
    }
	printf("%d",count_of_tri);
}



void
read_obj_data(char *file, object_data *o){
    FILE *fp;
	char line[256];
	int flag=0;
   
    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }

	do{
		fgets(line,256,fp);
		if(sscanf(line,"# faces: %d, points: %d, colours: %d",&o->num_of_init_tri,&o->num_of_ver,&o->Ncolor) == 3){
			flag =1;
			break;
		}
	}while(line[0] == '#');

	printf("f %d c %d v %d\n",o->num_of_init_tri,o->num_of_ver,o->Ncolor);

	fseek(fp,0,SEEK_SET);
	o->num_of_init_tri = o->num_of_ver = o->Ncolor = 0;
	count_data(fp,o);
	
	printf("f %d c %d v %d\n",o->num_of_init_tri,o->num_of_ver,o->Ncolor);
	
	o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
	////	o->texture_vertices = (texture_vertex_data *)malloc(sizeof(texture_vertex_data)*o->num_of_tex_ver);
	o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_init_tri);
	o->num_of_all_tri = o->num_of_init_tri;
	o->colors = (color_data *)malloc(sizeof(color_data)*o->Ncolor);
	
    fseek(fp,0,SEEK_SET);
    read_vertex_face_in_obj_file(fp,o);
    
    fclose(fp);
}


void
write_hierarchical_clusters_as_CMF(char *file,object_data *o){
    FILE *fp;
	int i;
	int cnt;
	vertex_data *v;
	cluster_data *c;
	triangle_data *t;


	if ((fp = fopen(file,"wb")) == NULL){
	  printf("Cannot open write cmf file!\n");
	  exit(-1);
	}

	fprintf(fp,"# dual_edges %d\n",o->num_of_all_tri-1);
	fprintf(fp,"# points %d\n",o->num_of_ver);
	fprintf(fp,"# faces %d\n",o->num_of_all_tri);
	
	v = o->vertices;
	c = o->clusters;
	t = o->triangles;
	cnt=0;

	for(i=0;i<o->num_of_ver;i++){
		v[i].number=i+1;
	}
	for(i=0;i<o->num_of_ver;i++){
		fprintf(fp,"v %lf %lf %lf\n",v[i].pos[0],v[i].pos[1],v[i].pos[2]);
	}

	for(i=o->number_of_cluster-1;i>=0;i--){
		c[i].check = DEAD;
	}
	for(i=0;i<o->num_of_init_tri;i++){
		c[t[i].cluster_number].check = ALIVE;
		c[t[i].cluster_number].number = ++cnt;
		fprintf(fp,"f %d %d %d\n",t[i].ver[0]->number,t[i].ver[1]->number,t[i].ver[2]->number);
	}
	for(i=o->number_of_cluster-1;i>0;i-=2){
		if(c[i].check == ALIVE && c[i-1].check == ALIVE){
			fprintf(fp,"f^ %d %d\n",c[i].number,c[i-1].number);
			fprintf(fp,"fc %e\n",0.0);
			fprintf(fp,"\tfe 1 0 0 0 1 0 0 0 1\n");
			c[i].parent->number = ++cnt;
			c[i].parent->check = ALIVE;
		}
		else{
			fprintf(stderr,"error multi resolution file is broken\n");
		}
	}
	fclose(fp);
//	exit(0);
}

void
read_vertex_face_in_cmf_file(FILE *fp, object_data *o){
    char line[256];
    double temp_pos[256];
    int itemp[8];
    int count_of_ver=0,count_of_tri=0,count_of_cluster=0;
//    int count_of_tex_ver=0,count_of_tex_before_ver= 1;
    int i;
    int readnumber;

    while(fgets(line,256,fp) != NULL){
        switch(line[0]){
        case '#':
            break;
        case 'v':
			sscanf(line,"v %lf %lf %lf",temp_pos,temp_pos+1,temp_pos+2);
			o->vertices[count_of_ver].pos[0] = temp_pos[0];
			o->vertices[count_of_ver].pos[1] = temp_pos[1];
			o->vertices[count_of_ver].pos[2] = temp_pos[2];
            count_of_ver++;
            break;
        case 'f':
			if(line[1] == '^'){
				sscanf(line,"f^ %d %d",itemp,itemp+1);
				o->clusters[count_of_cluster].children[0] = &o->clusters[itemp[0]-1];
				o->clusters[count_of_cluster].children[1] = &o->clusters[itemp[1]-1];
				o->clusters[itemp[0]-1].parent = &o->clusters[count_of_cluster];
				o->clusters[itemp[1]-1].parent = &o->clusters[count_of_cluster];
				o->clusters[count_of_cluster].parent = NULL;
				count_of_cluster++;
				break;
			}
			if(line[1] == ' '){
				readnumber = sscanf(line,"f%d%d%d",itemp,itemp+1,itemp+2);
				if(readnumber != 3){
					printf("Read Cmf File Error\n");
					break;
				}
				for (i=0; i<3; i++){
					o->triangles[count_of_tri].ver[i] = &o->vertices[itemp[i]-1];
				}
				o->clusters[count_of_cluster].parent = NULL;
				o->clusters[count_of_cluster].children[0] = NULL;
				o->clusters[count_of_cluster].children[1] = NULL;
#ifdef _MAKE_CLUSTER_HIERARCHY
				o->clusters[count_of_cluster].trianglelink = &o->triangles[count_of_tri];
				o->triangles[count_of_tri].cluster = &o->clusters[count_of_cluster];
#endif // MAKE_CLUSTER_HIERARCHY
				count_of_tri++;
				count_of_cluster++;
				break;
			}
        }	
    }
	printf("%d %d",count_of_tri,count_of_cluster);
}

void
read_cmf_data(char *file, object_data *o){
    FILE *fp;

    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }

	o->num_of_init_tri = o->num_of_ver = o->number_of_cluster = 0;

	count_data(fp,o);
	printf("f %d c %d v %d\n",o->num_of_init_tri,o->Ncolor,o->num_of_ver);

	o->num_of_all_tri = o->num_of_init_tri;
	o->number_of_cluster += o->num_of_init_tri;

	o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
	o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_init_tri);
	o->clusters = (cluster_data *)malloc(sizeof(cluster_data)*o->number_of_cluster);
	

	fseek(fp,0,SEEK_SET);
	read_vertex_face_in_cmf_file(fp,o);	
	fclose(fp);

}



void
read_smf_data(char *file, object_data *o){
    FILE *fp;
	char line[256];
	int flag=0;
	double *temp_pos;
	int itemp[3];
	int i;
   
    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }


	o->num_of_init_tri = o->num_of_ver = 0;
    while(fgets(line,256,fp) != NULL){
        switch(line[0]){
        case 'v':
            o->num_of_ver++;
            break;
        case 'f':
			o->num_of_init_tri++;
			break;
		}
   }
	printf("f %d v %d\n",o->num_of_init_tri,o->num_of_ver);
	fseek(fp,0,SEEK_SET);
	
	o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
	o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_init_tri);
	o->num_of_all_tri = o->num_of_init_tri;
	
	o->num_of_init_tri = o->num_of_ver = 0;
    while(fgets(line,256,fp) != NULL){
        switch(line[0]){
        case '#':
            break;
        case 'v':
			sscanf(line,"v %lf %lf %lf",temp_pos,temp_pos+1,temp_pos+2);
			o->vertices[o->num_of_ver].pos[0] = temp_pos[0];
			o->vertices[o->num_of_ver].pos[1] = temp_pos[1];
			o->vertices[o->num_of_ver].pos[2] = temp_pos[2];
            o->num_of_ver++;
            break;
        case 'f':
            sscanf(line,"f%d%d%d",itemp,itemp+1,itemp+2);
            for (i=0; i<3; i++){
				o->triangles[o->num_of_init_tri].ver[i] = &o->vertices[itemp[i]-1];
			}
            o->num_of_init_tri++;
            break;
        }	
    }
    
    fclose(fp);
}


void
read_stl_data(char *file, object_data *o){
    FILE *fp;
	char line[256];
	unsigned int itmp=0;
	int cnt = 0;
	int i,j;
	float ftmp[12];

	float *vpos;
	int *id;
	int nh;
	heap_data *heap;
	int *tid;
	float tpx;
	int ttid;
	bool isAscii = false;
	char header[126];


    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }
	fread(&header,sizeof(char),10,fp);
	if(strstr(header,"solid")){
		isAscii=true;
		fseek(fp,0,SEEK_SET);
		itmp=0;
		while(fgets(line,256,fp)!=NULL){
			if( strstr(line,"facet normal") ){
				itmp++;
			}
		}
		fseek(fp,0,SEEK_SET);
	}
	else{
		fseek(fp,80,SEEK_SET);
		o->num_of_init_tri = o->num_of_ver = o->number_of_cluster = 0;
		fread(&itmp,4,1,fp);
	}
		cnt = 0;
		o->num_of_init_tri = itmp;

	printf("%d\n",o->num_of_init_tri);


	id = (int *)malloc(sizeof(int)*o->num_of_init_tri*3);
	tid = (int *)malloc(sizeof(int)*o->num_of_init_tri*3);
	heap = (heap_data *)malloc(sizeof(heap_data)*(o->num_of_init_tri*3+1));
	vpos = (float *)malloc(sizeof(float)*9*o->num_of_init_tri);

	if(vpos == NULL) {
		printf("not enough memory stl\n");
		exit(0);
	}
	
	if(isAscii){
		int temp=0;
		while(fgets(line,256,fp)!=NULL){
			if( strstr(line,"vertex") ){
				sscanf(line,"vertex%f%f%f",vpos+temp,vpos+temp+1,vpos+temp+2);
				temp += 3;
			}
		}
	}		
	else{
		for(i=0;i<o->num_of_init_tri;i++){
			fread(&ftmp,sizeof(float),12,fp);
			for(j=3;j<12;j++){
				vpos[i*9+j-3] = ftmp[j];
			}
			fread(&ftmp,2,1,fp);
		}
	}
	
	nh = 0;
	for(i=0;i<o->num_of_init_tri*3;i++){
		id[i] = i;
		nh++;
		heap[nh].cluster_number = i;
		heap[nh].quadrics = vpos[i*3];
		upheap(nh,heap,nh);
	}

	tpx = heap[1].quadrics;
	cnt = 0;
	for(i=0;i<o->num_of_init_tri*3;i++){
		heap[1] = heap[nh];
		downheap(--nh,heap,1);
		if(heap[1].quadrics == tpx){
			ttid = tid[cnt++] =  heap[1].cluster_number;

			for(j=0;j<cnt-1;j++){
				if( (vpos[ttid*3] == vpos[tid[j]*3]) && (vpos[ttid*3+1] == vpos[tid[j]*3+1]) && (vpos[ttid*3+2] == vpos[tid[j]*3+2]) ){
					if(id[ttid] < id[tid[j]]) id[tid[j]] = id[ttid];
					else id[ttid] = id[tid[j]];

				}
			}

		}else{
			cnt = 0;
			tpx = heap[1].quadrics;
			tid[cnt++] =  heap[1].cluster_number;
		}
	}

	cnt = 0;

	for(i=0;i<o->num_of_init_tri*3;i++){
		if(id[i] == i) id[i] = cnt++;
		else id[i] = id[id[i]];
	}
	printf("%d\n",cnt);

	o->num_of_ver = cnt;
	o->num_of_all_tri = o->num_of_init_tri;
	o->number_of_cluster += o->num_of_init_tri;

	o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
	o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_init_tri);
	o->clusters = (cluster_data *)malloc(sizeof(cluster_data)*o->number_of_cluster);

	for(i=0;i<o->num_of_init_tri*3;i++){
		for(j=0;j<3;j++){
			o->vertices[id[i]].pos[j] = vpos[i*3+j];
		}
	}

	for(i=0;i<o->num_of_init_tri;i++){
		for(j=0;j<3;j++){
			o->triangles[i].ver[j] = &o->vertices[id[3*i+j]];
		}
	}

	free(vpos);
	free(id);
	free(tid);
	free(heap);

	fclose(fp);


/*

		cnt = 0;
        for (j=0; j<3; j++){
			tpoint = (char *)ftmp;
			tpoint[0] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[3] = line[cnt++];
			o->vertices[i].pos[j] =  ftmp[0];
		}




	printf("f %d c %d v %d\n",o->num_of_init_tri,o->Ncolor,o->num_of_ver);

	o->num_of_all_tri = o->num_of_init_tri;
	o->number_of_cluster += o->num_of_init_tri;

	o->vertices = (vertex_data *)malloc(sizeof(vertex_data)*o->num_of_ver);
	o->triangles = (triangle_data *)malloc(sizeof(triangle_data)*o->num_of_init_tri);
	o->clusters = (cluster_data *)malloc(sizeof(cluster_data)*o->number_of_cluster);
	

	fseek(fp,0,SEEK_SET);
	read_vertex_face_in_cmf_file(fp,o);	
	fclose(fp);
*/
}


void
read_mesh_data(char *name,object_data *o){
//	char *detect;
	
	//detect = strchr(name, '.' );
	if(strstr(name,".ply")){
		strcpy(o->filetype,"ply");
		read_ply_data(name,o);
		return;
	}
	if(strstr(name,".tri")){
		strcpy(o->filetype,"tri");
		read_tri_data(name,o);
		return;
	}
	if(strstr(name,".obj")){
		strcpy(o->filetype,"obj");
		read_obj_data(name,o);
		return;
	}
	if(strstr(name,".cmf")){
		strcpy(o->filetype,"cmf");
		read_cmf_data(name,o);
		return;
	}
	if(strstr(name,".smf")){
		strcpy(o->filetype,"smf");
		read_smf_data(name,o);
		return;
	}
	if(strstr(name,".stl")){
		strcpy(o->filetype,"stl");
		read_stl_data(name,o);
		return;
	}
#ifdef VIEW_CLUSTER_HIERARCHY
	if(strstr(name,".vch")){
		strcpy(o->filetype,"vch");
		read_vch_data(name,o);
		return;
	}
#endif

	printf("UnKnown Format %s\n ",name);
	exit(-1);

}



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <GL/glut.h>


#include "set_define.h"
#include "struct.h"
//#include "mymath.h"
#include "mymath_float.h"
//#include "tri_ver_math.h"
#include "list_handle.h"
#include "file_handle.h"



extern float 
    EYEPOS[3],
    INIT_EYEPOS[3],
    VIEWCENTER[3],
    LOOKAT_DIRECTION[3],
    SKY_DIRECTION[3],
    SIDE_DIRECTION[3],
    GAZE_DIRECTION[3];

extern float
	LEFT_CULLING[3],
	RIGHT_CULLING[3],
	UPPER_CULLING[3],
	UNDER_CULLING[3];

//float BOUNDING_SQUARE[4][3];

extern int MOUSE_POS[2];
//double THRESHOLD;

extern triangle_data BOUNDARY;
extern vertex_data   DUMMY_VERTEX,*TMP_VER;

extern double ORTHO_MIN[3],ORTHO_MAX[3];
extern int DISPLAY_SIZE[2];

extern int Counter,Counter2;
double  Error=0;


float SKY_DIRECTION_RASTER[3], SIDE_DIRECTION_RASTER[3];

double THRESHOLD = 0.99;

static view_cluster_data *CQ[1000000];
static GLuint qid[150000];
static float size[150000];
int TOP=0,END=0,QCNT=0;
//global variable for lod
/*int i;
view_vertex_tree_link_data *vl,*tvl;
view_vertex_tree_data *vt,*tvt,*v1;
int bn,cn;
float *normal;
*/
/**********************************************

‰ð‘œ“x§Œä•”

**********************************************/



void control_cluster_level_by_number(object_data *o, int cnt){
	static int top,end;
	static view_cluster_data **cq,*c;
	static int i;
	static view_vertex_tree_link_data *vl;
	static view_vertex_tree_data *vt;
	static view_vertex_tree_link_data *tvl;
	static view_vertex_tree_data *tvt,*v1;
	int bn;
	static int cnt1=0;
	extern int WIRE_FRAME;
	//extern double tor;
	static double tor=o->view_clusters[0].quadrics_error;

	double mx,my;
	extern int MOUSE_POS[2];
	extern int DISPLAY_SIZE[2];

	extern int TOP,END;

	extern int polygonNum;

//	WIRE_FRAME = 0;

//	printf("test\n");

	printf("%d\n",cnt1);


	o->Nclusterque = 0;
	top = TOP;
	end = END;
//	cq = o->clusterque;
	cq = CQ;
	vl = o->view_verlink;
	vt = o->view_vertex_trees;

//	cnt1+=cnt1;

	mx = o->size*2.0*((double)MOUSE_POS[0]/DISPLAY_SIZE[0]-0.5);
	my = -o->size*2.0*((double)MOUSE_POS[1]/DISPLAY_SIZE[1]-0.5);

	cq[end++] = &o->view_clusters[0];
	

	cnt1+=cnt1/100+1;

	while(top!=end){
		c = cq[top++];
//		if(c->children == NULL || end > polygonNum || tor > c->quadrics_error){
		if(c->children == NULL ||  tor > c->quadrics_error){
			c->check=ALIVE;
			continue;
		}
//#ifdef MOUSE
//		if(c->children == NULL || (c->position_range+o->size/20.0) < sqrt( (c->center[0]-mx)*(c->center[0]-mx)+(c->center[1]-my)*(c->center[1]-my)) ){
//#else
//#ifdef PROGRESSIVE
//		  if(c->quadrics_error < tor){
//			  c->check=ALIVE;
//			  continue;
//		  }
			  
//#endif
		    //		  if(c->children == NULL){


		for(i=c->verlinknumber;i<(c->verlinknumber+c->Ncluster_ver_link);i++){
			tvt = &vt[i];
			tvt->check=1;
			if(tvt->vertex->check++) continue;

			tvl = &vl[i];

			v1 = tvl->parent;
			
			if(v1){
				bn = tvt->boundary_order;
				while(v1->next && bn > v1->next->boundary_order) v1 = v1->next;
				tvt->next = v1->next;
				v1->next = tvt;
			}
			
			if(!tvl->alias) continue;
			tvt = tvl->alias;
			v1 = tvl->reverse_parent;
			
			bn = tvt->boundary_order;
			while(v1->next && bn > v1->next->boundary_order) v1 = v1->next;
			tvt->next = v1->next;
			v1->next = tvt;
		}
		c->check = DEAD;
		cq[end++] = &c->children[0];
		cq[end++] = &c->children[1];
	}
//	o->Nclusterque = end;
	tor = (tor/1.001 > cq[end-1]->quadrics_error/1.001) ? tor/1.001 : cq[end-1]->quadrics_error/1.001;
//	tor = tor/1.001 ;

	TOP = END = end;
}

void display_cluster_que(object_data *o){
	view_vertex_tree_data *vt;
	view_cluster_data **cq,*c;
	int i,j;

	Error = 0;

	cq = CQ;
	for(i=0;i<END;i++){
		if(cq[i]->check != ALIVE) continue;
		c = cq[i];

		Counter++;
		Error += c->quadrics_error*c->area;

		if(c->children != NULL){
			glBegin(GL_TRIANGLE_FAN);
			glNormal3fv(c->normal);
			
			glVertex3fv(c->center);
			for(j=0;j<c->number_of_vertex;j++){
				vt = c->polygon[j];
				glNormal3fv(vt->vertex->normal);
				//printf("%lf\n",vt->vertex->normal);
				glVertex3fv(vt->vertex->pos);
				while(vt->next && !vt->next->check){
					vt = vt->next;
					glNormal3fv(vt->vertex->normal);
					glVertex3fv(vt->vertex->pos);
					//printf("%lf\t%lf\t%lf\n",vt->vertex->normal[0],vt->vertex->normal[1],vt->vertex->normal[2]);
				}
			}
			vt = c->polygon[0];
			glNormal3fv(vt->vertex->normal);
			glVertex3fv(vt->vertex->pos);
			glEnd();
		}else{
			glBegin(GL_TRIANGLE_FAN);
//			glNormal3fv(c->normal);
//			glVertex3fv(c->center);
			for(j=0;j<c->number_of_vertex;j++){
				vt = c->polygon[j];
				glNormal3fv(vt->vertex->normal);
				glVertex3fv(vt->vertex->pos);
				while(vt->next && !vt->next->check){
					vt = vt->next;
					glNormal3fv(vt->vertex->normal);
					glVertex3fv(vt->vertex->pos);
				}
			}
			glEnd();
		}
	}
//	Counter2++;
}


int write_stl_from_cluster_que(object_data *o){
	view_vertex_tree_data *vt;
	view_cluster_data **cq,*c;
	int i,j;
	int count=0,count2=0;
	char header[80] = {0};
	char zero[16] = {0};
	float *pos[100];
	FILE *fp;

	cq = CQ;
	for(i=0;i<END;i++){
		if(cq[i]->check != ALIVE) continue;
		c = cq[i];

		count2=0;

		if(c->children != NULL){
//			glBegin(GL_TRIANGLE_FAN);
//			glNormal3fv(c->normal);
//			glVertex3fv(c->center);
			for(j=0;j<c->number_of_vertex;j++){
				vt = c->polygon[j];
				count2++;
//				glNormal3fv(vt->vertex->normal);
//				glVertex3fv(vt->vertex->pos);
				while(vt->next && !vt->next->check){
					vt = vt->next;
					count2++;
					//glNormal3fv(vt->vertex->normal);
					//glVertex3fv(vt->vertex->pos);
					//printf("%lf\t%lf\t%lf\n",vt->vertex->normal[0],vt->vertex->normal[1],vt->vertex->normal[2]);
				}
			}
			vt = c->polygon[0];
			//glNormal3fv(vt->vertex->normal);
			//glVertex3fv(vt->vertex->pos);
			//glEnd();
			count += (count2 > 2) ? count2 : 0;
		}else{
//			glBegin(GL_TRIANGLE_FAN);
////			glNormal3fv(c->normal);
////			glVertex3fv(c->center);
			for(j=0;j<c->number_of_vertex;j++){
				vt = c->polygon[j];
				count2++;
//				glNormal3fv(vt->vertex->normal);
//				glVertex3fv(vt->vertex->pos);
				while(vt->next && !vt->next->check){
					vt = vt->next;
					count2++;
//					glNormal3fv(vt->vertex->normal);
//					glVertex3fv(vt->vertex->pos);
				}
//				count--;
			}
//			glEnd();
			count2 -= 2;
			count += (count2 > 0) ? count2 : 0;
		}
	}

	fp = fopen("output.stl","wb");

	fwrite(header,1,80,fp);
	fwrite(&count,4,1,fp);

	printf("%d\n",count);

	cq = CQ;
	for(i=0;i<END;i++){
		if(cq[i]->check != ALIVE) continue;
		c = cq[i];

		Counter++;
		Error += c->quadrics_error*c->area;
		count2 = 0;

		if(c->children != NULL){
			for(j=0;j<c->number_of_vertex;j++){
				vt = c->polygon[j];
				pos[count2++] = vt->vertex->pos;
				while(vt->next && !vt->next->check){
					vt = vt->next;
					pos[count2++] = vt->vertex->pos;
				}
			}
			vt = c->polygon[0];


		}else{
			for(j=0;j<c->number_of_vertex;j++){
				vt = c->polygon[j];
				pos[count2++] = vt->vertex->pos;
				while(vt->next && !vt->next->check){
					vt = vt->next;
					pos[count2++] = vt->vertex->pos;
				}
			}
		}

		if(count2 > 2){
			if(c->children != NULL){
				for(j=0;j<count2;j++){
					fwrite(c->normal,sizeof(float),3,fp);
					fwrite(c->center,sizeof(float),3,fp);
					fwrite(pos[j],sizeof(float),3,fp);
					fwrite(pos[(j+1)%count2],sizeof(float),3,fp);
					fwrite(zero,1,2,fp);
				}
			}else{
				for(j=1;j<count2-1;j++){
					fwrite(c->normal,sizeof(float),3,fp);
					fwrite(pos[0],sizeof(float),3,fp);
					fwrite(pos[j],sizeof(float),3,fp);
					fwrite(pos[(j+1)%count2],sizeof(float),3,fp);
					fwrite(zero,1,2,fp);
				}
			}
		}

	}
	fclose(fp);

	return count;
//	Counter2++;
}




void display_polygon_cluster_que(object_data *o){
	view_vertex_tree_data *vt;
	view_cluster_data **cq,*c;
	int i,j;

	cq = CQ;
	for(i=0;i<END;i++){
		if(cq[i]->check != ALIVE) continue;
		Counter++;
		c = cq[i];
		glBegin(GL_POLYGON);
		for(j=0;j<c->number_of_vertex;j++){
			vt = c->polygon[j];
			glNormal3fv(vt->vertex->normal);
			glVertex3fv(vt->vertex->pos);
			while(vt->next && !vt->next->check){
				vt = vt->next;
				glNormal3fv(vt->vertex->normal);
				glVertex3fv(vt->vertex->pos);
			}
		}
		glEnd();
	}
//	Counter2++;
}

void display_occluder_cluster_que(object_data *o){
	view_vertex_tree_data *vt;
	view_cluster_data **cq,*c;
	int i,j;

	cq = CQ;
	for(i=0;i<END;i++){
		if(cq[i]->check != ALIVE) continue;
		c = cq[i];
		glBegin(GL_TRIANGLE_FAN);
		glVertex3fv(c->center);
		for(j=0;j<c->number_of_vertex;j++){
			vt = c->polygon[j];
			glVertex3fv(vt->vertex->pos);
			while(vt->next && !vt->next->check){
				vt = vt->next;
				glVertex3fv(vt->vertex->pos);
			}
		}
		glVertex3fv(c->polygon[0]->vertex->pos);
		glEnd();
	}
}

void reset_cluster_que(object_data *o){
	register view_vertex_tree_link_data *vl,*tvl;
	register view_vertex_tree_data *vt,*tvt;
	register view_cluster_data **cq,*c;
	int i,j;

//	cq = o->clusterque;
	vl = o->view_verlink;
	vt = o->view_vertex_trees;

	cq = CQ;
	for(i=0;i<END;i++){
//	for(i=0;i<o->Nclusterque;i++){
		if(cq[i]->check !=DEAD) continue;
		c = cq[i];
		for(j=0;j<c->Ncluster_ver_link;j++){
			tvt = &vt[c->verlinknumber+j];
			tvl = &vl[c->verlinknumber+j];
			tvt->check = tvt->vertex->check = 0;
			tvt->next = NULL;
			if(tvl->parent) tvl->parent->next = NULL;
			if(tvl->alias){
				tvl->alias->next = NULL;
				tvl->reverse_parent->next = NULL;
			}
		}
	}
//	Counter2++;
}

void reset_vertex_normal_recursive(view_cluster_data *c){
//	int j,flag0=DEAD,flag1=DEAD;
	int j;
	float *normal;
	view_vertex_tree_data *vt;


	if(c==NULL) return;

	if(c->check == DEAD){
		reset_vertex_normal_recursive(&c->children[0]);
		reset_vertex_normal_recursive(&c->children[1]);
		return;
	}
	if(c->check != ALIVE) return;
	if(c->number_of_vertex < 1) return;

	for(j=0;j<c->number_of_vertex;j++){
		vt = c->polygon[j];
		normal = vt->vertex->normal;
		normal[0] = normal[1] = normal[2] = 0;
	}
}

void make_vertex_normal_recursive(view_cluster_data *c){
//	int j,flag0=DEAD,flag1=DEAD;
	int j;
	float *normal;
	view_vertex_tree_data *vt;


	if(c==NULL) return;

	if(c->check == DEAD){
		make_vertex_normal_recursive(&c->children[0]);
		make_vertex_normal_recursive(&c->children[1]);
		return;
	}
	if(c->check != ALIVE) return;
	if(c->number_of_vertex < 1) return;

	for(j=0;j<c->number_of_vertex;j++){
		vt = c->polygon[j];
		normal = vt->vertex->normal;
		normal[0] += c->normal[0];
		normal[1] += c->normal[1];
		normal[2] += c->normal[2];
		while(vt->next && !vt->next->check){
			vt = vt->next;
			normal = vt->vertex->normal;
			normal[0] += c->normal[0];
			normal[1] += c->normal[1];
			normal[2] += c->normal[2];
		}
	}
}

void make_unit_vertex_normal_recursive(view_cluster_data *c){
//	int j,flag0=DEAD,flag1=DEAD;
	int j;
	float *normal;
	view_vertex_tree_data *vt;


	if(c==NULL) return;

	if(c->check == DEAD){
		make_unit_vertex_normal_recursive(&c->children[0]);
		make_unit_vertex_normal_recursive(&c->children[1]);
		return;
	}
	if(c->check != ALIVE) return;
	if(c->number_of_vertex < 1) return;

	for(j=0;j<c->number_of_vertex;j++){
		vt = c->polygon[j];
		normal = vt->vertex->normal;
		make_unit_vector_float(normal);
	}
}




void display_lod_model_by_number(object_data *o){
	static int flame=0;
	extern int WIRE_FRAME;
	static int cnt=0,cnt1=0;
	extern int SAVE_STL;
	static int first=1;
	
	extern char *outputFileName;

	object_data output;


//	int i;
//	view_vertex_data *v;

	GLfloat
		white[4] = {1.0, 1.0, 1.0, 1.0},
		ambient[4] = {0.3, 0.3, 0.3, 1.0},
		diffuse[4] = {1.0, 1.0, 1.0, 1.0},
		specular[4] = {1.0, 1.0, 1.0, 1.0},
		ns = 255;
	GLfloat mtr3[4] = {0.0,0.0,0.0,1.0};
	GLfloat mtr2[4] = {1.0,1.0,1.0,1.0};
	GLfloat mtr1[4] = {0.8,0.8,0.8,1.0};
//	GLfloat white[4] = {1.0,1.0,1.0,1.0};



	control_cluster_level_by_number(o, cnt);
//	cnt = 5;
	cnt += cnt/100 +2;
//	cnt = 200;

	if(WIRE_FRAME){
		glEnable(GL_LINE_SMOOTH);
//		glLineWidth(cnt1++);
//		glLineWidth(20);
//		glLightfv(GL_LIGHT0,GL_AMBIENT,white);
//		glLightfv(GL_LIGHT0,GL_DIFFUSE,white);
//		glLightfv(GL_LIGHT0,GL_SPECULAR,white);
//		glLightfv(GL_LIGHT0,GL_POSITION,white);
//		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
//		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr1);
//		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
//		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
//		glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
//		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
		
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
//		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr2);
//		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr2);

		glColorMask(GL_FALSE,GL_FALSE,GL_FALSE,GL_FALSE);

		display_cluster_que(o);
//		display_cluster_recursive_for_polygon(&o->view_clusters[0]);
	    glColorMask(GL_TRUE,GL_TRUE,GL_TRUE,GL_TRUE);

		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
//		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr3);
		Counter = 0;

		glCullFace(GL_BACK);
		glEnable(GL_CULL_FACE);
		glEnable(GL_POLYGON_OFFSET_LINE);
		glPolygonOffset(-10, -100);

		display_polygon_cluster_que(o);
		reset_cluster_que(o);
		fprintf(stdout,"%d, %lf\n",Counter, Error/o->view_clusters[0].area);


		return ;

	}


	Counter = 0;
	Error = 0;

//	reset_vertex_normal_recursive(o->view_clusters);
//	make_vertex_normal_recursive(o->view_clusters);
//	make_unit_vertex_normal_recursive(o->view_clusters);

	display_cluster_que(o);
//	if(SAVE_STL==1){
//	if(write_stl_from_cluster_que(o) > 16000){

	extern int TOP,END;
	extern int polygonNum;

	if(first && (END > polygonNum)){
		first=0;
		write_stl_from_cluster_que(o);
		read_stl_data("output.stl",&output);
		write_vrml_data(outputFileName,&output);
		exit(0);
	}

	


	reset_cluster_que(o);

//	fprintf(stdout,"%d, %lf\n",Counter, Error/o->view_clusters[0].area);
//	if(Counter >= 3000) exit (0);

}



void
read_vec_data(char *file, object_data *o){
    FILE *fp;
	char line[1024],*string,*stopstring;
	int i,j,cnt1,cnt2,tmp[256];
	double dtmp;
	view_vertex_data *v;
	view_vertex_tree_data *vt;
	view_cluster_data *c;
	view_vertex_tree_data **p;
	view_vertex_tree_link_data *vl;
	static int n=0;
	//	HANDLE hFileMap;
	char mapname[256];
	char *top;

    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open vec file!!  \n");
        exit(0);
    }


	fgets(line,1024,fp);
	sscanf(line,"%d%d%d%d%d",&o->num_of_ver,&o->Ntree_verticies,&o->number_of_cluster,&o->Nvpolygon,&o->Nvertexlink);
    fprintf(stderr,"%d %d %d %d %d\n",o->num_of_ver,o->Ntree_verticies,o->number_of_cluster,o->Nvpolygon,o->Nvertexlink);

	v = o->view_verticies = (view_vertex_data *)malloc(sizeof(view_vertex_data)*o->num_of_ver);
	vt = o->view_vertex_trees = (view_vertex_tree_data *)malloc(sizeof(view_vertex_tree_data)*o->Ntree_verticies);
	c = o->view_clusters = (view_cluster_data *)malloc(sizeof(view_cluster_data)*o->number_of_cluster);
	p = o->view_polygons = (view_vertex_tree_data **)malloc(sizeof(view_vertex_tree_data *)*o->Nvpolygon);
	vl = o->view_verlink = (view_vertex_tree_link_data *)malloc(sizeof(view_vertex_tree_link_data)*o->Nvertexlink);
	o->clusterque = (view_cluster_data **)malloc(sizeof(view_cluster_data *)*o->number_of_cluster);


	cnt1 = cnt2 = 0;
	cnt1 = sizeof(view_vertex_data)*o->num_of_ver; cnt2 += cnt1;
	printf("vsize \t%d  * \t%d = \t%d \n",sizeof(view_vertex_data),o->num_of_ver,cnt1);

	cnt1 =  sizeof(view_vertex_tree_data)*o->Ntree_verticies; cnt2 += cnt1;
	printf("vtsize \t%d  * \t%d = \t%d\n",sizeof(view_vertex_tree_data),o->Ntree_verticies,cnt1);

	cnt1 = sizeof(view_cluster_data)*o->number_of_cluster; cnt2 += cnt1;
	printf("csize \t%d  * \t%d = \t%d\n",sizeof(view_cluster_data),o->number_of_cluster,cnt1);

	cnt1 = sizeof(view_vertex_tree_data *)*o->Nvpolygon; cnt2 += cnt1;
	printf("pvsize \t%d  * \t%d = \t%d\n",sizeof(view_vertex_tree_data *),o->Nvpolygon,cnt1);

	cnt1 = sizeof(view_vertex_tree_link_data)*o->Nvertexlink; cnt2 += cnt1;
	printf("vlsize \t%d  * \t%d = \t%d\n",sizeof(view_vertex_tree_link_data),o->Nvertexlink,cnt1);

//	cnt1 = sizeof(view_cluster_data *)*o->number_of_cluster; cnt2 += cnt1;
//	printf("cqsize \t%d  * \t%d = \t%d\n",sizeof(view_cluster_data *),o->number_of_cluster,cnt1);

	printf("datasize \t%d \t%d\n", cnt2, cnt2*2/o->number_of_cluster);
	sprintf(mapname,"rvd%d",n++);
/*
	o->hFileMap = CreateFileMapping( (HANDLE)0xFFFFFFFF, NULL, PAGE_READWRITE, 0, cnt2 , mapname) ;
	if(o->hFileMap==NULL) exit(-1);

	top = (char *)MapViewOfFile( o->hFileMap, FILE_MAP_WRITE, 0, 0, 0 );

	cnt1=cnt2=0;

	v = o->view_verticies = top; 	cnt2 += sizeof(view_vertex_data)*o->num_of_ver;
	vt = o->view_vertex_trees = &top[cnt2];	cnt2 +=  sizeof(view_vertex_tree_data)*o->Ntree_verticies; 
	c = o->view_clusters = &top[cnt2];	cnt2 += sizeof(view_cluster_data)*o->number_of_cluster; 
	p = o->view_polygons = &top[cnt2]; 	cnt2 += sizeof(view_vertex_tree_data *)*o->Nvpolygon;
	vl = o->view_verlink = &top[cnt2];
*/

	cnt1 = cnt2 = 0;
	for(i=0;i<o->num_of_ver;i++){
		fgets(line,1024,fp);
        sscanf(line,"%f%f%f%f%f%f%d%d",
            &v[i].pos[0],
            &v[i].pos[1],
            &v[i].pos[2],
            &v[i].normal[0],
            &v[i].normal[1],
            &v[i].normal[2],
			&tmp[0],
			&tmp[1]
			);
//		v[i].bothcluster = (tmp[0] == -1) ? NULL : &c[tmp[0]];
//		v[i].ring_number = tmp[1];
    }

	for(i=0;i<o->Ntree_verticies;i++){
		fgets(line,1024,fp);
        sscanf(line,"%d%d%d%d%d",&tmp[0],&tmp[1],&tmp[2],&tmp[3],&tmp[4]);
		vt[i].vertex = (tmp[0] == -1) ? NULL : &v[tmp[0]];
//		vt[i].vnumber = tmp[0];
		vt[i].boundary_order = tmp[1];
		if(tmp[1] > 65535) printf("too large boundary\n");

		vt[i].check = 0;
		vt[i].next = NULL;


		if(i>=o->Nvertexlink) continue;

//		vl[i].myself = &vt[i];
		vl[i].parent = (tmp[2] == -1) ? NULL : &vt[tmp[2]];
		vl[i].reverse_parent = (tmp[3] == -1) ? NULL : &vt[tmp[3]];
		vl[i].alias = (tmp[4] == -1) ? NULL : &vt[tmp[4]];

		if(vl[i].reverse_parent == vl[i].alias) vl[i].reverse_parent = vl[i].alias = NULL;

		if(vl[i].parent == &vt[i]){
			vl[i].parent = &vt[i];
			vl[i].parent = NULL;
//			vl[i].parent = NULL;
//			vt[i].next = &vt[i];
//			vt[i].check = -10;
//			vl[i].alias->
		}

    }

	cnt1 = cnt2 = 0;

	for(i=0;i<o->number_of_cluster;i++){
		fgets(line,1024,fp); 
		string = line;
		tmp[0] = strtol(string,&stopstring,0); string = stopstring;
		//c[i].parent = (tmp[0] == -1) ? NULL : &c[tmp[0]];
		tmp[0] = strtol(string,&stopstring,0); string = stopstring;	c[i].children = (tmp[0] == -1) ? NULL : &c[tmp[0]];
		tmp[0] = strtol(string,&stopstring,0); string = stopstring;	
		//c[i].children[1] = (tmp[0] == -1) ? NULL : &c[tmp[0]];

		c[i].number_of_vertex = strtol(string,&stopstring,0); string = stopstring;
		c[i].polygon = &p[cnt1];
		for(j=0;j<c[i].number_of_vertex;j++){
			tmp[0] = strtol(string,&stopstring,0); string = stopstring; p[cnt1++] = &vt[tmp[0]];
		}

		c[i].Ncluster_ver_link = strtol(string,&stopstring,0); string = stopstring;
//		c[i].verlink = &vl[cnt2];
		c[i].verlinknumber = cnt2;
		for(j=0;j<c[i].Ncluster_ver_link;j++){
			tmp[0] = strtol(string,&stopstring,0); string = stopstring; 
			if(tmp[0] != cnt2++) printf("eroor\n");
		}

		dtmp = strtod(string,&stopstring); string = stopstring;	c[i].position_range = dtmp;
		for(j=0;j<3;j++){
				dtmp = strtod(string,&stopstring); string = stopstring; c[i].center[j] = dtmp;
		}
		dtmp = strtod(string,&stopstring); string = stopstring;	c[i].direction_range = dtmp;
		for(j=0;j<3;j++){
				dtmp = strtod(string,&stopstring); string = stopstring;	c[i].normal[j] = dtmp;
		}
		dtmp = strtod(string,&stopstring); string = stopstring;	c[i].area = dtmp;
		dtmp = strtod(string,&stopstring); string = stopstring;	c[i].quadrics_error = dtmp;

//		if(c[i].Ncluster_ver_link > 2) printf("\t%d",c[i].Ncluster_ver_link);
	}
	printf("Count %d %d\n",cnt1,cnt2);
	fclose(fp);
//	exit(0);

	for(i=0;i<o->number_of_cluster;i++){
//		c[i].number = i;
		c[i].check = PRE;
//		c[i].vtop = NULL;
	}
	for(i=0;i<o->num_of_ver;i++){
		v[i].check = 0;
//		v[i].number = i;
//		v[i].next = NULL;
	}
/*
	glInterleavedArrays(GL_N3F_V3F,sizeof(view_vertex_data),o->view_verticies[0].normal);

//	o->list_id = glNewObjectBufferATI((o->num_of_ver) * sizeof(view_vertex_data), v, GL_STATIC_ATI);
//	o->list_id = glNewObjectBufferATI(56, v, GL_STATIC_ATI);
	fprintf(stdout,"%d\n",o->list_id);
//	exit(0);

	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);

//	glArrayObjectATI(GL_NORMAL_ARRAY, 3, GL_FLOAT, sizeof(view_vertex_data),o->list_id, 0);
//	glArrayObjectATI(GL_VERTEX_ARRAY, 3, GL_FLOAT, sizeof(view_vertex_data),o->list_id, sizeof(float)*3);

//	fprintf(stdout,"%d %d\n",v->normal-v,v->pos-v);


	glNormalPointer(GL_FLOAT,sizeof(view_vertex_data),o->view_verticies[0].normal);
	glVertexPointer(3,GL_FLOAT,sizeof(view_vertex_data),o->view_verticies[0].pos);

	glLockArraysEXT(0,o->num_of_ver);
*/
}



void 
make_unit_10000mm_vec_data(object_data *o){
	int i,j;
	double size;
	view_vertex_data *v;

	v = o->view_verticies;

	for(j=0;j<3;j++){
		o->data_range_high[j] = MIN_DOUBLE;
		o->data_range_low[j] =  MAX_DOUBLE;
	}
	for(i=0;i<o->num_of_ver;i++){
		for(j=0;j<3;j++){
			o->data_range_high[j] = (o->data_range_high[j] < v[i].pos[j]) ? v[i].pos[j] : o->data_range_high[j];
			o->data_range_low[j] = (o->data_range_low[j] > v[i].pos[j]) ? v[i].pos[j] : o->data_range_low[j];
		}
	}
	size = 0;
	for(i=0;i<3;i++){
		if(size < (o->data_range_high[i] - o->data_range_low[i]))
			size = o->data_range_high[i] - o->data_range_low[i];
	}
}

void rotation_view(object_data *o, float r[3]){
	int i;

	for(i=0;i<o->num_of_ver;i++){
		rotation_vector_float(o->view_verticies[i].pos,r[0],r[1],r[2]);
		rotation_vector_float(o->view_verticies[i].normal,r[0],r[1],r[2]);
	}
	for(i=0;i<o->number_of_cluster;i++){
		rotation_vector_float(o->view_clusters[i].normal,r[0],r[1],r[2]);
		rotation_vector_float(o->view_clusters[i].center,r[0],r[1],r[2]);
	}
}


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glut.h>

#include "set_define.h"
#include "struct.h"
#include "list_handle.h"
#include "file_handle.h"
#include "object_handle.h"
#include "tri_ver_math.h"
#include "make_cluster.h"
#include "cluster2mesh.h"


//#define TEXTURE_SIZE_MAX_X 2048
//#define TEXTURE_SIZE_MAX_Y 2048

object_data OBJECT;
triangle_data BOUNDARY;char *OutputFileName;


double
//	ALPHA = -90.0,	//x-z平面となす角度 底面を見たいとき
	ALPHA = 0,	//x-z平面となす角度
	BETA  = -180.0,  //y-z平面となす角度
	GAMMA = 0.0,  //z軸周りの角度  
	DIST  = 15.0;	 //原点からの距離

int DISPLAY_SIZE[2] = {300,300};
//int DISPLAY_SIZE[2] = {600,600};
//int DISPLAY_SIZE[2] = {500,500};
//t DISPLAY_SIZE[2] = {700,700};
//double MAX_DOUBLE = 32654;
//double MIN_DOUBLE = -32654;
//double DATA_RANGE_HIGH[3],DATA_RANGE_LOW[3];
//double SIZE;

//int NUM_OF_VER,NUM_OF_TRI,NUM_OF_QUAD,NUM_OF_POLYGON;


//int TRUE = 1,FALSE = 0;
//int ALIVE = 1,DEAD = 0;


//unsigned int IMAGE[400][400];
//static GLuint TEXTURE;
//static GLubyte IMAGE_RGBA[TEXTURE_SIZE_MAX_X][TEXTURE_SIZE_MAX_Y][4];
//static char IMAGE_RGBA_2[TEXTURE_SIZE_MAX_X][TEXTURE_SIZE_MAX_Y][4];

//int TEXTURE_SIZE_X,TEXTURE_SIZE_Y,WINDOW_SIZE_X,WINDOW_SIZE_Y;

void
save_pnm_data(int x, int y)
{
	FILE *fp;
	int i,j;
	static int id=0,idk=0;
	char file[256];
	static char *image_r,*image_g,*image_b;
	static int firstflag=0;
	static int display_size[2];
	
	if(firstflag==0){
		display_size[0] = x;
		display_size[1] = y;
		image_r = (char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
		image_g = (char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
		image_b = (char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
	}
	firstflag = 1;
	
	glReadPixels(0,0,display_size[0],display_size[1],GL_RED,GL_UNSIGNED_BYTE,image_r);
	glReadPixels(0,0,display_size[0],display_size[1],GL_GREEN,GL_UNSIGNED_BYTE,image_g);
	glReadPixels(0,0,display_size[0],display_size[1],GL_BLUE,GL_UNSIGNED_BYTE,image_b);

	sprintf(file, "displayimage%06d.pnm", /*N,*/ idk+id++);

	while((fp = fopen(file,"rb"))!=NULL){
		fclose(fp);
		idk += 1000; id = 1;
		sprintf(file, "displayimage%06d.pnm", /*N,*/ idk);
	}

	
	if ((fp = fopen(file,"wb")) == NULL){
		printf("Cannot open file!\n");
		exit(0);
	}
	fprintf(fp,"P6\n");
	fprintf(fp,"# CREATOR OpenGLviewer written by T.T\n");
	fprintf(fp,"%d %d\n",display_size[0],display_size[1]);
	fprintf(fp,"255\n");
	
	for(j=display_size[1]-1;j>=0;j--){
		for(i=0;i<display_size[0];i++){
			fwrite(&image_r[i+display_size[0]*j],sizeof(char),1,fp);
			fwrite(&image_g[i+display_size[0]*j],sizeof(char),1,fp);
			fwrite(&image_b[i+display_size[0]*j],sizeof(char),1,fp);
		}
	}
	fclose(fp);
	puts(file);
}


void init_data_display(object_data *o){
	triangle_data *t;
	int num_of_init_tri,i,j;
	//float mtr[3];

	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glRotated(GAMMA,0,0,1);
		glRotated(BETA,0,1,0);
		glRotated(ALPHA,1,0,0);
		t = o->triangles;
		num_of_init_tri = o->num_of_init_tri;
		glBegin(GL_TRIANGLES);
		for(i=0;i<num_of_init_tri;i++){
			for(j=0;j<3;j++){
				glNormal3dv(t[i].ver[j]->normal);
				glVertex3dv(t[i].ver[j]->pos);
			}
		}
		glEnd();
		
		glLoadIdentity();
	glPopMatrix();

}



void make_cluster_color(int a, int cn, float mtr[3]){
	int rgb_int[3];
	int i,tmp;

	memset(rgb_int,0,sizeof(rgb_int));
	//printf("%3d %3d \n",cn,a);

	if(cn < a){
		rgb_int[0] = cn;
		for(i=0;i<3;i++){
			mtr[i] = (float)rgb_int[i]/(float)a;
			//printf("%3d %3d %3d \n",cn,a,rgb_int[i]);
		}
		//getchar();
		return;
	}
	rgb_int[0] = cn % a; 
	tmp = cn - rgb_int[0];
	if(tmp < a*a){
		rgb_int[1] = tmp / a;
		for(i=0;i<3;i++){
			mtr[i] = (float)rgb_int[i]/(float)a;
			//printf("%d ",rgb_int[i]);
		}
		//getchar();
		return;
	}
	rgb_int[1] = (tmp % (a*a))/a; 
	tmp -= tmp % (a*a);

	rgb_int[2] = tmp / (a*a);

	for(i=0;i<3;i++){
		mtr[i] = (float)rgb_int[i]/(float)a;
		//printf("%d ",rgb_int[i]);
	}
	if(cn%2){
		for(i=0;i<3;i++) mtr[i] = 1-mtr[i];
	}

	//getchar();

}


int display_cluster_color(object_data *o){
	triangle_data *t;
	int num_of_init_tri,i,j;
	float mtr[3];
	int a;
	static int flag = -100;

	a = 3;
	while(a*a*a < o->number_of_cluster){
		a++;
		//printf("%d\n",a);
	}


	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glRotated(GAMMA,0,0,1);
		glRotated(BETA,0,1,0);
		glRotated(ALPHA,1,0,0);
		t = o->triangles;
		num_of_init_tri = o->num_of_init_tri;
		glBegin(GL_TRIANGLES);
		for(i=0;i<num_of_init_tri;i++){
			make_cluster_color(a,t[i].cluster_number,mtr);
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr);
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr);
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mtr);
			for(j=0;j<3;j++){
				glNormal3dv(t[i].normal);
				glVertex3dv(t[i].ver[j]->pos);
			}
		}
		glEnd();
		
		glLoadIdentity();
	glPopMatrix();

	return flag;
}

int display_simaple_cluster_color(object_data *o){
	triangle_data *t;
	int num_of_init_tri,i,j;
	float mtr_white[3] = {1.0,1.0,1.0};
	float mtr_black[3] = {0.0,0.0,0.0};
	float mtr_red[3] = {1,0.0,0.0};
	float mtr_green[3] = {0.0,1.0,0.0};
	float mtr_blue[3] = {0,0.0,1.0};
	float *mtr; 
	static int flag = -100;


	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glRotated(GAMMA,0,0,1);
		glRotated(BETA,0,1,0);
		glRotated(ALPHA,1,0,0);
		t = o->triangles;
		num_of_init_tri = o->num_of_init_tri;
		glBegin(GL_TRIANGLES);
	
		for(i=0;i<num_of_init_tri;i++){
			if(t[i].cluster_number == 0) mtr = mtr_green;
			if(t[i].cluster_number == 1) mtr = mtr_blue;
			if(t[i].cluster_number == 2) mtr = mtr_red;
			glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr);
			glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr);
			glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mtr);
			for(j=0;j<3;j++){
				glNormal3dv(t[i].normal);
				glVertex3dv(t[i].ver[j]->pos);
			}
		}
		glEnd();
		
		glLoadIdentity();
	glPopMatrix();

	return flag;
}

int display_polygons_list(object_data *o){
	list_data *l;
	vertex_tree_data *tv;
	vertex_data *v;
	int i;
	float mtr[3] = {1.0,0.0,0.0};
	float mtr2[3] = {0.0,1.0,0.0};

//	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr);
//	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr);
//	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mtr);

	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glRotated(GAMMA,0,0,1);
		glRotated(BETA,0,1,0);
		glRotated(ALPHA,1,0,0);
	
		for(i=0;i<o->number_of_cluster;i++){
			if(o->clusters[i].check == DEAD) continue;
			l = &o->clusters[i].polygons;
			glBegin(GL_TRIANGLE_FAN);
			while(l->next != NULL){
				tv = (vertex_tree_data*)l->next->element;
				v = tv->vertex;
				glNormal3dv(v->normal);
				glVertex3dv(v->pos);
				l = l->next;
			}
			glEnd();
		}

/*		glDisable(GL_CULL_FACE);
		for(i=0;i<o->Nedge;i++){
//			if(i < 407 ) continue;
//			if(i < 409 ) continue;
			if(i < 411 ) continue;
			if(i > 411) continue; 

			printf("e ");

			tv = o->edges[i].vtop[0];
			glBegin(GL_POLYGON);
			while(tv!=NULL && tv->next != o->edges[i].vtop[0]){
				v = tv->vertex;

				glNormal3dv(v->normal);
				glVertex3dv(v->pos);
				tv = tv->next;
				printf(" %d", v->ring_number);
			}
			glEnd();
			printf("\n");
			tv = o->edges[i].vtop[1];
			glBegin(GL_POLYGON);
			while(tv!=NULL && tv->next != o->edges[i].vtop[1]){
				v = tv->vertex;
				glNormal3dv(v->normal);
				glVertex3dv(v->pos);
				tv = tv->next;
			}
			glEnd();
		}
*/	
		glLoadIdentity();
	glPopMatrix();
	return TRUE;
}


void display_meshes(object_data *o){
	triangle_data *t;
	int num_of_init_tri,i,j;
	float mtr[3] = {0.0,0.0,0.0};



	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glRotated(GAMMA,0,0,1);
		glRotated(BETA,0,1,0);
		glRotated(ALPHA,1,0,0);
		t = o->triangles;
		num_of_init_tri = o->num_of_all_tri;
		glBegin(GL_TRIANGLES);
		for(i=0;i<num_of_init_tri;i++){
//			if(t[i].check==ALIVE)
			for(j=0;j<3;j++){
				glNormal3dv(t[i].ver[j]->normal);
				glVertex3dv(t[i].ver[j]->pos);
			}
		}
		glEnd();
		
		glLoadIdentity();
	glPopMatrix();

}

double quadric_error(object_data *o){
	double error=0,error2;
	int i,cnt=0;
	cluster_data *c;
	static FILE *fp;
	static double be = 3.40282347e38;

	c = o->clusters;
	for(i=0;i<o->number_of_cluster;i++){
		if(c[i].check == DEAD) continue;
		cnt++;
		if(!(c[i].quadrics_error > 0)) continue;
		error += c[i].quadrics_error*c[i].area;
	}
	error2 = error /= c[0].area;
//	printf("%lf%lf\n",be,error);
	if(be < error ) error = be;
	be = error;
	if(error < 0.000001) error = 0.000001;
	if(i==3){
		fp = fopen("qe.txt","wb"); 
		fprintf(fp,"%lf\n",c->quadrics_error);
	}
	fprintf(fp,"%lf\n",error);
	if(cnt > 30000){
		fclose(fp);
		exit(0);
	}
	return error;
}



/*=============== 本プログラムの描画部 ===============*/
void  display(void){
	extern object_data OBJECT;
	static int mode = 0,count=0,first=0, repeat;
	static int flags=0;
	int state;
	int i;
	float mtr[4] = {0.5,0.5,0.5,1};
	float mtr1[4] = {0,0,0,1};
//	ring_list_data *l;

	if(first==0){
		count = OBJECT.number_of_cluster;
		//		timer_one_time_startstop(0,START);
		//timer_one_time_startstop(1,START);
	}
	first=1;

#ifndef NO_DISPLAY
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif

//	init_data_display(&OBJECT); mode = 10;
//	printf("%d %d %d\n",sizeof(char),sizeof(float),sizeof(double));


//	display_meshes(&OBJECT);
//	glutSwapBuffers();
//	return;



	switch(mode){
	case 0:
		repeat = count/5+1; //log10(count+1)+1;
//		repeat = 1;
//		repeat =1000000;
		for(i=0;i<repeat;i++){
//			if(count > 40){
//				mode=1;
//				break;
//			}

			state = hierarchical_clustering_by_quadrics(&OBJECT);

//			quadric_error(&OBJECT);
				
			if(count % 10000 == 0) fprintf(stderr,"%d \n",count);
/*			if(count == 100 || count == 400 || count == 1600 || count == 6400){
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				display_cluster_color(&OBJECT);
				save_pnm_data(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
				glutSwapBuffers();
//				save_pnm_data(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
			}
*/


#ifdef STOP_CLUSTER
			if(i==STOP_NUM/2){ mode=3; break; }
#endif
			if(state == FINISH){
/*				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				display_cluster_color(&OBJECT);
				glutSwapBuffers();
				save_pnm_data(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
*/
//				timer_one_time_startstop(0,STOP);
				mode++;
				write_vec_data(OutputFileName,&OBJECT);
				write_trivec_data((char*)"test.trivec",&OBJECT);
				exit(0);
				//				timer_one_time_startstop(2,START);
				break;
			}
			count++;
		}
		flags = 1;
#ifndef NO_DISPLAY

		if(mode < 1) display_cluster_color(&OBJECT);
//		if(mode < 1) display_polygons_list(&OBJECT);



		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mtr);
#endif
		break;
	case 1:
		display_cluster_color(&OBJECT);
//		exit(0);
//		display_polygons_list(&OBJECT);
//		display_polygons(&OBJECT);
		break;
		
	}
#ifndef NO_DISPLAY
	glutSwapBuffers();
#endif
	//getchar();

}

/**********************************************

OpenGL関連部

**********************************************/


/*************************** コールバック設定 *****************************/

/*========== ウインドウのリサイズに合わせて描画領域を変更 ==========*/
void
reshape(GLsizei width, GLsizei height)
{
	extern object_data OBJECT;
	int i;
	double size;

	
	OBJECT.size = 0;
	glViewport(0, 0, width, height);
	for(i=0;i<3;i++){
		if(OBJECT.size < (OBJECT.data_range_high[i] - OBJECT.data_range_low[i]))
			OBJECT.size = OBJECT.data_range_high[i] - OBJECT.data_range_low[i];
		printf("data_range[%d] %lf %lf\n",i,OBJECT.data_range_high[i],OBJECT.data_range_low[i]);
	}
	//OBJECT.size *= 3;
	size = OBJECT.size*.6;
	printf("size %lf\n",size);
	
	double offset[3];
	offset[0] = (OBJECT.data_range_high[0]+OBJECT.data_range_low[0])/2.0;
	offset[1] = (OBJECT.data_range_high[1]+OBJECT.data_range_low[1])/2.0;
	offset[2] = (OBJECT.data_range_high[2]+OBJECT.data_range_low[2])/2.0;
	

	//射影行列の設定
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	size *= 1.8;
//	glOrtho(size+offset[0],-size+offset[0],-size+offset[1],size+offset[1],size+offset[2],-size+offset[2]);
	glOrtho(size-offset[0],-size-offset[0],-size+offset[1],size+offset[1],size-offset[2],-size-offset[2]);
	
	glutSwapBuffers();
}




/*=============== CPUのアイドル時間の動作 ===============*/
void idle(void){
	glutPostRedisplay();
}

/**********************************************

操作部

**********************************************/

/*=============== キーボードからの ASCII文字入力 ===============*/
void keyboard(unsigned char key, int x, int y){
	
	switch(key)	{
		//qで終了
	case 'q' :	exit(0); break;
	case '1' :	GAMMA += 10.0;break;
	case '2' :	GAMMA -= 10.0;break;
	case 'c' :	save_pnm_data(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
	}
}


/*=============== キーボードからの 非ASCII文字入力 ===============*/
void special(int key, int x, int y){

	switch(key){
	case GLUT_KEY_UP : 
	  ALPHA += 10.0;
	  break;
	case GLUT_KEY_DOWN :
	  ALPHA -= 10.0;
	  break;
	case GLUT_KEY_LEFT :
	  BETA += 10.0;
	  break;
	case GLUT_KEY_RIGHT :
	  BETA -= 10.0;
	  break;
  	}
}


/*========== ウインドウが（非）可視領域に入ったときの処置 ==========*/
void Visible(int vis){

	if(vis != GLUT_VISIBLE)
		glutIdleFunc(NULL);
	else
		glutIdleFunc(idle);
}


/*************************************++++++++++++++
     
	   menu設定

***********************************************/

/*void polygon_mode(int value){

	if(value == 1) glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	if(value == 2) glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	if(value == 3) glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
	glutSwapBuffers();
}*/
void polygon_mode(int value){
	GLfloat mtr3[4] = {0.0,0.0,0.0,1.0};
	GLfloat mtr2[4] = {0.8,0.0,0.0,1.0};
	GLfloat mtr1[4] = {0.8,0.8,0.8,1.0};

	if(value == 1){
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr1);
	}
	if(value == 2){
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mtr3);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,mtr3);
	}
	if(value == 3){
		glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
		glPointSize(2.0);
	}
	glutSwapBuffers();
}

void shade_mode(int value){

	if(value == 1) glShadeModel(GL_SMOOTH);
	if(value == 2) glShadeModel(GL_FLAT);
	glutSwapBuffers();
}

void choice(int value){
	int xsize,ysize;

	if(value == 1) exit(0);
	if(value == 2){
		printf("WindowSizeX = ");
		scanf("%d",&xsize);
		printf("WindowSizeY = ");
		scanf("%d",&ysize);
		glutReshapeWindow(xsize,ysize);
	}

}

void make_menu(){
	int sub_polygon_mode,sub_shade_mode;

	sub_polygon_mode = glutCreateMenu(polygon_mode);
	glutAddMenuEntry("FILL",1);
	glutAddMenuEntry("LINES",2);
	glutAddMenuEntry("POINT",3);

	sub_shade_mode = glutCreateMenu(shade_mode);
	glutAddMenuEntry("SMOOTH",1);
	glutAddMenuEntry("FLAT",2);


	glutCreateMenu(choice);
	glutAddSubMenu("POLYGON MODE",sub_polygon_mode);
	glutAddSubMenu("SHADE MODE",sub_shade_mode);
	glutAddMenuEntry("WindowSize",2);
	glutAddMenuEntry("EXIT",1);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}


/************************** OpenGL環境の設定 ******************************/

/*=============== 光源設定 ===============*/
/*
void StartSunLight(void)
{
	GLfloat	
		Light_Ambient[]		= {1.0, 1.0, 1.0, 1.0}, //環境光
		Light_Diffuse[]		= {1.0, 1.0, 1.0, 1.0}, //拡散光
		Light_Specular[]	= {1.0, 1.0, 1.0, 1.0}, //鏡面光
		Light_Position[]	= {0.0, 0.0, 1.0, 0.0}; //位置


	glEnable(GL_LIGHTING);
}
*/

void init_light(){ /* 初期設定(OpenGL関連) */

	GLfloat
		l_amb[] = {1,1,1,1},
		l_dif[] = {.7,.7,.7,1},
		l_spe[] = {0,0,0,1},
		l_pos[] = {0,0,-1,0};
//		l_pos[] = {0,0,1,0};
  
  
	glLightfv(GL_LIGHT0,GL_AMBIENT,l_amb);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,l_dif);
	glLightfv(GL_LIGHT0,GL_SPECULAR,l_spe);
	glLightfv(GL_LIGHT0,GL_POSITION,l_pos);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

}

/*=============== OpenGLの環境の設定（詳細は省略）===============*/
void OpenGL_init(void){
	//クリアカラー（背景色）の設定
	glClearColor(1.0, 1.0, 1.0, 1.0);

	glClearAccum(0, 0, 0, 1);
	glClearDepth(1.0);
//	glDepthFunc(GL_GREATER);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	//glDisable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glPolygonMode(GL_FRONT, GL_FILL);
	glPolygonMode(GL_FRONT, GL_FILL);
	glCullFace(GL_FRONT);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glDisable(GL_CULL_FACE);

}


void glut_setting(int argc,char** argv){

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowPosition(500, 0);
	glutInitWindowSize(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
	glutCreateWindow("texture viewer");

	//make_draw_list();

	/*---------- イベントに同期するコールバックの設定 ----------*/
}

void glut_callback(){
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(special);
	glutVisibilityFunc(Visible);
}


double make_unit_vector(double v[3]);

/****************************** メイン関数 ********************************/

int main(int argc, char** argv){
	extern object_data OBJECT;
	int i,j;

	if (argc-1 != 2 ){
		printf("program inputfile outputfile\n");
		exit(0);
	}
	
	OutputFileName = argv[2];
	read_mesh_data(argv[1],&OBJECT);
	make_unit_10000mm_data(&OBJECT);
/*
	if(strstr(argv[1],"k5n.tri")){
		for(i=0;i<OBJECT.num_of_ver;i++){
			for(j=0;j<3;j++){
				OBJECT.vertices[i].normal[j] = OBJECT.vertices[i].pos[j];
			}
			make_unit_vector(OBJECT.vertices[i].normal);
		}
		write_tri_data("k5n2.tri",&OBJECT);
	}
*/
	//write_tri_data("mphone.tri",&OBJECT);



	OBJECT.nbr_status = DEAD;
	initial(&OBJECT);
	make_normal_of_vertices(&OBJECT);

	glut_setting(argc,argv);
	OpenGL_init();
	init_light();

	make_menu();
	glut_callback();
#ifndef NO_DISPLAY
	//init_clustering(&OBJECT);
	glutMainLoop();
#endif
	while(1) display();
	return(1);
}




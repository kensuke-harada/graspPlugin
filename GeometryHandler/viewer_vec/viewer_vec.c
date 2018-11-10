#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glut.h>


#include "set_define.h"

#include "struct.h"
#include "file_handle.h"
#include "mymath_float.h"
//#include "tri_ver_math.h"
#include "object_handle.h"
//#include "timer.h"
//#include "levels_of_detail.h"

//#include "winsocket.h"
#define MAXVERTEX 65536
#define MAXPACKETSIZE 65536

void read_vec_data(char *file, object_data *o);
void display_lod_model_by_number(object_data *o);
void make_unit_10000mm_vec_data(object_data *o);

//カメラ位置のパラメータ
float EYEPOS[3] = {0,0,0};	//カメラの座標
float VIEWCENTER[3] = {0,0,1};  //カメラの見てる向き
float LOOKAT_DIRECTION[3] = {0,0,1};
float SKY_DIRECTION[3] = {0,1,0};
float SIDE_DIRECTION[3] = {-1,0,0};
float GAZE_DIRECTION[3] = {0,0,1};

float
	LEFT_CULLING[3],
	RIGHT_CULLING[3],
	UPPER_CULLING[3],
	UNDER_CULLING[3];

//int DISPLAY_SIZE[2] = {512,512};
//int DISPLAY_SIZE[2] = {500,500};
int DISPLAY_SIZE[2] = {600,600};
int MOUSE_POS[2];
float GAZE_POS[2] = {400, 400};
int Counter,Counter2;
int SAVE_STL;


int LOOKAT_FLAG=ALIVE;

int NUMBER_OF_FILE;
triangle_data BOUNDARY;
//vertex_data   DUMMY_VERTEX,*TMP_VER;
//texture_data *TEXTURE;
//obj_mtl_file_data *FILE_DATAS,DOOR;

int NUMBER_OF_LOD_OBJECTS = 2;
int DATA_WIDTH = 1;
object_data LOD_OBJECT, LOD_OBJECT_DATAS[2];
object_data ORIGINAL_OBJECT_DATAS[2];
int DISPLAY_POSITION;
int DISPLAY_MODE;
double tor;

char* outputFileName;
int polygonNum=10000;


double YROTATION;

int WIRE_FRAME=0;

int LODMODE_FLAG=0;

void initial_setting();
void setting(int *argc, char** argv,int *display_size);
void OpenGL_init(void);
void make_menu();
void gl_callback();
void polygon_mode(int value);

/**;**************************** メイン関数 ********************************/
int main(int argc, char** argv)
{
	int count=0;
	//char file_list[256][256];
//	static texture_data tex;
	float pos[4] = {0, 0, 0, 0 },dir[3] = {0, 0, 0};
//	double t[3];
//	char *extensions;
//	object_data *o;


	
	//read_setting_file();
	fprintf(stdout,"start\n");

	initial_setting();
	setting(&argc,argv,DISPLAY_SIZE);
	OpenGL_init();
	make_menu();
	gl_callback();
	polygon_mode(1);
	OpenGL_init();


	LODMODE_FLAG = 0;
	if(argc < 2){
		printf("program input_file\n");
		exit(-1);
	}



	if(strstr(argv[1],".vec")){
		strcpy(LOD_OBJECT_DATAS[0].filetype,"vec");
		read_vec_data(argv[1],&LOD_OBJECT_DATAS[0]);
		LODMODE_FLAG = 1;
		make_unit_10000mm_vec_data(&LOD_OBJECT_DATAS[0]);

	}
	if(argc>2) outputFileName = argv[2];
	if(argc>3){
		sscanf(argv[3],"%d",&polygonNum);
		printf("polygonNum %d\n",polygonNum);
	}


	glutMainLoop();
	return(1);
}



void
save_pnm_data(int x, int y)
{
	FILE *fp;
	int i,j;
	static int id=0;
	char file[256];
	static char *image_r,*image_g,*image_b;
	static int firstflag=0;
	static int display_size[2];
	int count;
/*
//	count = Counter - Counter % 5;
//	if(count > 5000) count = Counter -Counter%20;
	if(count < 420) count = Counter - Counter % 20;
	if(count < 1650)count =  Counter - Counter % 50;
	if(count < 6600)count =  Counter - Counter % 200;

	if(!(count == 100 || count == 400 || count == 1600 || count == 6400)) return;
*/
//	if(Counter < 6400) return;
//	printf("c%d",Counter);
	count = Counter;

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

	sprintf(file, "Rbn%06d.pnm", /*N,*/ count);

//	while((fp = fopen(file,"rb"))!=NULL){
//		id += 1000;
//		sprintf(file, "displayimage%06d.pnm", /*N,*/ id++);
///		fclose(fp);
//	}

	
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
	printf("%s\n",file);
}


void
compare_images(int x, int y)
{
	static FILE *fp;
	int i;
	static int id=0,cnt;
	char file[256];
	static unsigned char *image_r,*image_g,*image_b,*image_rr,*image_gr,*image_br;
	static int firstflag=0;
	static int display_size[2];
	static int count=0;
	static double error;
	static double sum,diff;
	extern double Error;

	if(firstflag==0){
		display_size[0] = x;
		display_size[1] = y;
		image_r = (unsigned char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
//		image_g = (unsigned char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
//		image_b = (unsigned char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
		image_rr = (unsigned char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
//		image_gr = (unsigned char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
//		image_br = (unsigned char *)malloc(sizeof(char)*display_size[0]*display_size[1]);

		if(image_r == NULL || image_rr == NULL) exit(0);
		glReadPixels(0,0,display_size[0],display_size[1],GL_RED,GL_UNSIGNED_BYTE,image_rr);
//		glReadPixels(0,0,display_size[0],display_size[1],GL_GREEN,GL_UNSIGNED_BYTE,image_gr);
//		glReadPixels(0,0,display_size[0],display_size[1],GL_BLUE,GL_UNSIGNED_BYTE,image_br);
		sprintf(file, "error.txt");
		if ((fp = fopen(file,"wb")) == NULL){
			printf("Cannot open file!\n");
			exit(0);
		}
		fprintf(stdout,"%d %d\n",display_size[0],display_size[1]);
		sum = 0;
		cnt = 0;
		for(i=0;i<display_size[0]*display_size[1];i++){
			if(image_rr[i] == 255) continue;
			sum+=image_rr[i]*image_rr[i];
			cnt++;
		}
		firstflag = 1;
		return;
	}


	if(count < Counter) count = Counter;
	else return;

	
	glReadPixels(0,0,display_size[0],display_size[1],GL_RED,GL_UNSIGNED_BYTE,image_r);
//	glReadPixels(0,0,display_size[0],display_size[1],GL_GREEN,GL_UNSIGNED_BYTE,image_g);
//	glReadPixels(0,0,display_size[0],display_size[1],GL_BLUE,GL_UNSIGNED_BYTE,image_b);

	diff=0;
	for(i=0;i<display_size[0]*display_size[1];i++){
		if(image_rr[i] == 255 || image_r[i] == 255) continue;
		diff += (image_rr[i]-image_r[i])*(image_rr[i]-image_r[i]);
	}
	error = diff/sum;
//	error = diff/sum*cnt;
	fprintf(fp,"%d\t%lf\t%lf\n",count,error,tor);

	fprintf(stdout,"%d %lf \t",count,error);

	if(count > LOD_OBJECT_DATAS[0].number_of_cluster/2 -5){
		fclose(fp);
		exit(0);
	}


//	while((fp = fopen(file,"rb"))!=NULL){
//		id += 1000;
//		sprintf(file, "displayimage%06d.pnm", /*N,*/ id++);
///		fclose(fp);
//	}

	
	
}

void
read_depth_buffer(int x, int y)
{
	FILE *fp;
	int i,j;
	static int id=0;
	char file[256];
	static char *image_r,*image_g,*image_b;
	static char *depth,*depth2;
	static unsigned int *idepth;
	static int firstflag=0;
	static int display_size[2];

	
	if(firstflag==0){
		display_size[0] = x;
		display_size[1] = y;
		image_r = (char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
		image_g = (char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
		image_b = (char *)malloc(sizeof(char)*display_size[0]*display_size[1]);
//		depth = (short *)malloc(sizeof(short)*display_size[0]*display_size[1]);
//		idepth = (int *)malloc(sizeof(int)*display_size[0]*display_size[1]*2);
//		depth2 = (short *)malloc(sizeof(short)*display_size[0]*display_size[1]);
	}
	firstflag = 1;
//	idepth = (((int)idepth) - ((int)idepth)%32 + 32);
	
//	glReadPixels(0,0,display_size[0],display_size[1],GL_RED,GL_UNSIGNED_BYTE,image_r);
//	glReadPixels(0,0,display_size[0],display_size[1],GL_GREEN,GL_UNSIGNED_BYTE,image_g);
//	glReadPixels(0,0,display_size[0],display_size[1],GL_BLUE,GL_UNSIGNED_BYTE,image_b);
	glReadPixels(0,0,display_size[0],display_size[1],GL_DEPTH_COMPONENT,GL_UNSIGNED_INT,idepth);
//	glReadPixels(0,0,display_size[0],display_size[1],GL_DEPTH_COMPONENT,GL_UNSIGNED_INT_24_8_NV,idepth);
//	glReadPixels(0,0,display_size[0],display_size[1],GL_DEPTH_COMPONENT,GL_UNSIGNED_SHORT,idepth);
//	glReadPixels(0,display_size[1]/2,display_size[0],display_size[1],GL_DEPTH_COMPONENT,GL_UNSIGNED_BYTE,depth2);

	return;

//	return;
//	printf("\n%d\n",GL_UNSIGNED_INT);

//	depth = idepth;

//	return ;


	if(0) for(j=display_size[1]-1;j>=0;j--){
		for(i=0;i<display_size[0];i++){
			printf("%x ",idepth[i+display_size[0]*j]);
		}
		printf("\n");
	}
//	return ;

	sprintf(file, "displayimage%06d.pnm", id++);

	while(fopen(file,"rb")!=NULL){
		id += 1000;
		sprintf(file, "displayimage%06d.pnm", id++);
	}

	
	if ((fp = fopen(file,"wb")) == NULL){
		printf("Cannot open file!\n");
		exit(0);
	}
	fprintf(fp,"P6\n");
	fprintf(fp,"# CREATOR OpenGLviewer written by T.T\n");
	fprintf(fp,"%d %d\n",display_size[0],display_size[1]);
	fprintf(fp,"255\n");
	
	for(j=display_size[1];j>=0;j--){
		for(i=0;i<display_size[0];i++){
//			depth[i+display_size[0]*j] = idepth[i+display_size[0]*j];
//			fwrite(&image_r[i+display_size[0]*j],sizeof(char),1,fp);
//			fwrite(&image_g[i+display_size[0]*j],sizeof(char),1,fp);
//			fwrite(&image_b[i+display_size[0]*j],sizeof(char),1,fp);
//			fwrite(&depth[i+display_size[0]*j],sizeof(char),1,fp);
//			fwrite(&depth[i+display_size[0]*j],sizeof(char),1,fp);
//			fwrite(&depth[i+display_size[0]*j],sizeof(char),1,fp);
			fwrite(&depth[(i+display_size[0]*j)*4],sizeof(char),1,fp);
			fwrite(&depth[(i+display_size[0]*j)*4+1],sizeof(char),1,fp);
			fwrite(&depth[(i+display_size[0]*j)*4+2],sizeof(char),1,fp);
//			fwrite(&depth[(i+display_size[0]*j)*2],sizeof(char),1,fp);
//			fwrite(&depth[(i+display_size[0]*j)*2+1],sizeof(char),1,fp);
//			fwrite(&depth[(i+display_size[0]*j)*2+2],sizeof(char),1,fp);
		}
//		printf("%d",idepth[i+display_size[0]*j]);
	}

	fclose(fp);
	printf("displayimage%06d.pnm\n",id-1);

}



void initial_setting(){
	double gaze_p[2] = {0.5,0.5};
    
	
//	EYEPOS[0] = 1000;
//	EYEPOS[1] = 1500;
//	EYEPOS[2] = 5000;

	EYEPOS[0] = 0;
	EYEPOS[1] = 0;
	EYEPOS[2] = 0;
	
	LOOKAT_DIRECTION[0]= 0;
	LOOKAT_DIRECTION[1]= 0;
	LOOKAT_DIRECTION[2]= -1;
	
	SKY_DIRECTION[0] = 0; 
	SKY_DIRECTION[1] = 1; 
	SKY_DIRECTION[2] = 0; 
	
	outer_product_2_vector_float(SIDE_DIRECTION,SKY_DIRECTION,LOOKAT_DIRECTION);
	add_2_vector_float(VIEWCENTER,EYEPOS,LOOKAT_DIRECTION);

	//put_data_in_shared_memory(0,0x20,gaze_p,sizeof(double)*2);
}


/************************** OpenGL環境の設定 ******************************/

/*=============== 光源設定 ===============*/
void StartSunLight(void)
{
	GLfloat	
		Light_Ambient[]		= {1.0, 1.0, 1.0, 1.0}, //環境光
		Light_Diffuse[]		= {1.0, 1.0, 1.0, 1.0}, //拡散光
		Light_Specular[]	= {1.0, 1.0, 1.0, 1.0}, //鏡面光
		Light_Position[]	= {0.0, 0.0, 1.0, 0.0}; //位置


	glEnable(GL_LIGHTING);
}


void init_light() /* 初期設定(OpenGL関連) */
{
	GLfloat
		l_amb[] = {.4,.4,.4,1},
		l_dif[] = {.7,.7,.7,1},
		l_spe[] = {0.4,0.4,0.4,1},
		l_pos[] = {0,1,0,0},
		l2_amb[] = {1.0, 0.0, 0.0, 1.0},
		l2_dif[] = {1.0, 0.0, 0.0, 1.0},
		l2_spe[] = {1.0, 0.0, 0.0, 1.0},
		l2_pos[] = {0.0, 0.0, -1.0, 1.0},
		lmodel_ambient[] = {0.2,0.2,0.2,1.0},
		white[] = {1.0, 1.0, 1.0, 1.0};
  
#ifdef WIRE_FRAME  
#endif

//	glLightfv(GL_LIGHT2,GL_AMBIENT,l2_amb);
//	glLightfv(GL_LIGHT2,GL_DIFFUSE,l2_dif);
//	glLightfv(GL_LIGHT2,GL_SPECULAR,l2_spe);
//	glLightfv(GL_LIGHT2,GL_POSITION,l2_pos);

//	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lmodel_ambient);
//	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glDisable(GL_LIGHT2);
}

/*=============== OpenGLの環境の設定（詳細は省略）===============*/
void OpenGL_init(void)
{
	//クリアカラー（背景色）の設定
	glClearColor(1.0, 1.0, 1.0, 0.0);

	glClearAccum(0, 0, 0, 1);
	glDepthFunc(GL_LESS);
	glClearDepth(1.0);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
//	glPolygonMode(GL_FRONT, GL_LINE);
	glPolygonMode(GL_FRONT, GL_FILL);
///	glCullFace(GL_FRONT);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
//	glDisable(GL_CULL_FACE);

	init_light();
}




void setting(int *argc, char** argv,int *display_size){
	/*---------- OpenGL環境定義情報の読み出し ----------*/
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowPosition(500, 0);
	glutInitWindowSize(display_size[0],display_size[1]);
	glutCreateWindow("Demo");
}


//#include "mekemenu.h"


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
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
	}
	if(value == 3){
		glPolygonMode(GL_FRONT_AND_BACK,GL_POINT);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mtr3);
		glPointSize(10.0);
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

	if(value == 1){
		//exit_memory_board();
		exit(0);
	}
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


void keyboard(unsigned char key, int x, int y){
	float movement[3] = {0,0,0};
	double velocity = 100;
	
	switch(key){
	case 'q' : 
		//exit_memory_board();
		exit(0); break;
	case '5' :
		multiple_scalar_vector_float(movement,velocity,LOOKAT_DIRECTION);
		break;
	case '0' : 
		multiple_scalar_vector_float(movement,-velocity,LOOKAT_DIRECTION);
		break;
	case '8' : 
		multiple_scalar_vector_float(movement,velocity,SKY_DIRECTION);
		break;
	case '2' :
		multiple_scalar_vector_float(movement,-velocity,SKY_DIRECTION);
		break;
	case '4' :
		multiple_scalar_vector_float(movement,velocity,SIDE_DIRECTION);
		break;
	case '6' : 
		multiple_scalar_vector_float(movement,-velocity,SIDE_DIRECTION);
		break;
	case 'l' :
		LOOKAT_FLAG = !LOOKAT_FLAG;
		break;
	case 'c' :
		save_pnm_data(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
		break;
	case 's' :
	        SAVE_STL=1;
		break;
	case 'w' :
		WIRE_FRAME = !WIRE_FRAME;
		break;
	default:
		break;
	}
	//movement[1] = 0;  //////////////////////////////   上下固定モード　////////////////////////////////////////
	add_2_vector_float(EYEPOS,EYEPOS,movement);
#ifdef COMMENTOUT
	if(EYEPOS[0] > 5500) EYEPOS[0] = 5500;
	if(EYEPOS[0] < 500) EYEPOS[0] = 500;
	if(EYEPOS[1] > 2900) EYEPOS[1] = 2900;
	if(EYEPOS[1] < 500) EYEPOS[1] = 500;
	if(EYEPOS[2] > 5500) EYEPOS[2] = 5500;
	if(EYEPOS[2] < 500) EYEPOS[2] = 500;
#endif
	add_2_vector_float(VIEWCENTER,EYEPOS,LOOKAT_DIRECTION);
}


/*=============== キーボードからの 非ASCII文字入力 ===============*/
void special(int key, int x, int y){
	static int dra=0,drb=0,drg=0;
	double ra,rg,rb;
	float temp1[3],temp2[3],temp3[3];
	double rotation_velocity = 5.0;
	
	//dra=drb=drg=0;
	
	switch(key){
	case GLUT_KEY_UP : 
		dra += rotation_velocity;
		break;
	case GLUT_KEY_DOWN :
		dra -= rotation_velocity;
		break;
	case GLUT_KEY_LEFT :
		drb += rotation_velocity;
		break;
	case GLUT_KEY_RIGHT :
		drb -= rotation_velocity;
		break;
	default:
		return;
	}
	
//	if(dra > 90) dra = 90;
//	if(dra <- 90) dra = -90;
	
	ra = dra*2*M_PI/360.0;
	rb = drb*2*M_PI/360.0;
	rg = drg*2*M_PI/360.0;

	YROTATION = drb;

	
	LOOKAT_DIRECTION[0]= 0;
	LOOKAT_DIRECTION[1]= 0;
	LOOKAT_DIRECTION[2]= -1;
	
	SKY_DIRECTION[0] = 0; 
	SKY_DIRECTION[1] = 1; 
	SKY_DIRECTION[2] = 0; 

	SIDE_DIRECTION[0] = 1; 
	SIDE_DIRECTION[1] = 0; 
	SIDE_DIRECTION[2] = 0;
/*
	LOOKAT_DIRECTION[0]= 1;
	LOOKAT_DIRECTION[1]= 0;
	LOOKAT_DIRECTION[2]= 0;
	
	SKY_DIRECTION[0] = 0; 
	SKY_DIRECTION[1] = 1; 
	SKY_DIRECTION[2] = 0;
	
	SIDE_DIRECTION[0] = 0; 
	SIDE_DIRECTION[1] = 0; 
	SIDE_DIRECTION[2] = -1;
*/	
	multiple_scalar_vector_float(temp1,cos(rb),LOOKAT_DIRECTION);
	multiple_scalar_vector_float(temp2,sin(rb),SIDE_DIRECTION);
	add_2_vector_float(temp3,temp1,temp2);
	make_unit_vector_float(temp3);
	outer_product_2_vector_float(SIDE_DIRECTION,SKY_DIRECTION,temp3);
	make_unit_vector_float(SIDE_DIRECTION);
	multiple_scalar_vector_float(temp1,cos(ra),temp3);
	multiple_scalar_vector_float(temp2,sin(ra),SKY_DIRECTION);
	add_2_vector_float(LOOKAT_DIRECTION,temp1,temp2);
	make_unit_vector_float(LOOKAT_DIRECTION);
	outer_product_2_vector_float(SKY_DIRECTION,LOOKAT_DIRECTION,SIDE_DIRECTION);
	
	add_2_vector_float(VIEWCENTER,EYEPOS,LOOKAT_DIRECTION);
}


void mouse_pos_points(){
	float target[3];
    GLfloat
		ambient[4] = {1.0, 0.0, 0.0, 1.0},
		diffuse[4] = {1.0, 0.0, 0.0, 1.0},
		specular[4] = {1.0, 0.0, 0.0, 1.0},
		ns = 255;
	
    multiple_scalar_vector_float(target,100,GAZE_DIRECTION);
	add_2_vector_float(target,target,EYEPOS);
/*	
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diffuse);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
	glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,ns);
*/	
	glPointSize(10.0);
	glBegin(GL_POINTS);
	glVertex3fv(target);
	glNormal3f(0,0,1);
	glEnd();
}

/*  setting display  */

void make_culling_vector(){
    float lookat[3],sky[3],side[3];
	float left[3],right[3],upper[3],under[3];

    multiple_scalar_vector_float(lookat,(double)DISPLAY_SIZE[1]/2.0/tan(VIEW_RANGE*M_PI/360),LOOKAT_DIRECTION);
    multiple_scalar_vector_float(side,(double)DISPLAY_SIZE[0]/2.0,SIDE_DIRECTION);
    multiple_scalar_vector_float(sky,(double)DISPLAY_SIZE[1]/2.0,SKY_DIRECTION);

    add_2_vector_float(left,lookat,side);
    subtract_2_vector_float(right,lookat,side);
    add_2_vector_float(upper,lookat,sky);
    subtract_2_vector_float(under,lookat,sky);

	outer_product_2_vector_float(LEFT_CULLING,sky,left);
	outer_product_2_vector_float(RIGHT_CULLING,right,sky);
	outer_product_2_vector_float(UPPER_CULLING,upper,side);
	outer_product_2_vector_float(UNDER_CULLING,side,under);

    make_unit_vector_float(LEFT_CULLING);   
    make_unit_vector_float(RIGHT_CULLING);   
    make_unit_vector_float(UPPER_CULLING);   
    make_unit_vector_float(UNDER_CULLING);   
}

void make_gaze_direction(float screen_xy[2], float gaze_direction[3]){
    float lookat[3],sky[3],side[3],result[3];

    multiple_scalar_vector_float(lookat,(double)DISPLAY_SIZE[1]/2.0/tan(VIEW_RANGE*M_PI/360.0),LOOKAT_DIRECTION);
    multiple_scalar_vector_float(side,(double)DISPLAY_SIZE[0]/2.0-screen_xy[0],SIDE_DIRECTION);
    multiple_scalar_vector_float(sky,(double)DISPLAY_SIZE[1]/2.0-screen_xy[1],SKY_DIRECTION);
    add_2_vector_float(result,lookat,sky);
    add_2_vector_float(gaze_direction,result,side);

    make_unit_vector_float(gaze_direction);
//	printf("%lf %lf %lf \n" , gaze_direction[0],gaze_direction[1],gaze_direction[2]);
}

void setting_display(){

	make_gaze_direction(GAZE_POS,GAZE_DIRECTION);
	make_culling_vector();
}

void move_view(){
	double thres = 50;
	static FILE *fp;
	static int iflag=0;
	
#ifdef WRITE_MOVE_VIEW
	if(iflag==0) fp = fopen("movedata.txt","wa");
	iflag = 1;
#else
	if(iflag==0) fp = fopen("dummy_movedata.txt","wa");
	iflag = 1;
#endif

	if(GAZE_POS[0] > DISPLAY_SIZE[0] - thres){
		special(GLUT_KEY_RIGHT,0,0);
		fprintf(fp,"0\n");
	}		
	if(GAZE_POS[0] < thres){
		special(GLUT_KEY_LEFT,0,0);
		fprintf(fp,"1\n");
	}		
	if(GAZE_POS[1] < thres){
		keyboard('5',0,0);
		fprintf(fp,"2\n");
	}		
	if(GAZE_POS[1] > DISPLAY_SIZE[1] - thres){
		keyboard('0',0,0);
		fprintf(fp,"3\n");
	}

}

void move_view_from_file(){
	double thres = 50;
	static FILE *fp;
	static int iflag=0;
	int i;

	if(iflag==0) fp = fopen("movedata.txt","ra");
	iflag = 1;
	
	if(fscanf(fp,"%d",&i) != 1){
		exit(0);		
	}

	switch(i){
	case 0: special(GLUT_KEY_RIGHT,0,0); break;
	case 1: special(GLUT_KEY_LEFT,0,0); break;
	case 2: keyboard('5',0,0); break;
	case 3: keyboard('0',0,0); break;
	}
}

/*=============== 本プログラムの描画部 ===============*/
void  display(void){
	extern int TOP, END;
	int l;
	object_data *o;
	triangle_data *t;
	int num_of_init_tri,i,j;
	static int flame=0,fflag=0;
	static FILE *fp;
	static int f=0;

	static float
		eyepos[3],
		viewcenter[3],
		sky_direction[3],
		eyepos2[3],
		viewcenter2[3],
		sky_direction2[3],
		lookat_direction[3],
		lookat_direction2[3],
		side_direction[3],
		side_direction2[3];

//	timer2();    
//	printf("%lf %lf %lf \n" ,SKY_DIRECTION[0],SKY_DIRECTION[1],SKY_DIRECTION[2]);
//	printf("%lf %lf %lf \n" ,SIDE_DIRECTION[0],SIDE_DIRECTION[1],SIDE_DIRECTION[2]);
//	printf("%lf %lf %lf \n" ,LOOKAT_DIRECTION[0],LOOKAT_DIRECTION[1],LOOKAT_DIRECTION[2]);

	GAZE_POS[0] = MOUSE_POS[0];
	GAZE_POS[1] = MOUSE_POS[1];

	setting_display();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	gluLookAt(EYEPOS[0],EYEPOS[1],EYEPOS[2],
		VIEWCENTER[0],VIEWCENTER[1],VIEWCENTER[2],
		SKY_DIRECTION[0],SKY_DIRECTION[1],SKY_DIRECTION[2]);






	if(f){
		for(l=0;l<NUMBER_OF_LOD_OBJECTS;l++){
//			control_of_level(&LOD_OBJECT_DATAS[l]);
		}
		
		//timer_startstop(1,START);
		Counter= 0;
		Counter2= 0;
		glPushMatrix();
		for(l=0;l<1;l++){
			//		control_of_level(&LOD_OBJECT_DATAS[l]);
			TOP = END = 0;
			display_lod_model_by_number(&LOD_OBJECT_DATAS[l]);////////////////////////////////////
			//		glFlush();
		}
		//	display_lod_model(&LOD_OBJECT);////////////////////////////////////
		glPopMatrix();
//		fprintf(stderr,"%d %d\n",++flame,Counter);
		
		//timer_startstop(1,STOP);
		
		//printf("%d %d %d\n",++flame,Counter,Counter2);
		/*	if(fflag == 0){
		fp = fopen("CountTri.txt","wa");
		fflag=1;
		}
		fprintf(fp,"%d %d\n",++flame,Counter);
		*/
	}
	else{
		f=1;
		glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		//	for(l=0;l<NUMBER_OF_LOD_OBJECTS;l++){// モデルの描画
		//		glCallList(ORIGINAL_OBJECT_DATAS[l].list_id);
		//	}
		
		for(l=0;l<NUMBER_OF_LOD_OBJECTS;l++){
			glBegin(GL_TRIANGLES);
			o = &LOD_OBJECT_DATAS[l];
			t = o->triangles;
			num_of_init_tri = o->num_of_init_tri;
			for(i=0;i<num_of_init_tri;i++){
//				if(t[i].nbr[0] == &BOUNDARY || t[i].nbr[1] == &BOUNDARY || t[i].nbr[2] == &BOUNDARY){
				for(j=0;j<3;j++){
#ifdef FLOAT_VERTEX
					glNormal3fv(t[i].ver[j]->normal);
					glVertex3fv(t[i].ver[j]->pos);
//					glArrayElement(t[i].ver[j]->number);
#else
					glNormal3dv(t[i].ver[j]->normal);
					glVertex3dv(t[i].ver[j]->pos);
#endif
				}
//				}
			}
			glEnd();
		}
		
		glLoadIdentity();
		glPopMatrix();
	}

//	save_pnm_data(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);
//	printf("test\t");
	glutSwapBuffers();
//	compare_images(DISPLAY_SIZE[0],DISPLAY_SIZE[1]);

#ifdef MOVE_VIEW_FROM_FILE
//	move_view_from_file();
#else
//	move_view();
#endif
}

/**********************************************

OpenGL関連部

**********************************************/

/*======== ウインドウのリサイズに合わせて描画領域を変更 ==========*/
void
reshape(GLsizei width, GLsizei height){
  object_data *o;
  double size;
  int i;

  o = &LOD_OBJECT_DATAS[0];
	
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();


        o->size = 0;
        for(i=0;i<3;i++){
                if(o->size < (o->data_range_high[i] - o->data_range_low[i]))
                        o->size = o->data_range_high[i] - o->data_range_low[i];
                printf("data_range[%d] %lf %lf\n",i,o->data_range_high[i],o->data_range_low[i]);
        }
        //OBJECT.size *= 3;
        size = o->size;
        printf("size %lf\n",size);


        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
//        glOrtho(-size,size,-size,size,-size*2,size*2);

		double offset[3];
		offset[0] = (o->data_range_high[0]+o->data_range_low[0])/2.0;
		offset[1] = (o->data_range_high[1]+o->data_range_low[1])/2.0;
		offset[2] = (o->data_range_high[2]+o->data_range_low[2])/2.0;

	glOrtho(-size+offset[0],size+offset[0],-size+offset[1],size+offset[1],-size*2+offset[2],size*2+offset[2]);

	//	glOrtho(-500,500,-500,500,-5000,5000);
//	glOrtho(-1000,1000,-1000,1000,-5000,5000);
//	glutPostRedisplay();
}

void idle(void){
	glutPostRedisplay();
}



/*========== ウインドウが（非）可視領域に入ったときの処置 ==========*/
void Visible(int vis)
{
	if(vis != GLUT_VISIBLE)
		glutIdleFunc(NULL);
	else
		glutIdleFunc(idle);
}

void motion(int x,int y){
	MOUSE_POS[0] = x;
	MOUSE_POS[1] = y;
}

void mouse(int btn, int state, int x, int y){

	if(state == GLUT_DOWN){
		initial_setting();
	}
}

void gl_callback(){
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(special);
	glutVisibilityFunc(Visible);
	glutPassiveMotionFunc(motion);
	glutMouseFunc(mouse);
}

void make_original_models_list(){
	int i,j,l;
	int id_top;
	
	object_data *o;
	int num_of_init_tri;
	triangle_data *t;

    GLfloat
		ambient[4] = {1.0, 1.0, 1.0, 1.0},
		diffuse[4] = {1.0, 1.0, 1.0, 1.0},
		specular[4] = {1.0, 1.0, 1.0, 1.0},
		ns = 255;
	
	glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
	glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diffuse);
	glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
	glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,ns);

	id_top = glGenLists(NUMBER_OF_LOD_OBJECTS);
	printf("%d\n",id_top);
	//glListBase(id_top);

	for(l=0;l<NUMBER_OF_LOD_OBJECTS;l++){
		ORIGINAL_OBJECT_DATAS[l].list_id = id_top+l;
		glNewList(id_top+l,GL_COMPILE);
		
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,ambient);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diffuse);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
		glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,ns);
		glBegin(GL_TRIANGLES);
		o = &ORIGINAL_OBJECT_DATAS[l];
		t = o->triangles;
		num_of_init_tri = o->num_of_init_tri;
		for(i=0;i<num_of_init_tri;i++){
			for(j=0;j<3;j++){
#ifdef FLOAT_VERTEX
				glNormal3fv(t[i].ver[j]->normal);
				glVertex3fv(t[i].ver[j]->pos);
#else
				glNormal3dv(t[i].ver[j]->normal);
				glVertex3dv(t[i].ver[j]->pos);
#endif
			}
		}
		glEnd();

		glEndList();
	}
}



#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<float.h>
#include<iostream>

//#include "set_define.h"
//#include "shape.h"
//#include "file_handle.h"
//#include "list_handle.h"
#include "GeometryHandle.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

#ifdef __LINUX__
int _isnan(float f){};
#endif

class heap_data{
	public:
	int cluster_number;
	double quadrics;
};

void downheap(int n, heap_data a[], int k){
	int j;
	heap_data v;
	v = a[k];			// �e�m�[�h�̒l���m��
	while (1) {			// a[k]���e�m�[�h�Ƃ����؂��T��
		j = 2 * k ;		// ���̎q�̃C���f�b�N�Xj���v�Z
		if (j > n) break;		// j��r���z�����΂����q�m�[�h���Ȃ��Ɣ��f���C���[�v�����яo���D
		if (j != n) {		// j��r���菬�������C
			if (a[j + 1].quadrics > a[j].quadrics) {	// a[j+1]��a[j]�����傫���Ƃ��ɂ́C
				j = j + 1;		// j�����i�߂��D
			}
		}
		if (v.quadrics >= a[j].quadrics) break;	// �e�̕����傫���ꍇ�ɂ̓��[�v�����яo���D
		a[k] = a[j];		// �����ɓ��B�����Ƃ��ɂ́C
		// �q�m�[�h���e�m�[�ha[k]�����傫���̂�a[j]�̒l���������D
		k = j;			// ���̎q�m�[�h���V���Ȑe�m�[�h�ɂ��āC�q�[�v���\���������D
	}
	a[k] = v;			// ���̒l���召�֌W���ۂm�[�h�ɓ������D
}

// ���ڂ��Ă����v�f���K�؂ȏꏊ�܂ŕ��������点��
void upheap(int n, heap_data a[], int k) //n���� a�z�� ���ڔԍ�
{
    heap_data x;

    x = a[k];
	while (k > 1 && a[k/2].quadrics < x.quadrics) {
		a[k] = a[k/2];
		k /= 2;
	}
	a[k] = x;
}



void write_vrml_data(char *file,ObjectShape *wo){
	FILE *ofp;
	int i,cnt=0;
//	extern Triangle BOUNDARY;
//	VertexLink dummy;
	char header[] = "#VRML V2.0 utf8\n\n    Transform {\n      children  Shape {\n                    geometry    IndexedFaceSet {\n                      coord     Coordinate {\n                      point [\n\n";
    char middle[] = "  ]\n    }\n    coordIndex [\n";
	char footer[] = " ]\n                    }\n                }\n     translation 0 0 0\n  }\n";




	if ((ofp = fopen(file,"wb")) == NULL){
		printf("Cannot open file!\n");
		exit(0);
	}

//	fprintf(ofp,"%d %d %d\n", wo->nVerticies,wo->nTriangles,0);
	fprintf(ofp,"%s",header);

	for (i=0; i<wo->nVerticies; i++){
		fprintf(
			ofp,"%lf %lf %lf\n",
			wo->verticies[i].pos[0],
			wo->verticies[i].pos[1],
			wo->verticies[i].pos[2]
		);
		wo->verticies[i].id = i;
	}
	fprintf(ofp,"%s",middle);
	for (i=0;i<wo->nTriangles;i++){
			fprintf(ofp,"%d %d %d -1\n",
				wo->triangles[i].ver[0]->id,
				wo->triangles[i].ver[1]->id,
				wo->triangles[i].ver[2]->id
			);
	}
	fprintf(ofp,"%s",footer);
	fclose(ofp);
}

void
read_stl_data(char *file, ObjectShape *o){
    FILE *fp;
//	char line[256];
	unsigned int itmp=0;
	int cnt = 0;
	int i,j;
	float ftmp[12];

	float *vpos;
	int *id;
	int nh;
	heap_data *heap;
//	vector <Vertex*> heap;
	int *tid;
	float tpx;
	int ttid;


    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }

	fseek(fp,80,SEEK_SET);



	o->nTriangles = o->nVerticies = 0;

	int size= fread(&itmp,4,1,fp);
	if(!size) cout << "can't read file" << endl;
	cnt = 0;
	o->nTriangles = itmp;

//	printf("%ld\n",o->nTriangles);

	vpos = (float *)malloc(sizeof(float)*9*o->nTriangles);
	id = (int *)malloc(sizeof(int)*o->nTriangles*3);
	tid = (int *)malloc(sizeof(int)*o->nTriangles*3);



	heap = (heap_data *)malloc(sizeof(heap_data)*(o->nTriangles*3+1));

	if(vpos == NULL) {
		printf("not enough memory");
		exit(0);
	}

	for(i=0;i<o->nTriangles;i++){
		int size = fread(&ftmp,sizeof(float),12,fp);
		if(!size) cout << "can't read file" << endl;

		for(j=3;j<12;j++){
			vpos[i*9+j-3] = ftmp[j];
		}
		size = fread(&ftmp,2,1,fp);
		if(!size) cout << "can't read file" << endl;
	}

	nh = 0;
	for(i=0;i<o->nTriangles*3;i++){
		id[i] = i;
		nh++;
		heap[nh].cluster_number = i;
		heap[nh].quadrics = vpos[i*3];
		upheap(nh,heap,nh);
	}

	tpx = heap[1].quadrics;
	cnt = 0;
	for(i=0;i<o->nTriangles*3;i++){
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
		heap[1] = heap[nh];
		downheap(--nh,heap,1);
	}

	cnt = 0;

	for(i=0;i<o->nTriangles*3;i++){
		if(id[i] == i) id[i] = cnt++;
		else id[i] = id[id[i]];
	}
	printf("%d\n",cnt);

	o->nVerticies = cnt;
	o->nTriangles = o->nTriangles;
//	o->nClusters += o->nTriangles;

	//o->verticies = (VertexLink *)malloc(sizeof(VertexLink)*o->nVerticies);
	//o->triangles = (Triangle *)malloc(sizeof(Triangle)*o->nTriangles);

	o->verticies = new VertexLink[o->nVerticies];
	o->triangles =  new Triangle[o->nTriangles];

//	o->clusters = (cluster_data *)malloc(sizeof(cluster_data)*o->nClusters);

	for(i=0;i<o->nTriangles*3;i++){
		for(j=0;j<3;j++){
			o->verticies[id[i]].pos[j] = vpos[i*3+j];
		}
	}

	for(i=0;i<o->nTriangles;i++){
		for(j=0;j<3;j++){
			o->triangles[i].ver[j] = &o->verticies[id[3*i+j]];
		}
	}

	free(vpos);
	free(id);
	free(tid);
	free(heap);

	fclose(fp);

}

void write_hrp_vrml_data(char *file){
        FILE *ofp;
        FILE *ifp;
        char line[256];
        std::string sline;

        std::string base = strtok(file, ".");
        ofp = fopen((base+"hrp.wrl").c_str(),"wb" );

        int cnt =0;
				ifp =  fopen("/home/user/choreonoid/bin/hrp.wrl","rb" );

	if(ifp==NULL){
		printf("hrp.wrl is needed in this folder");
		return;
	}

    while(fgets(line,256,ifp) != NULL){
                if(strstr(line,"__name__")){
                        switch (cnt){
                                case 0:
                                        sline = "DEF " + base + " Humanoid {\n";
                                        break;
                                case 1:
                                        sline = "url \""+base+".wrl\"\n";
                                        break;
                                case 2:
                                        sline = "name \""+ base+"\"\n";
                                        break;
                        }
                        cnt++;
                        fputs(sline.c_str(),ofp);
                }else{
                        fputs(line,ofp);
                }
        }
}


void write_stl_data(char *file, ObjectShape *o){
    FILE *fp;
	float ftmp[12];
	char temp[81];



    if ((fp = fopen(file,"wb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }
	fwrite(&temp,80,1,fp);

	int size= fwrite(&o->nTriangles,4,1,fp);

	for(int i=0;i<o->nTriangles;i++){
		for(int j=0;j<3;j++){
			ftmp[j]=0;
		}
		int cnt=3;
		for(int j=0;j<3;j++){
			for(int k=0;k<3;k++){
				ftmp[cnt++] = o->triangles[i].ver[j]->pos[k];
			}
		}
		fwrite(&ftmp,sizeof(float),12,fp);

		short int zero=0;
		fwrite(&zero,2,1,fp);
	}

	fclose(fp);

}

void read_ply_ascii_data(char *file,ObjectShape *o){
    FILE *fp;
    int i,j;
    int tmp[6],itmp;
	char line[256];


    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ascii file!\n");
        exit(0);
    }
	bool r = fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	r = fgets(line,256,fp);
    if(memcmp(line,"format ascii 1.0",16)!=0){
		printf("Not format ascii 1.0\n");
		exit(0);
	}

    while(fgets(line,256,fp) != NULL){
		if(memcmp(line,"element vertex",14)==0){
            sscanf(line,"element vertex %d",&o->nVerticies);
        }
		if(memcmp(line,"element face",12)==0){
            sscanf(line,"element face %d",&o->nTriangles);
        }
        if(memcmp(line,"end_header",6)==0){
			break;
        }
	}


    o->verticies = (VertexLink *)malloc(sizeof(VertexLink)*o->nVerticies);
    o->triangles = (Triangle *)malloc(sizeof(Triangle)*o->nTriangles);

	if(o->verticies == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
	if(o->triangles == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
    for (i=0; i<o->nVerticies; i++){
		r = fgets(line,256,fp);
#ifdef FLOAT_VERTEX
        sscanf(line,"%f%f%f",
#else
		sscanf(line,"%lf%lf%lf",
#endif
            &o->verticies[i].pos[0],
            &o->verticies[i].pos[1],
            &o->verticies[i].pos[2]
			);
    }
    for (i=0; i<o->nTriangles; i++){
		r = fgets(line,256,fp);
        sscanf(line,"%d%d%d%d",&itmp,tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->verticies[tmp[j]];
    }
    fclose(fp);
}


void read_ply_bindary_big_endian_data(char *file,ObjectShape *o){
    FILE *fp;
    int i,j,cnt;
    int tmp[6],itmp;
	char line[256];
	char *tpoint;
	unsigned char uctmp;
	float ftmp[3];


    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ply file!\n");
        exit(0);
    }
	bool r = fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	r = fgets(line,256,fp);
    if(memcmp(line,"format binary_big_endian 1.0",28)!=0){
		printf("Not format binary_big_endian 1.0\n");
		exit(0);
	}

    while(fgets(line,256,fp) != NULL){
		if(memcmp(line,"element vertex",14)==0){
            sscanf(line,"element vertex %d",&o->nVerticies);
        }
		if(memcmp(line,"element face",12)==0){
            sscanf(line,"element face %d",&o->nTriangles);
        }
        if(memcmp(line,"end_header",6)==0){
			break;
        }
	}

    o->verticies = (VertexLink *)malloc(sizeof(VertexLink)*o->nVerticies);
    o->triangles = (Triangle *)malloc(sizeof(Triangle)*o->nTriangles);

	if(o->verticies == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
	if(o->triangles == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}

    for (i=0; i<o->nVerticies; i++){
		r= fread(line,sizeof(float)*6+sizeof(char)*3,1,fp);
		cnt = 0;
        for (j=0; j<3; j++){
			tpoint = (char *)ftmp;
			tpoint[3] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[0] = line[cnt++];
			o->verticies[i].pos[j] =  ftmp[0];
		}
	}
    for (i=0; i<o->nTriangles; i++){
		r = fread(&uctmp,sizeof(char),1,fp);
		r = fread(line,sizeof(int)*uctmp+3,1,fp);
		if(uctmp != 3) printf("not triangle %d",uctmp);
		cnt = 0;
		for(j=0;j<3;j++){
			tpoint = (char *)&itmp;
			tpoint[3] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[0] = line[cnt++];
            o->triangles[i].ver[j] = &o->verticies[itmp];
		}
/*
        sscanf(line,"%d%d%d%d",&itmp,tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->verticies[tmp[j]];
*/    }
    fclose(fp);
}

void read_ply_bindary_little_endian_data(char *file,ObjectShape *o){
    FILE *fp;
    int i,j,cnt;
    int tmp[6],itmp;
	char line[256];
	char *tpoint;
	unsigned char uctmp;
	float ftmp[3];


    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ply file!\n");
        exit(0);
    }
	bool r = fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	r = fgets(line,256,fp);
    if(memcmp(line,"format binary_little_endian 1.0",28)!=0){
		printf("Not format binary_little_endian 1.0\n");
		exit(0);
	}

    while(fgets(line,256,fp) != NULL){
		if(memcmp(line,"element vertex",14)==0){
            sscanf(line,"element vertex %d",&o->nVerticies);
        }
		if(memcmp(line,"element face",12)==0){
            sscanf(line,"element face %d",&o->nTriangles);
        }
        if(memcmp(line,"end_header",6)==0){
			break;
        }
	}

    o->verticies = (VertexLink *)malloc(sizeof(VertexLink)*o->nVerticies);
    o->triangles = (Triangle *)malloc(sizeof(Triangle)*o->nTriangles);

	if(o->verticies == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}
	if(o->triangles == NULL){
		fprintf(stderr, "Give me enough memory\n");
		exit(0);
	}

    for (i=0; i<o->nVerticies; i++){
		r = fread(line,sizeof(float)*5,1,fp);
		cnt = 0;
        for (j=0; j<3; j++){
			tpoint = (char *)ftmp;
			tpoint[0] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[3] = line[cnt++];
			o->verticies[i].pos[j] =  ftmp[0];
		}
	}
    for (i=0; i<o->nTriangles; i++){
		r = fread(&uctmp,sizeof(char),1,fp);
		r = fread(line,sizeof(int)*uctmp,1,fp);
		if(uctmp != 3) printf("not triangle %d",uctmp);
		cnt = 0;
		for(j=0;j<3;j++){
			tpoint = (char *)&itmp;
			tpoint[0] = line[cnt++];
			tpoint[1] = line[cnt++];
			tpoint[2] = line[cnt++];
			tpoint[3] = line[cnt++];
            o->triangles[i].ver[j] = &o->verticies[itmp];
		}
/*
        sscanf(line,"%d%d%d%d",&itmp,tmp,tmp+1,tmp+2);
        for (j=0; j<3; j++)
            o->triangles[i].ver[j] = &o->verticies[tmp[j]];
*/    }
    fclose(fp);
}

void read_ply_data(char *file,ObjectShape *o){
    FILE *fp;
    int i,j;
    int tmp[6],itmp;
	char line[256];
	float ftmp[3];

    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open ply file!\n");
        exit(0);
    }
    bool r= fgets(line,256,fp);
    if(memcmp(line,"ply",3)!=0){
		printf("Not ply file\n");
		exit(0);
	}
	r = fgets(line,256,fp);
    if(memcmp(line,"format ascii 1.0",16)==0){
		fclose(fp);
		printf("Format ascii 1.0\n");
		read_ply_ascii_data(file,o);
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

void read_quad_data(char *file, ObjectShape *o){
    FILE *fp;
    char line[256];
    int count_of_ver=0,count_of_tri=0,count_of_color=0;
    int i,j,itemp,flag;
	double temp_pos[3];

    if ((fp = fopen(file,"rb")) == NULL){
        printf("Cannot open obj file!!  \n");
        exit(0);
    }
	o->nTriangles = o->nVerticies = 0;



	//count data
	bool r = fgets(line,256,fp);
	while(fgets(line,256,fp) != NULL){
//		printf("tt%s",line);
		//{ OFF 3 1 1 # f49
		flag = sscanf(line,"{ OFF%d",&itemp);
		if(flag==0) continue;
		count_of_ver+=itemp;
		count_of_tri+=itemp-2;
		for(i=0;i<itemp+1;i++){
			r = fgets(line,256,fp);
//			printf("t1t%s",line);
		}
	}

	o->nTriangles =count_of_tri;
	o->nVerticies=count_of_ver;
	o->verticies = (VertexLink *)malloc(sizeof(VertexLink)*o->nVerticies);
	o->triangles = (Triangle *)malloc(sizeof(Triangle)*o->nTriangles);

	fseek(fp,0,SEEK_SET);


	count_of_ver = count_of_tri = 0;
	while(fgets(line,256,fp) != NULL){
		flag = sscanf(line,"{ OFF%d",&itemp);
		if(flag==0) continue;
		for(i=0;i<itemp;i++){
			bool r = fgets(line,256,fp);
			sscanf(line,"%lf%lf%lf",temp_pos,temp_pos+1,temp_pos+2);
			o->verticies[count_of_ver+i].pos[0] = temp_pos[0];
			o->verticies[count_of_ver+i].pos[1] = temp_pos[1];
			o->verticies[count_of_ver+i].pos[2] = temp_pos[2];
		}
		bool r = fgets(line,256,fp);
		//for(i=0;i<itemp;i++){
		//	o->verticies[count_of_ver+i].pos[0] = temp_pos[0];
		//	o->verticies[count_of_ver+i].pos[1] = temp_pos[1];
		//	o->verticies[count_of_ver+i].pos[2] = temp_pos[2];
		//}
		for(i=0;i<itemp-2;i++){
			o->triangles[count_of_tri+i].ver[0] = &o->verticies[count_of_ver];
			for(j=1;j<3;j++){
				o->triangles[count_of_tri+i].ver[j] = &o->verticies[count_of_ver+(j+i)];
			}
		}
		count_of_ver+=itemp;
		count_of_tri+=itemp-2;
//		fgets(line,256,fp);
	}

	printf("%d %d",count_of_ver,count_of_tri);

//	exit(0);



	fclose(fp);

}


void read_mesh_data(char *name,ObjectShape *o){
	if(strstr(name,".stl")){
//		strcpy(o->filetype,"stl");
		read_stl_data(name,o);
		return;
	}
	if(strstr(name,".ply")){
//		strcpy(o->filetype,"ply");
		read_ply_data(name,o);
		return;
	}
/*	if(strstr(name,".tri")){
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
*/	if(strstr(name,".quad")){
//		strcpy(o->filetype,"quad");
		read_quad_data(name,o);
		return;
	}

	printf("UnKnown Format %s\n ",name);
	exit(-1);

}

void write_mesh_data(char *name,ObjectShape *o){
	if(strstr(name,".stl")){
//		strcpy(o->filetype,"stl");
		write_stl_data(name,o);
		return;
	}
	if(strstr(name,".wrl")){
//		strcpy(o->filetype,"ply");
		write_vrml_data(name,o);
		return;
	}
/*	if(strstr(name,".tri")){
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
*/	if(strstr(name,".quad")){
//		strcpy(o->filetype,"quad");
		read_quad_data(name,o);
		return;
	}

	printf("UnKnown Format %s\n ",name);
	exit(-1);

}



int main(int argc, char* argv[]){
	if (argc != 3 ){
		cout << "program inputfile outputfile\n" << endl;
		exit(0);
	}
	ObjectShape object;
	read_mesh_data(argv[1], &object);
	write_mesh_data(argv[2], &object);
	write_hrp_vrml_data(argv[2]);

	cout << "Finish " << endl;
}

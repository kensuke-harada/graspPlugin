#include "ObjectShape.h"
#include <fstream>

using namespace grasp;
using namespace cnoid;
using namespace std;

#define WIDTH 320
#define HEIGHT 240

void
read_csv_data(char *file, ObjectShape *o){
	ifstream ifp (file);

	o->nTriangles = o->nVerticies = 0;
	if (!ifp){
		cout <<  "Cannot open obj file!!"  << endl;
		exit(0);
	}
	string line;
	for(int i=0;i<4;i++) getline(ifp, line);
	
	ifp.get(&line[0],25);
	
//	for(int i=0;i<20;i++){
//		getline(ifp, line);
	cout << line << endl;
	
	int cnt = 0;
	
	Vector3 vertexArray[HEIGHT][WIDTH];
	o->verticies = new VertexLink[WIDTH*HEIGHT];
	for(int i=0;i<HEIGHT;i++){
		for(int j=0;j<WIDTH;j++){
			for(int k=0;k<3;k++){
				 ifp >> vertexArray[i][j][k];
				ifp.get();
			}
			o->verticies[cnt].pos = vertexArray[i][j];
			o->verticies[cnt].id = cnt;
			cnt++;
		}
		ifp.get();
		ifp.get();
	}
	o->nVerticies = cnt;
	
}

void make_mesh_data(ObjectShape *o){
	int cnt2=0;
	o->triangles =  new Triangle[WIDTH*HEIGHT*2];
	for(int i=0;i<HEIGHT-1;i++){
		for(int j=0;j<WIDTH-1;j++){
			VertexLink* temp[4];
			temp[0] = &o->verticies[j+i*WIDTH];
			temp[1] = &o->verticies[j+1+i*WIDTH];
			temp[2] = &o->verticies[j+1+(i+1)*WIDTH];
			temp[3] = &o->verticies[j+(i+1)*WIDTH];
			if(temp[0]->pos[2] < -0.5) continue;
			if(temp[1]->pos[2] < -0.5) continue;
			if(temp[2]->pos[2] < -0.5) continue;
			if(temp[3]->pos[2] < -0.5) continue;
			if((temp[0]->pos - temp[1]->pos).norm() > 0.01) continue;
			if((temp[1]->pos - temp[2]->pos).norm() > 0.01) continue;
			if((temp[2]->pos - temp[3]->pos).norm() > 0.01) continue;
			if((temp[3]->pos - temp[0]->pos).norm() > 0.01) continue;
			
			o->triangles[cnt2].ver[0] = temp[0];
			o->triangles[cnt2].ver[1] = temp[1];
			o->triangles[cnt2].ver[2] = temp[2];
			cnt2++;
			o->triangles[cnt2].ver[0] = temp[0];
			o->triangles[cnt2].ver[1] = temp[2];
			o->triangles[cnt2].ver[2] = temp[3];
			cnt2++;
		}
	}
	o->nTriangles = cnt2;
}

void average_csv_data(vector<ObjectShape*> objects){
	
	for(int j=0;j<objects[0]->nVerticies;j++){
		if(objects[0]->verticies[j].pos[2] < -0.5) continue;
		int cnt = 1;
		for(int i=1;i<objects.size();i++){
			if(objects[i]->verticies[j].pos[2] < -0.5) continue;
			objects[0]->verticies[j].pos +=  objects[i]->verticies[j].pos;
			cnt++;
		}
		objects[0]->verticies[j].pos /= (double)cnt;
	}
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

void write_hrp_vrml_data(char *file){
        FILE *ofp;
        FILE *ifp;
        char line[256];
        std::string sline;
        
        std::string base = strtok(file, ".");
        ofp = fopen((base+"hrp.wrl").c_str(),"wb" );
        ifp =  fopen("hrp.wrl","rb" );
        
        int cnt =0;
	
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



int main(int argc, char* argv[]){
	if (argc < 3 ){
		cout << "program inputfile outputfile\n" << endl;
		exit(0);
	}
	cout << argc << endl;
	vector<ObjectShape*> objects;
	for(int i=1;i<argc-1;i++){
		ObjectShape* object = new ObjectShape;
		read_csv_data(argv[i],object);
		objects.push_back(object);
	}
	average_csv_data(objects);
	make_mesh_data(objects[0]);
//	read_mesh_data(argv[1], &object);
	write_vrml_data(argv[argc-1], objects[0]);
	write_hrp_vrml_data(argv[argc-1]);
	
	cout << "Finish " << endl;
}
